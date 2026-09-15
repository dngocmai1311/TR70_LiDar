// lidar_web_gateway.cpp
// Backend WebSocket gateway cho 4 LiDAR
// Nhận dữ liệu từ LiDAR SDK và đẩy lên trình duyệt.

#define _WEBSOCKETPP_CPP11_STL_
#define ASIO_STANDALONE
#define _WIN32_WINNT 0x0601

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>

#include "ladar_api.h"

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <set>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <cstring>
#include <chrono>

#ifdef _WIN32
#pragma comment(lib, "ladarsdk_gd.lib")
#endif

using json = nlohmann::json;

using WsServer = websocketpp::server<websocketpp::config::asio>;
using ConnHdl = websocketpp::connection_hdl;

static const double PI_D = 3.14159265358979323846;

static WsServer g_server;
static std::set<ConnHdl, std::owner_less<ConnHdl>> g_conns;
static std::mutex g_connMutex;

//---------------------------------------------------------
// Helpers: append little-endian binary
//---------------------------------------------------------
static void appendU8(std::vector<uint8_t>& b, uint8_t v)
{
    b.push_back(v);
}

static void appendU16(std::vector<uint8_t>& b, uint16_t v)
{
    b.push_back(uint8_t(v & 0xFF));
    b.push_back(uint8_t((v >> 8) & 0xFF));
}

static void appendU32(std::vector<uint8_t>& b, uint32_t v)
{
    b.push_back(uint8_t(v & 0xFF));
    b.push_back(uint8_t((v >> 8) & 0xFF));
    b.push_back(uint8_t((v >> 16) & 0xFF));
    b.push_back(uint8_t((v >> 24) & 0xFF));
}

static void appendF32(std::vector<uint8_t>& b, float v)
{
    uint8_t tmp[4];
    std::memcpy(tmp, &v, 4);
    b.insert(b.end(), tmp, tmp + 4);
}

//---------------------------------------------------------
// Extrinsic
//---------------------------------------------------------
struct Extrinsic2D
{
    float dx = 0.0f;
    float dy = 0.0f;
    float theta = 0.0f; // rad
};

class LidarSource;

static std::unordered_map<ILadar*, LidarSource*> g_lidarMap;
static std::mutex g_lidarMapMutex;

//---------------------------------------------------------
// LidarSource
//---------------------------------------------------------
class LidarSource
{
public:
    int id = 0;
    std::string ip;
    uint16_t port = 1112;

    ILadar* ladar = nullptr;
    ILadarPoller* poller = nullptr;
    std::thread pollerThread;

    std::atomic<bool> running{ false };
    std::atomic<uint32_t> frameCounter{ 0 };
    std::atomic<uint32_t> publishedId{ 0 };

    std::shared_ptr<std::vector<uint8_t>> latest;
    std::mutex frameMutex;
    std::mutex extrMutex;

    Extrinsic2D extr;

    LidarSource(int id_, const std::string& ip_, uint16_t port_, const Extrinsic2D& e)
        : id(id_), ip(ip_), port(port_), extr(e)
    {}

    ~LidarSource()
    {
        Stop();
    }

    void SetExtr(const Extrinsic2D& e)
    {
        std::lock_guard<std::mutex> lk(extrMutex);
        extr = e;
    }

    Extrinsic2D GetExtr()
    {
        std::lock_guard<std::mutex> lk(extrMutex);
        return extr;
    }

    static void OnFrameCallback(ILadar* ladar_, const uint8_t* frame, int frameLength)
    {
        LidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_lidarMapMutex);
            auto it = g_lidarMap.find(ladar_);
            if (it != g_lidarMap.end())
                self = it->second;
        }

        if (!self)
            return;

        if (frameLength < 4)
            return;

        // Code gốc đang lọc cmdWord == 110.
        // Nếu thiết bị của bạn trả frame dữ liệu khác, hãy chỉnh lại chỗ này.
        uint16_t cmdWord = uint16_t(frame[2] | (frame[3] << 8));
        if (cmdWord != 110)
            return;

        ladar_data_t data;
        int ret = ladar_->GetData(data, frame, frameLength);
        if (ret <= 0)
            return;

        uint32_t fid = self->frameCounter.fetch_add(1);
        Extrinsic2D e = self->GetExtr();

        // Đếm số điểm hợp lệ
        uint32_t count = 0;
        float angle = data.angle_min;
        for (size_t i = 0; i < data.ranges.size(); ++i, angle += data.angle_increment)
        {
            float d = data.ranges[i];
            if (d > 0.0f && d < data.range_max)
                count++;
        }

        std::vector<uint8_t> buf;
        buf.reserve(56 + size_t(count) * 12);

        // Header
        appendU8(buf, 1);                  // msgType
        appendU8(buf, uint8_t(self->id));  // sourceId
        appendU16(buf, 0);                 // reserved
        appendU32(buf, fid);
        appendU32(buf, data.ts_secs);
        appendU32(buf, data.ts_usecs);

        appendF32(buf, e.dx);
        appendF32(buf, e.dy);
        appendF32(buf, e.theta);

        appendF32(buf, data.angle_min);
        appendF32(buf, data.angle_max);
        appendF32(buf, data.angle_increment);
        appendF32(buf, data.range_min);
        appendF32(buf, data.range_max);
        appendF32(buf, data.scan_time);

        appendU32(buf, count);

        // Points: local x, y, intensity
        angle = data.angle_min;
        for (size_t i = 0; i < data.ranges.size(); ++i, angle += data.angle_increment)
        {
            float d = data.ranges[i];
            if (d <= 0.0f || d >= data.range_max)
                continue;

            float lx = d * cosf(angle);
            float ly = d * sinf(angle);

            float intensity = 0.0f;
            if (i < data.intensities.size())
                intensity = data.intensities[i];

            appendF32(buf, lx);
            appendF32(buf, ly);
            appendF32(buf, intensity);
        }

        {
            std::lock_guard<std::mutex> lk(self->frameMutex);
            self->latest = std::make_shared<std::vector<uint8_t>>(std::move(buf));
        }

        self->publishedId.store(fid);
    }

    static void OnStatusCallback(ILadar* ladar_, uint32_t oldStatus, uint32_t status)
    {
        (void)oldStatus;

        LidarSource* self = nullptr;
        {
            std::lock_guard<std::mutex> lk(g_lidarMapMutex);
            auto it = g_lidarMap.find(ladar_);
            if (it != g_lidarMap.end())
                self = it->second;
        }

        if (!self)
            return;

        if (status == csConnected)
        {
            ladar_->SetTime();
            std::cout << "[LiDAR " << self->id + 1 << "] connected, SetTime OK\n";
        }
        else if (status == csClosed)
        {
            std::cout << "[LiDAR " << self->id + 1 << "] closed\n";
        }
        else if (status == csConnecting)
        {
            std::cout << "[LiDAR " << self->id + 1 << "] connecting...\n";
        }
        else if (status == csConnectError)
        {
            std::cout << "[LiDAR " << self->id + 1 << "] connect error\n";
        }
    }

    bool Start()
    {
        if (running.load())
            return true;

        poller = CreateLadarPoller();
        if (!poller)
        {
            std::cout << "[ERROR] CreateLadarPoller failed for LiDAR " << id + 1 << "\n";
            return false;
        }

        ladar = poller->AddLadar(ip.c_str(), port, OnFrameCallback, OnStatusCallback);
        if (!ladar)
        {
            DeleteLadarPoller(poller);
            poller = nullptr;
            std::cout << "[ERROR] AddLadar failed: " << ip << ":" << port << "\n";
            return false;
        }

        {
            std::lock_guard<std::mutex> lk(g_lidarMapMutex);
            g_lidarMap[ladar] = this;
        }

        running = true;

        pollerThread = std::thread([this]() {
            poller->Run();
            });

        std::cout << "[OK] Started LiDAR " << id + 1 << " -> " << ip << ":" << port << "\n";
        return true;
    }

    void Stop()
    {
        if (!running.load())
            return;

        running = false;

        if (ladar)
        {
            std::lock_guard<std::mutex> lk(g_lidarMapMutex);
            g_lidarMap.erase(ladar);
        }

        if (poller)
        {
            poller->Stop();

            if (pollerThread.joinable())
                pollerThread.join();

            DeleteLadarPoller(poller);
            poller = nullptr;
            ladar = nullptr;
        }

        std::cout << "[STOPPED] LiDAR " << id + 1 << "\n";
    }
};

static std::vector<std::unique_ptr<LidarSource>> g_sources;

//---------------------------------------------------------
// WebSocket helpers
//---------------------------------------------------------
static bool HasClients()
{
    std::lock_guard<std::mutex> lk(g_connMutex);
    return !g_conns.empty();
}

static void SendText(ConnHdl hdl, const std::string& text)
{
    websocketpp::lib::error_code ec;
    g_server.send(hdl, text, websocketpp::frame::opcode::value::text, ec);
}

static void BroadcastBinary(const std::vector<uint8_t>& payload)
{
    if (payload.empty())
        return;

    std::lock_guard<std::mutex> lk(g_connMutex);

    for (auto hdl : g_conns)
    {
        websocketpp::lib::error_code ec;
        g_server.send(
            hdl,
            payload.data(),
            payload.size(),
            websocketpp::frame::opcode::value::binary,
            ec
        );
    }
}

static void HandleMessage(ConnHdl hdl, WsServer::message_ptr msg)
{
    try
    {
        auto j = json::parse(msg->get_payload());
        std::string cmd = j.value("cmd", "");

        if (cmd == "set_extrinsic")
        {
            int id = j.at("id").get<int>();

            if (id >= 0 && id < int(g_sources.size()))
            {
                Extrinsic2D e;
                e.dx = float(j.value("dx", 0.0));
                e.dy = float(j.value("dy", 0.0));
                double thetaDeg = j.value("thetaDeg", 0.0);
                e.theta = float(thetaDeg * PI_D / 180.0);

                g_sources[id]->SetExtr(e);

                json ack;
                ack["type"] = "ack";
                ack["cmd"] = "set_extrinsic";
                ack["id"] = id;
                ack["dx"] = e.dx;
                ack["dy"] = e.dy;
                ack["thetaDeg"] = thetaDeg;

                SendText(hdl, ack.dump());

                std::cout << "[EXTRINSIC] LiDAR " << id + 1
                    << " dx=" << e.dx
                    << " dy=" << e.dy
                    << " thetaDeg=" << thetaDeg << "\n";
            }
        }
        else if (cmd == "get_state")
        {
            json arr = json::array();

            for (auto& s : g_sources)
            {
                Extrinsic2D e = s->GetExtr();

                json item;
                item["id"] = s->id;
                item["ip"] = s->ip;
                item["port"] = s->port;
                item["dx"] = e.dx;
                item["dy"] = e.dy;
                item["thetaDeg"] = e.theta * 180.0 / PI_D;

                arr.push_back(item);
            }

            json out;
            out["type"] = "state";
            out["lidars"] = arr;

            SendText(hdl, out.dump());
        }
        else
        {
            SendText(hdl, "{\"error\":\"unknown command\"}");
        }
    }
    catch (...)
    {
        SendText(hdl, "{\"error\":\"bad command\"}");
    }
}

//---------------------------------------------------------
// main
//---------------------------------------------------------
int main()
{
    std::cout << "LiDAR Web Gateway\n";

    // Sửa IP tại đây nếu cần
    const char* ips[4] = {
        "192.168.201.15",
        "192.168.201.16",
        "192.168.201.17",
        "192.168.201.18"
    };

    const uint16_t port = 1112;

    Extrinsic2D extrs[4];
    extrs[0] = { 0.0f, 0.0f, 0.0f };
    extrs[1] = { 5.0f, 0.0f, 0.0f };
    extrs[2] = { 0.0f, 5.0f, float(90.0 * PI_D / 180.0) };
    extrs[3] = { 5.0f, 5.0f, float(180.0 * PI_D / 180.0) };

    for (int i = 0; i < 4; ++i)
    {
        auto src = std::make_unique<LidarSource>(i, ips[i], port, extrs[i]);
        src->Start();
        g_sources.push_back(std::move(src));
    }

    try
    {
        g_server.init_asio();
        g_server.set_reuse_addr(true);

        g_server.set_open_handler([](ConnHdl hdl) {
            std::lock_guard<std::mutex> lk(g_connMutex);
            g_conns.insert(hdl);
            std::cout << "[WEB] client connected\n";
            });

        g_server.set_close_handler([](ConnHdl hdl) {
            std::lock_guard<std::mutex> lk(g_connMutex);
            g_conns.erase(hdl);
            std::cout << "[WEB] client disconnected\n";
            });

        g_server.set_message_handler([](ConnHdl hdl, WsServer::message_ptr msg) {
            HandleMessage(hdl, msg);
            });

        g_server.listen(9002);
        g_server.start_accept();

        std::cout << "WebSocket server listening on ws://0.0.0.0:9002\n";
    }
    catch (std::exception& ex)
    {
        std::cout << "[ERROR] WebSocket init: " << ex.what() << "\n";
        return 1;
    }

    std::thread wsThread([]() {
        try
        {
            g_server.run();
        }
        catch (...)
        {
        }
        });

    std::atomic<bool> run{ true };

    std::thread broadcastThread([&]() {
        uint32_t lastIds[4] = { 0, 0, 0, 0 };

        while (run.load())
        {
            if (HasClients())
            {
                for (int i = 0; i < 4 && i < int(g_sources.size()); ++i)
                {
                    auto& s = g_sources[i];

                    uint32_t pid = s->publishedId.load();
                    if (pid != lastIds[i])
                    {
                        std::shared_ptr<std::vector<uint8_t>> frame;
                        {
                            std::lock_guard<std::mutex> lk(s->frameMutex);
                            frame = s->latest;
                        }

                        if (frame)
                            BroadcastBinary(*frame);

                        lastIds[i] = pid;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        });

    std::cout << "Press q + Enter to quit.\n";

    char c;
    while (std::cin.get(c))
    {
        if (c == 'q' || c == 'Q')
            break;
    }

    run = false;

    if (broadcastThread.joinable())
        broadcastThread.join();

    g_server.stop_listening();
    g_server.stop();

    if (wsThread.joinable())
        wsThread.join();

    for (auto& s : g_sources)
        s->Stop();

    std::cout << "Exited.\n";
    return 0;
}