#pragma once

//---------------------------------------------------------------------------
#include <cstdint>
#include <vector>
#include <string>
#include <functional>

//---------------------------------------------------------------------------
#define csClosed       ((uint32_t)(0x00))
#define csConnecting   ((uint32_t)(0x01))
#define csConnected    ((uint32_t)(0x02))
#define csConnectError ((uint32_t)(0x03))

//---------------------------------------------------------------------------
#define dsLogin     ((uint32_t)(0x01))
#define dsParam     ((uint32_t)(0x02))
#define dsDataFmt   ((uint32_t)(0x04))
#define dsSyncInfo  ((uint32_t)(0x08))
#define dsTimeStamp ((uint32_t)(0x10))
#define dsStandby   ((uint32_t)(0x20))

//---------------------------------------------------------------------------
#define wtNone     ((uint32_t)(0x00))
#define wtFar      ((uint32_t)(0x01))
#define wtBig      ((uint32_t)(0x02))
#define wtNear     ((uint32_t)(0x03))
#define wtFar2     ((uint32_t)(0x04))

//---------------------------------------------------------------------------
class ILadarPoller;
struct sockaddr_in;

//---------------------------------------------------------------------------
#ifndef GLSDK_API
#ifdef _WIN32
#define GLSDK_API extern "C" __declspec(dllimport)
#else
#define GLSDK_API
#endif
#endif

//---------------------------------------------------------------------------
GLSDK_API ILadarPoller* CreateLadarPoller();
GLSDK_API void DeleteLadarPoller(ILadarPoller* poller);

//---------------------------------------------------------------------------
typedef struct ladar_param_t
{
	uint32_t flags;           //dsLogin, dsParam, dsDataFmt位域
	float radius;             //雷达扫描半径：米，dsParam有效时
	float ticks;              //雷达扫描周期：秒，dsParam有效时
	float angle_start;        //雷达数据输出起始角度：弧度，dsDataFmt有效时
	float angle_stop;         //雷达数据输出终止角度（不包含此角度）：弧度，dsDataFmt有效时
	float angle_step;         //雷达数据输出角分辨率：弧度，dsDataFmt有效时
	int32_t wave_count;       //雷达输出数据的回波个数，dsDataFmt有效时
	uint32_t wave_types[2];   //雷达输出数据的回波，dsDataFmt有效时
	int32_t intensity;        //1/0，雷达输出数据是否包含反射率，dsDataFmt有效时

	ladar_param_t()
	{
		this->flags = 0;
		this->radius = 0;
		this->ticks = 0;
		this->angle_start = 0;
		this->angle_stop = 0;
		this->angle_step = 0;
		this->wave_count = 0;
		this->wave_types[0] = wtNone;
		this->wave_types[1] = wtNone;
		this->intensity = 0;
	}
} ladar_param_t;

//---------------------------------------------------------------------------
typedef struct ladar_data_t
{
	uint32_t flags;                  //dsSyncInfo与dsTimeStamp的组合，表示对应信息可用
	float range_min;                 //米，雷达数据输出最小距离
	float range_max;                 //米，雷达数据输出最大有效距离
	float angle_min;                 //弧度，雷达数据输出最小角度
	float angle_max;                 //弧度，雷达数据输出最大角度
	float angle_increment;           //弧度，雷达数据输出两个点最小角度间隔，角度分辨率
	float scan_time;                 //秒，扫描一圈儿的时间
	float time_increment;            //秒，扫描一个点的时间
	int32_t wave_count;              //数据回波个数
	uint32_t ts_syncs;               //雷达同步数据，flags中包含dsSyncInfo时可用
	uint32_t ts_secs;                //雷达时间戳（秒），flags中包含dsTimeStamp时可用
	uint32_t ts_usecs;               //雷达时间戳（微秒），flags中包含dsTimeStamp时可用
	std::vector<float> ranges;       //按照角度从angle_min到angle_max的排列的距离数组
	std::vector<float> intensities;  //按照角度从angle_min到angle_max的排列的反射率数组（可能没有这个数据）

	ladar_data_t()
	{
		this->flags = 0;
		this->range_min = 0.f;                 //米
		this->range_max = 0.f;                 //米
		this->angle_min = 0.f;                 //弧度
		this->angle_max = 0.f;                 //弧度
		this->angle_increment = 0.f;           //弧度
		this->scan_time = 0.f;                 //秒
		this->time_increment = 0.f;            //秒
		this->wave_count = 0;
		this->ts_syncs = 0;
		this->ts_secs = 0;
		this->ts_usecs = 0;
	}

	ladar_data_t(ladar_data_t&& rhs) noexcept
	{
		this->flags = rhs.flags;
		this->range_min = rhs.range_min;
		this->range_max = rhs.range_max;
		this->angle_min = rhs.angle_min;
		this->angle_max = rhs.angle_max;
		this->angle_increment = rhs.angle_increment;
		this->scan_time = rhs.scan_time;
		this->time_increment = rhs.time_increment;
		this->wave_count = rhs.wave_count;
		this->ts_syncs = rhs.ts_syncs;
		this->ts_secs = rhs.ts_secs;
		this->ts_usecs = rhs.ts_usecs;
		this->ranges = std::move(rhs.ranges);
		this->intensities = std::move(rhs.intensities);
	}

	ladar_data_t& operator=(ladar_data_t&& rhs) noexcept
	{
		this->flags = rhs.flags;
		this->range_min = rhs.range_min;
		this->range_max = rhs.range_max;
		this->angle_min = rhs.angle_min;
		this->angle_max = rhs.angle_max;
		this->angle_increment = rhs.angle_increment;
		this->scan_time = rhs.scan_time;
		this->time_increment = rhs.time_increment;
		this->wave_count = rhs.wave_count;
		this->ts_syncs = rhs.ts_syncs;
		this->ts_secs = rhs.ts_secs;
		this->ts_usecs = rhs.ts_usecs;
		this->ranges = std::move(rhs.ranges);
		this->intensities = std::move(rhs.intensities);
		return *this;
	}
} ladar_data_t;


//---------------------------------------------------------------------------
typedef struct ladar_wave_data_t
{
	uint32_t wave_type;
	std::vector<float> ranges;       //按照角度从angle_min到angle_max的排列的距离数组
	std::vector<float> intensities;  //按照角度从angle_min到angle_max的排列的反射率数组（可能没有这个数据）
} ladar_wave_data_t;

typedef struct ladar_datas_t
{
	uint32_t flags;                  //dsSyncInfo与dsTimeStamp的组合，表示对应信息可用
	float range_min;                 //米，雷达数据输出最小距离
	float range_max;                 //米，雷达数据输出最大有效距离
	float angle_min;                 //弧度，雷达数据输出最小角度
	float angle_max;                 //弧度，雷达数据输出最大角度
	float angle_increment;           //弧度，雷达数据输出两个点最小角度间隔，角度分辨率
	float scan_time;                 //秒，扫描一圈儿的时间
	float time_increment;            //秒，扫描一个点的时间
	int32_t wave_count;              //数据回波个数
	uint32_t ts_syncs;               //雷达同步数据，flags中包含dsSyncInfo时可用
	uint32_t ts_secs;                //雷达时间戳（秒），flags中包含dsTimeStamp时可用
	uint32_t ts_usecs;               //雷达时间戳（微秒），flags中包含dsTimeStamp时可用
	std::vector<ladar_wave_data_t> datas; //包含多个回波数据

	ladar_datas_t()
	{
		this->flags = 0;
		this->range_min = 0.f;                 //米
		this->range_max = 0.f;                 //米
		this->angle_min = 0.f;                 //弧度
		this->angle_max = 0.f;                 //弧度
		this->angle_increment = 0.f;           //弧度
		this->scan_time = 0.f;                 //秒
		this->time_increment = 0.f;            //秒
		this->wave_count = 0;
		this->ts_syncs = 0;
		this->ts_secs = 0;
		this->ts_usecs = 0;
	}

	ladar_datas_t(ladar_datas_t&& rhs) noexcept
	{
		this->flags = rhs.flags;
		this->range_min = rhs.range_min;
		this->range_max = rhs.range_max;
		this->angle_min = rhs.angle_min;
		this->angle_max = rhs.angle_max;
		this->angle_increment = rhs.angle_increment;
		this->scan_time = rhs.scan_time;
		this->time_increment = rhs.time_increment;
		this->wave_count = rhs.wave_count;
		this->ts_syncs = rhs.ts_syncs;
		this->ts_secs = rhs.ts_secs;
		this->ts_usecs = rhs.ts_usecs;
		this->datas = std::move(rhs.datas);
	}

	ladar_datas_t& operator=(ladar_datas_t&& rhs) noexcept
	{
		this->flags = rhs.flags;
		this->range_min = rhs.range_min;
		this->range_max = rhs.range_max;
		this->angle_min = rhs.angle_min;
		this->angle_max = rhs.angle_max;
		this->angle_increment = rhs.angle_increment;
		this->scan_time = rhs.scan_time;
		this->time_increment = rhs.time_increment;
		this->wave_count = rhs.wave_count;
		this->ts_syncs = rhs.ts_syncs;
		this->ts_secs = rhs.ts_secs;
		this->ts_usecs = rhs.ts_usecs;
		this->datas = std::move(rhs.datas);
		return *this;
	}
} ladar_datas_t;

//---------------------------------------------------------------------------
typedef struct float3_t
{
	float x, y, z;
} float3_t;

typedef struct ladar_data3d_t
{
	float range_min;                 //米，雷达线扫数据输出最小距离
	float range_max;                 //米，雷达线扫数据输出最大有效距离
	std::vector<float3_t> points;
	std::vector<float> intensities;  //points中每个点对应的的反射率（可能没有这个数据）
} ladar_data3d_t;

//---------------------------------------------------------------------------
class IHandle
{
public:
	virtual ~IHandle() {};
	virtual ILadarPoller* GetPoller() = 0;
	virtual void Delete() = 0;
	virtual void* GetUserData() const = 0;
};

//---------------------------------------------------------------------------
class ITimer : public IHandle
{
public:
	virtual int GetInterval() const = 0;
	virtual void Enable() = 0;
	virtual void Disable() = 0;
	virtual bool Enabled() = 0;
};

//---------------------------------------------------------------------------
class ITipClient : public IHandle
{
public:
	virtual const char* GetAddress() const = 0;
	virtual uint16_t GetPort() const = 0;
	virtual bool UpdateEndpoint(const char* address, uint16_t port) = 0;
	virtual void WantConnect() = 0;
	virtual void WantDisconnect() = 0;
	virtual int Send(const uint8_t* frame, int frameLength) = 0;
	virtual bool IsConnected() = 0;
	virtual bool IsWantConnected() = 0;
};

//---------------------------------------------------------------------------
class ILadar : public IHandle
{
public:
	virtual const char* GetAddress() const = 0;
	virtual uint16_t GetPort() const = 0;
	virtual bool UpdateEndpoint(const char* address, uint16_t port) = 0;
	virtual void WantConnect() = 0;
	virtual void WantDisconnect() = 0;
	virtual int Send(const uint8_t* frame, int frameLength) = 0;
	virtual int IOControl(uint16_t index, uint16_t flag) = 0;
	virtual int GetDIStatus() = 0;
	virtual int SetTime() = 0;
	virtual int Standby(uint16_t standby) = 0;
	virtual int SetDustCapStatus(int isOpen) = 0;
	virtual void GetParam(ladar_param_t& param) = 0;
	virtual int GetData(ladar_data_t& data, const uint8_t* frame, int frameLength) = 0;
	virtual int GetData(ladar_datas_t& data, const uint8_t* frame, int frameLength) = 0;
	virtual int GetData(ladar_data3d_t& data, const uint8_t* frame, int frameLength) = 0;
	virtual bool IsConnected() = 0;
	virtual bool IsWantConnected() = 0;
	virtual int SwitchMode(bool is2d) = 0;
};

//---------------------------------------------------------------------------
class ILadar94 : public IHandle
{
public:
	virtual const char* GetAddress() const = 0;
	virtual uint16_t GetPort() const = 0;
	virtual bool UpdateEndpoint(const char* address, uint16_t port) = 0;
	virtual void WantConnect() = 0;
	virtual void WantDisconnect() = 0;
	virtual void GetRange(float& rmin, float& rmax) const = 0;
	virtual int Send(const uint8_t* frame, int frameLength) = 0;
	virtual int SetTime() = 0;
	virtual int GetScanMode() = 0;
	virtual int StartScan(uint16_t isLoop, uint16_t speed, uint16_t angleStart, uint16_t angleEnd, uint16_t cBrush = 0, uint16_t isStandby = 0) = 0;
	virtual int StopScan() = 0;
	virtual int StartBrush(uint16_t cBrush) = 0;
	virtual int GetData(ladar_data3d_t& data, const uint8_t* frame, int frameLength) = 0;
	virtual bool IsConnected() = 0;
	virtual bool IsWantConnected() = 0;
	virtual int ResetAngle() = 0;
	virtual int GetCurrAngle() = 0;
};

//---------------------------------------------------------------------------
class IPelco : public IHandle
{
public:
	virtual const char* GetName() const = 0;
	virtual int Send(const uint8_t* frame, int frameLength) = 0;
	virtual uint8_t GetAddr() const = 0;
	virtual int SetAngleRange(int start, int end) = 0;
	virtual int StopScan() = 0;
	virtual int ControlUp(uint8_t speed) = 0;
	virtual int ControlDown(uint8_t speed) = 0;
	virtual int Reset() = 0;
	virtual int GetCurrentAngle() = 0;
	virtual int GetCurrentStatus() = 0;
};

//---------------------------------------------------------------------------

typedef std::function<void(ITimer*)> TTimerHandler;
typedef std::function<void(ILadar* ladar, const uint8_t* frame, int frameLength)> TLadarFrameHandler;
typedef std::function<void(ILadar* ladar, uint32_t old_status, uint32_t status)> TLadarStatusHandler;
typedef std::function<void(ILadar94* ladar, const uint8_t* frame, int frameLength)> TLadarFrameHandler94;
typedef std::function<void(ILadar94* ladar, uint32_t old_status, uint32_t status)> TLadarStatusHandler94;
typedef std::function<void(IPelco* c, const uint8_t* frame, int frameLength)> TPelcoFrameHandler;
typedef std::function<void(ITipClient* c, const uint8_t* frame, int frameLength)> TTipFrameHandler;
typedef std::function<void(ITipClient* c, uint32_t old_status, uint32_t status)> TTipStatusHandler;

class ILadarPoller
{
public:
	virtual ~ILadarPoller() {}
	virtual void Run() = 0;
	virtual bool Stopped() const = 0;
	virtual void Stop() = 0;
	virtual void Post(const std::function<void()>& handler) = 0;

	virtual ITimer* AddTimer(int interval, const TTimerHandler& handler, void* userdata = NULL) = 0;
	virtual void RemoveTimer(ITimer* timer) = 0;
	virtual void ClearAllTimer() = 0;

	virtual ILadar* AddLadar(const char* address, uint16_t port,
		const TLadarFrameHandler& onFrame, const TLadarStatusHandler& onStatus, void* userdata = NULL) = 0;
	virtual void RemoveLadar(ILadar* ladar) = 0;
	virtual void ClearAllLadar() = 0;

	virtual ILadar94* AddLadar94(const char* address, uint16_t port,
		const TLadarFrameHandler94& onFrame, const TLadarStatusHandler94& onStatus, void* userdata = NULL) = 0;
	virtual void RemoveLadar94(ILadar94* ladar) = 0;
	virtual void ClearAllLadar94() = 0;

	virtual IPelco* AddPelco(const char* name, uint8_t addr,
		const TPelcoFrameHandler& onFrame, void* userdata = NULL) = 0;
	virtual void RemovePelco(IPelco* pelco) = 0;
	virtual void ClearAllPelco() = 0;

	virtual ITipClient* AddTipClient(const char* address, uint16_t port,
		const TTipFrameHandler& onFrame, const TTipStatusHandler& onStatus, void* userdata = NULL) = 0;
	virtual void RemoveTipClient(ITipClient* c) = 0;
	virtual void ClearAllTipClient() = 0;
};

//---------------------------------------------------------------------------
