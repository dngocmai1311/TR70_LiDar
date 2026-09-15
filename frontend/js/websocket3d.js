/* Binary V2: local XYI, 64-byte header. V3: world XYZI, 76-byte header.
 * All numbers little endian; wire rotation angles are radians. */
(function(root){
  'use strict';

  // Tính ma trận quay 3x3 từ các góc Euler (roll, pitch, oth/yaw) theo thứ tự ZYX
  function rotation(e){
    const cr = Math.cos(e.roll || 0), sr = Math.sin(e.roll || 0);
    const cp = Math.cos(e.pitch || 0), sp = Math.sin(e.pitch || 0);
    const cy = Math.cos(e.oth || 0),   sy = Math.sin(e.oth || 0);
    return [
      cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
      sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
      -sp,     cp * sr,               cp * cr
    ];
  }

  // Chuyển đổi điểm cục bộ (x, y, z) sang tọa độ thế giới 3D dựa trên pose 'e'
  function localPointToWorld3D(e, x, y, z = 0){
    const r = rotation(e);
    return [
      r[0] * x + r[1] * y + r[2] * z + (e.ox ?? e.x ?? e.dx ?? 0),
      r[3] * x + r[4] * y + r[5] * z + (e.oy ?? e.y ?? e.dy ?? 0),
      r[6] * x + r[7] * y + r[8] * z + (e.oz ?? e.z ?? 0)
    ];
  }

  function parse(buf){
    if(!(buf instanceof ArrayBuffer) || buf.byteLength < 4) return null;
    const d = new DataView(buf), v = d.getUint8(0);
    if(v !== 2 && v !== 3) return null;

    const header = v === 3 ? 76 : 64, stride = v === 3 ? 16 : 12;
    if(buf.byteLength < header) return null;

    const n = d.getUint32(header - 4, true);
    if(n > 1000000 || buf.byteLength !== header + n * stride) return null;

    const f = {
      version: v,
      sourceId: d.getUint8(1),
      frameId: d.getUint32(4, true),
      tsSec: d.getUint32(8, true),
      tsUsec: d.getUint32(12, true),
      ox: d.getFloat32(16, true),
      oy: d.getFloat32(20, true),
      oz: v === 3 ? d.getFloat32(24, true) : 0,
      roll: v === 3 ? d.getFloat32(28, true) : 0,
      pitch: v === 3 ? d.getFloat32(32, true) : 0,
      oth: d.getFloat32(v === 3 ? 36 : 24, true),
      n
    };

    let off = v === 3 ? 40 : 28;
    for(const key of ['aMin', 'aMax', 'aInc', 'rMin', 'rMax', 'scan', 'tInc']){
      f[key] = d.getFloat32(off, true);
      off += 4;
    }
    f.wave = d.getUint32(off, true);

    if(f.tsUsec >= 1000000 || Object.values(f).some(x => !Number.isFinite(x))) return null;

    f.timestamp = f.tsSec * 1000 + f.tsUsec / 1000;
    f.recvTime = performance.now();
    f.pts = new Float32Array(n * 3);
    f.xyz = new Float32Array(n * 3);

    const r = rotation(f);
    for(let i = 0, o = header; i < n; i++, o += stride){
      const x = d.getFloat32(o, true);
      const y = d.getFloat32(o + 4, true);
      const z = v === 3 ? d.getFloat32(o + 8, true) : 0;
      const intensity = d.getFloat32(o + stride - 4, true);

      if(![x, y, z, intensity].every(Number.isFinite)) return null;

      const k = i * 3;
      if(v === 3){
        f.xyz.set([x, y, z], k);
        // Transform ngược về tọa độ local để phục vụ các công cụ 2D/calibration
        const a = x - f.ox, b = y - f.oy, c = z - f.oz;
        f.pts.set([r[0]*a + r[3]*b + r[6]*c, r[1]*a + r[4]*b + r[7]*c, intensity], k);
      }else{
        f.pts.set([x, y, intensity], k);
        f.xyz.set([r[0]*x + r[1]*y + f.ox, r[3]*x + r[4]*y + f.oy, 0], k);
      }
    }
    return f;
  }

  function reproject(f, e){
    const out = {
      ...f,
      ox: e.x ?? e.dx ?? f.ox,
      oy: e.y ?? e.dy ?? f.oy,
      oz: e.z ?? f.oz,
      roll: (e.roll ?? f.roll * 180 / Math.PI) * Math.PI / 180,
      pitch: (e.pitch ?? f.pitch * 180 / Math.PI) * Math.PI / 180,
      oth: (e.yaw ?? e.thetaDeg ?? f.oth * 180 / Math.PI) * Math.PI / 180,
      xyz: new Float32Array(f.n * 3)
    };
    const r = rotation(out);
    for(let k = 0; k < f.pts.length; k += 3){
      const x = f.pts[k], y = f.pts[k + 1];
      out.xyz.set([r[0]*x + r[1]*y + out.ox, r[3]*x + r[4]*y + out.oy, r[6]*x + r[7]*y + out.oz], k);
    }
    return out;
  }

  class FusionFrames {
    constructor(){
      this.hist = new Map();
      this.maxFusionDeltaMs = 100;
      this.maxAgeMs = 1000;
    }
    clear(){
      this.hist.clear();
    }
    push(f){
      let h = this.hist.get(f.sourceId) || [];
      if(h.length && f.timestamp < h[h.length - 1].timestamp) h = []; // Đặt lại clock / CSV wrap
      h.push(f);
      if(h.length > 32) h.shift();
      this.hist.set(f.sourceId, h);
    }
    reproject(id, e){
      if(this.hist.has(id)){
        this.hist.set(id, this.hist.get(id).map(f => reproject(f, e)));
      }
    }
    select(count, now = performance.now(), paused = false){
      const latest = Array.from({length: count}, (_, id) => this.hist.get(id)?.at(-1));
      const live = latest.filter(f => f && (paused || now - f.recvTime <= this.maxAgeMs));
      
      const times = live.map(f => f.timestamp).sort((a, b) => a - b);
      const timestamp = times.length ? times[Math.floor(times.length / 2)] : null;
      
      const layers = latest.map((last, id) => {
        let frame = null, delta = null;
        if(last && timestamp !== null){
          frame = this.hist.get(id).reduce((a, b) => 
            Math.abs(a.timestamp - timestamp) <= Math.abs(b.timestamp - timestamp) ? a : b
          );
          delta = frame.timestamp - timestamp;
        }
        const stale = !frame || (!paused && now - last.recvTime > this.maxAgeMs) || Math.abs(delta) > this.maxFusionDeltaMs;
        return {id, frame, delta, stale};
      });
      return {
        timestamp,
        layers,
        maxDelta: Math.max(0, ...layers.filter(l => !l.stale).map(l => Math.abs(l.delta)))
      };
    }
  }

  // Xuất đầy đủ các API bao gồm cả localPointToWorld3D
  root.LidarProtocol = {
    parse,
    rotation,
    localPointToWorld3D,
    reproject,
    FusionFrames
  };

  if(typeof module !== 'undefined') module.exports = root.LidarProtocol;
})(globalThis);