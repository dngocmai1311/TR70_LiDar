const assert=require('node:assert/strict');
const fs=require('fs'),vm=require('vm');
const {parse,reproject,FusionFrames}=require('../frontend/js/websocket3d.js');
function packet(version=3,source=0,timestamp=100000,z=.4){
  const h=version===3?76:64,b=new ArrayBuffer(h+(version===3?16:12)),d=new DataView(b);
  d.setUint8(0,version);d.setUint8(1,source);d.setUint32(4,42,true);d.setUint32(8,Math.floor(timestamp/1000),true);d.setUint32(12,timestamp%1000*1000,true);
  if(version===3)d.setFloat32(24,z,true);d.setUint32(h-8,1,true);d.setUint32(h-4,1,true);
  d.setFloat32(h,3,true);d.setFloat32(h+4,1,true);if(version===3)d.setFloat32(h+8,z,true);d.setFloat32(b.byteLength-4,77,true);return b;
}
for(const v of [2,3]){
  const b=packet(v),f=parse(b);assert.equal(f.n,1);assert.equal(f.frameId,42);assert.equal(f.pts[2],77);assert.equal(f.xyz[0],3);
  for(let len=0;len<b.byteLength;len++)assert.equal(parse(b.slice(0,len)),null);
  const oversized=b.slice(0);new DataView(oversized).setUint32((v===3?76:64)-4,0xffffffff,true);assert.equal(parse(oversized),null);
  new DataView(b).setFloat32(v===3?76:64,NaN,true);assert.equal(parse(b),null);
}
const fusion=new FusionFrames();
for(let id=0;id<4;id++)fusion.push(parse(packet(3,id,100000+[0,12,-7,19][id],[.4,.7,1,1.3][id])));
let selected=fusion.select(4);assert.equal(selected.layers.filter(l=>!l.stale).length,4);
for(let id=0;id<4;id++){assert.ok(Math.abs(selected.layers[id].frame.xyz[2]-[.4,.7,1,1.3][id])<1e-6);assert.equal(selected.layers[id].frame.xyz[0],3);}
const rotated=reproject(parse(packet()),{x:0,y:0,z:.7,roll:0,pitch:0,yaw:90});
assert.ok(Math.abs(rotated.xyz[0]+1)<1e-5);assert.ok(Math.abs(rotated.xyz[1]-3)<1e-5);
const tilted=reproject(parse(packet()),{x:0,y:0,z:1,roll:0,pitch:30,yaw:0});assert.ok(Math.abs(tilted.xyz[2]+.5)<1e-5);
fusion.maxFusionDeltaMs=1;assert.equal(fusion.select(4).layers.filter(l=>!l.stale).length,1);
assert.ok(fusion.select(4,performance.now()+2000).layers.every(l=>l.stale));
fusion.maxFusionDeltaMs=100;assert.ok(fusion.select(4,performance.now()+2000,true).layers.every(l=>!l.stale));
for(let i=0;i<100;i++)fusion.push(parse(packet(3,0,101000+i)));assert.equal(fusion.hist.get(0).length,32);
fusion.push(parse(packet(3,0,1000)));assert.equal(fusion.hist.get(0).length,1);
fusion.clear();assert.equal(fusion.hist.size,0);
const html=fs.readFileSync(require('path').join(__dirname,'../frontend/index.html'),'utf8');
for(const match of html.matchAll(/<script>([\s\S]*?)<\/script>/g))new vm.Script(match[1]);
console.log('PASS: V2/V3 layout, truncated buffers, invalid data, wall layers, yaw, pitch, fusion, stale, bounded history, inline JS syntax');
