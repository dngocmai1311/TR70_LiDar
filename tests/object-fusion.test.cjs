const test=require('node:test');
const assert=require('node:assert/strict');
const F=require('../frontend/js/object-fusion.js');
const roi={id:7,scope:'fusion',type:0,x1:-2,y1:-2,x2:5,y2:5};
function frame(sensorId,timestamp,xy,z=sensorId){
  const xyz=new Float32Array(xy.length*3),pts=new Float32Array(xy.length*3);
  xy.forEach((p,i)=>{xyz.set([p[0],p[1],z],i*3);pts.set([p[0]-sensorId,p[1],10],i*3);});
  return {sourceId:sensorId,timestamp,xyz,pts};
}
const blob=(x,y)=>[[x,y],[x+.04,y],[x,y+.04],[x+.04,y+.04]];

test('one lidar creates one single-sensor object',()=>{
  const r=F.fuseObjects([frame(0,1000,blob(0,0))],[roi]);
  assert.equal(r.objects.length,1);assert.deepEqual(r.objects[0].sensors,[0]);assert.equal(r.objects[0].roiId,7);
});
test('L1 and L2 points on the same target become one object with source counts',()=>{
  const r=F.fuseObjects([frame(0,1000,blob(0,0)),frame(1,1010,blob(.08,.03))],[roi]);
  assert.equal(r.objects.length,1);assert.deepEqual(r.objects[0].sensors,[0,1]);
  assert.deepEqual(r.objects[0].pointsPerSensor,{0:4,1:4});assert.equal(r.objects[0].pointCount,8);
});
test('two distant targets remain two objects and may each contain two sensors',()=>{
  const a=[...blob(0,0),...blob(3,3)],b=[...blob(.05,.03),...blob(3.05,3.03)];
  const r=F.fuseObjects([frame(0,1000,a),frame(1,1005,b)],[roi]);
  assert.equal(r.objects.length,2);assert.ok(r.objects.every(o=>o.sensors.length===2));
});
test('points outside ROI do not create an object',()=>{
  assert.equal(F.fuseObjects([frame(0,1000,blob(9,9))],[roi]).objects.length,0);
});
test('frames outside fusion tolerance are never combined',()=>{
  const r=F.fuseObjects([frame(0,1000,blob(0,0)),frame(1,1400,blob(.05,.02))],[roi],{fusionToleranceMs:100});
  assert.equal(r.objects.length,1);assert.deepEqual(r.objects[0].sensors,[1]);
});
test('different world Z does not split an XY object and point metadata is retained',()=>{
  const r=F.fuseObjects([frame(0,1000,blob(0,0),.5),frame(1,1000,blob(.05,0),2.5)],[roi]);
  assert.equal(r.objects.length,1);assert.deepEqual(new Set(r.objects[0].points.map(p=>p.z)),new Set([.5,2.5]));
  assert.ok(r.objects[0].points.every(p=>Number.isFinite(p.localX)&&Number.isFinite(p.timestamp)));
});
