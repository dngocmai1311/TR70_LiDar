const test=require('node:test');
const assert=require('node:assert/strict');
const {MapView}=require('../frontend/js/map-view.js');
const map={width:40,height:40,originX:-20,originY:-20};
const close=(a,b)=>assert.ok(Math.abs(a-b)<1e-9,`${a} != ${b}`);

test('worldToScreen and screenToWorld are exact inverses after zoom pan rotate',()=>{
  const v=new MapView();v.zoom=4.2;v.offsetX=123;v.offsetY=-71;v.rotation=Math.PI/3;
  const s=v.worldToScreen(2.5,-7.25,1200,700,map),w=v.screenToWorld(s.x,s.y,1200,700,map);
  close(w.x,2.5);close(w.y,-7.25);
});
test('zoom-at-cursor keeps the same world point under cursor',()=>{
  const v=new MapView(),before=v.screenToWorld(317,211,1000,800,map);
  v.zoomAt(317,211,5,1000,800,map);
  const after=v.worldToScreen(before.x,before.y,1000,800,map);
  close(after.x,317);close(after.y,211);assert.equal(v.zoom,5);
});
test('pan moves content in screen direction and reset restores fit view',()=>{
  const v=new MapView(),a=v.worldToScreen(1,2,900,600,map);v.pan(40,-25);
  const b=v.worldToScreen(1,2,900,600,map);close(b.x-a.x,40);close(b.y-a.y,-25);
  v.rotation=1;v.zoom=8;v.reset();assert.deepEqual({zoom:v.zoom,x:v.offsetX,y:v.offsetY,r:v.rotation},{zoom:1,x:0,y:0,r:0});
});
test('rotation keeps the current view center fixed',()=>{
  const v=new MapView();v.zoom=3;v.pan(90,45);
  const before=v.screenToWorld(500,350,1000,700,map);v.rotateAtCenter(Math.PI/2,1000,700,map);
  const after=v.screenToWorld(500,350,1000,700,map);close(after.x,before.x);close(after.y,before.y);
});
