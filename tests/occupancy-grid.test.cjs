const test=require('node:test');
const assert=require('node:assert/strict');
const {OccupancyGrid,CellState}=require('../frontend/js/occupancy-grid.js');

const cfg={width:10,height:10,resolution:1,originX:-5,originY:-5,
  hitIncrement:4,freeDecrement:1,occupiedThreshold:3,freeThreshold:-1,decayPerSecond:1};

test('one sensor marks the ray free and endpoint occupied',()=>{
  const g=new OccupancyGrid(cfg); g.updateRay(0,0,3,0,1000);
  assert.equal(g.getCell(5,5),CellState.FREE);
  assert.equal(g.getCell(5,7),CellState.FREE);
  assert.equal(g.getCell(5,8),CellState.OCCUPIED);
});

test('two lidar measurements fuse into the same world cell',()=>{
  const g=new OccupancyGrid(cfg);
  g.updateScan({id:0,x:0,y:0},[{x:3,y:2}],1000);
  g.updateScan({id:1,x:2,y:0},[{x:3.2,y:2.1}],1000);
  assert.equal(g.getCell(7,8),CellState.OCCUPIED);
  assert.deepEqual(g.getStats().sensors,[0,1]);
});

test('translated and 30 degree yaw local endpoint lands at expected world cell',()=>{
  const g=new OccupancyGrid(cfg), yaw=Math.PI/6, local={x:2,y:0};
  const world={x:2+local.x*Math.cos(yaw)-local.y*Math.sin(yaw),
    y:1+local.x*Math.sin(yaw)+local.y*Math.cos(yaw)};
  g.updateRay(2,1,world.x,world.y,1000);
  const c=g.worldToGrid(world.x,world.y);
  assert.equal(g.getCell(c.row,c.col),CellState.OCCUPIED);
});

test('old occupied cells decay instead of persisting forever',()=>{
  const g=new OccupancyGrid(cfg); g.updateRay(0,0,2,0,1000);
  g.decay(1000); g.decay(6000);
  const c=g.worldToGrid(2,0);
  assert.equal(g.getCell(c.row,c.col),CellState.UNKNOWN);
});

test('out-of-bounds endpoint still clears free space up to map edge',()=>{
  const g=new OccupancyGrid(cfg); g.updateRay(0,0,20,0,1000);
  assert.equal(g.getCell(5,9),CellState.FREE);
  assert.equal(g.getStats().occupied,0);
});

test('world ROI vertices convert to the same grid coordinates',()=>{
  const g=new OccupancyGrid(cfg);
  assert.deepEqual([{x:-1,y:-1},{x:2,y:3}].map(p=>g.worldToGrid(p.x,p.y)),
    [{row:4,col:4},{row:8,col:7}]);
});
