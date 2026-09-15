(function(root){
  'use strict';

  const CellState = Object.freeze({UNKNOWN:-1, FREE:0, OCCUPIED:100});
  const defaults = Object.freeze({
    width:40, height:40, resolution:0.05, originX:-20, originY:-20,
    hitIncrement:4, freeDecrement:1, occupiedThreshold:3, freeThreshold:-1,
    minScore:-20, maxScore:20, decayPerSecond:1, currentScanTtlMs:350
  });

  class OccupancyGrid {
    constructor(config={}){
      this.config={...defaults,...config};
      const c=this.config;
      this.cols=Math.ceil(c.width/c.resolution);
      this.rows=Math.ceil(c.height/c.resolution);
      this.scores=new Int16Array(this.rows*this.cols);
      this.observed=new Uint8Array(this.rows*this.cols);
      this.currentHitAt=new Float64Array(this.rows*this.cols);
      this.lastDecayAt=0;
      this.contributingSensors=new Set();
    }
    clear(){
      this.scores.fill(0); this.observed.fill(0); this.currentHitAt.fill(0);
      this.lastDecayAt=0; this.contributingSensors.clear();
    }
    worldToGrid(x,y){
      const col=Math.floor((x-this.config.originX)/this.config.resolution);
      const row=Math.floor((y-this.config.originY)/this.config.resolution);
      return row>=0&&row<this.rows&&col>=0&&col<this.cols?{row,col}:null;
    }
    gridToWorld(row,col){
      if(row<0||row>=this.rows||col<0||col>=this.cols)return null;
      return {x:this.config.originX+(col+0.5)*this.config.resolution,
        y:this.config.originY+(row+0.5)*this.config.resolution};
    }
    index(row,col){return row*this.cols+col;}
    getCell(row,col){
      if(row<0||row>=this.rows||col<0||col>=this.cols)return null;
      const i=this.index(row,col);
      if(!this.observed[i])return CellState.UNKNOWN;
      if(this.scores[i]>=this.config.occupiedThreshold)return CellState.OCCUPIED;
      if(this.scores[i]<=this.config.freeThreshold)return CellState.FREE;
      return CellState.UNKNOWN;
    }
    setCell(row,col,state,now=Date.now()){
      if(row<0||row>=this.rows||col<0||col>=this.cols)return false;
      const i=this.index(row,col); this.observed[i]=1;
      if(state===CellState.OCCUPIED){this.scores[i]=this.config.occupiedThreshold;this.currentHitAt[i]=now;}
      else if(state===CellState.FREE)this.scores[i]=this.config.freeThreshold;
      else {this.scores[i]=0;this.observed[i]=0;this.currentHitAt[i]=0;}
      return true;
    }
    _add(row,col,delta,now,isHit){
      if(row<0||row>=this.rows||col<0||col>=this.cols)return;
      const i=this.index(row,col), c=this.config; this.observed[i]=1;
      this.scores[i]=Math.max(c.minScore,Math.min(c.maxScore,this.scores[i]+delta));
      if(isHit)this.currentHitAt[i]=now;
    }
    updateRay(sensorX,sensorY,hitX,hitY,now=Date.now()){
      const start=this.worldToGrid(sensorX,sensorY);
      if(!start)return false;
      let end=this.worldToGrid(hitX,hitY), endpointIsHit=!!end;
      if(!end){
        // Keep the visible part of a long beam: cells up to the map edge are FREE,
        // but the out-of-map measured endpoint must not create an OCCUPIED cell.
        const dx=hitX-sensorX,dy=hitY-sensorY,c=this.config,eps=c.resolution*1e-6;
        let t=1;
        if(dx>0)t=Math.min(t,(c.originX+c.width-eps-sensorX)/dx);
        else if(dx<0)t=Math.min(t,(c.originX+eps-sensorX)/dx);
        if(dy>0)t=Math.min(t,(c.originY+c.height-eps-sensorY)/dy);
        else if(dy<0)t=Math.min(t,(c.originY+eps-sensorY)/dy);
        if(!Number.isFinite(t)||t<0)return false;
        end=this.worldToGrid(sensorX+dx*t,sensorY+dy*t);
        if(!end)return false;
      }
      let x=start.col,y=start.row, x1=end.col,y1=end.row;
      const dx=Math.abs(x1-x), sx=x<x1?1:-1, dy=-Math.abs(y1-y), sy=y<y1?1:-1;
      let err=dx+dy;
      while(x!==x1||y!==y1){
        this._add(y,x,-this.config.freeDecrement,now,false);
        const e2=2*err;
        if(e2>=dy){err+=dy;x+=sx;}
        if(e2<=dx){err+=dx;y+=sy;}
      }
      this._add(y1,x1,endpointIsHit?this.config.hitIncrement:-this.config.freeDecrement,now,endpointIsHit);
      return true;
    }
    updateScan(sensor,points,now=Date.now()){
      if(!sensor||!Number.isFinite(sensor.x)||!Number.isFinite(sensor.y))return 0;
      this.decay(now);
      const stride=points instanceof Float32Array?3:1;
      let rays=0;
      for(let i=0;i<points.length;i+=stride){
        const p=stride===3?{x:points[i],y:points[i+1]}:points[i];
        if(p&&Number.isFinite(p.x)&&Number.isFinite(p.y)&&this.updateRay(sensor.x,sensor.y,p.x,p.y,now))rays++;
      }
      if(rays&&sensor.id!==undefined)this.contributingSensors.add(sensor.id);
      return rays;
    }
    decay(now=Date.now()){
      if(!this.lastDecayAt){this.lastDecayAt=now;return;}
      const steps=Math.floor((now-this.lastDecayAt)/1000*this.config.decayPerSecond);
      if(steps<1)return;
      for(let i=0;i<this.scores.length;i++){
        if(!this.observed[i])continue;
        if(this.scores[i]>0)this.scores[i]=Math.max(0,this.scores[i]-steps);
        else if(this.scores[i]<0)this.scores[i]=Math.min(0,this.scores[i]+steps);
        if(this.scores[i]===0)this.observed[i]=0;
      }
      this.lastDecayAt=now;
    }
    isCurrentHit(row,col,now=Date.now()){
      const t=this.currentHitAt[this.index(row,col)];
      return t>0&&now-t<=this.config.currentScanTtlMs;
    }
    getStats(){
      let occupied=0,free=0,unknown=0;
      for(let i=0;i<this.scores.length;i++){
        if(!this.observed[i])unknown++;
        else if(this.scores[i]>=this.config.occupiedThreshold)occupied++;
        else if(this.scores[i]<=this.config.freeThreshold)free++;
        else unknown++;
      }
      return {occupied,free,unknown,sensors:Array.from(this.contributingSensors).sort((a,b)=>a-b)};
    }
  }
  root.OccupancyGrid=OccupancyGrid;
  root.OccupancyCellState=CellState;
  if(typeof module!=='undefined')module.exports={OccupancyGrid,CellState,defaults};
})(globalThis);
