(function(root){
  'use strict';
  class MapView {
    constructor(config={}){
      this.minZoom=config.minZoom??0.5;this.maxZoom=config.maxZoom??20;
      this.zoom=1;this.offsetX=0;this.offsetY=0;this.rotation=0;
    }
    reset(){this.zoom=1;this.offsetX=0;this.offsetY=0;this.rotation=0;}
    baseScale(canvasWidth,canvasHeight,mapConfig){
      return Math.min(canvasWidth/mapConfig.width,canvasHeight/mapConfig.height);
    }
    worldToScreen(x,y,canvasWidth,canvasHeight,mapConfig){
      const scale=this.baseScale(canvasWidth,canvasHeight,mapConfig);
      const centerX=mapConfig.originX+mapConfig.width/2,centerY=mapConfig.originY+mapConfig.height/2;
      const vx=(x-centerX)*scale,vy=-(y-centerY)*scale,c=Math.cos(this.rotation),s=Math.sin(this.rotation);
      return {x:canvasWidth/2+this.offsetX+this.zoom*(vx*c-vy*s),
        y:canvasHeight/2+this.offsetY+this.zoom*(vx*s+vy*c)};
    }
    screenToWorld(x,y,canvasWidth,canvasHeight,mapConfig){
      const scale=this.baseScale(canvasWidth,canvasHeight,mapConfig),c=Math.cos(this.rotation),s=Math.sin(this.rotation);
      const rx=(x-canvasWidth/2-this.offsetX)/this.zoom,ry=(y-canvasHeight/2-this.offsetY)/this.zoom;
      const vx=rx*c+ry*s,vy=-rx*s+ry*c;
      return {x:mapConfig.originX+mapConfig.width/2+vx/scale,
        y:mapConfig.originY+mapConfig.height/2-vy/scale};
    }
    zoomAt(screenX,screenY,factor,canvasWidth,canvasHeight,mapConfig){
      const world=this.screenToWorld(screenX,screenY,canvasWidth,canvasHeight,mapConfig);
      this.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.zoom*factor));
      const after=this.worldToScreen(world.x,world.y,canvasWidth,canvasHeight,mapConfig);
      this.offsetX+=screenX-after.x;this.offsetY+=screenY-after.y;
    }
    rotateAtCenter(delta,canvasWidth,canvasHeight,mapConfig){
      const world=this.screenToWorld(canvasWidth/2,canvasHeight/2,canvasWidth,canvasHeight,mapConfig);
      this.rotation+=delta;
      const after=this.worldToScreen(world.x,world.y,canvasWidth,canvasHeight,mapConfig);
      this.offsetX+=canvasWidth/2-after.x;this.offsetY+=canvasHeight/2-after.y;
    }
    pan(dx,dy){this.offsetX+=dx;this.offsetY+=dy;}
  }
  root.MapView=MapView;
  if(typeof module!=='undefined')module.exports={MapView};
})(globalThis);
