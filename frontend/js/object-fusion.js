(function(root){
  'use strict';
  const defaults=Object.freeze({clusterEps:0.25,clusterMinPoints:3,fusionToleranceMs:100});

  function pointInsideRoi(roi,x,y){
    if(roi.type===0)return x>=roi.x1&&x<=roi.x2&&y>=roi.y1&&y<=roi.y2;
    if(roi.type===1||roi.type===4){
      const v=roi.vertices||[];if(v.length<3)return false;let inside=false;
      for(let i=0,j=v.length-1;i<v.length;j=i++){
        const a=v[i],b=v[j];
        if(((a[1]>y)!==(b[1]>y))&&(x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]))inside=!inside;
      }
      return inside;
    }
    if(roi.type===2)return Math.hypot(x-roi.cx,y-roi.cy)<=roi.radius;
    if(roi.type===3){
      const dx=x-roi.sx,dy=y-roi.sy;if(Math.hypot(dx,dy)>roi.sRadius)return false;
      const norm=a=>(a%(2*Math.PI)+2*Math.PI)%(2*Math.PI),a=norm(Math.atan2(dy,dx));
      const a1=norm((roi.sAngleStart||0)*Math.PI/180),a2=norm((roi.sAngleEnd||0)*Math.PI/180);
      return a1<=a2?a>=a1&&a<=a2:a>=a1||a<=a2;
    }
    return false;
  }

  function buildFusionFrame(frames,toleranceMs=defaults.fusionToleranceMs){
    const valid=frames.filter(f=>f&&Number.isFinite(f.timestamp)&&f.xyz&&f.pts);
    if(!valid.length)return {timestamp:null,points:[],sensorIds:[]};
    const times=valid.map(f=>f.timestamp).sort((a,b)=>a-b),timestamp=times[Math.floor(times.length/2)];
    const accepted=valid.filter(f=>Math.abs(f.timestamp-timestamp)<=toleranceMs),points=[];
    for(const f of accepted)for(let i=0;i<f.xyz.length;i+=3)points.push({
      x:f.xyz[i],y:f.xyz[i+1],z:f.xyz[i+2]||0,sensorId:f.sourceId,
      localX:f.pts[i],localY:f.pts[i+1],intensity:f.pts[i+2],timestamp:f.timestamp
    });
    return {timestamp,points,sensorIds:[...new Set(accepted.map(f=>f.sourceId))].sort((a,b)=>a-b)};
  }

  function dbscan(points,eps=defaults.clusterEps,minPoints=defaults.clusterMinPoints){
    const buckets=new Map(),labels=new Int32Array(points.length);labels.fill(-2);
    const cell=(x,y)=>[Math.floor(x/eps),Math.floor(y/eps)],key=(x,y)=>x+','+y;
    points.forEach((p,i)=>{const c=cell(p.x,p.y),k=key(c[0],c[1]);if(!buckets.has(k))buckets.set(k,[]);buckets.get(k).push(i);});
    const neighbors=i=>{const p=points[i],c=cell(p.x,p.y),out=[],e2=eps*eps;
      for(let dx=-1;dx<=1;dx++)for(let dy=-1;dy<=1;dy++)for(const j of buckets.get(key(c[0]+dx,c[1]+dy))||[]){
        const q=points[j],xx=p.x-q.x,yy=p.y-q.y;if(xx*xx+yy*yy<=e2)out.push(j);
      }return out;};
    let clusterId=0;
    for(let i=0;i<points.length;i++){
      if(labels[i]!==-2)continue;const first=neighbors(i);
      if(first.length<minPoints){labels[i]=-1;continue;}
      labels[i]=clusterId;const queue=[];
      for(const j of first)if(labels[j]===-2){labels[j]=clusterId;if(j!==i)queue.push(j);}else if(labels[j]===-1)labels[j]=clusterId;
      for(let qi=0;qi<queue.length;qi++){
        const j=queue[qi],near=neighbors(j);if(near.length>=minPoints)for(const k of near){
          if(labels[k]===-2){labels[k]=clusterId;queue.push(k);}else if(labels[k]===-1)labels[k]=clusterId;
        }
      }clusterId++;
    }
    return Array.from({length:clusterId},(_,id)=>points.filter((_,i)=>labels[i]===id));
  }

  function buildObject(points,roiId,id){
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity,sx=0,sy=0,sz=0;
    const pointsPerSensor={};
    for(const p of points){minX=Math.min(minX,p.x);maxX=Math.max(maxX,p.x);minY=Math.min(minY,p.y);maxY=Math.max(maxY,p.y);sx+=p.x;sy+=p.y;sz+=p.z;pointsPerSensor[p.sensorId]=(pointsPerSensor[p.sensorId]||0)+1;}
    const sensors=Object.keys(pointsPerSensor).map(Number).sort((a,b)=>a-b);
    return {id,roiId,points,center:{x:sx/points.length,y:sy/points.length,z:sz/points.length},
      bbox:{minX,maxX,minY,maxY},width:maxX-minX,length:maxY-minY,pointCount:points.length,sensors,pointsPerSensor};
  }

  function fuseObjects(frames,rois,config={}){
    const c={...defaults,...config},fusion=buildFusionFrame(frames,c.fusionToleranceMs),objects=[];
    for(const roi of rois.filter(r=>r&&(r.scope||'fusion')==='fusion')){
      const inside=fusion.points.filter(p=>pointInsideRoi(roi,p.x,p.y));
      const clusters=dbscan(inside,c.clusterEps,c.clusterMinPoints);
      for(const cluster of clusters)objects.push(buildObject(cluster,roi.id,objects.length+1));
    }
    return {timestamp:fusion.timestamp,sensorIds:fusion.sensorIds,points:fusion.points,objects};
  }
  root.ObjectFusion={defaults,pointInsideRoi,buildFusionFrame,dbscan,buildObject,fuseObjects};
  if(typeof module!=='undefined')module.exports=root.ObjectFusion;
})(globalThis);
