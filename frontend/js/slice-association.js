(function(root){
  'use strict';
  const defaults=Object.freeze({enabled:true,maxTimeDiffMs:100,maxCentroidDistanceXY:0.60,minAssociationScore:0.55,
    weights:Object.freeze({distance:0.50,overlap:0.35,shape:0.15}),enableDebug:true});
  const center=s=>s.world.center||s.world.centroid;
  const bbox=s=>s.world.bbox||s.world.measuredBBox;
  function bboxMetrics(a,b){
    const A=bbox(a),B=bbox(b),iw=Math.max(0,Math.min(A.maxX,B.maxX)-Math.max(A.minX,B.minX)),ih=Math.max(0,Math.min(A.maxY,B.maxY)-Math.max(A.minY,B.minY));
    const intersection=iw*ih,areaA=Math.max(0,(A.maxX-A.minX)*(A.maxY-A.minY)),areaB=Math.max(0,(B.maxX-B.minX)*(B.maxY-B.minY));
    const minArea=Math.min(areaA,areaB),union=areaA+areaB-intersection;
    return {intersection,areaA,areaB,overlapRatio:minArea>0?intersection/minArea:0,iou:union>0?intersection/union:0};
  }
  function shapeSimilarity(a,b){
    const m=bboxMetrics(a,b),ratio=(x,y)=>x>0&&y>0?Math.min(x,y)/Math.max(x,y):0;
    const A=bbox(a),B=bbox(b),aspectA=Math.max(A.maxX-A.minX,A.maxY-A.minY)/Math.max(1e-6,Math.min(A.maxX-A.minX,A.maxY-A.minY));
    const aspectB=Math.max(B.maxX-B.minX,B.maxY-B.minY)/Math.max(1e-6,Math.min(B.maxX-B.minX,B.maxY-B.minY));
    return 0.6*ratio(m.areaA,m.areaB)+0.4*ratio(aspectA,aspectB);
  }
  function computeAssociationScore(a,b,config={}){
    const c={...defaults,...config,weights:{...defaults.weights,...config.weights}},ca=center(a),cb=center(b);
    const timeDiffMs=Math.abs(a.timestamp-b.timestamp),dx=ca.x-cb.x,dy=ca.y-cb.y,centroidDistanceXY=Math.hypot(dx,dy);
    const overlap=bboxMetrics(a,b),shape=shapeSimilarity(a,b),metrics={timeDiffMs,centroidDistanceXY,overlapRatio:overlap.overlapRatio,iou:overlap.iou,shapeSimilarity:shape};
    if(a.sensorId===b.sensorId)return {valid:false,score:0,reason:'same-sensor',metrics};
    if(a.roiId!==b.roiId)return {valid:false,score:0,reason:'different-roi',metrics};
    if(timeDiffMs>c.maxTimeDiffMs)return {valid:false,score:0,reason:'timestamp',metrics};
    if(centroidDistanceXY>c.maxCentroidDistanceXY)return {valid:false,score:0,reason:'centroid-distance',metrics};
    const distance=Math.max(0,1-centroidDistanceXY/c.maxCentroidDistanceXY),w=c.weights,sum=w.distance+w.overlap+w.shape||1;
    const score=(w.distance*distance+w.overlap*overlap.overlapRatio+w.shape*shape)/sum;
    return {valid:score>=c.minAssociationScore,score,reason:score>=c.minAssociationScore?null:'score-threshold',metrics:{...metrics,distanceScore:distance}};
  }
  function buildMultiObject(slices,pairScores,index,frameTimestamp){
    const timestamps=slices.map(s=>s.timestamp).sort((a,b)=>a-b),timestamp=timestamps[Math.floor(timestamps.length/2)];
    const sensorIds=slices.map(s=>s.sensorId).sort((a,b)=>a-b),boxes=slices.map(bbox),centers=slices.map(center);
    const measuredBBox={minX:Math.min(...boxes.map(b=>b.minX)),maxX:Math.max(...boxes.map(b=>b.maxX)),minY:Math.min(...boxes.map(b=>b.minY)),maxY:Math.max(...boxes.map(b=>b.maxY)),minZ:Math.min(...boxes.map(b=>b.minZ)),maxZ:Math.max(...boxes.map(b=>b.maxZ))};
    const relevant=pairScores.filter(p=>slices.includes(p.a)&&slices.includes(p.b)),confidence=relevant.length?relevant.reduce((n,p)=>n+p.score,0)/relevant.length:null;
    return {multiObjectId:'MO-F'+Math.round(frameTimestamp??timestamp)+'-'+index,roiId:slices[0].roiId,timestamp,slices,sensorIds,sensorCount:sensorIds.length,
      totalPointCount:slices.reduce((n,s)=>n+s.pointCount,0),status:sensorIds.length>1?'multi-sensor':'single-sensor',
      world:{centroidXY:{x:centers.reduce((n,p)=>n+p.x,0)/centers.length,y:centers.reduce((n,p)=>n+p.y,0)/centers.length},measuredBBox},
      association:{confidence,pairScores:relevant.map(p=>({sliceA:p.a.sliceId,sliceB:p.b.sliceId,score:p.score,metrics:p.metrics}))}};
  }
  function associateSliceFrame(sliceFrame,config={}){
    const c={...defaults,...config,weights:{...defaults.weights,...config.weights}},nodes=Object.values(sliceFrame.bySensor||{}).flat(),comparisons=[],edges=[];
    if(c.enabled)for(let i=0;i<nodes.length;i++)for(let j=i+1;j<nodes.length;j++){
      if(nodes[i].sensorId===nodes[j].sensorId)continue;const result=computeAssociationScore(nodes[i],nodes[j],c),entry={a:nodes[i],b:nodes[j],...result};
      if(c.enableDebug)comparisons.push(entry);if(result.valid)edges.push(entry);
    }
    const parent=nodes.map((_,i)=>i),sensors=nodes.map(s=>new Set([s.sensorId])),find=i=>parent[i]===i?i:(parent[i]=find(parent[i]));
    edges.sort((a,b)=>b.score-a.score);
    for(const e of edges){const ia=nodes.indexOf(e.a),ib=nodes.indexOf(e.b),ra=find(ia),rb=find(ib);if(ra===rb)continue;
      if([...sensors[ra]].some(id=>sensors[rb].has(id)))continue;parent[rb]=ra;for(const id of sensors[rb])sensors[ra].add(id);
    }
    const groups=new Map();nodes.forEach((s,i)=>{const r=find(i);if(!groups.has(r))groups.set(r,[]);groups.get(r).push(s);});
    const pairScores=edges.filter(e=>find(nodes.indexOf(e.a))===find(nodes.indexOf(e.b)));
    const multiObjects=[...groups.values()].map((s,i)=>buildMultiObject(s,pairScores,i,sliceFrame.timestamp));
    const sliceToMultiObject={};for(const o of multiObjects)for(const s of o.slices)sliceToMultiObject[s.sliceId]=o.multiObjectId;
    return {timestamp:sliceFrame.timestamp,multiObjects,sliceToMultiObject,comparisons};
  }
  root.SliceAssociation={defaults,bboxMetrics,shapeSimilarity,computeAssociationScore,buildMultiObject,associateSliceFrame};
  if(typeof module!=='undefined')module.exports=root.SliceAssociation;
})(globalThis);
