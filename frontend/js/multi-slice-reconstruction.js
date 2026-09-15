(function(root){
  'use strict';
  const defaults=Object.freeze({enabled:true,contourSamples:32,interpolationSteps:5,enableEndCaps:true,maxRecommendedSliceGap:1.0,
    showMeasuredContours:true,showEstimatedSurface:true,showWireframe:false,enableSmoothing:false});
  const center=s=>s.world.center||s.world.centroid;
  function orderSlicesForReconstruction(slices){return slices.slice().sort((a,b)=>center(a).z-center(b).z);}
  function validateSliceContour(slice){const c=slice?.world?.contour;return !!(c&&c.length>=3&&c.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)&&Number.isFinite(p.z)));}
  function resampleClosedContour(contour,sampleCount){
    if(!contour||contour.length<2||sampleCount<3)return [];
    const lengths=[],distance=(a,b)=>Math.hypot(b.x-a.x,b.y-a.y,b.z-a.z);let total=0;
    for(let i=0;i<contour.length;i++){const len=distance(contour[i],contour[(i+1)%contour.length]);lengths.push(len);total+=len;}
    if(total<=1e-9)return [];
    const out=[];let edge=0,edgeStart=0;
    for(let k=0;k<sampleCount;k++){
      const target=total*k/sampleCount;while(edge<lengths.length-1&&edgeStart+lengths[edge]<target){edgeStart+=lengths[edge];edge++;}
      const a=contour[edge],b=contour[(edge+1)%contour.length],t=lengths[edge]>0?(target-edgeStart)/lengths[edge]:0;
      out.push({x:a.x+(b.x-a.x)*t,y:a.y+(b.y-a.y)*t,z:a.z+(b.z-a.z)*t});
    }return out;
  }
  function signedAreaXY(c){let area=0;for(let i=0;i<c.length;i++){const a=c[i],b=c[(i+1)%c.length];area+=a.x*b.y-b.x*a.y;}return area/2;}
  function normalizeContourOrientation(reference,contour){return signedAreaXY(reference)*signedAreaXY(contour)<0?contour.slice().reverse():contour.slice();}
  function alignContourCorrespondence(a,b){
    if(a.length!==b.length||!a.length)return {alignedA:a.slice(),alignedB:b.slice(),offset:0,cost:Infinity};
    let bestOffset=0,bestCost=Infinity;
    for(let offset=0;offset<b.length;offset++){let cost=0;for(let i=0;i<a.length;i++){const q=b[(i+offset)%b.length],dx=a[i].x-q.x,dy=a[i].y-q.y;cost+=dx*dx+dy*dy;}if(cost<bestCost){bestCost=cost;bestOffset=offset;}}
    return {alignedA:a.slice(),alignedB:a.map((_,i)=>b[(i+bestOffset)%b.length]),offset:bestOffset,cost:bestCost};
  }
  function interpolateContours(a,b,steps){const sections=[];for(let step=1;step<=steps;step++){const t=step/(steps+1);sections.push(a.map((p,i)=>({x:p.x+(b[i].x-p.x)*t,y:p.y+(b[i].y-p.y)*t,z:p.z+(b[i].z-p.z)*t,estimated:true})));}return sections;}
  function buildMeshFromRings(rings,enableEndCaps){
    const vertices=[],indices=[],n=rings[0]?.length||0;if(rings.length<2||n<3)return {vertices,indices,closed:false,ringCount:rings.length};
    rings.forEach(r=>r.forEach(p=>vertices.push({x:p.x,y:p.y,z:p.z})));
    for(let r=0;r<rings.length-1;r++)for(let i=0;i<n;i++){const j=(i+1)%n,a=r*n+i,b=r*n+j,c=(r+1)*n+j,d=(r+1)*n+i;indices.push(a,b,c,a,c,d);}
    if(enableEndCaps){
      const addCap=(ringIndex,reverse)=>{const ring=rings[ringIndex],centerIndex=vertices.length,centroid=ring.reduce((q,p)=>({x:q.x+p.x,y:q.y+p.y,z:q.z+p.z}),{x:0,y:0,z:0});vertices.push({x:centroid.x/n,y:centroid.y/n,z:centroid.z/n});const base=ringIndex*n;for(let i=0;i<n;i++){const j=(i+1)%n;indices.push(centerIndex,base+(reverse?j:i),base+(reverse?i:j));}};
      addCap(0,true);addCap(rings.length-1,false);
    }
    return {vertices,indices,closed:!!enableEndCaps,ringCount:rings.length};
  }
  function computeEstimatedBBox(vertices){if(!vertices.length)return null;return{minX:Math.min(...vertices.map(p=>p.x)),maxX:Math.max(...vertices.map(p=>p.x)),minY:Math.min(...vertices.map(p=>p.y)),maxY:Math.max(...vertices.map(p=>p.y)),minZ:Math.min(...vertices.map(p=>p.z)),maxZ:Math.max(...vertices.map(p=>p.z))};}
  function computeSurfaceArea(mesh){let area=0;for(let i=0;i<mesh.indices.length;i+=3){const a=mesh.vertices[mesh.indices[i]],b=mesh.vertices[mesh.indices[i+1]],c=mesh.vertices[mesh.indices[i+2]],u={x:b.x-a.x,y:b.y-a.y,z:b.z-a.z},v={x:c.x-a.x,y:c.y-a.y,z:c.z-a.z};area+=.5*Math.hypot(u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x);}return area;}
  function computeMeshVolume(mesh){if(!mesh.closed)return null;let v=0;for(let i=0;i<mesh.indices.length;i+=3){const a=mesh.vertices[mesh.indices[i]],b=mesh.vertices[mesh.indices[i+1]],c=mesh.vertices[mesh.indices[i+2]];v+=a.x*(b.y*c.z-b.z*c.y)-a.y*(b.x*c.z-b.z*c.x)+a.z*(b.x*c.y-b.y*c.x);}return Math.abs(v)/6;}
  function reconstructMultiSliceObject(object,config={}){
    const c={...defaults,...config},ordered=orderSlicesForReconstruction(object.slices||[]),warnings=[],valid=ordered.filter(s=>{const ok=validateSliceContour(s);if(!ok)warnings.push({code:'invalid-contour',sliceId:s.sliceId});return ok;});
    const measuredSlices=ordered.map(s=>({sliceId:s.sliceId,sensorId:s.sensorId,contour:(s.world.contour||[]).map(p=>({...p})),points:s.points}));
    if(!c.enabled||valid.length<2)return {...object,reconstruction:{status:'insufficient-slices',method:'multi-slice-linear',measuredSliceCount:ordered.length,usableSliceCount:valid.length,measuredSensorIds:ordered.map(s=>s.sensorId),contourSamples:c.contourSamples,interpolationSteps:c.interpolationSteps,measuredSlices,interpolatedSections:[],segments:[],mesh:{vertices:[],indices:[],closed:false,ringCount:0},estimated:{centroid:null,bbox:null,height:0,width:0,depth:0,surfaceArea:null,volume:null},confidence:{score:0,measuredSliceCount:ordered.length,associationConfidence:object.association?.confidence??null,warnings:[...warnings,{code:'insufficient-slices'}]}}};
    const sampled=valid.map(s=>resampleClosedContour(s.world.contour,c.contourSamples)),rings=[],segments=[],interpolatedSections=[];
    let previous=sampled[0];rings.push(previous);
    for(let i=0;i<valid.length-1;i++){
      let next=normalizeContourOrientation(previous,sampled[i+1]);const aligned=alignContourCorrespondence(previous,next);next=aligned.alignedB;
      const gap=Math.abs(center(valid[i+1]).z-center(valid[i]).z);if(gap>c.maxRecommendedSliceGap)warnings.push({code:'large-slice-gap',from:valid[i].sliceId,to:valid[i+1].sliceId,gap});
      const intermediate=interpolateContours(previous,next,c.interpolationSteps);interpolatedSections.push(...intermediate.map((contour,k)=>({fromSliceId:valid[i].sliceId,toSliceId:valid[i+1].sliceId,t:(k+1)/(c.interpolationSteps+1),contour})));
      segments.push({fromSliceId:valid[i].sliceId,toSliceId:valid[i+1].sliceId,gap,alignmentOffset:aligned.offset,alignmentCostXY:aligned.cost,interpolatedCount:intermediate.length});
      rings.push(...intermediate,next);previous=next;
    }
    const mesh=buildMeshFromRings(rings,c.enableEndCaps),box=computeEstimatedBBox(mesh.vertices),centroid=mesh.vertices.reduce((q,p)=>({x:q.x+p.x,y:q.y+p.y,z:q.z+p.z}),{x:0,y:0,z:0});centroid.x/=mesh.vertices.length;centroid.y/=mesh.vertices.length;centroid.z/=mesh.vertices.length;
    const sliceFactor=Math.min(valid.length/4,1),association=object.association?.confidence??0.35,quality=valid.length/Math.max(1,ordered.length),gapPenalty=warnings.some(w=>w.code==='large-slice-gap')?.75:1;
    return {...object,reconstruction:{status:'reconstructed-'+valid.length+'-slices',method:'multi-slice-linear',measuredSliceCount:ordered.length,usableSliceCount:valid.length,measuredSensorIds:valid.map(s=>s.sensorId),contourSamples:c.contourSamples,interpolationSteps:c.interpolationSteps,measuredSlices,interpolatedSections,segments,mesh,
      estimated:{centroid,bbox:box,height:box.maxZ-box.minZ,width:box.maxX-box.minX,depth:box.maxY-box.minY,surfaceArea:computeSurfaceArea(mesh),volume:null},
      confidence:{score:Math.max(0,Math.min(1,(.4*sliceFactor+.4*association+.2*quality)*gapPenalty)),measuredSliceCount:valid.length,associationConfidence:object.association?.confidence??null,warnings}}};
  }
  function reconstructFrame(associationFrame,config={}){return{timestamp:associationFrame.timestamp,objects:(associationFrame.multiObjects||[]).map(o=>reconstructMultiSliceObject(o,config))};}
  function serializeReconstructedObject(object){const r=object.reconstruction;return{objectId:object.multiObjectId,timestamp:object.timestamp,measured:{sensorCount:object.sensorCount,slices:r.measuredSlices.map(s=>({sensorId:s.sensorId,sliceId:s.sliceId,points:s.points,contour:s.contour}))},estimated:{status:r.status,method:r.method,bbox:r.estimated.bbox,mesh:r.mesh,surfaceArea:r.estimated.surfaceArea,volume:r.estimated.volume,confidence:r.confidence}};}
  root.MultiSliceReconstruction={defaults,orderSlicesForReconstruction,validateSliceContour,resampleClosedContour,signedAreaXY,normalizeContourOrientation,alignContourCorrespondence,interpolateContours,buildMeshFromRings,computeEstimatedBBox,computeSurfaceArea,computeMeshVolume,reconstructMultiSliceObject,reconstructFrame,serializeReconstructedObject};
  if(typeof module!=='undefined')module.exports=root.MultiSliceReconstruction;
})(globalThis);
