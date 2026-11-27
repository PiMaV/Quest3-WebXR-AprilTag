// Camera bundle adjustment and static-target hand/eye calibration. Runs in a Worker.
// No device profile or user recording is embedded in this solver.
import {fitPlanarPose,incrementPose} from './rig.mjs';
import {cvToGL,product,inverse,rigidVector,vectorRigid,meanRigid,poseAt} from './runtime.mjs';
import {CALIBRATION_SOLVER_VERSION} from './profile.mjs';
const sq=a=>a.reduce((s,x)=>s+x*x,0);
const percentile=(a,p)=>{const s=a.slice().sort((x,y)=>x-y);return s[Math.min(s.length-1,Math.floor(s.length*p))]??Infinity;};
export function linear(A,b) {
  const a=A.map((r,i)=>[...r,b[i]]),n=b.length;
  for(let c=0;c<n;c++){
    let k=c;for(let r=c+1;r<n;r++)if(Math.abs(a[r][c])>Math.abs(a[k][c]))k=r;
    if(Math.abs(a[k][c])<1e-16)return null;
    [a[c],a[k]]=[a[k],a[c]];const d=a[c][c];for(let j=c;j<=n;j++)a[c][j]/=d;
    for(let r=0;r<n;r++)if(r!==c){const f=a[r][c];for(let j=c;j<=n;j++)a[r][j]-=f*a[c][j];}
  }
  return a.map(r=>r[n]);
}
function normal(J,r,n=J[0].length){
  const A=Array.from({length:n},()=>Array(n).fill(0)),b=Array(n).fill(0);
  for(let k=0;k<r.length;k++)for(let i=0;i<n;i++){b[i]-=J[k][i]*r[k];for(let j=0;j<n;j++)A[i][j]+=J[k][i]*J[k][j];}
  return {A,b};
}
export function projectResidual(pose,tags,K){
  const k1=Number.isFinite(K?.k1)?K.k1:0;
  const r=[];
  for(const t of tags)for(let i=0;i<t.points.length;i++){
    const [x,y]=t.points[i],p=pose.t.map((v,j)=>v+pose.R[0][j]*x+pose.R[1][j]*y);
    if(p[2]<=.01)return null;
    const xn=p[0]/p[2],yn=p[1]/p[2],sc=k1?1+k1*(xn*xn+yn*yn):1;
    r.push(K.fx*xn*sc+K.cx-t.pixels[i].x,K.fy*yn*sc+K.cy-t.pixels[i].y);
  }
  return r.every(Number.isFinite)?r:null;
}
const isRadialModel=model=>model==='centered-radial'||model==='pinhole-radial';
const isFreePinhole=model=>model==='pinhole'||model==='pinhole-radial';
const modelK=(g,size,model)=>{
  const free=isFreePinhole(model),rad=isRadialModel(model);
  return {width:size[0],height:size[1],fx:Math.exp(g[0]),fy:Math.exp(free?g[1]:g[0]),
    cx:free?g[2]*size[0]:size[0]/2,cy:free?g[3]*size[1]:size[1]/2,k1:rad?g[free?4:1]:0};
};
const initialModelG=(size,model,initK=null)=>{
  const fx=initK?.fx??.8*size[0],fy=initK?.fy??.8*size[0];
  const cx=initK?initK.cx/size[0]:.5,cy=initK?initK.cy/size[1]:.5;
  const k1=Number.isFinite(initK?.k1)?initK.k1:0;
  if(model==='pinhole')return [Math.log(fx),Math.log(fy),cx,cy];
  if(model==='pinhole-radial')return [Math.log(fx),Math.log(fy),cx,cy,k1];
  if(model==='centered-radial')return [Math.log(fx),k1];
  return [Math.log(fx)];
};
const isPlausibleK=(KK,size)=>KK.fx>size[0]*.2&&KK.fx<size[0]*4&&KK.fy>size[0]*.2&&KK.fy<size[0]*4&&
  KK.cx>0&&KK.cx<size[0]&&KK.cy>0&&KK.cy<size[1]&&Math.abs(KK.k1||0)<.45;
const huberDelta=3;
const huberCost=r=>r.reduce((s,v)=>{const u=v/huberDelta;return s+2*huberDelta*huberDelta*(Math.sqrt(1+u*u)-1);},0);
function buildSchur(rows,poses,g,size,model,damping){
  const n=g.length,K=modelK(g,size,model),S=Array.from({length:n},()=>Array(n).fill(0)),b=Array(n).fill(0),locals=[];
  for(let k=0;k<rows.length;k++){
    const r=projectResidual(poses[k],rows[k].tags,K);if(!r)return null;
    const J=r.map(()=>Array(n+6));
    for(let j=0;j<n+6;j++){
      const eps=1e-6,gg=g.slice(),d=Array(6).fill(0);if(j<n)gg[j]+=eps;else d[j-n]=eps;
      const rr=projectResidual(j<n?poses[k]:incrementPose(poses[k],d),rows[k].tags,modelK(gg,size,model));
      if(!rr)return null;rr.forEach((v,i)=>J[i][j]=(v-r[i])/eps);
    }
    const w=r.map(v=>Math.pow(1+(v/huberDelta)**2,-.25));
    const {A,b:rhs}=normal(J.map((row,i)=>row.map(v=>v*w[i])),r.map((v,i)=>v*w[i]));
    const C=A.slice(n).map(x=>x.slice(n)),B=A.slice(0,n).map(x=>x.slice(n));
    for(let j=0;j<6;j++)C[j][j]+=damping*Math.max(1,C[j][j]);
    const z=linear(C,rhs.slice(n)),W=B.map(row=>linear(C,row));
    if(!z||W.some(x=>!x))return null;
    for(let i=0;i<n;i++){
      b[i]+=rhs[i]-B[i].reduce((s,v,j)=>s+v*z[j],0);
      for(let j=0;j<n;j++)S[i][j]+=A[i][j]-B[i].reduce((s,v,l)=>s+v*W[j][l],0);
    }
    locals.push({z,W});
  }
  return {S,b,locals};
}
/** Fit camera intrinsics (`centered-square`, `centered-radial`, `pinhole`, or `pinhole-radial`) via Pseudo-Huber Schur-complement BA. */
export function fitIntrinsics(rows,size,{model='centered-square',maxIterations=60}={}){
  let g=initialModelG(size,model);
  const n=g.length;
  let poses=rows.map(r=>fitPlanarPose(r.tags,modelK(g,size,model)));
  if(poses.some(p=>!p))throw new Error('Could not solve marker perspectives');
  let damping=.001;
  const cost=(gs,ps)=>ps.reduce((s,p,i)=>{const r=projectResidual(p,rows[i].tags,modelK(gs,size,model));return s+(r?huberCost(r):1e20);},0);
  const rawSq=(gs,ps)=>ps.reduce((s,p,i)=>{const r=projectResidual(p,rows[i].tags,modelK(gs,size,model));return s+(r?sq(r):1e20);},0);
  let error=cost(g,poses);
  for(let iteration=0;iteration<maxIterations;iteration++){
    const schur=buildSchur(rows,poses,g,size,model,damping);
    if(!schur)throw new Error('Too few distinct perspectives');
    const {S,b,locals}=schur;
    for(let i=0;i<n;i++)S[i][i]+=damping*Math.max(1,S[i][i]);
    const dg=linear(S,b);if(!dg)throw new Error('Camera intrinsics are not yet observable');
    const next=g.map((v,i)=>v+dg[i]),KK=modelK(next,size,model);
    const plausible=isPlausibleK(KK,size);
    const pp=poses.map((p,k)=>{
      const updated=incrementPose(p,locals[k].z.map((v,i)=>v-locals[k].W.reduce((s,w,j)=>s+w[i]*dg[j],0)));
      // A planar pose has competing minima near frontal views. Restart from
      // its independent homography as K changes rather than locking a bad basin.
      const fresh=plausible?fitPlanarPose(rows[k].tags,KK):null;
      const a=projectResidual(updated,rows[k].tags,KK),b=fresh&&projectResidual(fresh,rows[k].tags,KK);
      return b&&(!a||huberCost(b)<huberCost(a))?fresh:updated;
    });
    const e=plausible?cost(next,pp):Infinity;
    if(e<error){const improvement=error-e;g=next;poses=pp;error=e;damping=Math.max(1e-9,damping/3);if(improvement<1e-8)break;}
    else damping*=10;
    if(damping>1e12)break;
  }
  const finalSchur=buildSchur(rows,poses,g,size,model,0)?.S??buildSchur(rows,poses,g,size,model,1e-9)?.S??null;
  const points=rows.reduce((s,r)=>s+r.tags.length*4,0),variance=error/Math.max(1,2*points-6*rows.length-n);
  const std=g.map((_,i)=>{if(!finalSchur)return Infinity;const e=Array(n).fill(0);e[i]=1;const col=linear(finalSchur,e);return col&&col[i]>0?Math.sqrt(col[i]*variance):Infinity;});
  return {K:modelK(g,size,model),model,rmsPx:Math.sqrt(rawSq(g,poses)/points),std,poses,schur:finalSchur};
}
function cholesky(S){
  if(!Array.isArray(S)||!S.length)return null;
  const n=S.length,L=Array.from({length:n},()=>Array(n).fill(0));
  for(let i=0;i<n;i++)for(let j=0;j<=i;j++){
    let sum=S[i][j];
    for(let k=0;k<j;k++)sum-=L[i][k]*L[j][k];
    if(i===j){if(!(sum>1e-12))return null;L[i][j]=Math.sqrt(sum);}
    else L[i][j]=sum/L[j][j];
  }
  return L;
}
function lm(fun,initial,{maxIterations=70}={}){
  let x=initial.slice(),r=fun(x),lambda=.001;
  if(!r)throw new Error('Missing time-matched XR poses');
  const robust=a=>a.reduce((s,v)=>s+2*(Math.sqrt(1+v*v)-1),0);let cost=robust(r);
  for(let iteration=0;iteration<maxIterations;iteration++){
    const J=r.map(()=>Array(x.length));
    for(let j=0;j<x.length;j++){
      const eps=j===12?.00001:.000001,xx=x.slice();xx[j]+=eps;const rr=fun(xx);if(!rr)return {x,cost};
      rr.forEach((v,i)=>J[i][j]=(v-r[i])/eps);
    }
    const weights=r.map(v=>Math.pow(1+v*v,-.25));
    const {A,b}=normal(J.map((row,i)=>row.map(v=>v*weights[i])),r.map((v,i)=>v*weights[i]));
    for(let j=0;j<x.length;j++)A[j][j]+=lambda*Math.max(1,A[j][j]);
    const step=linear(A,b);if(!step)break;
    const xx=x.map((v,j)=>v+step[j]);if(xx.length>12)xx[12]=Math.max(-.2499,Math.min(.2499,xx[12]));
    const rr=fun(xx),next=rr?robust(rr):Infinity;
    if(next<cost){const change=cost-next;x=xx;r=rr;cost=next;lambda=Math.max(1e-9,lambda/3);if(change<1e-9)break;}
    else lambda*=10;
    if(lambda>1e12)break;
  }
  return {x,cost};
}
function hasContinuousHistory(history,tMin,tMax){
  if(history.length<2||tMin<history[0].xrRafTimeMs||tMax>history.at(-1).xrRafTimeMs)return false;
  let lo=0,hi=history.length-1;
  while(hi-lo>1){const m=(lo+hi)>>1;if(history[m].xrRafTimeMs<=tMin)lo=m;else hi=m;}
  for(let i=lo;i<history.length-1;i++){
    const gap=history[i+1].xrRafTimeMs-history[i].xrRafTimeMs;
    if(gap<=0||gap>100)return false;
    if(history[i+1].xrRafTimeMs>=tMax)return true;
  }
  return false;
}
export function worldPoses(rows,history,K,X,offset){
  return rows.map(r=>{
    const h=poseAt(history,r.captureTime+offset),c=fitPlanarPose(r.tags,K);
    return h&&c?product(h,X,cvToGL(c)):null;
  });
}
/** Measure 95th-percentile position (mm) and orientation (deg) scatter of world rig poses. */
export function consistency(worlds,reference=null){
  if(!worlds.length||worlds.some(x=>!x))return {ok:false,positionP95Mm:Infinity,angleP95Deg:Infinity};
  const ref=reference??meanRigid(worlds),inv=inverse(ref),errors=worlds.map(w=>rigidVector(product(inv,w)));
  const position=errors.map(e=>Math.hypot(...e.slice(3))*1000),angle=errors.map(e=>Math.hypot(...e.slice(0,3))*180/Math.PI);
  return {ok:true,positionP95Mm:percentile(position,.95),angleP95Deg:percentile(angle,.95),reference:ref};
}
/** Solve `viewerFromCameraGL`, `worldFromRig`, and `timeOffsetMs` across multiple delay seeds. */
export function fitHandEye(rows,history,K){
  const validRows=[],cameras=[];
  for(const r of rows){
    if(!hasContinuousHistory(history,r.captureTime-250,r.captureTime+250))continue;
    const pose=fitPlanarPose(r.tags,K);if(!pose)continue;
    validRows.push(r);cameras.push(cvToGL(pose));
  }
  if(validRows.length<24)throw new Error('Missing time-matched XR poses');
  const seeds=[];
  const fun=x=>{
    const X=vectorRigid(x.slice(0,6)),invL=inverse(vectorRigid(x.slice(6,12))),out=[];
    for(let i=0;i<validRows.length;i++){
      const H=poseAt(history,validRows[i].captureTime+x[12]*1000);if(!H)return null;
      const e=rigidVector(product(invL,H,X,cameras[i]));out.push(...e.map(v=>v/.01));
    }return out;
  };
  for(const delay of [-.12,0,.12]){
    const aligned=validRows.map((r,i)=>{const H=poseAt(history,r.captureTime+delay*1000);return H?product(H,cameras[i]):null;}).filter(Boolean);
    if(aligned.length<24)continue;
    const L=meanRigid(aligned);
    seeds.push(lm(fun,[0,0,0,0,0,0,...rigidVector(L),delay]));
  }
  if(!seeds.length)throw new Error('Missing time-matched XR poses');
  const fit=seeds.sort((a,b)=>a.cost-b.cost)[0];
  const equivalent=seeds.filter(s=>s.cost<=fit.cost+Math.max(.01,fit.cost*.01));
  const delaySpreadMs=(Math.max(...equivalent.map(s=>s.x[12]))-Math.min(...equivalent.map(s=>s.x[12])))*1000;
  return {viewerFromCameraGL:vectorRigid(fit.x.slice(0,6)),worldFromRig:vectorRigid(fit.x.slice(6,12)),
    timeOffsetMs:fit.x[12]*1000,cost:fit.cost,delaySpreadMs,convergedSeeds:equivalent.length,usedViews:validRows.length};
}
/** Check whether calibration views span two non-collinear head rotation axes and >= 10 cm translation. */
export function diversity(rows,history,imageSize=null){
  const poses=rows.map(r=>poseAt(history,r.captureTime)).filter(Boolean);
  if(poses.length<3)return {ok:false,reason:'Time-matched XR poses missing'};
  const inv=inverse(poses[0]),vectors=poses.map(p=>rigidVector(product(inv,p)).slice(0,3));
  const maxAngle=Math.max(...vectors.map(v=>Math.hypot(...v)));
  // Two non-collinear rotation axes AND real 3D translation span are essential to decouple fx and hand/eye depth.
  let area=0,maxTransM=0;
  for(const a of vectors)for(const b of vectors)area=Math.max(area,Math.hypot(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]));
  for(let i=0;i<poses.length;i++){
    const a=poses[i];
    for(let j=i+1;j<poses.length;j++){
      const b=poses[j];
      const d=Math.hypot(b[12]-a[12],b[13]-a[13],b[14]-a[14]);
      if(d>maxTransM)maxTransM=d;
    }
  }
  const ok=maxAngle>.2&&area>.015&&maxTransM>=.10;
  const reason=(maxAngle>.2&&area>.015&&maxTransM<.10)
    ?'Move head wider & closer/farther around multiple axes (>= 10 cm)'
    :'Rotate head around multiple axes and vary distance/side-to-side (>= 10 cm)';
  let maxCornerRadiusFrac=null,visitedQuadrants=null;
  if(Array.isArray(imageSize)&&imageSize[0]>0&&imageSize[1]>0){
    const hw=imageSize[0]*.5,hh=imageSize[1]*.5;
    let maxR=0;const quads=new Set();
    for(const r of rows){
      const ps=r.tags.flatMap(t=>t.pixels);if(!ps.length)continue;
      let sx=0,sy=0;
      for(const p of ps){
        sx+=p.x;sy+=p.y;
        const rf=Math.hypot((p.x-hw)/hw,(p.y-hh)/hh);
        if(rf>maxR)maxR=rf;
      }
      const cx=sx/ps.length,cy=sy/ps.length;
      quads.add((cx>=hw?1:0)|(cy>=hh?2:0));
    }
    maxCornerRadiusFrac=maxR;visitedQuadrants=quads.size;
  }
  return {ok,maxAngleDeg:maxAngle*180/Math.PI,rotationArea:area,translationSpanM:maxTransM,imageCentreSpanPx:(()=>{
    const centres=rows.map(r=>{const ps=r.tags.flatMap(t=>t.pixels);return ['x','y'].map(k=>ps.reduce((s,p)=>s+p[k],0)/ps.length);});
    return [0,1].map(i=>Math.max(...centres.map(p=>p[i]))-Math.min(...centres.map(p=>p[i])));
  })(),maxCornerRadiusFrac,visitedQuadrants,reason};
}
function jointReprojectionErrors(rows,history,K,X,L,offsetMs){
  const invX=inverse(X),k1=Number.isFinite(K?.k1)?K.k1:0,out=[];
  for(const r of rows){
    const H=poseAt(history,r.captureTime+offsetMs);if(!H)return null;
    const C=product(invX,inverse(H),L);
    for(const t of r.tags)for(let i=0;i<t.points.length;i++){
      const [tx,ty]=t.points[i];
      const px=C[12]+C[0]*tx+C[4]*ty,py=C[13]+C[1]*tx+C[5]*ty,pz=C[14]+C[2]*tx+C[6]*ty;
      const zCV=-pz;if(!(zCV>.02))return null;
      const xn=px/zCV,yn=-py/zCV,sc=k1?1+k1*(xn*xn+yn*yn):1;
      out.push(K.fx*xn*sc+K.cx-t.pixels[i].x,K.fy*yn*sc+K.cy-t.pixels[i].y);
    }
  }
  return out.length&&out.every(Number.isFinite)?out:null;
}
function jointReprojectionRms(rows,history,K,X,L,offsetMs){
  const e=jointReprojectionErrors(rows,history,K,X,L,offsetMs);
  return e?Math.sqrt(sq(e)/(e.length/2)):Infinity;
}
/** Aggregate corner reprojection residuals across normalized radius bins and image quadrants. */
export function summarizeRadialResiduals(rows,K,{poses=null,history=null,viewerFromCameraGL=null,worldFromRig=null,timeOffsetMs=0}={}){
  const mkAcc=()=>({count:0,sumSq:0,sumRad:0,sumDx:0,sumDy:0});
  const fin=a=>({count:a.count,rmsPx:a.count?Math.sqrt(a.sumSq/a.count):0,
    meanRadialPx:a.count?a.sumRad/a.count:0,meanDxPx:a.count?a.sumDx/a.count:0,meanDyPx:a.count?a.sumDy/a.count:0});
  const inner=mkAcc(),mid=mkAcc(),outer=mkAcc(),all=mkAcc();
  const qAcc={tl:mkAcc(),tr:mkAcc(),bl:mkAcc(),br:mkAcc()},radii=[];
  const k1=Number.isFinite(K?.k1)?K.k1:0;
  const useJoint=Boolean(history&&viewerFromCameraGL&&worldFromRig);
  const invX=useJoint?inverse(viewerFromCameraGL):null;
  const pushAcc=(acc,dx,dy,rad)=>{acc.count++;acc.sumSq+=dx*dx+dy*dy;acc.sumRad+=rad;acc.sumDx+=dx;acc.sumDy+=dy;};
  for(let ri=0;ri<rows.length;ri++){
    const row=rows[ri];
    let C=null,pose=null;
    if(useJoint){
      const H=poseAt(history,row.captureTime+timeOffsetMs);if(!H)continue;
      C=product(invX,inverse(H),worldFromRig);
    }else{
      pose=poses?.[ri]??fitPlanarPose(row.tags,K);if(!pose)continue;
    }
    for(const tag of row.tags)for(let i=0;i<tag.points.length;i++){
      const [tx,ty]=tag.points[i],obs=tag.pixels[i];
      let xn,yn;
      if(useJoint){
        const px=C[12]+C[0]*tx+C[4]*ty,py=C[13]+C[1]*tx+C[5]*ty,pz=C[14]+C[2]*tx+C[6]*ty;
        const zCV=-pz;if(!(zCV>.02))continue;
        xn=px/zCV;yn=-py/zCV;
      }else{
        const px=pose.t[0]+pose.R[0][0]*tx+pose.R[1][0]*ty;
        const py=pose.t[1]+pose.R[0][1]*tx+pose.R[1][1]*ty;
        const pz=pose.t[2]+pose.R[0][2]*tx+pose.R[1][2]*ty;
        if(!(pz>.01))continue;
        xn=px/pz;yn=py/pz;
      }
      const sc=k1?1+k1*(xn*xn+yn*yn):1;
      const dx=K.fx*xn*sc+K.cx-obs.x,dy=K.fy*yn*sc+K.cy-obs.y;
      if(!Number.isFinite(dx)||!Number.isFinite(dy))continue;
      const rx=(obs.x-K.cx)/K.fx,ry=(obs.y-K.cy)/K.fy,r=Math.hypot(rx,ry);
      const rPx=Math.hypot(obs.x-K.cx,obs.y-K.cy);
      const radPx=rPx>1e-9?(dx*(obs.x-K.cx)+dy*(obs.y-K.cy))/rPx:0;
      radii.push(r);pushAcc(all,dx,dy,radPx);
      if(r<.25)pushAcc(inner,dx,dy,radPx);
      else if(r<.45)pushAcc(mid,dx,dy,radPx);
      else pushAcc(outer,dx,dy,radPx);
      const qKey=(obs.y<K.cy?'t':'b')+(obs.x<K.cx?'l':'r');
      pushAcc(qAcc[qKey],dx,dy,radPx);
    }
  }
  radii.sort((a,b)=>a-b);
  return {totalCorners:all.count,rmsPx:fin(all).rmsPx,
    rMax:radii.length?radii.at(-1):0,rP95:radii.length?radii[Math.min(radii.length-1,Math.floor(radii.length*.95))]:0,
    inner:fin(inner),mid:fin(mid),outer:fin(outer),
    quadrants:{tl:fin(qAcc.tl),tr:fin(qAcc.tr),bl:fin(qAcc.bl),br:fin(qAcc.br)}};
}
function refineJointCalibration(validRows,history,initK,hand,{imageSize,model='centered-square',opticalPrior=null}){
  const init=[
    ...rigidVector(hand.viewerFromCameraGL),
    ...rigidVector(hand.worldFromRig),
    Math.max(-.24,Math.min(.24,hand.timeOffsetMs/1000)),
    ...initialModelG(imageSize,model,initK),
  ];
  const fun=x=>{
    const KK=modelK(x.slice(13),imageSize,model);
    if(!isPlausibleK(KK,imageSize))return null;
    const err=jointReprojectionErrors(validRows,history,KK,vectorRigid(x.slice(0,6)),vectorRigid(x.slice(6,12)),x[12]*1000);
    if(!err)return null;
    const out=err.map(v=>v*.5);
    if(opticalPrior?.L&&opticalPrior?.g0&&opticalPrior.wOpt>0){
      const dg=x.slice(13).map((v,i)=>v-opticalPrior.g0[i]);
      const s=.5*opticalPrior.wOpt;
      for(let i=0;i<dg.length;i++){
        let v=0;
        for(let j=i;j<dg.length;j++)v+=opticalPrior.L[j][i]*dg[j];
        out.push(s*v);
      }
    }
    return out;
  };
  const fit=lm(fun,init,{maxIterations:30});
  const x=fit.x;
  const K=modelK(x.slice(13),imageSize,model);
  const viewerFromCameraGL=vectorRigid(x.slice(0,6));
  const worldFromRig=vectorRigid(x.slice(6,12));
  const timeOffsetMs=x[12]*1000;
  const r0=fun(x);
  const gLen=x.length-13;
  let intrinsicStd=Array(gLen).fill(Infinity);
  if(r0){
    const n=x.length,J=r0.map(()=>Array(n));
    let ok=true;
    for(let j=0;j<n;j++){
      const eps=j===12?1e-5:1e-6,xx=x.slice();xx[j]+=eps;
      const rr=fun(xx);if(!rr){ok=false;break;}
      for(let i=0;i<r0.length;i++)J[i][j]=(rr[i]-r0[i])/eps;
    }
    if(ok){
      const {A}=normal(J,r0,n);
      const dof=Math.max(1,r0.length-n);
      const variance=sq(r0)/dof;
      intrinsicStd=intrinsicStd.map((_,k)=>{
        const idx=13+k,e=Array(n).fill(0);e[idx]=1;
        const col=linear(A,e);
        return col&&col[idx]>0?Math.sqrt(col[idx]*variance):Infinity;
      });
    }
  }
  const jointTrainingRmsPx=jointReprojectionRms(validRows,history,K,viewerFromCameraGL,worldFromRig,timeOffsetMs);
  return {K,viewerFromCameraGL,worldFromRig,timeOffsetMs,jointTrainingRmsPx,fxStd:intrinsicStd[0],intrinsicStd};
}
/** Full camera intrinsics, hand-eye, capture delay, joint BA refinement, and 80/20 holdout solver. */
export function solveCalibration({rows,history,imageSize,model='centered-square'},progress=()=>{}){
  if(rows.length<30)throw new Error('At least 30 distinct multi-tag views required');
  const scope=rows[0].scope;
  if(rows.some(r=>r.scope!==scope)||history.some(p=>p.scope!==scope))throw new Error('Camera or XR space changed: collect again');
  if(!diversity(rows,history,imageSize).ok)throw new Error(diversity(rows,history,imageSize).reason);
  // Last fifth is never fitted: this also exposes motion-dependent overfitting.
  const cut=Math.floor(rows.length*.8),training=rows.slice(0,cut),held=rows.slice(cut);
  if(!diversity(training,history,imageSize).ok)throw new Error('Collect more diverse head motions and distance');
  progress('Fitting camera model');const fit=fitIntrinsics(training,imageSize,{model});
  progress('Solving camera-headset transform and delay');const hand0=fitHandEye(training,history,fit.K);
  const validTraining=training.filter(r=>hasContinuousHistory(history,r.captureTime-250,r.captureTime+250));
  const jointStep=Math.max(1,validTraining.length/48);
  const jointRows=validTraining.length<=48?validTraining:Array.from({length:48},(_,i)=>validTraining[Math.min(validTraining.length-1,Math.floor(i*jointStep))]);
  const initJointRms=jointReprojectionRms(jointRows,history,fit.K,hand0.viewerFromCameraGL,hand0.worldFromRig,hand0.timeOffsetMs);
  progress('Refining focal length & hand-eye jointly');
  let finalK=fit.K,finalX=hand0.viewerFromCameraGL,finalL=hand0.worldFromRig,finalOffset=hand0.timeOffsetMs;
  let jointTrainingRmsPx=initJointRms,jointIntrinsicStd=fit.std.map(()=>Infinity);
  const priorL=fit.schur?cholesky(fit.schur):null;
  const wOpt=Math.min(4.0,Math.max(1.0,(initJointRms/Math.max(.25,fit.rmsPx))*.7));
  const opticalPrior=priorL?{g0:initialModelG(imageSize,model,fit.K),L:priorL,wOpt}:null;
  try{
    const joint=refineJointCalibration(jointRows,history,fit.K,hand0,{imageSize,model,opticalPrior});
    if(Number.isFinite(joint.jointTrainingRmsPx)&&joint.jointTrainingRmsPx<=initJointRms+.15){
      finalK=joint.K;finalX=joint.viewerFromCameraGL;finalL=joint.worldFromRig;finalOffset=joint.timeOffsetMs;
      jointTrainingRmsPx=joint.jointTrainingRmsPx;jointIntrinsicStd=joint.intrinsicStd;
    }
  }catch{ /* keep decoupled fit if joint step fails */ }
  const worlds=worldPoses(held,history,finalK,finalX,finalOffset);
  const validation=consistency(worlds,finalL);
  const heldPoses=held.map(r=>fitPlanarPose(r.tags,finalK));
  const errors=held.flatMap((r,i)=>{
    const pose=heldPoses[i],e=pose&&projectResidual(pose,r.tags,finalK);return e??[1e6];
  });
  const heldRms=Math.sqrt(sq(errors)/(errors.length/2));
  const jointHeldOutRmsPx=jointReprojectionRms(held,history,finalK,finalX,finalL,finalOffset);
  const intrinsicStd=fit.std.map((s,i)=>Number.isFinite(jointIntrinsicStd[i])?Math.min(s,jointIntrinsicStd[i]):s);
  const free=isFreePinhole(model),rad=isRadialModel(model);
  const uncertainty=intrinsicStd[0]<.05&&
    (!free||(intrinsicStd[1]<.05&&intrinsicStd[2]<.04&&intrinsicStd[3]<.04))&&
    (!rad||intrinsicStd[free?4:1]<.06);
  const camT=finalX.slice(12,15);
  const offsetSafe=Math.abs(finalOffset)<230,translationSafe=Math.hypot(camT[0],camT[1],camT[2])<.16&&Math.abs(camT[2])<.09;
  const delayIdentified=hand0.convergedSeeds>=2&&hand0.delaySpreadMs<15;
  const k1=finalK.k1??0,k1Safe=Math.abs(k1)<.45;
  const passed=uncertainty&&offsetSafe&&translationSafe&&delayIdentified&&k1Safe&&fit.rmsPx<1&&heldRms<1.2&&
    validation.positionP95Mm<8.5&&validation.angleP95Deg<1.5&&jointHeldOutRmsPx<10.0;
  const assumptions=[
    ...(free?[]:['square pixels','centered principal point']),
    rad?'radial distortion k1':'zero distortion',
    'fixed stationary target','nominal print scale',
  ];
  const radialResiduals=summarizeRadialResiduals(held,finalK,{poses:heldPoses});
  const jointRadialResiduals=summarizeRadialResiduals(held,finalK,{history,viewerFromCameraGL:finalX,worldFromRig:finalL,timeOffsetMs:finalOffset});
  return {K:finalK,distCoeffs:[k1,0,0,0,0],
    viewerFromCameraGL:finalX,worldFromRig:finalL,timeOffsetMs:finalOffset,
    cost:hand0.cost,delaySpreadMs:hand0.delaySpreadMs,convergedSeeds:hand0.convergedSeeds,usedViews:hand0.usedViews,
    model,solverVersion:CALIBRATION_SOLVER_VERSION,assumptions,
    validation:{...validation,passed,trainingViews:training.length,heldOutViews:held.length,
      trainingRmsPx:fit.rmsPx,heldOutRmsPx:heldRms,jointTrainingRmsPx,jointHeldOutRmsPx,
      planarFx:fit.K.fx,jointFx:finalK.fx,planarK1:fit.K.k1??0,jointK1:k1,intrinsicStd,
      radialResiduals,jointRadialResiduals,
      uncertainty,offsetSafe,translationSafe,delayIdentified},
    independentlyValidated:false};
}
