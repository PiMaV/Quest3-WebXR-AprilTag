// Zero-dependency column-major 4x4 rigid-transform and quaternion math.
export const identity=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];

function mul4(a,b){
  const out=new Array(16);
  for(let col=0;col<4;col++){
    const c=col*4,b0=b[c],b1=b[c+1],b2=b[c+2],b3=b[c+3];
    for(let row=0;row<4;row++)
      out[c+row]=a[row]*b0+a[4+row]*b1+a[8+row]*b2+a[12+row]*b3;
  }
  return out;
}
export const product=(...mats)=>mats.reduce((acc,m)=>mul4(acc,m),identity());

export function det4(m){
  const n11=m[0],n21=m[1],n31=m[2],n41=m[3],n12=m[4],n22=m[5],n32=m[6],n42=m[7];
  const n13=m[8],n23=m[9],n33=m[10],n43=m[11],n14=m[12],n24=m[13],n34=m[14],n44=m[15];
  const t11=n23*n34*n42-n24*n33*n42+n24*n32*n43-n22*n34*n43-n23*n32*n44+n22*n33*n44;
  const t12=n14*n33*n42-n13*n34*n42-n14*n32*n43+n12*n34*n43+n13*n32*n44-n12*n33*n44;
  const t13=n13*n24*n42-n14*n23*n42+n14*n22*n43-n12*n24*n43-n13*n22*n44+n12*n23*n44;
  const t14=n14*n23*n32-n13*n24*n32-n14*n22*n33+n12*n24*n33+n13*n22*n34-n12*n23*n34;
  return n11*t11+n21*t12+n31*t13+n41*t14;
}

export function inverse(m){
  const n11=m[0],n21=m[1],n31=m[2],n41=m[3],n12=m[4],n22=m[5],n32=m[6],n42=m[7];
  const n13=m[8],n23=m[9],n33=m[10],n43=m[11],n14=m[12],n24=m[13],n34=m[14],n44=m[15];
  const t11=n23*n34*n42-n24*n33*n42+n24*n32*n43-n22*n34*n43-n23*n32*n44+n22*n33*n44;
  const t12=n14*n33*n42-n13*n34*n42-n14*n32*n43+n12*n34*n43+n13*n32*n44-n12*n33*n44;
  const t13=n13*n24*n42-n14*n23*n42+n14*n22*n43-n12*n24*n43-n13*n22*n44+n12*n23*n44;
  const t14=n14*n23*n32-n13*n24*n32-n14*n22*n33+n12*n24*n33+n13*n22*n34-n12*n23*n34;
  const det=n11*t11+n21*t12+n31*t13+n41*t14;
  if(det===0)return new Array(16).fill(0);
  const d=1/det;
  return [
    t11*d,
    (n24*n33*n41-n23*n34*n41-n24*n31*n43+n21*n34*n43+n23*n31*n44-n21*n33*n44)*d,
    (n22*n34*n41-n24*n32*n41+n24*n31*n42-n21*n34*n42-n22*n31*n44+n21*n32*n44)*d,
    (n23*n32*n41-n22*n33*n41-n23*n31*n42+n21*n33*n42+n22*n31*n43-n21*n32*n43)*d,
    t12*d,
    (n13*n34*n41-n14*n33*n41+n14*n31*n43-n11*n34*n43-n13*n31*n44+n11*n33*n44)*d,
    (n14*n32*n41-n12*n34*n41-n14*n31*n42+n11*n34*n42+n12*n31*n44-n11*n32*n44)*d,
    (n12*n33*n41-n13*n32*n41+n13*n31*n42-n11*n33*n42-n12*n31*n43+n11*n32*n43)*d,
    t13*d,
    (n14*n23*n41-n13*n24*n41-n14*n21*n43+n11*n24*n43+n13*n21*n44-n11*n23*n44)*d,
    (n12*n24*n41-n14*n22*n41+n14*n21*n42-n11*n24*n42-n12*n21*n44+n11*n22*n44)*d,
    (n13*n22*n41-n12*n23*n41-n13*n21*n42+n11*n23*n42+n12*n21*n43-n11*n22*n43)*d,
    t14*d,
    (n13*n24*n31-n14*n23*n31+n14*n21*n33-n11*n24*n33-n13*n21*n34+n11*n23*n34)*d,
    (n14*n22*n31-n12*n24*n31-n14*n21*n32+n11*n24*n32+n12*n21*n34-n11*n22*n34)*d,
    (n12*n23*n31-n13*n22*n31+n13*n21*n32-n11*n23*n32-n12*n21*n33+n11*n22*n33)*d,
  ];
}

export function quatNormalize(q){
  const l=Math.hypot(q[0],q[1],q[2],q[3]);
  return l>0?[q[0]/l,q[1]/l,q[2]/l,q[3]/l]:[0,0,0,1];
}
export const quatDot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
export const quatAngle=(a,b)=>2*Math.acos(Math.min(1,Math.max(-1,Math.abs(quatDot(a,b)))));

export function quatSlerp(a,b,t){
  let d=quatDot(a,b),bx=b[0],by=b[1],bz=b[2],bw=b[3];
  if(d<0){d=-d;bx=-bx;by=-by;bz=-bz;bw=-bw;}
  if(d>0.999999)return quatNormalize([a[0]+(bx-a[0])*t,a[1]+(by-a[1])*t,a[2]+(bz-a[2])*t,a[3]+(bw-a[3])*t]);
  const th=Math.acos(Math.min(1,d)),s=Math.sin(th),wa=Math.sin((1-t)*th)/s,wb=Math.sin(t*th)/s;
  return quatNormalize([a[0]*wa+bx*wb,a[1]*wa+by*wb,a[2]*wa+bz*wb,a[3]*wa+bw*wb]);
}

export function matToQuat(m){
  const m11=m[0],m21=m[1],m31=m[2],m12=m[4],m22=m[5],m32=m[6],m13=m[8],m23=m[9],m33=m[10];
  const trace=m11+m22+m33;
  let x,y,z,w;
  if(trace>0){
    const s=0.5/Math.sqrt(trace+1);
    w=0.25/s;x=(m32-m23)*s;y=(m13-m31)*s;z=(m21-m12)*s;
  }else if(m11>m22&&m11>m33){
    const s=2*Math.sqrt(1+m11-m22-m33);
    w=(m32-m23)/s;x=0.25*s;y=(m12+m21)/s;z=(m13+m31)/s;
  }else if(m22>m33){
    const s=2*Math.sqrt(1+m22-m11-m33);
    w=(m13-m31)/s;x=(m12+m21)/s;y=0.25*s;z=(m23+m32)/s;
  }else{
    const s=2*Math.sqrt(1+m33-m11-m22);
    w=(m21-m12)/s;x=(m13+m31)/s;y=(m23+m32)/s;z=0.25*s;
  }
  return quatNormalize([x,y,z,w]);
}

export function composeRigid(p,q){
  const [x,y,z,w]=quatNormalize(q);
  const x2=x+x,y2=y+y,z2=z+z,xx=x*x2,xy=x*y2,xz=x*z2,yy=y*y2,yz=y*z2,zz=z*z2,wx=w*x2,wy=w*y2,wz=w*z2;
  return [
    1-(yy+zz),xy+wz,xz-wy,0,
    xy-wz,1-(xx+zz),yz+wx,0,
    xz+wy,yz-wx,1-(xx+yy),0,
    p[0],p[1],p[2],1,
  ];
}

export function cvToGL(pose){
  const [a,b,c]=pose.R,t=pose.t;
  return [a[0],-a[1],-a[2],0,b[0],-b[1],-b[2],0,c[0],-c[1],-c[2],0,t[0],-t[1],-t[2],1];
}

export function rigidVector(a){
  let q=matToQuat(a);
  if(q[3]<0)q=[-q[0],-q[1],-q[2],-q[3]];
  const s=Math.hypot(q[0],q[1],q[2]),k=s>1e-10?2*Math.atan2(s,q[3])/s:2;
  return [q[0]*k,q[1]*k,q[2]*k,a[12],a[13],a[14]];
}

export function vectorRigid(v){
  const a=Math.hypot(v[0],v[1],v[2]);
  const q=a>1e-12?[v[0]/a*Math.sin(a/2),v[1]/a*Math.sin(a/2),v[2]/a*Math.sin(a/2),Math.cos(a/2)]:[0,0,0,1];
  return composeRigid([v[3],v[4],v[5]],q);
}

export function meanRigid(poses){
  const qs=poses.map(matToQuat),q=[0,0,0,0];
  for(const v of qs){const k=quatDot(v,qs[0])<0?-1:1;q[0]+=k*v[0];q[1]+=k*v[1];q[2]+=k*v[2];q[3]+=k*v[3];}
  const p=[12,13,14].map(i=>poses.reduce((s,m)=>s+m[i],0)/poses.length);
  return composeRigid(p,q);
}

export function poseAt(history,time){
  let lo=0,hi=history.length-1;
  if(hi<1||time<history[0].xrRafTimeMs||time>history[hi].xrRafTimeMs)return null;
  while(hi-lo>1){const m=(lo+hi)>>1;if(history[m].xrRafTimeMs<=time)lo=m;else hi=m;}
  const a=history[lo],b=history[hi],gap=b.xrRafTimeMs-a.xrRafTimeMs;
  if(gap<=0||gap>100)return null;
  const t=(time-a.xrRafTimeMs)/gap;
  const q=quatSlerp(matToQuat(a.worldFromViewer),matToQuat(b.worldFromViewer),t);
  const p=[12,13,14].map(i=>a.worldFromViewer[i]*(1-t)+b.worldFromViewer[i]*t);
  return composeRigid(p,q);
}

export function anchorFeatureState(session,frame){
  const enabled=session.enabledFeatures?Array.from(session.enabledFeatures).includes('anchors'):null;
  return {enabled,hasMethod:typeof frame?.createAnchor==='function',canAttempt:enabled!==false&&typeof frame?.createAnchor==='function'};
}

export function poseToRigidParts(a){
  if(!a||a.length!==16||!a.every(Number.isFinite))return null;
  const q=matToQuat(a);
  return {position:{x:a[12],y:a[13],z:a[14]},orientation:{x:q[0],y:q[1],z:q[2],w:q[3]}};
}

export const intrinsics=(w,h,fov=65)=>({width:w,height:h,fx:w/2/Math.tan(fov*Math.PI/360),fy:w/2/Math.tan(fov*Math.PI/360),cx:w/2,cy:h/2});

export function observationIsFresh(s,c,now,maxAge=650){
  return s?.context?.sessionId===c?.sessionId&&s?.context?.spaceEpoch===c?.spaceEpoch&&
    Number.isFinite(s?.submittedAtPerformanceMs)&&now>=s.submittedAtPerformanceMs&&now-s.submittedAtPerformanceMs<=maxAge&&s.frameEvidence?.fresh===true;
}
