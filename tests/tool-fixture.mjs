import {readFileSync} from 'node:fs';
import {synthetic} from './synthetic.mjs';
import {product,inverse,vectorRigid} from '../src/runtime.mjs';
import {associateAnchorCapture} from '../src/anchor-audit.mjs';
export const toolSpec=JSON.parse(readFileSync(new URL('../rigs/ms-qr-independent-tools.json',import.meta.url)));
export const rig=JSON.parse(readFileSync(new URL('../rigs/ms-qr-main-l.json',import.meta.url)));
export function fixture(){
  const d=synthetic({noise:0,count:80}),K=d.truth.K,L=d.truth.L,X=d.truth.X;
  const profile={id:'synthetic-profile',viewerFromCameraGL:X,timeOffsetMs:-55};
  const history=d.history.map(p=>({...p,worldFromAnchor:L.slice(),anchorTrackingState:'tracking'}));
  const transforms=[vectorRigid([.15,-.2,.3,.35,.06,.10]),vectorRigid([-.12,.15,-.2,.34,.25,.05]),vectorRigid([.05,.1,.15,.12,.33,.08])];
  function frame(i,{generation=1,epoch=0,frozen=false,stationary=false,noise=0,shift=0}={}){
    const target=1000+i*340,cap=target+55,now=cap+250;
    const H=history.find(p=>p.xrRafTimeMs===(stationary?1000:target)).worldFromViewer,C=product(H,X);
    const sample={serial:i+1,generation,context:{sessionId:'synthetic',spaceEpoch:epoch},imageSize:[1280,1080],sourceSize:[1280,1080],
      submittedAtPerformanceMs:cap+50,receivedAtPerformanceMs:now,videoFrameMetadata:{captureTime:cap},frameEvidence:{fresh:true},settings:{experimentalCalibration:profile}};
    const association={worldFromAnchor:L,worldFromCamera:C,targetXrTimeMs:target,
      candidate:{...sample.context,calibrationProfileId:profile.id,association:'validated-profile-at-capture-time'}};
    const project=(points,world)=>{
      const m=product(inverse(C),world);
      return points.map(([x,y],j)=>{const p=[0,1,2].map(k=>m[12+k]+m[k]*x+m[4+k]*y);
        return {x:-K.fx*p[0]/p[2]+K.cx+noise*Math.sin(i*13+j*7)+shift,y:K.fy*p[1]/p[2]+K.cy+noise*Math.cos(i*11+j*5)};});
    };
    const unassignedDetections=toolSpec.tools.map((t,j)=>({id:t.tagId,family:toolSpec.family,corners:project(t.tagCorners,product(L,transforms[j])),registered:false,pose:null}));
    return {sample,association,now,K,unassignedDetections,observation:{cameraModel:K,unassignedDetections},project,
      history:history.filter(p=>p.xrRafTimeMs<=now)};
  }
  return {K,L,X,profile,history,transforms,frame};
}
