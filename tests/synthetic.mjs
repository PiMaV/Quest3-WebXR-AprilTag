import {Matrix4,Vector3,Quaternion,Euler} from '../vendor/three/three.module.js';
import {product,inverse,vectorRigid} from '../src/runtime.mjs';
export const rig={schema:'webxr-apriltag-rig-v1',id:'synthetic-l',units:'m',family:'apriltag-36h11',tools:[{id:'origin-l',tags:[
  {id:350,corners:[[.0075,.005],[.0275,.005],[.0275,.025],[.0075,.025]]},
  {id:351,corners:[[.2375,.005],[.2575,.005],[.2575,.025],[.2375,.025]]},
  {id:352,corners:[[.0075,.175],[.0275,.175],[.0275,.195],[.0075,.195]]}]}]};
export function synthetic({pinhole=false,k1=0,noise=.05,count=72}={}){
  const K={width:1280,height:1080,fx:890,fy:pinhole?920:890,cx:pinhole?665:640,cy:pinhole?513:540,k1};
  const X=vectorRigid([.012,.018,-.01,.035,-.014,-.042]),L=vectorRigid([0,0,.2,-.12,-.1,-.9]),delay=-55;
  const head=t=>{
    const s=t/1000,q=new Quaternion().setFromEuler(new Euler(.28*Math.sin(s*1.7),.38*Math.cos(s*.9),.12*Math.sin(s*1.1)));
    return new Matrix4().compose(new Vector3(.12*Math.sin(s*.8),.1*Math.cos(s*.65),.08*Math.sin(s*1.3)),q,new Vector3(1,1,1)).toArray();
  };
  const history=[];for(let t=0;t<=12000;t+=10)history.push({xrRafTimeMs:t,worldFromViewer:head(t),scope:'s',sessionId:'synthetic',spaceEpoch:0});
  const rows=[];
  for(let i=0;i<count;i++){
    const captureTime=500+i*140,C=product(inverse(X),inverse(head(captureTime+delay)),L);
    const tags=rig.tools[0].tags.map(tag=>({id:tag.id,points:tag.corners,pixels:tag.corners.map(([x,y],j)=>{
      const p=[0,1,2].map(k=>C[12+k]+C[k]*x+C[4+k]*y),u=-p[0]/p[2],v=p[1]/p[2];
      const sc=k1?1+k1*(u*u+v*v):1;
      return {x:K.fx*u*sc+K.cx+noise*Math.sin(i*15+j*7+tag.id),y:K.fy*v*sc+K.cy+noise*Math.cos(i*9+j*5+tag.id)};
    })}));
    rows.push({scope:'s',captureTime,tags,serial:i+1});
  }
  return {rows,history,imageSize:[1280,1080],truth:{K,X,L,delay}};
}
