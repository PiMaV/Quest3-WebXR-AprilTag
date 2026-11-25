import {solveCalibration} from './solver.mjs';
self.onmessage=({data})=>{
  try{const result=solveCalibration(data.payload,message=>postMessage({id:data.id,type:'progress',message}));
    postMessage({id:data.id,type:'result',result});}
  catch(error){postMessage({id:data.id,type:'error',message:error.message});}
};
