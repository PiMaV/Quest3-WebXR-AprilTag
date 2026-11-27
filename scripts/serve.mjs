#!/usr/bin/env node
// Standalone zero-dependency static server for Quest 3 WebXR testing.
// Usage:
//   npm run serve:lan                                 (http://0.0.0.0:8765 for LAN / Quest 3 access)
//   npm run serve                                     (http://127.0.0.1:8443 for localhost / adb reverse)
//   node scripts/serve.mjs --base /Quest3-WebXR-AprilTag
//   node scripts/serve.mjs --cert c.pem --key k.pem --host 0.0.0.0 --port 8765
import http from 'node:http';
import https from 'node:https';
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';

const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const MIME={
  '.html':'text/html; charset=utf-8',
  '.mjs':'text/javascript; charset=utf-8',
  '.js':'text/javascript; charset=utf-8',
  '.json':'application/json; charset=utf-8',
  '.wasm':'application/wasm',
  '.pdf':'application/pdf',
  '.css':'text/css; charset=utf-8',
  '.svg':'image/svg+xml',
  '.png':'image/png',
  '.webp':'image/webp',
  '.jpg':'image/jpeg',
  '.ico':'image/x-icon',
};

function normalizeBase(raw){
  if(!raw||raw==='/')return '';
  const trimmed=String(raw).replace(/\/+$/,'');
  if(!trimmed)return '';
  return trimmed.startsWith('/')?trimmed:'/'+trimmed;
}

function parseArgs(argv){
  const out={host:'127.0.0.1',port:8443,cert:null,key:null,base:''};
  for(let i=0;i<argv.length;i++){
    const a=argv[i];
    if(a==='--host'&&argv[i+1])out.host=argv[++i];
    else if(a==='--port'&&argv[i+1])out.port=Number(argv[++i]);
    else if(a==='--cert'&&argv[i+1])out.cert=argv[++i];
    else if(a==='--key'&&argv[i+1])out.key=argv[++i];
    else if(a==='--base'&&argv[i+1])out.base=normalizeBase(argv[++i]);
  }
  return out;
}

export function createStaticHandler(rootDir=root,{base=''}={}){
  const prefix=normalizeBase(base);
  return (req,res)=>{
    if(!['GET','HEAD'].includes(req.method)){res.writeHead(405);res.end('Method Not Allowed');return;}
    let pathname='/';
    try{pathname=decodeURIComponent(new URL(req.url,'http://localhost').pathname);}catch{res.writeHead(400);res.end('Bad Request');return;}
    if(prefix){
      if(pathname===prefix){
        res.writeHead(302,{Location:prefix+'/','Cache-Control':'no-store'});
        res.end();
        return;
      }
      if(!pathname.startsWith(prefix+'/')){res.writeHead(404);res.end('Not Found');return;}
      pathname=pathname.slice(prefix.length)||'/';
    }
    if(pathname==='/')pathname='/index.html';
    const resolved=path.resolve(rootDir,'.'+pathname);
    if(resolved!==rootDir&&!resolved.startsWith(rootDir+path.sep)){res.writeHead(403);res.end('Forbidden');return;}
    fs.stat(resolved,(err,stat)=>{
      if(err||!stat.isFile()){res.writeHead(404);res.end('Not Found');return;}
      const ext=path.extname(resolved).toLowerCase();
      res.writeHead(200,{
        'Content-Type':MIME[ext]||'application/octet-stream',
        'Content-Length':stat.size,
        'Cache-Control':'no-store',
      });
      if(req.method==='HEAD'){res.end();return;}
      fs.createReadStream(resolved).pipe(res);
    });
  };
}

if(process.argv[1]&&path.resolve(process.argv[1])===fileURLToPath(import.meta.url)){
  const opts=parseArgs(process.argv.slice(2));
  const handler=createStaticHandler(root,{base:opts.base});
  const useTls=Boolean(opts.cert&&opts.key);
  const server=useTls
    ?https.createServer({cert:fs.readFileSync(opts.cert),key:fs.readFileSync(opts.key)},handler)
    :http.createServer(handler);
  server.listen(opts.port,opts.host,()=>{
    const proto=useTls?'https':'http';
    const basePath=opts.base?`${opts.base}/`:'/';
    console.log(`Quest3-WebXR-AprilTag server running at ${proto}://${opts.host}:${opts.port}${basePath}`);
    console.log(`Quest demo:      ${proto}://${opts.host}:${opts.port}${opts.base}/examples/quest.html`);
    console.log(`Minimal starter: ${proto}://${opts.host}:${opts.port}${opts.base}/examples/minimal.html`);
    if(!useTls){
      if(opts.host==='127.0.0.1'||opts.host==='localhost'){
        console.log(`For Meta Quest 3 over USB: adb reverse tcp:${opts.port} tcp:${opts.port}`);
        console.log(`Then open in Quest Browser: http://localhost:${opts.port}${opts.base}/examples/quest.html`);
      }else{
        console.log(`LAN access: http://<LAN-IP>:${opts.port}${opts.base}/ (pass --cert <pem> --key <pem> for HTTPS)`);
      }
    }
  });
}
