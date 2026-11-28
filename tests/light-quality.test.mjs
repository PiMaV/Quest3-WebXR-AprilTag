import test from 'node:test';
import assert from 'node:assert/strict';
import {analyzeLight} from '../src/light-quality.mjs';

const W=160,H=120;
const frame=fn=>{const g=new Uint8Array(W*H);for(let y=0;y<H;y++)for(let x=0;x<W;x++)g[y*W+x]=fn(x,y);return g;};
const tag={corners:[{x:60,y:40},{x:100,y:40},{x:100,y:80},{x:60,y:80}]};
const inTag=(x,y)=>x>=60&&x<100&&y>=40&&y<80;

test('well lit tag scores good',()=>{
  const g=frame((x,y)=>inTag(x,y)?((x+y)%8<4?20:220):120);
  const r=analyzeLight(g,W,H,[tag]);
  assert.equal(r.level,'good');assert.equal(r.flags.length,0);
});
test('backlight: bright blown area, dark tag -> flagged and not good',()=>{
  const g=frame((x,y)=>inTag(x,y)?((x+y)%8<4?25:50):(x<70?255:60));
  const r=analyzeLight(g,W,H,[tag]);
  assert.ok(r.flags.includes('backlight'));assert.ok(r.flags.includes('tag-low-contrast'));
  assert.notEqual(r.level,'good');
});
test('dark frame is bad',()=>{
  const r=analyzeLight(frame(()=>10),W,H,[]);
  assert.equal(r.level,'bad');assert.ok(r.flags.includes('dark'));
});
test('auto-exposure hunting detected from mean drift',()=>{
  const r=analyzeLight(frame(()=>150),W,H,[],{meanLuma:100});
  assert.ok(r.flags.includes('ae-hunting'));
});
