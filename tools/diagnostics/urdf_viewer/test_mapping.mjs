import test from 'node:test';
import assert from 'node:assert/strict';
import {rad,deg,sdkTarget,relativeAngle,withinLimits,freshSnapshot,gripperGap} from './static/mapping.js';

const mapping={base_rad:.979,direction:-1,scale:.859866,zero_deg:-.3};
test('rest subtraction preserves reference, sign, and the SDK angular scale',()=>{
  assert.equal(relativeAngle(40,40,1,-1,mapping),1);
  assert.ok(Math.abs(deg(relativeAngle(50,40,0,1,mapping)) + 8.59866)<1e-9);
  assert.ok(Math.abs(deg(relativeAngle(50,40,0,-1,mapping))-8.59866)<1e-9);
  assert.ok(Math.abs(deg(relativeAngle(50,40,0,1,mapping,'one'))-10)<1e-9);
});
test('absolute SDK target and relative URDF angle are distinct quantities',()=>{
  const lerobot={servo:1,direction:-0.859866,range_deg:[-175,-3]};
  assert.ok(Math.abs(deg(sdkTarget(100,lerobot))+85.9866)<1e-9);
  assert.ok(Math.abs(deg(sdkTarget(100-360,lerobot))+85.9866)<1e-9);   // whole turns unwrapped
  assert.equal(deg(sdkTarget(0,lerobot)),-3);                             // clipped to range_deg
  assert.equal(relativeAngle(-.3,-.3,0,1,lerobot),0);
  assert.ok(Math.abs(deg(relativeAngle(10,0,0,1,lerobot))+8.59866)<1e-9);  // scale folded into direction
});
test('out-of-range values remain visible rather than being clamped',()=>{
  const q=relativeAngle(180,0,0,1,mapping,'one');
  assert.equal(q,Math.PI);
  assert.equal(withinLimits(q,{lower:-1,upper:1}),false);
  assert.equal(withinLimits(NaN,{lower:-1,upper:1}),false);
});
test('packet freshness expires even if the browser stops receiving responses',()=>{
  const state={readings:[{servo:0,angle_deg:0,age_ms:40,fresh:true},{servo:1,angle_deg:null,age_ms:null,fresh:false}]};
  assert.equal(freshSnapshot(state,400)[0].fresh,true);
  assert.equal(freshSnapshot(state,500)[0].fresh,false);
  assert.equal(freshSnapshot(state,0)[1].fresh,false);
  assert.equal(state.readings[0].fresh,true);
});

test('gripper endpoint proxy produces symmetric travel and reports saturation',()=>{
  const m={direction:-1,scale:1.983613};
  const cal={closed_encoder_rad:.0067132066834521,commanded_open_encoder_rad:-2.078984206912338};
  const starRange=deg((cal.commanded_open_encoder_rad-cal.closed_encoder_rad)/(m.direction*m.scale));
  assert.equal(gripperGap(-4.7,-4.7,m,cal).applied,0);
  const full=gripperGap(starRange-4.7,-4.7,m,cal);
  assert.ok(Math.abs(full.applied-.104825)<1e-10);
  assert.ok(Math.abs(gripperGap(starRange/2-4.7,-4.7,m,cal).applied-.0524125)<1e-10);
  const beyond=gripperGap(2*starRange-4.7,-4.7,m,cal);
  assert.equal(beyond.applied,.104825);
  assert.ok(beyond.requested>.104825);
  assert.ok(Math.abs(gripperGap(-starRange-4.7,-4.7,m,cal,true).applied-.104825)<1e-10);
});
