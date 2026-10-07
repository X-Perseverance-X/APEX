const assert = require('node:assert/strict');
const {frontFov, anchoredPoint, Controller} = require('../visualization/web/sweep3d.js');

const close = (actual, expected) => assert.ok(Math.abs(actual-expected) < 1e-8,
  `${actual} != ${expected}`);

assert.equal(frontFov(0,100),true);
assert.equal(frontFov(100*Math.PI/180,100),true);
assert.equal(frontFov(260*Math.PI/180,100),true); // right side wraps to -100°
assert.equal(frontFov(Math.PI,100),false);         // operator behind the LiDAR
assert.equal(frontFov(-120*Math.PI/180,100),false);

const offset = [0.04,0,-0.045];
const point = [2,0,0];
const identity = [1,0,0,0];
for (const [actual,expected] of anchoredPoint(identity,identity,point,offset)
  .map((value,index)=>[value,point[index]])) close(actual,expected);

const halfTurnAboutY = [Math.SQRT1_2,0,Math.SQRT1_2,0];
const tilted = anchoredPoint(identity,halfTurnAboutY,point,offset);
close(tilted[0],0.085);
close(tilted[1],0);
close(tilted[2],-2.005);

function element() {
  return {textContent:'',disabled:false,value:'',addEventListener() {},
    getBoundingClientRect() {return {width:0,height:0};}};
}
const ui = {canvas:element(),status:element(),details:element(),start:element(),
  stop:element(),clear:element(),export:element(),fov:element(),fovLabel:element()};
const controller = new Controller(ui);
const data = {sensors:{lidar:'STREAMING'},geometry:{ready:true,imu_offset_from_lidar_m:offset},
  lidar:{sequence:1,scan_quaternion_wxyz:identity,imu_scan_skew_ms:5,scan_motion_deg:0,
    scan:[[0,1],[0.2,1],[Math.PI,1],[Math.PI+0.1,1]]}};
controller.update(data);
controller.start();
controller.update(data);
assert.equal(controller.points.length,1); // front kept, person behind rejected
controller.update(data);
assert.equal(controller.acceptedScans,1); // duplicate WebSocket state not recaptured
controller.update({...data,lidar:{...data.lidar,sequence:2,scan_motion_deg:5}});
assert.equal(controller.skippedScans,1); // fast sweep distorted during 134 ms scan

console.log('3D sweep geometry, front FOV, and capture gating passed');
