const assert = require('node:assert/strict');
const {frontFov, anchoredPoint, stationTranslation, Controller} =
  require('../visualization/web/sweep3d.js');

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
const translated = anchoredPoint(identity,identity,point,offset,[.4,-.2,0]);
close(translated[0],2.4);
close(translated[1],-.2);
const station = stationTranslation({x_m:1,y_m:2,yaw_deg:90},{x_m:1,y_m:3});
close(station[0],1);
close(station[1],0);

function element() {
  return {textContent:'',disabled:false,value:'',addEventListener() {},
    getBoundingClientRect() {return {width:0,height:0};}};
}
const ui = {canvas:element(),status:element(),details:element(),start:element(),
  stop:element(),move:element(),resume:element(),height:element(),
  clear:element(),export:element(),fov:element(),fovLabel:element()};
const controller = new Controller(ui);
const data = {sensors:{lidar:'STREAMING'},geometry:{ready:true,imu_offset_from_lidar_m:offset},
  slam:{x_m:0,y_m:0,yaw_deg:0,score:.9,mapping_status:'ACTIVE'},
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
controller.pauseForMove();
assert.equal(controller.relocating,true);
controller.update({...data,slam:{...data.slam,x_m:.4},lidar:{...data.lidar,sequence:3}});
assert.equal(controller.acceptedScans,1); // no points while carrying between stations
controller.resumeAtStation();
assert.equal(controller.stations,2);
close(controller.stationOffset[0],.4);
controller.update({...data,lidar:{...data.lidar,sequence:4}});
assert.equal(controller.acceptedScans,2);
assert.equal(controller.points.some(p=>p.xyz[0]>1.3),true);
controller.pauseForMove();
controller.update({...data,slam:{...data.slam,x_m:.05},
  lidar:{...data.lidar,sequence:5}});
controller.resumeAtStation();
assert.equal(controller.stations,2); // stationary pose jitter does not create a new station
close(controller.stationOffset[0],.4);
controller.onDisconnect();
assert.equal(ui.start.disabled,true); // a stale browser state cannot start capture
controller.update(data);
assert.equal(ui.start.disabled,false);

console.log('3D station transform, front FOV, and capture gating passed');
