/* Laptop point cloud from stationary sweeps at manually selected stations.
 * The Pi sends scan attitude and its existing level 2D pose. Translation
 * between stations is approximate and is never inferred from IMU acceleration.
 */
(function (root, makeExports) {
  const api = makeExports();
  if (typeof module === 'object' && module.exports) module.exports = api;
  root.Sweep3D = api;
})(globalThis, function () {
  const DEG = Math.PI / 180;
  const IDENTITY = [1, 0, 0, 0];

  function unitQuaternion(value) {
    if (!Array.isArray(value) || value.length !== 4 || !value.every(Number.isFinite)) return null;
    const norm = Math.hypot(...value);
    return norm > 1e-9 ? value.map(x => x / norm) : null;
  }

  function conjugate(q) { return [q[0], -q[1], -q[2], -q[3]]; }

  function multiply(a, b) {
    const [w, x, y, z] = a, [v, i, j, k] = b;
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
      w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v];
  }

  function rotate(q, p) {
    const [w, x, y, z] = q, [px, py, pz] = p;
    const tx = 2*(y*pz-z*py), ty = 2*(z*px-x*pz), tz = 2*(x*py-y*px);
    return [px+w*tx+y*tz-z*ty, py+w*ty+z*tx-x*tz, pz+w*tz+x*ty-y*tx];
  }

  function frontFov(angleRad, halfFovDeg) {
    if (!Number.isFinite(angleRad) || !Number.isFinite(halfFovDeg)) return false;
    const angle = Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    return Math.abs(angle) <= halfFovDeg * DEG + 1e-10;
  }

  function anchoredPoint(q0, qScan, pointLidar, imuOffsetFromLidar,
                         stationOffset = [0, 0, 0]) {
    const first = unitQuaternion(q0), current = unitQuaternion(qScan);
    if (!first || !current || !pointLidar.every(Number.isFinite) ||
        !imuOffsetFromLidar.every(Number.isFinite) ||
        !stationOffset.every(Number.isFinite)) return null;
    // World axes coincide with the first LiDAR scan. Under the stated fixed
    // IMU-origin assumption, LiDAR moves around the offset IMU location.
    const relative = unitQuaternion(multiply(conjugate(first), current));
    const local = pointLidar.map((value, index) => value - imuOffsetFromLidar[index]);
    return rotate(relative, local).map((value, index) =>
      value + imuOffsetFromLidar[index] + stationOffset[index]);
  }

  function stationTranslation(firstPose, nextPose) {
    if (!firstPose || !nextPose ||
        !['x_m', 'y_m', 'yaw_deg'].every(key => Number.isFinite(firstPose[key])) ||
        !['x_m', 'y_m'].every(key => Number.isFinite(nextPose[key]))) return null;
    const dx = nextPose.x_m - firstPose.x_m, dy = nextPose.y_m - firstPose.y_m;
    const yaw = firstPose.yaw_deg * DEG, c = Math.cos(yaw), s = Math.sin(yaw);
    return [c*dx+s*dy, -s*dx+c*dy, 0];
  }

  function heightColor(z) {
    const t = Math.max(0, Math.min(1, (z + 1.5) / 3));
    return [Math.round(70 + 185*t), Math.round(170 + 45*t), Math.round(240 - 170*t)];
  }

  class Controller {
    constructor(elements) {
      this.canvas = elements.canvas;
      this.status = elements.status;
      this.details = elements.details;
      this.startButton = elements.start;
      this.stopButton = elements.stop;
      this.clearButton = elements.clear;
      this.exportButton = elements.export;
      this.moveButton = elements.move;
      this.resumeButton = elements.resume;
      this.heightInput = elements.height;
      this.fovSlider = elements.fov;
      this.fovLabel = elements.fovLabel;
      this.halfFovDeg = 100;
      this.maxPoints = 60000;
      this.voxelM = 0.02;
      this.maxScanMotionDeg = 3.0;
      this.cameraYaw = -Math.PI/4;
      this.cameraElevation = 0.48;
      this.pixelsPerMeter = 180;
      this.latest = null;
      this.connected = false;
      this.active = false;
      this.relocating = false;
      this.points = [];
      this.voxels = new Set();
      this.firstQuaternion = null;
      this.firstPose = null;
      this.departurePose = null;
      this.stationOffset = [0, 0, 0];
      this.stations = 0;
      this.floorHeightM = null;
      this.floorPoints = 0;
      this.minZ = Infinity;
      this.lastSequence = null;
      this.acceptedScans = 0;
      this.skippedScans = 0;
      this.message = 'Hazır: ilk konumda düz tutup başlatın, sonra yavaşça eğin.';

      this.startButton.addEventListener('click', () => this.start());
      this.stopButton.addEventListener('click', () => this.stop('Durduruldu.'));
      this.clearButton.addEventListener('click', () => this.clear());
      this.exportButton.addEventListener('click', () => this.exportPly());
      this.moveButton.addEventListener('click', () => this.pauseForMove());
      this.resumeButton.addEventListener('click', () => this.resumeAtStation());
      this.heightInput.addEventListener('change', () => {
        const cm = Number(this.heightInput.value);
        this.floorHeightM = this.heightInput.value.trim() !== '' &&
          Number.isFinite(cm) && cm > 0 && cm <= 300 ? cm / 100 : null;
        this.floorPoints = this.points.filter(point => this.isFloor(point.xyz[2])).length;
        this.refreshText(); this.draw();
      });
      this.fovSlider.value = String(this.halfFovDeg);
      this.fovSlider.addEventListener('input', () => {
        const selected = Number(this.fovSlider.value);
        if (selected !== this.halfFovDeg) {
          this.halfFovDeg = selected;
          this.clear('Görüş alanı değişti; önceki noktalar temizlendi.');
        }
      });

      let dragging = false, lastX = 0, lastY = 0;
      this.canvas.addEventListener('pointerdown', event => {
        dragging = true;
        lastX = event.clientX; lastY = event.clientY;
        this.canvas.setPointerCapture(event.pointerId);
      });
      this.canvas.addEventListener('pointermove', event => {
        if (!dragging) return;
        this.cameraYaw += (event.clientX - lastX) * 0.008;
        this.cameraElevation = Math.max(-1.35, Math.min(1.35,
          this.cameraElevation + (event.clientY - lastY) * 0.008));
        lastX = event.clientX; lastY = event.clientY;
        this.draw();
      });
      this.canvas.addEventListener('pointerup', () => { dragging = false; });
      this.canvas.addEventListener('pointercancel', () => { dragging = false; });
      this.canvas.addEventListener('wheel', event => {
        event.preventDefault();
        this.pixelsPerMeter = Math.max(20, Math.min(600,
          this.pixelsPerMeter * (event.deltaY > 0 ? 0.9 : 1.1)));
        this.draw();
      }, {passive: false});
      this.refreshText();
    }

    start() {
      const state = this.latest;
      if (!this.connected || !state || !state.geometry || !state.geometry.ready ||
          state.sensors.lidar !== 'STREAMING' ||
          !unitQuaternion(state.lidar.scan_quaternion_wxyz)) {
        this.message = 'LiDAR, IMU zaman eşleşmesi ve montaj ölçüsü hazır değil.';
        this.refreshText();
        return;
      }
      if (!state.slam || state.slam.mapping_status !== 'ACTIVE' ||
          !stationTranslation(state.slam, state.slam)) {
        this.message = 'İlk konum için cihazı düz tutun ve 2D harita ACTIVE olunca başlayın.';
        this.refreshText();
        return;
      }
      this.clear('Yakalama başladı; IMU noktasını yerinde tutun.');
      this.firstPose = {x_m: state.slam.x_m, y_m: state.slam.y_m,
        yaw_deg: state.slam.yaw_deg};
      this.firstQuaternion = unitQuaternion(state.lidar.scan_quaternion_wxyz);
      this.stations = 1;
      this.active = true;
      this.refreshText();
    }

    pauseForMove() {
      if (!this.active) return;
      const pose = this.latest && this.latest.slam;
      this.departurePose = pose && Number.isFinite(pose.x_m) && Number.isFinite(pose.y_m) ?
        {x_m: pose.x_m, y_m: pose.y_m, yaw_deg: this.firstPose.yaw_deg} : null;
      this.active = false;
      this.relocating = true;
      this.message = 'Cihazı düz tutup yeni konuma taşıyın; 2D harita ACTIVE olunca sürdürün.';
      this.refreshText();
    }

    resumeAtStation() {
      if (!this.relocating || !this.latest) return;
      const state = this.latest;
      if (state.sensors.lidar !== 'STREAMING' ||
          state.slam.mapping_status !== 'ACTIVE' ||
          !unitQuaternion(state.lidar.scan_quaternion_wxyz) ||
          !Number.isFinite(state.slam.score) || state.slam.score < 0.5) {
        this.message = 'Yeni konum eşleşmedi: cihazı düz ve sabit tutup 2D harita ACTIVE/score ≥0.5 bekleyin.';
        this.refreshText();
        return;
      }
      const movement = stationTranslation(this.departurePose, state.slam);
      if (!movement) {
        this.message = '2D konum bilgisi yok; cihazı düz tutup harita güncellemesini bekleyin.';
        this.refreshText();
        return;
      }
      const distance = Math.hypot(movement[0], movement[1]);
      if (distance >= 0.15) {
        this.stationOffset = this.stationOffset.map((value,index) => value + movement[index]);
        this.stations++;
      }
      this.relocating = false;
      this.active = true;
      this.lastSequence = null;
      this.message = distance >= 0.15 ?
        `Durak ${this.stations}: yaklaşık ${distance.toFixed(2)} m taşındı; sabit tutup eğin.` :
        '2D konum farkı 15 cm altında; aynı durakta tarama sürüyor.';
      this.departurePose = null;
      this.refreshText();
    }

    stop(message) {
      this.active = false;
      this.relocating = false;
      this.message = message;
      this.refreshText();
    }

    clear(message = 'Bulut temizlendi.') {
      this.active = false;
      this.relocating = false;
      this.points = [];
      this.voxels.clear();
      this.firstQuaternion = null;
      this.firstPose = null;
      this.departurePose = null;
      this.stationOffset = [0, 0, 0];
      this.stations = 0;
      this.floorPoints = 0;
      this.minZ = Infinity;
      this.lastSequence = null;
      this.acceptedScans = 0;
      this.skippedScans = 0;
      this.message = message;
      this.refreshText();
      this.draw();
    }

    onDisconnect() {
      this.connected = false;
      if (this.active || this.relocating) this.stop('Bağlantı koptu; yakalama durduruldu.');
      else {
        this.message = 'Pi bağlantısı koptu; canlı veri bekleniyor.';
        this.refreshText();
      }
    }

    update(data) {
      const reconnected = !this.connected;
      this.connected = true;
      this.latest = data;
      if (reconnected && !this.active && !this.relocating && !this.points.length)
        this.message = 'Hazır: ilk konumda düz tutup başlatın, sonra yavaşça eğin.';
      const lidar = data.lidar || {};
      if (this.active && data.sensors.lidar !== 'STREAMING') {
        this.stop('LiDAR akışı durdu; yakalama sonlandırıldı.');
      }
      if (this.active && Number.isInteger(lidar.sequence)) {
        if (this.lastSequence !== null && lidar.sequence < this.lastSequence) {
          this.stop('LiDAR akışı yeniden başladı; bulut birleştirilmedi.');
        } else if (lidar.sequence !== this.lastSequence) {
          this.lastSequence = lidar.sequence;
          this.captureScan(data);
        }
      }
      this.refreshText();
      this.draw();
    }

    captureScan(data) {
      const lidar = data.lidar;
      const q = unitQuaternion(lidar.scan_quaternion_wxyz);
      const offset = data.geometry && data.geometry.imu_offset_from_lidar_m;
      if (data.sensors.lidar !== 'STREAMING' || !q ||
          !Array.isArray(offset) || offset.length !== 3 ||
          !offset.every(Number.isFinite) ||
          !Number.isFinite(lidar.imu_scan_skew_ms) ||
          !Number.isFinite(lidar.scan_motion_deg) ||
          lidar.scan_motion_deg > this.maxScanMotionDeg) {
        this.skippedScans++;
        this.message = 'Hızlı dönüş veya IMU zaman eşleşmesi yüzünden tarama atlandı.';
        return;
      }
      if (!this.firstQuaternion) this.firstQuaternion = q;
      let added = 0;
      for (let i = 0; i < lidar.scan.length; i += 2) {
        const [angle, range] = lidar.scan[i];
        if (!frontFov(angle, this.halfFovDeg) || !Number.isFinite(range) || range <= 0) continue;
        const point = anchoredPoint(this.firstQuaternion, q,
          [range*Math.cos(angle), range*Math.sin(angle), 0], offset,
          this.stationOffset);
        if (!point) continue;
        const voxel = point.map(value => Math.floor(value / this.voxelM)).join(',');
        if (this.voxels.has(voxel)) continue;
        if (this.points.length >= this.maxPoints) {
          this.stop('60.000 nokta sınırı doldu; dosyayı kaydedebilirsiniz.');
          break;
        }
        this.voxels.add(voxel);
        this.points.push({xyz: point, rgb: heightColor(point[2])});
        this.minZ = Math.min(this.minZ, point[2]);
        if (this.isFloor(point[2])) this.floorPoints++;
        added++;
      }
      this.acceptedScans++;
      if (this.active) this.message = added ?
        `Durak ${this.stations}: yakalama sürüyor; ön görüş ${2*this.halfFovDeg}°.` :
        'Bu taramada yeni 2 cm hücre bulunmadı.';
    }

    isFloor(z) {
      return this.floorHeightM !== null && Math.abs(z + this.floorHeightM) <= 0.06;
    }

    refreshText() {
      this.fovLabel.textContent = `ÖN ${2*this.halfFovDeg}° (±${this.halfFovDeg}°)`;
      this.status.textContent = this.message;
      const lidar = this.latest && this.latest.lidar || {};
      const motion = Number.isFinite(lidar.scan_motion_deg) ? lidar.scan_motion_deg.toFixed(1) : '—';
      const skew = Number.isFinite(lidar.imu_scan_skew_ms) ? lidar.imu_scan_skew_ms.toFixed(1) : '—';
      this.details.textContent = `${this.points.length} nokta · ${this.acceptedScans} tarama · ` +
        `${this.skippedScans} atlanan · ${this.stations} durak · ` +
        `en alt Z ${Number.isFinite(this.minZ) ? this.minZ.toFixed(2) : '—'} m · ` +
        `zemin adayı ${this.floorPoints} · tarama içi ${motion}° · IMU farkı ${skew} ms`;
      const ready = this.connected && this.latest && this.latest.geometry?.ready &&
        this.latest.sensors?.lidar === 'STREAMING' &&
        unitQuaternion(this.latest.lidar?.scan_quaternion_wxyz) &&
        this.latest.slam?.mapping_status === 'ACTIVE';
      this.startButton.disabled = this.active || this.relocating || !ready;
      this.stopButton.disabled = !(this.active || this.relocating);
      this.moveButton.disabled = !this.active;
      this.resumeButton.disabled = !this.relocating;
      this.exportButton.disabled = this.points.length === 0;
    }

    project(point, width, height) {
      const cy = Math.cos(this.cameraYaw), sy = Math.sin(this.cameraYaw);
      const ce = Math.cos(this.cameraElevation), se = Math.sin(this.cameraElevation);
      const horizontal = cy*point[0] - sy*point[1];
      const depth = sy*point[0] + cy*point[1];
      return [width/2 + horizontal*this.pixelsPerMeter,
        height/2 - (ce*point[2] - se*depth)*this.pixelsPerMeter];
    }

    draw() {
      const rect = this.canvas.getBoundingClientRect();
      if (!rect.width || !rect.height) return;
      const pixelRatio = window.devicePixelRatio || 1;
      const pw = Math.round(rect.width*pixelRatio), ph = Math.round(rect.height*pixelRatio);
      if (this.canvas.width !== pw || this.canvas.height !== ph) {
        this.canvas.width = pw; this.canvas.height = ph;
      }
      const ctx = this.canvas.getContext('2d');
      ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
      const w = rect.width, h = rect.height;
      ctx.fillStyle = '#07131d'; ctx.fillRect(0, 0, w, h);
      if (this.floorHeightM !== null) {
        const z = -this.floorHeightM;
        ctx.strokeStyle = 'rgba(246,195,96,0.24)'; ctx.lineWidth = 1;
        for (let n = -4; n <= 4; n++) {
          for (const segment of [
            [[n*0.5,-2,z],[n*0.5,2,z]],
            [[-2,n*0.5,z],[2,n*0.5,z]],
          ]) {
            const a = this.project(segment[0],w,h), b = this.project(segment[1],w,h);
            ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
          }
        }
        const label = this.project([0,0,z],w,h);
        ctx.fillStyle = '#f6c360'; ctx.font = '12px sans-serif';
        ctx.fillText(`ZEMİN ≈ ${this.floorHeightM.toFixed(2)} m aşağı`,label[0]+8,label[1]-7);
      }
      const origin = this.project([0,0,0],w,h);
      for (const [axis, color, label] of [
        [[1,0,0],'#ef7272','X ÖN'], [[0,1,0],'#7ce79f','Y SOL'],
        [[0,0,1],'#83b8ff','Z YUKARI']]) {
        const end = this.project(axis,w,h);
        ctx.strokeStyle = color; ctx.lineWidth = 2;
        ctx.beginPath(); ctx.moveTo(...origin); ctx.lineTo(...end); ctx.stroke();
        ctx.fillStyle = color; ctx.font = '12px sans-serif';
        ctx.fillText(label,end[0]+5,end[1]-4);
      }
      for (const point of this.points) {
        const [x,y] = this.project(point.xyz,w,h);
        if (x < 0 || x >= w || y < 0 || y >= h) continue;
        ctx.fillStyle = this.isFloor(point.xyz[2]) ? '#f6c360' :
          `rgba(${point.rgb.join(',')},0.78)`;
        ctx.fillRect(x,y,2,2);
      }
      const state = this.latest;
      if (state && state.lidar && Array.isArray(state.lidar.scan)) {
        const scanQ = unitQuaternion(state.lidar.scan_quaternion_wxyz);
        const offset = state.geometry && state.geometry.imu_offset_from_lidar_m;
        if (scanQ) {
          ctx.fillStyle = '#66f5da';
          for (let i = 0; i < state.lidar.scan.length; i += 2) {
            const [angle, range] = state.lidar.scan[i];
            if (!frontFov(angle,this.halfFovDeg) || !Number.isFinite(range)) continue;
            const local = [range*Math.cos(angle),range*Math.sin(angle),0];
            const point = this.firstQuaternion && Array.isArray(offset) ?
              anchoredPoint(this.firstQuaternion,scanQ,local,offset,this.stationOffset) :
              rotate(scanQ,local);
            if (!point) continue;
            const [x,y] = this.project(point,w,h);
            if (x >= 0 && x < w && y >= 0 && y < h) ctx.fillRect(x,y,2,2);
          }
        }
        if (this.firstQuaternion && Array.isArray(offset)) {
          const imu = offset.map((value,index) => value + this.stationOffset[index]);
          const [x,y] = this.project(imu,w,h);
          ctx.fillStyle = '#ff85d6'; ctx.beginPath(); ctx.arc(x,y,4,0,Math.PI*2); ctx.fill();
          ctx.fillText('IMU ≈ +4 cm X / −4–5 cm Z',x+7,y-6);
        }
      }
      ctx.fillStyle = '#89a7b6'; ctx.font = '11px sans-serif';
      ctx.fillText('Sürükle: döndür · Tekerlek: yakınlaştır · her durakta sabit IMU noktası',10,h-10);
    }

    exportPly() {
      if (!this.points.length) return;
      const header = [
        'ply', 'format ascii 1.0',
        'comment SLAM Lab V0 experimental multi-station IMU-pivot sweep',
        'comment frame initial_lidar_center x_forward y_left z_up metres',
        'comment station XY from approximate 2D scan matching; no full 6DoF tracking',
        `comment stations ${this.stations}`,
        `comment voxel_m ${this.voxelM}`,
        `comment assumed_lidar_height_above_floor_m ${this.floorHeightM ?? 'unknown'}`,
        `comment front_fov_degrees ${2*this.halfFovDeg}`,
        `comment imu_offset_from_lidar_m ${(this.latest.geometry.imu_offset_from_lidar_m || []).join(' ')} approximate; y assumed centerline`,
        `element vertex ${this.points.length}`,
        'property float x', 'property float y', 'property float z',
        'property uchar red', 'property uchar green', 'property uchar blue',
        'end_header',
      ];
      const rows = this.points.map(point => {
        const color = this.isFloor(point.xyz[2]) ? [246,195,96] : point.rgb;
        return `${point.xyz.map(value => value.toFixed(5)).join(' ')} ${color.join(' ')}`;
      });
      const blob = new Blob([header.concat(rows).join('\n')+'\n'], {type:'application/octet-stream'});
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `slam-lab-pivot-sweep-${new Date().toISOString().replace(/[:.]/g,'-')}.ply`;
      link.click();
      setTimeout(() => URL.revokeObjectURL(url), 1000);
      this.message = 'Yaklaşık 3D nokta bulutu PLY olarak indirildi.';
      this.refreshText();
    }
  }

  return {unitQuaternion, conjugate, multiply, rotate, frontFov,
    anchoredPoint, stationTranslation, Controller};
});
