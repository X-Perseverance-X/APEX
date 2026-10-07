/* Laptop-only, bounded point cloud for a short stationary IMU-pivot sweep.
 * The Pi sends a scan-level attitude; point projection and PLY export happen
 * in the browser. This does not estimate 3D translation or occupancy.
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

  function anchoredPoint(q0, qScan, pointLidar, imuOffsetFromLidar) {
    const first = unitQuaternion(q0), current = unitQuaternion(qScan);
    if (!first || !current || !pointLidar.every(Number.isFinite) ||
        !imuOffsetFromLidar.every(Number.isFinite)) return null;
    // World axes coincide with the first LiDAR scan. Under the stated fixed
    // IMU-origin assumption, LiDAR moves around the offset IMU location.
    const relative = unitQuaternion(multiply(conjugate(first), current));
    const local = pointLidar.map((value, index) => value - imuOffsetFromLidar[index]);
    return rotate(relative, local).map((value, index) => value + imuOffsetFromLidar[index]);
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
      this.fovSlider = elements.fov;
      this.fovLabel = elements.fovLabel;
      this.halfFovDeg = 100;
      this.maxPoints = 15000;
      this.maxDurationMs = 30000;
      this.voxelM = 0.05;
      this.maxScanMotionDeg = 3.0;
      this.cameraYaw = -Math.PI/4;
      this.cameraElevation = 0.48;
      this.pixelsPerMeter = 42;
      this.latest = null;
      this.active = false;
      this.points = [];
      this.voxels = new Set();
      this.firstQuaternion = null;
      this.lastSequence = null;
      this.acceptedScans = 0;
      this.skippedScans = 0;
      this.startedAtMs = 0;
      this.message = 'Hazır: cihazı bir noktada tutup yavaşça eğin.';

      this.startButton.addEventListener('click', () => this.start());
      this.stopButton.addEventListener('click', () => this.stop('Durduruldu.'));
      this.clearButton.addEventListener('click', () => this.clear());
      this.exportButton.addEventListener('click', () => this.exportPly());
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
        this.pixelsPerMeter = Math.max(8, Math.min(180,
          this.pixelsPerMeter * (event.deltaY > 0 ? 0.9 : 1.1)));
        this.draw();
      }, {passive: false});
      this.refreshText();
    }

    start() {
      const state = this.latest;
      if (!state || !state.geometry || !state.geometry.ready ||
          state.sensors.lidar !== 'STREAMING' ||
          !unitQuaternion(state.lidar.scan_quaternion_wxyz)) {
        this.message = 'LiDAR, IMU zaman eşleşmesi ve montaj ölçüsü hazır değil.';
        this.refreshText();
        return;
      }
      this.clear('Yakalama başladı; IMU noktasını yerinde tutun.');
      this.active = true;
      this.startedAtMs = performance.now();
      this.refreshText();
    }

    stop(message) {
      this.active = false;
      this.message = message;
      this.refreshText();
    }

    clear(message = 'Bulut temizlendi.') {
      this.active = false;
      this.points = [];
      this.voxels.clear();
      this.firstQuaternion = null;
      this.lastSequence = null;
      this.acceptedScans = 0;
      this.skippedScans = 0;
      this.message = message;
      this.refreshText();
      this.draw();
    }

    onDisconnect() {
      if (this.active) this.stop('Bağlantı koptu; yakalama durduruldu.');
    }

    update(data) {
      this.latest = data;
      const lidar = data.lidar || {};
      if (this.active && data.sensors.lidar !== 'STREAMING') {
        this.stop('LiDAR akışı durdu; yakalama sonlandırıldı.');
      }
      if (this.active && performance.now() - this.startedAtMs > this.maxDurationMs) {
        this.stop('30 saniyelik sınır doldu; dosyayı kaydedebilirsiniz.');
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
          [range*Math.cos(angle), range*Math.sin(angle), 0], offset);
        if (!point) continue;
        const voxel = point.map(value => Math.floor(value / this.voxelM)).join(',');
        if (this.voxels.has(voxel)) continue;
        if (this.points.length >= this.maxPoints) {
          this.stop('15.000 nokta sınırı doldu; dosyayı kaydedebilirsiniz.');
          break;
        }
        this.voxels.add(voxel);
        this.points.push({xyz: point, rgb: heightColor(point[2])});
        added++;
      }
      this.acceptedScans++;
      if (this.active) this.message = added ? 'Yakalama sürüyor; sadece öndeki 200° alınıyor.' :
        'Bu taramada yeni 5 cm hücre bulunmadı.';
    }

    refreshText() {
      this.fovLabel.textContent = `ÖN ${2*this.halfFovDeg}° (±${this.halfFovDeg}°)`;
      this.status.textContent = this.message;
      const lidar = this.latest && this.latest.lidar || {};
      const motion = Number.isFinite(lidar.scan_motion_deg) ? lidar.scan_motion_deg.toFixed(1) : '—';
      const skew = Number.isFinite(lidar.imu_scan_skew_ms) ? lidar.imu_scan_skew_ms.toFixed(1) : '—';
      this.details.textContent = `${this.points.length} nokta · ${this.acceptedScans} tarama · ` +
        `${this.skippedScans} atlanan · tarama içi ${motion}° · IMU farkı ${skew} ms`;
      this.startButton.disabled = this.active;
      this.stopButton.disabled = !this.active;
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
        ctx.fillStyle = `rgba(${point.rgb.join(',')},0.78)`;
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
              anchoredPoint(this.firstQuaternion,scanQ,local,offset) : rotate(scanQ,local);
            if (!point) continue;
            const [x,y] = this.project(point,w,h);
            if (x >= 0 && x < w && y >= 0 && y < h) ctx.fillRect(x,y,2,2);
          }
        }
        if (this.firstQuaternion && Array.isArray(offset)) {
          const [x,y] = this.project(offset,w,h);
          ctx.fillStyle = '#ff85d6'; ctx.beginPath(); ctx.arc(x,y,4,0,Math.PI*2); ctx.fill();
          ctx.fillText('IMU ≈ +4 cm X / −4–5 cm Z',x+7,y-6);
        }
      }
      ctx.fillStyle = '#89a7b6'; ctx.font = '11px sans-serif';
      ctx.fillText('Sürükle: döndür · Tekerlek: yakınlaştır · sabit IMU noktası varsayımı',10,h-10);
    }

    exportPly() {
      if (!this.points.length) return;
      const header = [
        'ply', 'format ascii 1.0',
        'comment SLAM Lab V0 experimental stationary IMU-pivot sweep',
        'comment frame initial_lidar_center x_forward y_left z_up metres',
        'comment translation unknown; not a metrically verified 3D SLAM map',
        `comment front_fov_degrees ${2*this.halfFovDeg}`,
        `comment imu_offset_from_lidar_m ${(this.latest.geometry.imu_offset_from_lidar_m || []).join(' ')} approximate; y assumed centerline`,
        `element vertex ${this.points.length}`,
        'property float x', 'property float y', 'property float z',
        'property uchar red', 'property uchar green', 'property uchar blue',
        'end_header',
      ];
      const rows = this.points.map(point =>
        `${point.xyz.map(value => value.toFixed(5)).join(' ')} ${point.rgb.join(' ')}`);
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

  return {unitQuaternion, conjugate, multiply, rotate, frontFov, anchoredPoint, Controller};
});
