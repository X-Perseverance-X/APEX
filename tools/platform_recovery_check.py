"""Explicit, bounded bench checks. Never run at import/startup."""
import argparse
import json
import time
import httpx


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=['status', 'stand', 'manual'])
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    args = parser.parse_args()
    with httpx.Client(base_url=args.url, timeout=1.0, headers={'Connection': 'close'}) as client:
        def request(path, **params):
            response = client.get(path, params=params)
            response.raise_for_status()
            return response

        def telemetry():
            data = request('/telemetry').json()
            if data.get('proxyStale'):
                raise RuntimeError('Telemetry is stale')
            return data

        def report(data):
            fields = ('firmwareVersion', 'spineState', 'mode', 'outputsArmed',
                      'pcaRightOnline', 'pcaLeftOnline', 'mpuOnline',
                      'mpuP', 'mpuR', 'batR', 'batL', 'failsafe',
                      'failsafeCount', 'gaitPhase', 'gaitParams', 'vx', 'vy')
            print(json.dumps({k: data.get(k) for k in fields}), flush=True)

        data = telemetry()
        report(data)
        if args.action == 'status':
            return
        if not all(data.get(k) for k in ('pcaRightOnline', 'pcaLeftOnline', 'configValid', 'homeSet', 'mpuOnline')):
            raise RuntimeError('Hardware/config prerequisites missing')
        if min(data['batR'], data['batL']) < 10.2:
            raise RuntimeError('Battery below test threshold')
        if max(abs(data['mpuP']), abs(data['mpuR'])) > 45:
            raise RuntimeError('IMU tilt above test threshold')
        if data.get('firmwareVersion') not in ('v57.3-smooth-rise', 'v57.4-motion-cadence'):
            raise RuntimeError('Updated firmware required')

        if args.action == 'stand':
            if data['spineState'] != 'DISARMED':
                raise RuntimeError('Stand check requires DISARMED')
            request('/goHome', confirm='start')
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                time.sleep(.6)
                data = telemetry()
                report(data)
                if data['spineState'] == 'STAND':
                    return
                if data['spineState'] not in ('CAPTURE', 'RISING'):
                    raise RuntimeError('Stand sequence interrupted')
            # No automatic IDLE command: that would select a different IK target.
            raise RuntimeError('Stand timeout; operator inspection required')

        if data['spineState'] != 'STAND' or not data['outputsArmed']:
            raise RuntimeError('Manual check requires STAND')
        saved = data['gaitParams']
        initial_phase = data['gaitPhase']
        try:
            request('/params', z=saved['z'], len=40, lift=10, spd=1000)
            request('/mode', m='bezier_joy')
            until = time.monotonic() + 1.5
            while time.monotonic() < until:
                request('/joy', x=0, y=.12, r=0)
                time.sleep(.06)
        finally:
            try:
                request('/joy', x=0, y=0, r=0)
            finally:
                try:
                    request('/mode', m='idle')
                finally:
                    request('/params', **saved)
        time.sleep(.3)
        data = telemetry()
        report(data)
        assert data['spineState'] == 'STAND' and not data['failsafe']
        assert data['gaitParams'] == saved
        assert data['gaitPhase'] != initial_phase
        print('Manual command accepted, gait advanced, STAND and profile restored.', flush=True)


if __name__ == '__main__':
    main()
