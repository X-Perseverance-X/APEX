"""Route/control regressions; mocked transport, no lifespan or hardware startup."""
import asyncio
import json
import time
import unittest
from unittest.mock import AsyncMock, Mock, patch

import httpx
from fastapi.responses import JSONResponse
import apex_server as server
from mapping_v2 import MappingEngine


class MotionRecoveryTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.engine = MappingEngine()
        self.engine.grid.observed[:] = True
        self.engine.grid.log_odds[:] = -2
        self.engine.set_goal(1.0, 0.0)
        self.patch = patch.object(server, '_mapping_engine', self.engine)
        self.patch.start()
        server._navigation_platform_task = None
        server._navigation_platform_status.clear()
        server._navigation_platform_status.update(active=False, state='IDLE')
        server._lidar_nav_enabled = False
        server._ai_tracking_enabled = False
        self.client = httpx.AsyncClient(transport=httpx.ASGITransport(app=server.app), base_url='http://test')

    async def asyncTearDown(self):
        await self.client.aclose()
        self.patch.stop()
        server._navigation_platform_status.update(active=False)
        server._lidar_nav_enabled = False
        server._ai_tracking_enabled = False

    async def test_rejected_approval_stays_retryable_and_reports_reason(self):
        with patch.object(server, '_navigation_read_telemetry', AsyncMock(return_value={'outputsArmed': False})), \
             patch.object(server, '_run_navigation_platform_test', AsyncMock()) as runner:
            response = await self.client.post('/api/navigation/approve')
        self.assertEqual(response.status_code, 409)
        self.assertFalse(self.engine.approved)
        self.assertEqual(server._navigation_platform_status['state'], 'BLOCKED')
        runner.assert_not_called()

    async def test_navigation_safety_preflight_bypasses_hud_telemetry_cache(self):
        payload = {'outputsArmed': False, 'spineState': 'DISARMED'}
        with patch.object(
            server, '_proxy_to_esp32',
            AsyncMock(return_value=JSONResponse(payload)),
        ) as proxy:
            self.assertEqual(await server._navigation_read_telemetry(), payload)
        proxy.assert_awaited_once_with('/telemetry', {}, force_fresh=True)

    async def test_latched_failsafe_is_cleared_only_with_zero_and_idle(self):
        samples = [
            {'failsafe': True, 'spineState': 'FAILSAFE'},
            {'failsafe': False, 'spineState': 'STAND', 'outputsArmed': True},
        ]
        ok = JSONResponse({'ok': True})
        with patch.object(server, '_navigation_read_telemetry', AsyncMock(side_effect=samples)), \
             patch.object(server, '_proxy_to_esp32', AsyncMock(side_effect=[ok, ok])) as proxy:
            telemetry = await server._motion_start_telemetry()
        self.assertEqual(telemetry['spineState'], 'STAND')
        self.assertEqual(proxy.await_args_list[0].args, ('/joy', {'x': '0', 'y': '0', 'r': '0'}))
        self.assertEqual(proxy.await_args_list[1].args, ('/mode', {'m': 'idle'}))

    async def test_ai_stop_uses_complete_idle_sequence(self):
        server._ai_tracking_enabled = True
        with patch.object(server, '_navigation_send_stop', AsyncMock()) as stop:
            response = await self.client.get('/ai_tracking?state=0')
            await asyncio.sleep(0)
        self.assertEqual(response.status_code, 200)
        stop.assert_awaited_once_with()

    async def test_parameterless_lidar_motor_get_is_read_only(self):
        server._lidar_enabled = False
        server._lidar_online = False
        with patch.object(server, '_set_lidar_motor', AsyncMock()) as setter:
            response = await self.client.get('/lidar/motor')
        self.assertEqual(response.status_code, 200)
        self.assertFalse(response.json()['enabled'])
        setter.assert_not_awaited()

    async def test_new_goal_clears_previous_completed_direction(self):
        server._navigation_platform_status.update(state='COMPLETED', joy_x=-.2, joy_y=0)
        response = await self.client.post('/api/navigation/goal', json={'x_m': 1, 'y_m': 1})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()['platform_test']['state'], 'READY')
        self.assertNotIn('joy_x', response.json()['platform_test'])
        self.assertFalse(self.engine.approved)

    async def test_switching_to_auto_starts_the_common_route_executor(self):
        started = {
            'decision': 'AUTO_APPROVED_FOR_EXECUTION',
            'platform_test': {'state': 'STARTING', 'active': True},
        }
        with patch.object(server, '_begin_navigation_execution', AsyncMock(return_value=started)) as begin:
            response = await self.client.post('/api/navigation/mode', json={'mode': 'auto'})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()['platform_test']['state'], 'STARTING')
        begin.assert_awaited_once_with()

    async def test_new_goal_in_auto_mode_starts_the_common_route_executor(self):
        self.engine.set_approval_mode('auto')
        started = {
            'decision': 'AUTO_APPROVED_FOR_EXECUTION',
            'platform_test': {'state': 'STARTING', 'active': True},
        }
        with patch.object(server, '_begin_navigation_execution', AsyncMock(return_value=started)) as begin:
            response = await self.client.post('/api/navigation/goal', json={'x_m': 0.5, 'y_m': 0.5})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()['platform_test']['state'], 'STARTING')
        begin.assert_awaited_once_with()

    async def test_changed_target_during_preflight_does_not_execute_old_route(self):
        async def changed():
            self.engine.set_goal(0, 1)
            return {'gaitParams': {'z': -100, 'len': 100, 'lift': 50, 'spd': 1000}}
        with patch.object(server, '_navigation_read_telemetry', changed), \
             patch.object(server, '_navigation_telemetry_error', return_value=None), \
             patch.object(server, '_set_lidar_motor', AsyncMock(return_value={'applied': True})), \
             patch.object(server, '_run_navigation_platform_test', AsyncMock()) as runner:
            response = await self.client.post('/api/navigation/approve')
        self.assertEqual(response.status_code, 409)
        self.assertIn('değişti', response.json()['error'])
        self.assertFalse(self.engine.approved)
        runner.assert_not_called()

    async def test_route_completion_restores_previous_manual_profile(self):
        profile = {'z': -110, 'len': 90, 'lift': 30, 'spd': 1200}
        snapshots = [
            {'pose': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0, 'trusted': True},
             'local_slam': {'state': 'TRACKING'}, 'sensors': {'lidar': {'fresh': True}},
             'goal': {'x_m': 1.0, 'y_m': 0.0}, 'approval_mode': 'manual'},
            {'pose': {'x_m': 1.0, 'y_m': 0.0, 'yaw_rad': 0.0, 'trusted': True},
             'local_slam': {'state': 'TRACKING'}, 'sensors': {'lidar': {'fresh': True}},
             'goal': {'x_m': 1.0, 'y_m': 0.0}, 'approval_mode': 'manual'},
        ]
        fake_engine = Mock()
        fake_engine.snapshot.side_effect = snapshots
        clear_scan = [(180.0 + index * 0.1, 1000.0) for index in range(40)]
        with patch.object(server, '_mapping_engine', fake_engine), \
             patch.object(server, '_lidar_online', True), \
             patch.object(server, '_lidar_last_scan_ts', time.monotonic()), \
             patch.object(server, '_lidar_points', clear_scan), \
             patch.object(server, '_navigation_direct_request', AsyncMock(return_value=httpx.Response(200))), \
             patch.object(server, '_navigation_send_stop', AsyncMock()) as stop, \
             patch.object(server, '_proxy_to_esp32', AsyncMock(return_value=JSONResponse({'ok': True}))) as proxy:
            await server._run_navigation_platform_test([[0, 0], [1, 0]], profile)
        stop.assert_awaited_once()
        proxy.assert_awaited_once_with('/params', {k: str(v) for k, v in profile.items()})
        self.assertEqual(server._navigation_platform_status['state'], 'COMPLETED')

    async def test_platform_rejected_mode_also_stops_and_restores_profile(self):
        profile = {'z': -100, 'len': 60, 'lift': 25, 'spd': 1000}
        with patch.object(server, '_navigation_direct_request', AsyncMock(side_effect=[httpx.Response(200), httpx.Response(409, text='not armed')])), \
             patch.object(server, '_navigation_send_stop', AsyncMock()) as stop, \
             patch.object(server, '_proxy_to_esp32', AsyncMock(return_value=JSONResponse({'ok': True}))) as proxy:
            await server._run_navigation_platform_test([[0, 0], [1, 0]], profile)
        stop.assert_awaited_once()
        proxy.assert_awaited_once_with('/params', {k: str(v) for k, v in profile.items()})
        self.assertEqual(server._navigation_platform_status['state'], 'ABORTED')

    async def test_route_test_cannot_be_zeroed_by_idle_browser_heartbeat(self):
        server._navigation_platform_status['active'] = True
        with patch.object(server, '_proxy_to_esp32', AsyncMock()) as proxy:
            response = await self.client.get('/joy?x=0&y=0&r=0')
        self.assertEqual(response.status_code, 409)
        proxy.assert_not_called()

    async def test_manual_and_lidar_command_sources_do_not_fight(self):
        server._lidar_nav_enabled = True
        with patch.object(server, '_proxy_to_esp32', AsyncMock(return_value=JSONResponse({'ok': True}))) as proxy:
            blocked = await self.client.get('/joy?x=0&y=0&r=0')
            allowed = await self.client.get('/joy?x=0&y=.2&r=0&source=lidar')
        self.assertEqual(blocked.status_code, 409)
        self.assertEqual(allowed.status_code, 200)
        proxy.assert_awaited_once_with('/joy', {'x': '0', 'y': '.2', 'r': '0'})

    async def test_manual_heartbeat_cannot_overwrite_ai_tracking(self):
        server._ai_tracking_enabled = True
        with patch.object(server, '_proxy_to_esp32', AsyncMock(return_value=JSONResponse({'ok': True}))) as proxy:
            blocked = await self.client.get('/joy?x=0&y=0&r=0')
            allowed = await self.client.get('/joy?x=0&y=-.2&r=0&source=ai')
        self.assertEqual(blocked.status_code, 409)
        self.assertEqual(allowed.status_code, 200)
        proxy.assert_awaited_once_with('/joy', {'x': '0', 'y': '-.2', 'r': '0'})

    async def test_lidar_enable_cannot_claim_success_when_spine_is_disarmed(self):
        with patch.object(server, '_navigation_read_telemetry', AsyncMock(return_value={'outputsArmed': False})), \
             patch.object(server, '_proxy_to_esp32', AsyncMock()) as proxy:
            response = await self.client.get('/lidar_nav?state=1')
        self.assertEqual(response.status_code, 409)
        self.assertFalse(server._lidar_nav_enabled)
        proxy.assert_not_called()

    async def test_busy_esp_does_not_silently_drop_zero_command(self):
        lock = asyncio.Lock()
        await lock.acquire()
        sent = []
        async def get(url, params):
            sent.append(params)
            return httpx.Response(200, text='OK')
        client = AsyncMock()
        client.__aenter__.return_value = client
        client.get.side_effect = get
        async def release():
            await asyncio.sleep(.02)
            lock.release()
        with patch.object(server, 'esp32_lock', lock), patch.object(server.httpx, 'AsyncClient', return_value=client):
            task = asyncio.create_task(release())
            response = await server._proxy_to_esp32('/joy', {'x':'0','y':'0','r':'0'})
            await task
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(sent), 1)


if __name__ == '__main__':
    unittest.main()
