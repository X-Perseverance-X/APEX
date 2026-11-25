import asyncio
import time
import unittest
from unittest.mock import AsyncMock, patch

from fastapi.responses import JSONResponse
import apex_server as server


class AiBodyPoseGateTests(unittest.IsolatedAsyncioTestCase):
    async def test_ai_body_pose_requires_fresh_lidar_and_bounded_angles(self):
        safe = {
            'outputsArmed': True, 'spineState': 'STAND', 'mode': 'IDLE',
            'failsafe': False, 'mpuOnline': True, 'batR': 7.4,
            'mpuP': 0.0, 'mpuR': 0.0,
        }
        with patch.object(server, '_ai_tracking_enabled', True), \
             patch.object(server, '_lidar_nav_enabled', False), \
             patch.object(server, '_lidar_enabled', True), \
             patch.object(server, '_lidar_online', True), \
             patch.object(server, '_lidar_last_scan_ts', time.monotonic()), \
             patch.object(server, '_navigation_read_telemetry', AsyncMock(return_value=safe)), \
             patch.object(server, '_proxy_to_esp32', AsyncMock(return_value=JSONResponse({'ok': True}))) as proxy:
            valid = await server._operator_proxy('/bodyIk', {'p': '3', 'r': '0', 'y': '-4', 'source': 'ai'})
            self.assertEqual(valid.status_code, 200)
            proxy.assert_awaited_once_with('/bodyIk', {'p': '3', 'r': '0', 'y': '-4'})
            invalid = await server._operator_proxy('/bodyIk', {'p': '9', 'r': '0', 'y': '0', 'source': 'ai'})
            self.assertEqual(invalid.status_code, 400)
            manual = await server._operator_proxy('/bodyIk', {'p': '3', 'r': '0', 'y': '0'})
            self.assertEqual(manual.status_code, 409)

    async def test_stale_lidar_blocks_body_motion(self):
        with patch.object(server, '_ai_tracking_enabled', True), \
             patch.object(server, '_lidar_nav_enabled', False), \
             patch.object(server, '_lidar_enabled', True), \
             patch.object(server, '_lidar_online', True), \
             patch.object(server, '_lidar_last_scan_ts', time.monotonic() - 3), \
             patch.object(server, '_proxy_to_esp32', AsyncMock()) as proxy:
            response = await server._operator_proxy('/bodyIk', {'p': '3', 'r': '0', 'y': '0', 'source': 'ai'})
            self.assertEqual(response.status_code, 409)
            proxy.assert_not_awaited()
