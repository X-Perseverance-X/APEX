import hashlib
import importlib
import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import AsyncMock


class AuthContractTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.previous = {key: os.environ.get(key) for key in (
            'APEX_AUTH_ENABLED', 'APEX_AUTH_USER', 'APEX_AUTH_PASSWORD_HASH', 'APEX_AUTH_SECRET')}
        salt = bytes.fromhex('00112233445566778899aabbccddeeff')
        digest = hashlib.pbkdf2_hmac('sha256', b'correct', salt, 1000).hex()
        os.environ.update({
            'APEX_AUTH_ENABLED': '1', 'APEX_AUTH_USER': 'apex',
            'APEX_AUTH_PASSWORD_HASH': f'1000${salt.hex()}${digest}',
            'APEX_AUTH_SECRET': 'test-secret-not-for-production',
        })
        sys.modules.pop('apex_server', None)
        self.server = importlib.import_module('apex_server')

    def tearDown(self):
        for key, value in self.previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        sys.modules.pop('apex_server', None)

    def test_password_and_signed_session(self):
        self.assertTrue(self.server._password_matches('correct'))
        self.assertFalse(self.server._password_matches('wrong'))
        cookie = self.server._session_value('apex', int(self.server.time.time()) + 60)
        self.assertTrue(self.server._session_valid(cookie))
        self.assertFalse(self.server._session_valid(cookie + 'tampered'))

    def test_expired_or_wrong_user_session_is_rejected(self):
        expired = self.server._session_value('apex', int(self.server.time.time()) - 1)
        wrong_user = self.server._session_value('other', int(self.server.time.time()) + 60)
        self.assertFalse(self.server._session_valid(expired))
        self.assertFalse(self.server._session_valid(wrong_user))

    async def test_local_helpers_bypass_browser_login(self):
        request = SimpleNamespace(
            url=SimpleNamespace(path='/telemetry'),
            client=SimpleNamespace(host='127.0.0.1'),
            cookies={},
        )
        call_next = AsyncMock(return_value='ok')
        self.assertEqual(await self.server.require_login(request, call_next), 'ok')
        call_next.assert_awaited_once_with(request)
