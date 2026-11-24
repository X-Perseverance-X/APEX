#!/usr/bin/env python3
from pathlib import Path

p = Path('/home/perseverance/detection_hud.py')
if not p.exists():
    print('[CAMERA PATCH] detection_hud.py bulunamadı; atlanıyor.')
    raise SystemExit(0)

s = p.read_text(encoding='utf-8')
orig = s
s = s.replace(
    'from http.server import BaseHTTPRequestHandler, HTTPServer',
    'from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer',
)
s = s.replace('server = HTTPServer(("0.0.0.0", STREAM_PORT), HUDStreamHandler)',
              'server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), HUDStreamHandler)')
s = s.replace("server = HTTPServer(('0.0.0.0', STREAM_PORT), HUDStreamHandler)",
              "server = ThreadingHTTPServer(('0.0.0.0', STREAM_PORT), HUDStreamHandler)")

if 'ThreadingHTTPServer' in s:
    if s != orig:
        backup = p.with_suffix('.py.bak_before_threaded_http')
        if not backup.exists():
            backup.write_text(orig, encoding='utf-8')
        p.write_text(s, encoding='utf-8')
        print('[CAMERA PATCH] detection_hud.py ThreadingHTTPServer yapıldı.')
    else:
        print('[CAMERA PATCH] detection_hud.py zaten threaded HTTP kullanıyor.')
else:
    print('[CAMERA PATCH] Uygun HTTPServer kalıbı bulunamadı; dosyaya dokunulmadı.')
