# Copyright (c) 2026 Alice Zenina and Alexander Grachev RTU MIREA (Russia)
# SPDX-License-Identifier: MIT
# Details in the LICENSE file in the root of the package.

"""
HTTP-сервер веб-настройщика на стандартной библиотеке Python.

    GET  /                          страница
    GET  /static/<файл>             app.js, style.css
    GET  /events                    телеметрия, Server-Sent Events
    GET  /api/step_test/recording   записанный отклик ступенчатого теста (JSON)
    GET  /api/step_test/result.csv  то же в CSV
    POST /api/<команда>             команды (JSON), см. Backend.handle_api

Сервер ничего не знает о ROS: всё, что нужно, он берёт у backend
(нода WebConfiguratorNode).
"""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import threading

MAX_BODY = 4096
CONTENT_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'text/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon',
}


class _Handler(BaseHTTPRequestHandler):
    server_version = 'jetbot-web-configurator/1.0'
    protocol_version = 'HTTP/1.1'

    # ---------------------------------------------------------------- utils
    @property
    def backend(self):
        return self.server.backend

    def log_message(self, fmt, *args):  # noqa: D102 - тихий лог в ROS-логгер
        self.backend.log_debug('%s - %s' % (self.address_string(), fmt % args))

    def _send(self, code, body, content_type, extra_headers=None):
        data = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (extra_headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj, ensure_ascii=False), 'application/json; charset=utf-8')

    # ---------------------------------------------------------------- GET
    def do_GET(self):  # noqa: N802
        path = self.path.split('?', 1)[0]
        if path in ('/', '/index.html'):
            return self._static('index.html')
        if path.startswith('/static/'):
            return self._static(path[len('/static/'):])
        if path == '/events':
            return self._events()
        if path == '/api/step_test/recording':
            return self._json(HTTPStatus.OK, self.backend.step_recording())
        if path == '/api/step_test/result.csv':
            csv_text = self.backend.step_csv()
            if csv_text is None:
                return self._json(HTTPStatus.NOT_FOUND, {'error': 'нет завершённого теста'})
            return self._send(HTTPStatus.OK, csv_text, 'text/csv; charset=utf-8',
                              {'Content-Disposition': 'attachment; filename="step_test.csv"'})
        self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})

    def _static(self, name):
        # только файлы прямо в каталоге static, без подкаталогов и '..'
        if '/' in name or '\\' in name or name.startswith('.'):
            return self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
        path = os.path.join(self.server.static_dir, name)
        ext = os.path.splitext(name)[1]
        if ext not in CONTENT_TYPES or not os.path.isfile(path):
            return self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
        with open(path, 'rb') as f:
            self._send(HTTPStatus.OK, f.read(), CONTENT_TYPES[ext])

    def _events(self):
        """Поток SSE: сначала config, затем телеметрия с частотой telemetry_rate."""
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'keep-alive')
        self.end_headers()
        self.close_connection = True
        cursor = self.backend.telemetry_cursor_start()
        self.server.client_connected(+1)
        try:
            self._write_event('config', self.backend.config())
            period = 1.0 / self.backend.telemetry_rate
            while not self.server.stopping.wait(period):
                cursor, payload = self.backend.telemetry_payload(cursor)
                self._write_event('telemetry', payload)
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass   # страница закрыта или связь пропала
        finally:
            self.server.client_connected(-1)

    def _write_event(self, name, obj):
        data = json.dumps(obj, ensure_ascii=False, separators=(',', ':'))
        self.wfile.write(f'event: {name}\ndata: {data}\n\n'.encode('utf-8'))
        self.wfile.flush()

    # ---------------------------------------------------------------- POST
    def do_POST(self):  # noqa: N802
        path = self.path.split('?', 1)[0]
        if not path.startswith('/api/'):
            return self._json(HTTPStatus.NOT_FOUND, {'error': 'not found'})
        try:
            length = int(self.headers.get('Content-Length', '0'))
        except ValueError:
            length = -1
        if length < 0 or length > MAX_BODY:
            return self._json(HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                              {'error': 'слишком большой запрос'})
        raw = self.rfile.read(length) if length else b''
        try:
            body = json.loads(raw.decode('utf-8')) if raw else {}
        except (UnicodeDecodeError, json.JSONDecodeError):
            return self._json(HTTPStatus.BAD_REQUEST, {'error': 'тело запроса - не JSON'})
        if not isinstance(body, dict):
            return self._json(HTTPStatus.BAD_REQUEST, {'error': 'ожидался JSON-объект'})
        code, result = self.backend.handle_api(path[len('/api/'):], body)
        self._json(code, result)


class WebServer:
    """HTTP-сервер в отдельном потоке (по потоку на соединение)."""

    def __init__(self, backend, host, port, static_dir):
        self._httpd = ThreadingHTTPServer((host, port), _Handler)
        self._httpd.daemon_threads = True
        self._httpd.backend = backend
        self._httpd.static_dir = static_dir
        self._httpd.stopping = threading.Event()
        self._clients = 0
        self._clients_lock = threading.Lock()
        self._httpd.client_connected = self._client_connected
        self._thread = threading.Thread(target=self._httpd.serve_forever,
                                        kwargs={'poll_interval': 0.5},
                                        name='web_configurator_http', daemon=True)

    @property
    def address(self):
        return self._httpd.server_address

    @property
    def clients(self):
        with self._clients_lock:
            return self._clients

    def _client_connected(self, delta):
        with self._clients_lock:
            self._clients += delta

    def start(self):
        self._thread.start()

    def stop(self):
        self._httpd.stopping.set()
        self._httpd.shutdown()
        self._httpd.server_close()
        self._thread.join(timeout=2.0)
