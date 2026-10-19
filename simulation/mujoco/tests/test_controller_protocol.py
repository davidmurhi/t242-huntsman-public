"""The drive protocol shared by the keyboard window, the drive pad page
and the team's Svelte dashboard (controller.py). Uses a real WebSocket
connection on localhost; needs no extra packages."""

import base64
import json
import os
import socket
import struct
import time

import pytest

import controller
import gait as gaitlib


@pytest.fixture()
def commands():
    return controller.Commands(gaitlib.from_spider())


def test_drive_hold_gait_estop(commands):
    commands.apply({"type": "drive", "vx": 0.1, "vy": 0.0, "wz": 20})
    assert commands.vx == pytest.approx(0.1)
    assert commands.gait.cmd[0] == pytest.approx(0.1)
    commands.apply({"type": "gait", "id": "trot"})
    assert commands.gait.gait == "trot"
    commands.apply({"type": "estop", "engaged": True})
    assert commands.gait.estop and commands.vx == 0.0
    commands.apply({"type": "estop", "engaged": False})
    commands.apply({"type": "hold"})
    assert not commands.gait.cmd.any()


def test_dead_man_stops_a_silent_remote(commands, monkeypatch):
    monkeypatch.setattr(controller, "DEADMAN_S", 0.05)
    commands.apply({"type": "drive", "vx": 0.1})
    commands.tick()
    assert commands.vx == pytest.approx(0.1)
    time.sleep(0.08)
    commands.tick()
    assert commands.vx == 0.0 and not commands.gait.cmd.any()


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _masked(payload, op=1):
    mask = os.urandom(4)
    data = bytes(b ^ mask[i % 4] for i, b in enumerate(payload))
    return struct.pack(">BB", 0x80 | op, 0x80 | len(data)) + mask + data


def _read_frame(sock):
    b0, b1 = sock.recv(2)
    n = b1 & 0x7F
    if n == 126:
        n = struct.unpack(">H", sock.recv(2))[0]
    data = b""
    while len(data) < n:
        data += sock.recv(n - len(data))
    return b0 & 0x0F, data


@pytest.fixture()
def server(commands):
    srv = controller.Server(commands, host="127.0.0.1", port=_free_port())
    srv.start()
    assert srv.ready.is_set()
    return srv


def _connect(srv):
    sock = socket.create_connection(("127.0.0.1", srv.port), timeout=3)
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f"GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n"
                  f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                  "Sec-WebSocket-Version: 13\r\n\r\n").encode())
    head = b""
    while b"\r\n\r\n" not in head:
        head += sock.recv(1)
    assert head.startswith(b"HTTP/1.1 101")
    return sock


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_drive_pad_page_served(server):
    sock = socket.create_connection(("127.0.0.1", server.port), timeout=3)
    sock.sendall(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n")
    reply = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            break
        reply += chunk
    assert reply.startswith(b"HTTP/1.1 200")
    assert b"<html" in reply.lower()


def test_websocket_drive_telemetry_and_disconnect(server, commands):
    sock = _connect(server)
    sock.sendall(_masked(json.dumps(
        {"type": "drive", "vx": 0.08, "vy": 0.0, "wz": 0.0}).encode()))
    assert _wait(lambda: abs(commands.vx - 0.08) < 1e-9)

    assert _wait(lambda: len(server.clients) == 1)
    server.broadcast({"type": "telemetry", "rollDeg": 1.5})
    op, data = _read_frame(sock)
    assert op == 1 and json.loads(data)["rollDeg"] == 1.5

    sock.close()                       # a dropped operator must stop it
    assert _wait(lambda: commands.vx == 0.0)


def test_bad_messages_are_ignored(server, commands):
    sock = _connect(server)
    sock.sendall(_masked(b"not json"))
    sock.sendall(_masked(json.dumps({"type": "gait", "id": "rubble"}).encode()))
    assert _wait(lambda: commands.gait.gait == "rubble")
    sock.close()
