#!/usr/bin/env python3
"""
controller.py

Drive the Huntsman simulation live: from the keyboard, from the drive
pad page this serves, or from the team's operator dashboard. Telemetry
streams back out on the same connection.

    python controller.py                  3D window + drive pad (every OS)
    python controller.py --headless       no 3D window (server only)

    (Plain python, not mjpython: the window is our own, so it can read
    exactly which keys are held down.)

Then open http://localhost:8765 for the drive pad, or point the
dashboard at ws://localhost:8765/ws (telemetry.svelte.js: connect()).

No extra packages: the WebSocket server is Python's standard library.

PROTOCOL (JSON text messages)
    to the robot
        {"type": "drive", "vx": 0.08, "vy": 0.0, "wz": 0.0}
              vx forward m/s (negative = backwards), vy left m/s,
              wz turn deg/s (positive = left). Send it every 100-200 ms
              while the operator holds a control: if none arrives for
              0.5 s the robot holds (dead-man).
        {"type": "hold"}                         stop and stand
        {"type": "gait", "id": "walk"}           walk | trot | rubble | crouch
        {"type": "estop", "engaged": true}       freeze every servo / release
    from the robot, 30 times a second
        {"rollDeg", "pitchDeg", "yawDeg", "speedMps", "turnDps", "xM", "yM",
         "heightM", "feetDown", "gait", "estop", "policyHz", "command"}
    rollDeg / pitchDeg / yawDeg match the names Tom's dashboard store
    already uses, and the server Prab started (port 8765).

DRIVING, like any game controller
    hold UP / W             walk forward; let go and it stops
    hold DOWN / S           walk backwards; let go and it stops
    hold LEFT/RIGHT, A/D    turn on the spot; let go and it stops
    UP + LEFT / RIGHT       walk forward curving that way (same backwards)
    Q / E                   step sideways
    SPACE hold   X e-stop   1 2 3 4 gait (walk, trot, rubble, crouch)
    R reset      mouse: left-drag rotate, right-drag pan, scroll zoom
    The window reads the keys' real up/down state every frame. The drive
    pad page and a gamepad plugged into the computer drive the same way.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import struct
import sys
import threading
import time

import gait as gaitlib

HERE = os.path.dirname(os.path.abspath(__file__))
HOST, PORT = "0.0.0.0", 8765
CONTROL_HZ = 100
TELEMETRY_HZ = 30
DEADMAN_S = 0.5
GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


# =====================================================================
# COMMANDS - shared between the network, the keyboard and the loop
# =====================================================================

class Commands:
    def __init__(self, gait):
        self.gait = gait
        self.lock = threading.Lock()
        self.last_drive = 0.0
        self.held = False            # a remote control is being held
        self.vx = self.vy = self.wz = 0.0

    def apply(self, msg, remote=True):
        kind = msg.get("type")
        with self.lock:
            if kind == "drive":
                self.vx = float(msg.get("vx", 0.0))
                self.vy = float(msg.get("vy", 0.0))
                self.wz = float(msg.get("wz", 0.0))
                self.gait.drive(self.vx, self.vy, self.wz)
                self.last_drive = time.monotonic()
                self.held = remote
            elif kind == "hold":
                self.vx = self.vy = self.wz = 0.0
                self.gait.hold()
                self.held = False
            elif kind == "gait":
                self.gait.set_gait(str(msg.get("id", "walk")))
            elif kind == "estop":
                self.gait.set_estop(bool(msg.get("engaged", True)))
                if self.gait.estop:
                    self.vx = self.vy = self.wz = 0.0
                    self.held = False

    def tick(self):
        """Dead-man: a remote drive that stops arriving means stop."""
        with self.lock:
            if self.held and time.monotonic() - self.last_drive > DEADMAN_S:
                self.vx = self.vy = self.wz = 0.0
                self.gait.hold()
                self.held = False

    def nudge(self, dvx=0.0, dvy=0.0, dwz=0.0):
        g = gaitlib.GAITS[self.gait.gait]
        with self.lock:
            self.vx = max(-g["speed"], min(g["speed"], self.vx + dvx))
            self.vy = max(-g["speed"], min(g["speed"], self.vy + dvy))
            self.wz = max(-g["turn"], min(g["turn"], self.wz + dwz))
            for name in ("vx", "vy", "wz"):
                if abs(getattr(self, name)) < 1e-9:
                    setattr(self, name, 0.0)
            self.gait.drive(self.vx, self.vy, self.wz)
            self.held = False

    def describe(self):
        g = self.gait
        if g.estop:
            return "E-STOP"
        return (f"{g.gait}  vx {self.vx:+.2f} m/s  vy {self.vy:+.2f} m/s  "
                f"turn {self.wz:+.0f} deg/s")


# =====================================================================
# WEBSOCKET + DRIVE PAD SERVER (standard library only)
# =====================================================================

class Server:
    def __init__(self, commands, host=HOST, port=PORT):
        self.commands = commands
        self.host, self.port = host, port
        self.clients = set()
        self.loop = asyncio.new_event_loop()
        self.ready = threading.Event()
        with open(os.path.join(HERE, "drive_pad.html"), "rb") as fh:
            self.page = fh.read()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()
        self.ready.wait(5)

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(
            asyncio.start_server(self._client, self.host, self.port))
        self.ready.set()
        self.loop.run_forever()

    async def _client(self, reader, writer):
        try:
            head = await reader.readuntil(b"\r\n\r\n")
        except (asyncio.IncompleteReadError, asyncio.LimitOverrunError):
            writer.close()
            return
        lines = head.decode("latin-1").split("\r\n")
        path = lines[0].split(" ")[1] if " " in lines[0] else "/"
        hdr = {}
        for line in lines[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                hdr[k.strip().lower()] = v.strip()
        if hdr.get("upgrade", "").lower() != "websocket":
            body = self.page if path in ("/", "/index.html") else b"not found"
            code = "200 OK" if path in ("/", "/index.html") else "404 Not Found"
            writer.write((f"HTTP/1.1 {code}\r\nContent-Type: text/html; "
                          f"charset=utf-8\r\nContent-Length: {len(body)}\r\n"
                          f"Connection: close\r\n\r\n").encode() + body)
            await writer.drain()
            writer.close()
            return
        accept = base64.b64encode(hashlib.sha1(
            (hdr["sec-websocket-key"] + GUID).encode()).digest()).decode()
        writer.write(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket"
                      "\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: "
                      f"{accept}\r\n\r\n").encode())
        await writer.drain()
        self.clients.add(writer)
        print(f"  operator connected ({len(self.clients)})")
        try:
            while True:
                op, data = await self._read_frame(reader)
                if op == 8:
                    break
                if op == 9:
                    writer.write(self._frame(data, 10))
                    continue
                if op == 1:
                    try:
                        self.commands.apply(json.loads(data.decode()))
                    except (ValueError, TypeError, AttributeError):
                        pass
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            self.clients.discard(writer)
            writer.close()
            # A dropped operator must not leave the robot walking.
            self.commands.apply({"type": "hold"})
            print(f"  operator disconnected ({len(self.clients)})")

    @staticmethod
    async def _read_frame(reader):
        b0, b1 = await reader.readexactly(2)
        n = b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", await reader.readexactly(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", await reader.readexactly(8))[0]
        mask = await reader.readexactly(4) if b1 & 0x80 else b"\0\0\0\0"
        data = bytearray(await reader.readexactly(n))
        for i in range(n):
            data[i] ^= mask[i % 4]
        return b0 & 0x0F, bytes(data)

    @staticmethod
    def _frame(data, op=1):
        n = len(data)
        if n < 126:
            head = struct.pack(">BB", 0x80 | op, n)
        elif n < 65536:
            head = struct.pack(">BBH", 0x80 | op, 126, n)
        else:
            head = struct.pack(">BBQ", 0x80 | op, 127, n)
        return head + data

    def broadcast(self, obj):
        if not self.clients:
            return
        frame = self._frame(json.dumps(obj).encode())

        def send():
            for w in list(self.clients):
                try:
                    w.write(frame)
                except (ConnectionError, RuntimeError):
                    self.clients.discard(w)
        self.loop.call_soon_threadsafe(send)


# =====================================================================
# THE SIMULATION
# =====================================================================

class Robot:
    """The MuJoCo model, stepped in real time."""

    def __init__(self):
        import mujoco
        self.mj = mujoco
        path = os.path.join(HERE, "spider.xml")
        if not os.path.exists(path):
            sys.exit("spider.xml not found. Run first:  python spider.py --save")
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                      "body")
        self.order = [mujoco.mj_name2id(self.model,
                                        mujoco.mjtObj.mjOBJ_ACTUATOR,
                                        f"{leg}_{j}")
                      for leg in gaitlib.LEGS for j in gaitlib.JOINTS]
        if min(self.order) < 0:
            sys.exit("spider.xml is missing servos; rebuild it with spider.py")
        ground = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                   "ground")
        self.ground = ground
        self.tips = {mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM,
                                       f"{leg}_tip") for leg in gaitlib.LEGS}
        self.reset()

    def reset(self):
        self.mj.mj_resetData(self.model, self.data)
        for _ in range(400):
            self.mj.mj_step(self.model, self.data)

    def step(self, targets, seconds):
        for k, a in enumerate(self.order):
            self.data.ctrl[a] = targets[k]
        for _ in range(max(1, round(seconds / self.model.opt.timestep))):
            self.mj.mj_step(self.model, self.data)

    def pose(self):
        d = self.data
        return [float(v) for v in d.qpos[:7]], [float(v) for v in d.qvel[:6]]

    def feet_down(self):
        n = 0
        for c in range(self.data.ncon):
            con = self.data.contact[c]
            pair = {con.geom1, con.geom2}
            if self.ground in pair and pair & self.tips:
                n += 1
        return n


def telemetry(robot, commands, hz):
    (x, y, z, w, qx, qy, qz), vel = robot.pose()
    roll = math.atan2(2 * (w * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * qy - qz * qx))))
    yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    fwd = vel[0] * math.cos(yaw) + vel[1] * math.sin(yaw)
    g = commands.gait
    return {"rollDeg": round(math.degrees(roll), 2),
            "pitchDeg": round(math.degrees(pitch), 2),
            "yawDeg": round(math.degrees(yaw), 1),
            "speedMps": round(fwd, 3),
            "turnDps": round(math.degrees(vel[5]), 1),
            "xM": round(x, 3), "yM": round(y, 3), "heightM": round(z, 3),
            "feetDown": robot.feet_down(), "gait": g.gait,
            "estop": g.estop, "policyHz": round(hz, 1),
            "command": {"vx": commands.vx, "vy": commands.vy,
                        "wz": commands.wz}}


def heading_and_position(robot):
    (x, y, _z, w, qx, qy, qz), _v = robot.pose()
    yaw = math.atan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return yaw, (x, y)


# =====================================================================
# THE 3D WINDOW - our own, so held keys are read exactly
# =====================================================================

def window(robot, gait, commands, loop_once, dt):
    import glfw
    mj = robot.mj
    if not glfw.init():
        sys.exit("could not open a window (glfw)")
    glfw.window_hint(glfw.SAMPLES, 4)
    win = glfw.create_window(1400, 900, "Huntsman - drive with the arrow keys",
                             None, None)
    if not win:
        glfw.terminate()
        sys.exit("could not open a window (glfw)")
    glfw.make_context_current(win)
    glfw.swap_interval(1)

    model, data = robot.model, robot.data
    cam, opt = mj.MjvCamera(), mj.MjvOption()
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)
    cam.distance, cam.elevation, cam.azimuth = 1.1, -22.0, 135.0
    scene = mj.MjvScene(model, maxgeom=6000)
    con = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150)

    mouse = {"left": False, "right": False, "x": 0.0, "y": 0.0}
    gaits = {glfw.KEY_1: "walk", glfw.KEY_2: "trot", glfw.KEY_3: "rubble",
             glfw.KEY_4: "crouch"}

    def on_key(_w, key, _sc, action, _mods):
        if action != glfw.PRESS:
            return
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(win, True)
        elif key in gaits:
            commands.apply({"type": "gait", "id": gaits[key]}, False)
        elif key == glfw.KEY_X:
            commands.apply({"type": "estop", "engaged": not gait.estop},
                           False)
        elif key == glfw.KEY_SPACE:
            commands.apply({"type": "hold"}, False)
        elif key == glfw.KEY_R:
            robot.reset()

    def on_button(_w, button, action, _mods):
        mouse["left"] = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_LEFT) \
            == glfw.PRESS
        mouse["right"] = glfw.get_mouse_button(win, glfw.MOUSE_BUTTON_RIGHT) \
            == glfw.PRESS
        mouse["x"], mouse["y"] = glfw.get_cursor_pos(win)

    def move_camera(action, dx, dy):
        # MuJoCo 3.3+ dropped the scene argument; older versions need it.
        try:
            mj.mjv_moveCamera(model, action, dx, dy, cam)
        except TypeError:
            mj.mjv_moveCamera(model, action, dx, dy, scene, cam)

    def on_move(_w, x, y):
        dx, dy = x - mouse["x"], y - mouse["y"]
        mouse["x"], mouse["y"] = x, y
        if not (mouse["left"] or mouse["right"]):
            return
        _w2, h = glfw.get_window_size(win)
        action = (mj.mjtMouse.mjMOUSE_ROTATE_V if mouse["left"]
                  else mj.mjtMouse.mjMOUSE_MOVE_V)
        move_camera(action, dx / max(1, h), dy / max(1, h))

    def on_scroll(_w, _x, y):
        move_camera(mj.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * y)

    glfw.set_key_callback(win, on_key)
    glfw.set_mouse_button_callback(win, on_button)
    glfw.set_cursor_pos_callback(win, on_move)
    glfw.set_scroll_callback(win, on_scroll)

    def down(*keys):
        return any(glfw.get_key(win, k) == glfw.PRESS for k in keys)

    driving = False
    last_tel, ticks, t_rate = 0.0, 0, time.monotonic()
    sim_clock, wall0 = 0.0, time.monotonic()
    print("  hold the arrow keys (or W A S D) to drive; Q/E sideways, "
          "SPACE hold, X e-stop, 1-4 gait, R reset, ESC quit\n")
    while not glfw.window_should_close(win):
        glfw.poll_events()
        # Held keys -> command, every frame. Nothing held -> stop.
        fwd = down(glfw.KEY_UP, glfw.KEY_W) - down(glfw.KEY_DOWN, glfw.KEY_S)
        turn = down(glfw.KEY_LEFT, glfw.KEY_A) - down(glfw.KEY_RIGHT,
                                                       glfw.KEY_D)
        side = down(glfw.KEY_Q) - down(glfw.KEY_E)
        if fwd or turn or side:
            g = gaitlib.GAITS[gait.gait]
            commands.apply({"type": "drive", "vx": g["speed"] * fwd,
                            "vy": g["speed"] * side, "wz": g["turn"] * turn},
                           remote=False)
            driving = True
        elif driving:
            commands.apply({"type": "hold"}, False)
            driving = False

        # Run the physics in step with the wall clock.
        now = time.monotonic() - wall0
        n = 0
        while sim_clock < now and n < 10:
            last_tel, ticks, t_rate = loop_once(last_tel, ticks, t_rate)
            sim_clock += dt
            n += 1
        if sim_clock < now - 0.2:
            sim_clock = now                    # fell behind: don't spiral

        cam.lookat[:] = [data.xpos[robot.body][0], data.xpos[robot.body][1],
                         0.08]
        w, h = glfw.get_framebuffer_size(win)
        view = mj.MjrRect(0, 0, w, h)
        mj.mjv_updateScene(model, data, opt, None, cam,
                           mj.mjtCatBit.mjCAT_ALL, scene)
        mj.mjr_render(view, scene, con)
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL, mj.mjtGridPos.mjGRID_TOPLEFT,
                       view, "arrows / WASD\nQ E\nSPACE\nX\n1 2 3 4\nR",
                       "drive and turn\nsideways\nhold\ne-stop\n"
                       "walk trot rubble crouch\nreset", con)
        mj.mjr_overlay(mj.mjtFont.mjFONT_NORMAL,
                       mj.mjtGridPos.mjGRID_BOTTOMLEFT, view,
                       "gait\ncommand", f"{gait.gait}"
                       f"{'   E-STOP' if gait.estop else ''}\n"
                       f"{commands.describe()}", con)
        glfw.swap_buffers(win)
    glfw.terminate()
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    robot = Robot()
    gait = gaitlib.from_spider()
    commands = Commands(gait)
    server = Server(commands, port=args.port)
    server.start()
    print(f"\n  drive pad    http://localhost:{args.port}")
    print(f"  dashboard    ws://localhost:{args.port}/ws\n")

    dt = 1.0 / CONTROL_HZ
    state = {"live": True, "hz": float(CONTROL_HZ)}

    def loop_once(last_tel, ticks, t_rate):
        commands.tick()
        heading, pos = heading_and_position(robot)
        targets = gait.step(dt, heading=heading, position=pos)
        robot.step(targets, dt)
        now = time.monotonic()
        if now - last_tel >= 1.0 / TELEMETRY_HZ:
            server.broadcast(telemetry(robot, commands, state["hz"]))
            last_tel = now
        ticks += 1
        if now - t_rate >= 1.0:
            state["hz"] = ticks / (now - t_rate)
            ticks, t_rate = 0, now
        return last_tel, ticks, t_rate

    last_tel, ticks, t_rate = 0.0, 0, time.monotonic()
    next_t = time.monotonic()
    if args.headless:
        print("  running headless; Ctrl+C to stop")
        try:
            while True:
                last_tel, ticks, t_rate = loop_once(last_tel, ticks, t_rate)
                next_t += dt
                time.sleep(max(0.0, next_t - time.monotonic()))
        except KeyboardInterrupt:
            return 0

    return window(robot, gait, commands, loop_once, dt)


if __name__ == "__main__":
    sys.exit(main())
