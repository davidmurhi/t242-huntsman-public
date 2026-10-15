import asyncio
import json
import math
import websockets
from spider_env import SpiderEnv

HOST = "localhost"
PORT = 8765

# Part A: every dashboard that connects gets added here, removed when it disconnects
connected_clients = set()

async def handle_client(websocket):
    connected_clients.add(websocket)
    print(f"Dashboard connected. Total: {len(connected_clients)}")
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        print(f"Dashboard disconnected. Total: {len(connected_clients)}")

async def broadcast(message):
    if connected_clients:
        await asyncio.gather(
            *(ws.send(message) for ws in connected_clients),
            return_exceptions=True,
        )

# Part B: step the sim, pull real values out, send only what the sim can honestly provide
async def run_sim():
    env = SpiderEnv(render=False)
    obs, info = env.reset()

    while True:
        action = env.action_space.sample()  # placeholder - swap for a trained policy later
        obs, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            obs, info = env.reset()

        roll, pitch, yaw = obs[24], obs[25], obs[26]  # radians, from SpiderEnv's observation layout

        telemetry = {
            "rollDeg": math.degrees(roll),
            "pitchDeg": math.degrees(pitch),
            "yawDeg": math.degrees(yaw),
        }

        await broadcast(json.dumps(telemetry))
        await asyncio.sleep(1 / 30)  # ~30 updates per second

# Part C: run the WebSocket server and the sim loop together
async def main():
    async with websockets.serve(handle_client, HOST, PORT):
        print(f"Telemetry bridge running on ws://{HOST}:{PORT}")
        await run_sim()

if __name__ == "__main__":
    asyncio.run(main())