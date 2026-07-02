"""PyAvionics server.

FastAPI app that samples the sensor suite at 20 Hz and streams JSON frames
to every connected browser over a WebSocket.

Run:  python server.py [--sim] [--host 0.0.0.0] [--port 8000]
"""
from __future__ import annotations

import argparse
import asyncio
import contextlib
import json
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from avionics.manager import Avionics

ROOT = Path(__file__).resolve().parent
SAMPLE_HZ = 20

log = logging.getLogger("avionics.server")

avionics: Avionics | None = None
clients: set[WebSocket] = set()
_closers: set[asyncio.Task] = set()


async def _send_frame(ws: WebSocket, text: str) -> None:
    # A client that stops reading without closing TCP would otherwise park
    # this send on transport flow control forever and freeze the whole
    # broadcast; evict it instead (the browser auto-reconnects).
    await asyncio.wait_for(ws.send_text(text), timeout=1.0)


async def _close_quietly(ws: WebSocket) -> None:
    with contextlib.suppress(Exception):
        await asyncio.wait_for(ws.close(), timeout=2.0)


def _evict(ws: WebSocket) -> None:
    clients.discard(ws)
    task = asyncio.create_task(_close_quietly(ws))
    _closers.add(task)
    task.add_done_callback(_closers.discard)


async def sampler() -> None:
    period = 1.0 / SAMPLE_HZ
    loop = asyncio.get_running_loop()
    next_t = loop.time()
    while True:
        try:
            frame = avionics.frame()
        except Exception:
            log.exception("frame sampling failed; retrying")
            frame = None
        # One snapshot for both the sends and the result pairing: `clients`
        # can mutate while the gather is awaited, which would misattribute
        # failures and evict the wrong websocket.
        targets = list(clients) if frame is not None else []
        if targets:
            text = None
            try:
                # allow_nan=False: a stray NaN would make every browser's
                # JSON.parse throw, so fail loudly here instead.
                text = json.dumps(frame, allow_nan=False)
            except ValueError:
                log.exception("frame contained non-finite values; dropped")
            if text is not None:
                results = await asyncio.gather(
                    *(_send_frame(ws, text) for ws in targets),
                    return_exceptions=True,
                )
                for ws, res in zip(targets, results):
                    if isinstance(res, Exception):
                        _evict(ws)
        next_t += period
        delay = next_t - loop.time()
        if delay < 0:  # fell behind (heavy load); resync instead of bursting
            next_t = loop.time()
            delay = 0.0
        await asyncio.sleep(delay)


@asynccontextmanager
async def lifespan(app: FastAPI):
    global avionics
    avionics = Avionics(force_sim=os.environ.get("AVIONICS_SIM") == "1")
    await avionics.start()
    task = asyncio.create_task(sampler())
    try:
        yield
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        await avionics.stop()


app = FastAPI(title="PyAvionics", lifespan=lifespan)
app.mount("/static", StaticFiles(directory=ROOT / "static"), name="static")


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(ROOT / "static" / "index.html")


@app.get("/api/status")
async def status() -> dict:
    f = avionics.frame()
    return {"mode": f["status"]["mode"], "sensors": f["status"]["sensors"]}


@app.post("/api/zero")
async def zero() -> dict:
    return avionics.zero()


@app.get("/debug")
async def debug_page() -> FileResponse:
    return FileResponse(ROOT / "static" / "debug.html")


@app.get("/api/debug")
async def debug_data() -> dict:
    return avionics.debug()


@app.post("/api/magcal/reset")
async def magcal_reset() -> dict:
    avionics.mag.reset()
    return {"reset": True}


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    clients.add(ws)
    log.info("client connected (%d total)", len(clients))
    try:
        while True:
            await ws.receive_text()  # keepalive; content ignored
    except WebSocketDisconnect:
        pass
    finally:
        clients.discard(ws)
        log.info("client disconnected (%d total)", len(clients))


def main() -> None:
    parser = argparse.ArgumentParser(description="PyAvionics PFD server")
    parser.add_argument("--sim", action="store_true",
                        help="force full simulation (ignore real sensors)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    if args.sim:
        os.environ["AVIONICS_SIM"] = "1"
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
