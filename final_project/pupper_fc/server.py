"""Loopback-only viewer/API. This module has no robot or ROS transport."""
import asyncio
from contextlib import asynccontextmanager, suppress
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from .runtime import DT, Sandbox
from .simulation import SCENARIOS


def create_app(sandbox=None):
    sandbox = sandbox or Sandbox()

    @asynccontextmanager
    async def lifespan(app):
        async def pump():
            while True:
                await asyncio.sleep(DT)
                sandbox.advance()
        task = asyncio.create_task(pump())
        try:
            yield
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task

    app = FastAPI(title="Pupper FC offline practice", lifespan=lifespan)
    app.state.sandbox = sandbox

    @app.get("/api/state")
    async def state():
        return sandbox.snapshot()

    @app.get("/api/scenarios")
    async def scenarios():
        return [dict(id=key, **value) for key, value in SCENARIOS.items()]

    @app.post("/api/control")
    async def control(payload: dict):
        try:
            return sandbox.control(payload)
        except (ValueError, TypeError) as exc:
            raise HTTPException(status_code=422, detail=str(exc)) from exc

    @app.get("/api/trace")
    async def trace():
        return JSONResponse(sandbox.trace(), headers={
            "Content-Disposition": 'attachment; filename="pupper-fc-trace.json"'})

    app.mount("/", StaticFiles(directory=Path(__file__).with_name("static"), html=True), name="viewer")
    return app
