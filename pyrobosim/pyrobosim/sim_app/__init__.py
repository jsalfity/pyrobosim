"""PyRoboSim simulation app with GUI and HTTP control API.

This module provides a standalone simulation server with:
- GUI visualization of the PyRoboSim world
- HTTP control API for executing behavior trees
- Client utilities for connecting to the server

Components:
    server: Main simulation app server (run with `python -m pyrobosim.sim_app.server`)
    run_bt_client: Client script for executing single BTs
    utils: HTTP client utilities
"""

from pyrobosim.sim_app.server import (
    ControlHandler,
    SimContext,
    load_world,
    main,
    start_control_server,
)
from pyrobosim.sim_app.utils import call_control

__all__ = [
    "SimContext",
    "ControlHandler",
    "start_control_server",
    "load_world",
    "main",
    "call_control",
]
