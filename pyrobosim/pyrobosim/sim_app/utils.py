"""Utility functions for sim app client."""

from __future__ import annotations

import json
import urllib.request
from typing import Any


def call_control(url: str, endpoint: str, args: dict[str, Any]) -> dict[str, Any]:
    """
    Call a control server endpoint.

    Args:
        url: Base URL of control server (e.g., http://localhost:9001)
        endpoint: Endpoint path (e.g., /run_bt)
        args: Request payload

    Returns:
        Response payload

    Raises:
        Exception: If request fails or returns error
    """
    request_url = url.rstrip("/") + endpoint
    data = json.dumps(args).encode("utf-8")
    req = urllib.request.Request(
        request_url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )

    try:
        with urllib.request.urlopen(req, timeout=10) as response:
            body = response.read().decode("utf-8")
            result = json.loads(body)
            if "error" in result:
                raise Exception(f"Server error: {result['error']}")
            return result
    except urllib.error.URLError as e:
        raise Exception(f"Failed to connect to {request_url}: {e}") from e
