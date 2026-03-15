#!/usr/bin/env python3
"""
Execute a single BT on a running sim app server.

Usage:
    python3 -m pyrobosim.sim_app.run_bt_client <bt-file.json> [options]

Example:
    python3 -m pyrobosim.sim_app.run_bt_client generated/task_10_5ffbb8.json --realtime-factor 5
"""

import argparse
import json
import sys
import time
from pathlib import Path

from pyrobosim.sim_app.utils import call_control


def main():
    parser = argparse.ArgumentParser(
        description="Execute a behavior tree on a running sim app server with GUI visualization."
    )
    parser.add_argument(
        "bt_file",
        type=Path,
        help="Path to BT JSON file",
    )
    parser.add_argument(
        "--control-url",
        default="http://localhost:8080",
        help="URL of sim app server (default: http://localhost:8080)",
    )
    parser.add_argument(
        "--realtime-factor",
        type=float,
        default=1.0,
        help="Simulation speed (default: 1.0 = realtime, higher = faster)",
    )
    parser.add_argument(
        "--timeout",
        type=int,
        default=60,
        help="Timeout in seconds (default: 60)",
    )
    parser.add_argument(
        "--robot",
        default=None,
        help="Robot name (default: use first robot)",
    )
    args = parser.parse_args()

    # Load BT
    if not args.bt_file.exists():
        print(f"❌ Error: BT file not found: {args.bt_file}")
        sys.exit(1)

    print(f"📄 Loading BT from: {args.bt_file}")
    with open(args.bt_file) as f:
        bt = json.load(f)

    bt_name = bt.get("name", "Unnamed BT")
    print(f"🤖 BT name: {bt_name}")
    print(f"🌐 Control server: {args.control_url}")
    print(f"⚡ Realtime factor: {args.realtime_factor}x")
    print()

    # Execute BT on server
    print("▶️  Starting BT execution on server...")
    print("👀 Watch the GUI window for robot movements")
    print()

    # Start BT execution
    try:
        result = call_control(
            args.control_url,
            "/run_bt",
            {
                "bt_json": bt,
                "robot": args.robot,
                "tick_ms": 100,
                "realtime_factor": args.realtime_factor,
                "print_every": 1,  # Server will generate tree text every tick
            },
        )
    except Exception as e:
        print(f"❌ Error starting BT: {e}")
        print()
        print("💡 Make sure the sim app server is running:")
        print("   python3 -m pyrobosim.sim_app.server --world-file roscon_2024_workshop_world.yaml --port 9001")
        sys.exit(1)

    run_id = result.get("run_id")
    if not run_id:
        print(f"❌ Error: No run_id in response: {result}")
        sys.exit(1)

    # Poll for status and print tree visualization
    print("=" * 60)
    start_time = time.time()
    last_tick = -1
    tick_count = 0
    final_status = None

    try:
        while True:
            status = call_control(args.control_url, "/bt_status", {"run_id": run_id})
            tick_count = int(status.get("tick_count", 0))
            state = status.get("status")
            tree_text = status.get("tree", "")

            # Print tree when tick count changes
            if tree_text and tick_count != last_tick:
                print(tree_text)
                print()
                last_tick = tick_count

            # Check if done
            if state in ("SUCCESS", "FAILURE", "CANCELED"):
                # Print final tree state if available
                if tree_text and tick_count == last_tick:
                    print(tree_text)
                    print()
                final_status = state
                break

            # Check timeout
            if time.time() - start_time > args.timeout:
                final_status = "TIMEOUT"
                break

            time.sleep(0.1)  # Poll every 100ms

    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
        final_status = "CANCELED"
    except Exception as e:
        print(f"\n❌ Error polling status: {e}")
        final_status = "ERROR"

    runtime_ms = int((time.time() - start_time) * 1000)

    # Create result dict
    result = {
        "exec_status": final_status,
        "runtime_ms": runtime_ms,
        "tick_count": tick_count,
    }

    # Print results
    print()
    print("=" * 60)
    status = result['exec_status']
    if status == 'SUCCESS':
        print(f"✅ Status: {status}")
    elif status == 'FAILURE':
        print(f"❌ Status: {status}")
    elif status == 'TIMEOUT':
        print(f"⏱️  Status: {status}")
    else:
        print(f"⚠️  Status: {status}")

    print(f"⏱️  Runtime: {result['runtime_ms']/1000:.2f}s")
    print(f"🔄 Ticks: {result['tick_count']}")

    if result.get('error'):
        print(f"💥 Error: {result['error']}")
    print("=" * 60)

    # Exit with appropriate code
    if result['exec_status'] == 'SUCCESS':
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
