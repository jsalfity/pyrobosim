"""PyRoboSim MCP Server - exposes skills and accepts BT JSONs via MCP protocol."""

from __future__ import annotations

import argparse
from pathlib import Path

from bt_mcp.config import ServerValidationConfig
from bt_mcp.server import BTMCPServer

from .providers import (
    PyRoboSimBTSchemaProvider,
    PyRoboSimRootstocksProvider,
    PyRoboSimSkillProvider,
    PyRoboSimWorldProvider,
)


def create_server(
    generated_dir: Path | None = None,
    submission_file: Path | None = None,
    world_file: str | None = None,
    control_url: str | None = None,
    restrict_control_flow: bool = False,
) -> BTMCPServer:
    """
    Create PyRoboSim MCP server instance.

    Args:
        generated_dir: Directory to save generated BT JSONs
        submission_file: Path to evaluation submission log (JSONL)
        world_file: Deprecated, not used (vocabulary comes from sim_app)
        control_url: URL of sim_app server for live vocabulary (e.g., http://localhost:8080)
        restrict_control_flow: If True, restrict to sequence-only BTs

    Returns:
        Configured BTMCPServer instance
    """
    # Default paths - use current working directory (configurable via CLI)
    if generated_dir is None:
        generated_dir = Path.cwd() / "jsons"
        generated_dir.mkdir(parents=True, exist_ok=True)

    if submission_file is None:
        submission_file = Path.cwd() / "submissions.jsonl"

    # Create PyRoboSim providers
    skill_provider = PyRoboSimSkillProvider()
    bt_schema_provider = PyRoboSimBTSchemaProvider()
    world_provider = PyRoboSimWorldProvider(control_url=control_url)
    rootstocks_provider = PyRoboSimRootstocksProvider()

    # Server configuration
    config = ServerValidationConfig(
        generated_dir=generated_dir,
        submission_file=submission_file,
        send_static_enforce=False,  # Return validation results gracefully (don't throw error)
        expose_validate_tool=True,  # Expose validation tool
        check_vocabulary=bool(control_url),  # Enable vocab check if sim_app URL provided
        expose_rootstocks=True,
        restrict_control_flow=restrict_control_flow,
    )

    # Create MCP server with PyRoboSim providers
    server = BTMCPServer(
        server_name="pyrobosim",
        skill_provider=skill_provider,
        bt_schema_provider=bt_schema_provider,
        world_entities_provider=world_provider,
        rootstocks_provider=rootstocks_provider,
        config=config,
        # Vocabulary validation configuration
        location_entity_keys=["rooms", "locations", "object_spawns"],
        object_entity_keys=["objects", "object_categories"],
        location_param_names=["target_location", "source_location", "room"],
        object_param_names=["object"],
    )

    return server


def main():
    """Run PyRoboSim MCP server."""
    parser = argparse.ArgumentParser(description="PyRoboSim MCP Server")
    parser.add_argument(
        "--generated-dir",
        type=Path,
        help="Directory to save generated BT JSONs",
    )
    parser.add_argument(
        "--submission-file",
        type=Path,
        help="Path to evaluation submission log (JSONL)",
    )
    parser.add_argument(
        "--sim-app-url",
        type=str,
        required=True,
        help="URL of sim_app server for live vocabulary (e.g., http://localhost:8080)",
    )
    parser.add_argument(
        "--restrict-control-flow",
        action="store_true",
        help="Restrict to sequence-only BTs (no selector/parallel/decorator)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port for MCP server (default: 8000)",
    )

    args = parser.parse_args()

    server = create_server(
        generated_dir=args.generated_dir,
        submission_file=args.submission_file,
        world_file=None,
        control_url=args.sim_app_url,
        restrict_control_flow=args.restrict_control_flow,
    )

    print("Starting PyRoboSim MCP server...")
    print(f"  Port: {args.port}")
    print(f"  Generated BTs: {server.config.generated_dir}")
    print(f"  Submission log: {server.config.submission_file}")
    print(f"  Sim app server: {args.sim_app_url} (live vocabulary)")
    print("\nMCP tools available:")
    print("  - list_skills()")
    print("  - get_bt_format()")
    print("  - send_to_robot(bt_json, ...)")
    print("  - list_world_entities()")
    print("  - list_rootstocks_tool()")
    print("\nServer running...")

    # Run the FastMCP server
    server.run(port=args.port)


if __name__ == "__main__":
    main()
