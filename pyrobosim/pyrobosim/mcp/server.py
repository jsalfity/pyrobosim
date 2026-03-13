"""PyRoboSim MCP Server - exposes skills and accepts BT JSONs via MCP protocol."""

from __future__ import annotations

import argparse
from pathlib import Path

from bt_mcp.config import ServerValidationConfig
from bt_mcp.server import BTMCPServer

from .providers import (
    PyRoboSimBTSchemaProvider,
    PyRoboSimSkillProvider,
    PyRoboSimWorldProvider,
)


def create_server(
    generated_dir: Path | None = None,
    submission_file: Path | None = None,
    world_file: str | None = None,
    restrict_control_flow: bool = False,
) -> BTMCPServer:
    """
    Create PyRoboSim MCP server instance.

    Args:
        generated_dir: Directory to save generated BT JSONs
        submission_file: Path to evaluation submission log (JSONL)
        world_file: Path to world YAML (for vocabulary validation)
        restrict_control_flow: If True, restrict to sequence-only BTs

    Returns:
        Configured BTMCPServer instance
    """
    # Default paths - use current working directory (configurable via CLI)
    if generated_dir is None:
        generated_dir = Path.cwd() / "generated"
        generated_dir.mkdir(parents=True, exist_ok=True)

    if submission_file is None:
        submission_file = Path.cwd() / "submissions.jsonl"

    # Create PyRoboSim providers
    skill_provider = PyRoboSimSkillProvider()
    bt_schema_provider = PyRoboSimBTSchemaProvider()
    world_provider = PyRoboSimWorldProvider(world_file=world_file)

    # Server configuration
    config = ServerValidationConfig(
        generated_dir=generated_dir,
        submission_file=submission_file,
        send_static_enforce=False,  # Return validation results gracefully (don't throw error)
        expose_validate_tool=True,  # Expose validation tool
        check_vocabulary=bool(world_file),  # Enable vocab check if world provided
        restrict_control_flow=restrict_control_flow,
    )

    # Create MCP server with PyRoboSim providers
    server = BTMCPServer(
        server_name="pyrobosim",
        skill_provider=skill_provider,
        bt_schema_provider=bt_schema_provider,
        world_entities_provider=world_provider,
        config=config,
        # Vocabulary validation configuration
        location_entity_keys=["rooms", "locations"],
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
        "--world-file",
        type=str,
        help="Path to world YAML for vocabulary validation",
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

    # Resolve world file path to absolute if provided
    world_file = None
    if args.world_file:
        world_file_path = Path(args.world_file).expanduser()

        # If not absolute, try relative to current dir first, then PyRoboSim data dir
        if not world_file_path.is_absolute():
            if not world_file_path.exists():
                # Try PyRoboSim data directory
                from pyrobosim.utils.general import get_data_folder
                pyrobosim_data_path = get_data_folder() / args.world_file
                if pyrobosim_data_path.exists():
                    world_file_path = pyrobosim_data_path
                else:
                    print(f"ERROR: World file not found: {args.world_file}")
                    print(f"  Tried: {Path(args.world_file).resolve()}")
                    print(f"  Tried: {pyrobosim_data_path}")
                    return

        world_file_path = world_file_path.resolve()
        if not world_file_path.exists():
            print(f"ERROR: World file not found: {world_file_path}")
            return
        world_file = str(world_file_path)

    server = create_server(
        generated_dir=args.generated_dir,
        submission_file=args.submission_file,
        world_file=world_file,
        restrict_control_flow=args.restrict_control_flow,
    )

    print("Starting PyRoboSim MCP server...")
    print(f"  Port: {args.port}")
    print(f"  Generated BTs: {server.config.generated_dir}")
    print(f"  Submission log: {server.config.submission_file}")
    if args.world_file:
        print(f"  World file: {args.world_file}")
    print("\nMCP tools available:")
    print("  - list_skills()")
    print("  - get_bt_format()")
    print("  - send_to_robot(bt_json, ...)")
    print("  - list_world_entities()")
    print("\nServer running...")

    # Run the FastMCP server
    server.run(port=args.port)


if __name__ == "__main__":
    main()
