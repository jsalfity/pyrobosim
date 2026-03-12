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
    # Default paths
    if generated_dir is None:
        generated_dir = Path(__file__).resolve().parent.parent / "behaviors" / "generated"
    if submission_file is None:
        repo_root = Path(__file__).resolve().parents[3]
        submission_file = repo_root / "eval" / "submissions.jsonl"

    # Create PyRoboSim providers
    skill_provider = PyRoboSimSkillProvider()
    bt_schema_provider = PyRoboSimBTSchemaProvider()
    world_provider = PyRoboSimWorldProvider(world_file=world_file)

    # Server configuration
    config = ServerValidationConfig(
        generated_dir=generated_dir,
        submission_file=submission_file,
        send_static_enforce=True,  # Enforce validation before saving
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

    args = parser.parse_args()

    server = create_server(
        generated_dir=args.generated_dir,
        submission_file=args.submission_file,
        world_file=args.world_file,
        restrict_control_flow=args.restrict_control_flow,
    )

    print("Starting PyRoboSim MCP server...")
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
    server.run()


if __name__ == "__main__":
    main()
