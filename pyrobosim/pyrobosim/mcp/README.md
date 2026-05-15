# PyRoboSim MCP Server

Model Context Protocol (MCP) server that exposes PyRoboSim skills and world vocabulary to Claude for behavior tree generation.

## Quick Start

### 1. Start the sim_app server

```bash
cd pyrobosim/pyrobosim
python3 -m pyrobosim.sim_app --world-file examples/01_simple.yaml
```

The sim_app GUI will launch and serve on `http://localhost:8080` by default.

### 2. Create a batch directory and start MCP server

```bash
cd /path/to/bt-eval-harness
mkdir -p generated/$(date +%Y-%m-%d_%H-%M)
cd generated/$(date +%Y-%m-%d_%H-%M)

python3 -m pyrobosim.mcp.server --sim-app-url http://localhost:8080
```

For pass@1 evaluation runs, also pass `--no-validate-tool` so every BT goes through `send_to_robot` (and is counted as an attempt):

```bash
python3 -m pyrobosim.mcp.server --sim-app-url http://localhost:8080 --no-validate-tool
```

The MCP server will:
- Load skill schemas from `data/skills.yaml`
- Load BT schema from `data/bt_schema.yaml`
- Load rootstock BT templates from `data/rootstocks.yaml`
- Fetch live world vocabulary (rooms, locations, object spawns, objects) from sim_app server
- Save generated BTs to `jsons/` subdirectory (in current working directory)
- Log generations to `submissions.jsonl` with `valid`, `issues`, and `attempt_number` per submission

**Batch Directory Structure:**
```
generated/2026-03-16_18-07/
  jsons/               # BT JSON files (created by MCP)
  submissions.jsonl    # Generation log (created by MCP)
  results.json         # Evaluation results (created by bt-eval-harness)
  results_summary.json # Summary (created by bt-eval-harness)
```

### 3. Connect Claude Desktop

Add to your Claude Desktop MCP config (`~/Library/Application Support/Claude/claude_desktop_config.json`):

```json
{
  "mcpServers": {
    "pyrobosim": {
      "command": "python3",
      "args": [
        "-m",
        "pyrobosim.mcp.server",
        "--sim-app-url",
        "http://localhost:8080"
      ],
      "cwd": "/path/to/pyrobosim/pyrobosim"
    }
  }
}
```

Restart Claude Desktop to connect.

## Architecture

- **MCP Server**: Defines what's possible (skills, BT schema, world vocabulary)
- **sim_app**: Executes behavior trees with GUI visualization
- **bt-eval-harness**: Batch evaluation tool for research

## Available Tools

When connected, Claude can:
- `get_skills` - Get available robot skills (navigate, detect, pick, place, open, close)
- `get_bt_schema` - Get behavior tree JSON schema
- `get_world_entities` - Get current world vocabulary (rooms, locations, objects)
- `list_rootstocks_tool` - Get canonical BT composition templates for common task patterns
- `send_to_robot` - Validate, save, and log a BT; returns `attempt_number` and `static_validation`. Returns `success=false` with `reason="max_attempts_reached"` once the per-session cap (default 10) is hit.
- `validate_bt` - Validate a behavior tree against schema and skills *(hidden when `--no-validate-tool` is passed)*
- `execute_bt` - Execute a behavior tree via sim_app server

## Rootstocks

Rootstocks are optional composition templates for common BT patterns. They are not executable skills and they do not extend the BT runtime semantics.

Current rootstocks:
- `detect_pick` - direct detect-then-pick sequence
- `search_pick_selector` - two-branch selector search with guarded pick branches
- `search_pick_deliver` - selector search followed by either place or return delivery flow

Use them as scaffolding for synthesis, then fill slots with exact world vocabulary names.

## Files

- `server.py` - MCP server implementation
- `providers.py` - Skill, schema, and world entity providers
- `skills.py` - Helper to load skill schemas for validation
- `bt_schema.py` - Helper to load BT schema
- `data/skills.yaml` - PyRoboSim skill definitions
- `data/bt_schema.yaml` - Behavior tree JSON schema
