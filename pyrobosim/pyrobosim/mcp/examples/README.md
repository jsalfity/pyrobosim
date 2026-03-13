# PyRoboSim MCP Examples

Simple examples demonstrating how to use the PyRoboSim MCP server and execute behavior trees.

## Quick Start

### 1. Start the MCP Server

```bash
python -m pyrobosim.mcp.server --world-file roscon_2024_workshop_world.yaml --port 8000
```

This exposes PyRoboSim skills via the Model Context Protocol.

### 2. Generate a BT (using MCP client)

Use an MCP client (e.g., Claude Code) to generate a behavior tree JSON file. The server provides these tools:

- `list_skills()` - Get available PyRoboSim skills
- `get_bt_format()` - Get the BT JSON schema
- `send_to_robot(bt_json, ...)` - Validate and save a BT
- `list_world_entities()` - Get world vocabulary (rooms, objects, etc.)

Generated BTs are saved to `./generated/` by default.

### 3. Execute the BT

**Headless (no GUI, fast):**
```bash
python run_bt_headless.py ../../../generated/my_bt.json
```

**With GUI (visual debugging):**
```bash
python run_bt_gui.py ../../../generated/my_bt.json
```

## Example Workflow

```bash
# Terminal 1: Start MCP server
cd pyrobosim/pyrobosim/pyrobosim/mcp
python server.py --world-file roscon_2024_workshop_world.yaml

# Terminal 2: Generate BT using Claude Code
claude  # then use MCP tools to generate BT

# Terminal 3: Execute BT with GUI
cd examples
python run_bt_gui.py ../../../generated/fetch_apple.json
```

## Arguments

### run_bt_headless.py

- `bt_file` - Path to BT JSON file (required)
- `--world-file` - World YAML (default: roscon_2024_workshop_world.yaml)
- `--realtime-factor` - Simulation speed, 0.0 = instant (default: 0.0)

### run_bt_gui.py

- `bt_file` - Path to BT JSON file (required)
- `--world-file` - World YAML (default: roscon_2024_workshop_world.yaml)
- `--realtime-factor` - Simulation speed, 1.0 = realtime (default: 1.0)
- `--tick-rate` - Seconds between tree ticks (default: 0.5)

## Notes

- These are simple examples for getting started with PyRoboSim MCP
- For production evaluation workflows, see `bt-eval-harness/executors/`
- BT files must match the schema from `get_bt_format()`
- World vocabulary is validated if `--world-file` is provided to the server
