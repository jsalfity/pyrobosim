# PyRoboSim Simulation App

A standalone simulation server with GUI visualization and HTTP control API for executing behavior trees against a PyRoboSim world.

## Features

- **GUI Visualization**: Real-time visualization of the robot and world state
- **HTTP Control API**: RESTful API for remote BT execution
- **Background Execution**: BTs run in background threads while GUI remains responsive
- **World Management**: Reset and reload world state via API

## Quick Start

### 1. Start the Simulation Server

```bash
python -m pyrobosim.sim_app.server --world-file roscon_2024_workshop_world.yaml --port 8080
```

This will:
- Load the specified world file
- Start the HTTP control server on port 9001
- Open a GUI window showing the world

### 2. Execute a Behavior Tree

In a separate terminal:

```bash
python -m pyrobosim.sim_app.run_bt_client path/to/behavior_tree.json --realtime-factor 5
```

This will:
- Connect to the running server
- Execute the BT
- Display the tree structure in the terminal as it ticks
- Show robot movements in the GUI window

## API Endpoints

The control server exposes the following HTTP endpoints:

### `POST /run_bt`
Execute a behavior tree.

**Request:**
```json
{
  "bt_json": { ... },
  "robot": "robot_name",
  "tick_ms": 100,
  "realtime_factor": 1.0,
  "print_every": 1
}
```

**Response:**
```json
{
  "run_id": "uuid"
}
```

### `POST /bt_status`
Get status of a running BT.

**Request:**
```json
{
  "run_id": "uuid"
}
```

**Response:**
```json
{
  "status": "RUNNING|SUCCESS|FAILURE|CANCELED",
  "tick_count": 42,
  "tree": "...",
  "blackboard": { ... }
}
```

### `POST /world_state`
Get current world state.

**Request:**
```json
{
  "robot": "robot_name"
}
```

**Response:**
```json
{
  "robot": "robot",
  "robot_location": "kitchen",
  "location": "kitchen",
  "held_object": {"name": "apple", "category": "food"},
  "objects": [...]
}
```

### `POST /world_entities`
Get world vocabulary (locations, objects, categories).

**Request:**
```json
{
  "mode": "vocab|observed|full",
  "robot": "robot_name",
  "allow_full": false
}
```

**Response:**
```json
{
  "mode": "vocab",
  "rooms": ["kitchen", "dining", ...],
  "locations": ["pantry", "fridge", ...],
  "object_spawns": ["pantry_storage", "fridge_storage", ...],
  "object_categories": ["food", "drinks", ...],
  "objects": ["apple", "soda", ...],
  "hallways": [...]
}
```

### `POST /reset_world`
Reset world to initial state.

**Request:**
```json
{
  "deterministic": false,
  "seed": -1
}
```

**Response:**
```json
{
  "reset": true
}
```

### `POST /reload_world`
Reload world from file.

**Request:**
```json
{}
```

**Response:**
```json
{
  "reload": true
}
```

## Command-Line Options

### Server (`pyrobosim.sim_app.server`)

```
--world-file WORLD_FILE  Path to world YAML file (required)
--host HOST              Server host (default: 127.0.0.1)
--port PORT              Server port (default: 9001)
```

### Client (`pyrobosim.sim_app.run_bt_client`)

```
bt_file                  Path to BT JSON file (required)
--control-url URL        Server URL (default: http://localhost:9001)
--realtime-factor FLOAT  Simulation speed (default: 1.0)
--timeout INT            Timeout in seconds (default: 60)
--robot ROBOT            Robot name (default: first robot)
```

## Example Usage

### Terminal 1: Start Server
```bash
cd /path/to/pyrobosim
python -m pyrobosim.sim_app.server \
  --world-file roscon_2024_workshop_world.yaml \
  --port 9001
```

### Terminal 2: Execute BT
```bash
cd /path/to/pyrobosim
python -m pyrobosim.sim_app.run_bt_client \
  /path/to/bt.json \
  --realtime-factor 10 \
  --timeout 120
```

## Integration with Other Tools

The sim app is designed to work with:

- **bt-eval-harness**: Batch evaluation of behavior trees
- **MCP Server**: BT generation via Claude (uses `/world_entities` for vocabulary)
- **Custom Clients**: Any HTTP client can interact with the API

Example with bt-eval-harness:
```bash
# Terminal 1: Start sim app
python -m pyrobosim.sim_app.server --world-file roscon_2024_workshop_world.yaml

# Terminal 2: Run batch evaluation
cd /path/to/bt-eval-harness
python run_eval.py \
  --tasks-file tasks.yaml \
  --control-url http://localhost:9001 \
  --realtime-factor 10
```

## Architecture

- **`server.py`**: Main server with HTTP API and GUI
- **`run_bt_client.py`**: CLI client for executing single BTs
- **`utils.py`**: HTTP client utilities
- **`world_entities.py`**: World vocabulary extraction
- **`__init__.py`**: Module exports
- **`__main__.py`**: Module entry point

## Dependencies

This module depends on:
- `pyrobosim.behaviors.build_pyrobosim_tree_from_json` (from feature/json_bt_integration)
- `pyrobosim.core.World`, `WorldYamlLoader`
- `pyrobosim.gui.start_gui`
- `py_trees` for BT execution
