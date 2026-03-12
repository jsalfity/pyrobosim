"""Create submissions.jsonl for B3 baseline from generated BT files."""

import json
import uuid
from pathlib import Path
from datetime import datetime
import sys

# Add parent to path to import from lib
sys.path.insert(0, str(Path(__file__).parent))
from lib.io_utils import load_yaml

# Load tasks to get prompts
tasks_file = Path(__file__).parent / "tasks50_roscon.yaml"
config = load_yaml(tasks_file)
tasks = config.get('tasks', [])

# Find all B3 BT files
repo_root = Path(__file__).parent.parent
bt_dir = repo_root / "pyrobosim" / "pyrobosim" / "behaviors" / "generated" / "b3_prompt"
bt_files = sorted(bt_dir.glob("*.json"))

print(f"Found {len(bt_files)} B3 BT files in {bt_dir}")

# Create submissions
submissions = []
for bt_file in bt_files:
    # Extract task number from filename (e.g., b3_task_0.json -> 0, b3_task_10.json -> 10)
    # Handle both b3_task_X.json and task_X.json formats
    stem = bt_file.stem
    if 'task_' in stem:
        task_num_str = stem.split('task_')[-1]
        try:
            task_num = int(task_num_str)
        except ValueError:
            print(f"Warning: Could not parse task number from {bt_file.name}")
            continue
    else:
        print(f"Warning: Unexpected filename format: {bt_file.name}")
        continue

    # Convert to task ID format (e.g., 0 -> task01, 10 -> task11)
    task_id_prefix = f"task{task_num + 1:02d}_"

    # Find matching task
    matching_task = None
    for task in tasks:
        if task['id'].startswith(task_id_prefix):
            matching_task = task
            break

    if not matching_task:
        print(f"Warning: No matching task for {bt_file.name}")
        continue

    # Create submission entry (path relative to repo root)
    relative_path = bt_file.relative_to(repo_root)
    submission = {
        "submission_id": uuid.uuid4().hex,
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "generated": str(relative_path),
        "task_prompt": matching_task.get('task_prompt', ''),
    }
    submissions.append(submission)
    print(f"✓ {bt_file.name} -> {matching_task['id']}")

# Write submissions file
output_file = Path(__file__).parent / "submissions_b3_prompt.jsonl"
with open(output_file, 'w') as f:
    for sub in submissions:
        f.write(json.dumps(sub) + '\n')

print(f"\n✓ Wrote {len(submissions)} submissions to {output_file}")
