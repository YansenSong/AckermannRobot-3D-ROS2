# Navigation Agent Design

## Goal

Add a terminal-based ROS 2 agent that accepts natural-language navigation requests, uses LangChain with an OpenAI-compatible model to identify a predefined destination, asks the user for confirmation, and then sends the goal through the existing Hybrid A* and NeuPAN navigation pipeline.

The first version supports navigation only. It does not execute shell commands, accept model-generated coordinates, manage system processes, or implement emergency-stop behavior.

## Architecture

Create a Python package at `src/navigation_agent/` with these responsibilities:

- `terminal_agent_node.py`: own the ROS 2 node, terminal loop, confirmation prompt, and user-visible status.
- `command_parser.py`: invoke LangChain and return a schema-constrained intent containing only `action` and `destination`.
- `destination_store.py`: load `config/destinations.yaml`, validate entries, and resolve destination names to fixed `x`, `y`, and `yaw` values.
- `goal_dispatcher.py`: construct and publish `geometry_msgs/msg/PoseStamped` goals.
- `config/agent.yaml`: store the model name, timeout, retry count, and other non-secret settings.
- `test/`: contain parser, configuration, safety, confirmation, and node tests.

The API key and compatible endpoint are read from `NAV_AGENT_API_KEY` and `NAV_AGENT_BASE_URL`. Secrets must never be stored in repository files or logs.

## Data Flow

1. On startup, the node loads and validates the destination configuration.
2. The terminal accepts a request such as `前往装卸区`.
3. LangChain requests a structured result such as `{"action":"navigate","destination":"装卸区"}`.
4. Local code requires `action` to equal `navigate` and resolves `destination` exclusively through the YAML whitelist. Coordinates returned by the model, if any, are ignored and rejected as invalid output.
5. The terminal displays the resolved name and pose. Only an explicit `y` confirmation authorizes execution; all other input cancels it.
6. The node publishes a `PoseStamped` on `/goal_pose` with frame `map` and converts YAML `yaw` to a quaternion.
7. The existing `hybrid_astar_planner` publishes the resulting path on `/plan`, which NeuPAN consumes without modification.

`quit` ends the terminal session. Unknown commands return to the prompt without publishing.

## Validation and Failure Handling

Destination entries require a unique, non-empty name and finite numeric `x`, `y`, and `yaw` fields. Invalid configuration prevents node startup with an actionable error. Unknown destinations list the available names.

Malformed model output, unsupported actions, API failures, timeouts, exhausted retries, unavailable ROS interfaces, and rejected confirmations must publish no goal. Errors are reported concisely and the interactive session remains available when safe. Request headers, API keys, and raw authentication errors are excluded from logs.

## Testing and Acceptance

Automated tests mock all model calls so they require neither network access nor API cost. Unit tests cover destination loading, schema enforcement, whitelist resolution, yaw conversion, and error paths. Node tests verify that `/goal_pose` receives exactly one matching message after confirmation and no message after rejection or any validation failure.

A Gazebo smoke test starts the existing navigation workflow, enters natural-language variants such as `前往装卸区` and `请去装卸区`, confirms the resolved target, and verifies that Hybrid A* publishes `/plan` and NeuPAN begins execution. Existing RViz goal submission must continue to work unchanged.
