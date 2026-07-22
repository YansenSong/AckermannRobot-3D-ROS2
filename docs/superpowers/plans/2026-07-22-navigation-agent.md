# Navigation Agent Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a terminal-based ROS 2 agent that uses LangChain with an OpenAI-compatible API to resolve natural-language requests to whitelisted destinations, asks for confirmation, and publishes `/goal_pose`.

**Architecture:** A new `ament_python` package separates configuration, model parsing, destination lookup, ROS message publication, and terminal orchestration. The model may select only an action and destination name; local code owns coordinates, validation, confirmation, and ROS execution.

**Tech Stack:** ROS 2 Humble, Python 3.10, `rclpy`, `geometry_msgs`, PyYAML, Pydantic, LangChain, `langchain-openai`, pytest, ament lint.

## Global Constraints

- Publish only `geometry_msgs/msg/PoseStamped` on `/goal_pose` with `header.frame_id = "map"`.
- Accept only the `navigate` action and destinations defined in `config/destinations.yaml`.
- Never accept model-generated coordinates or execute shell commands.
- Publish only after an explicit lowercase-insensitive `y` confirmation.
- Read secrets only from `NAV_AGENT_API_KEY` and `NAV_AGENT_BASE_URL`; never log them.
- Mock every model call in automated tests.
- Preserve the existing RViz, Hybrid A*, `/plan`, and NeuPAN behavior.

---

### Task 1: Package Scaffold and Validated Configuration

**Files:**
- Create: `src/navigation_agent/package.xml`
- Create: `src/navigation_agent/setup.py`
- Create: `src/navigation_agent/setup.cfg`
- Create: `src/navigation_agent/resource/navigation_agent`
- Create: `src/navigation_agent/navigation_agent/__init__.py`
- Create: `src/navigation_agent/navigation_agent/config.py`
- Create: `src/navigation_agent/config/agent.yaml`
- Create: `src/navigation_agent/config/destinations.yaml`
- Create: `src/navigation_agent/test/test_config.py`

**Interfaces:**
- Produces: `Destination(name: str, x: float, y: float, yaw: float)`.
- Produces: `AgentConfig(model: str, timeout_seconds: float, max_retries: int)`.
- Produces: `load_destinations(path: Path) -> dict[str, Destination]`.
- Produces: `load_agent_config(path: Path) -> AgentConfig`.

- [ ] **Step 1: Create the ROS package metadata and install config files**

Use `ament_python` metadata. In `setup.py`, include both YAML files under `share/navigation_agent/config`, declare `setuptools`, `PyYAML`, `pydantic`, `langchain`, and `langchain-openai`, and reserve this console entry point for Task 4:

```python
entry_points={
    'console_scripts': [
        'terminal_agent = navigation_agent.terminal_agent_node:main',
    ],
}
```

In `package.xml`, declare `ament_python` as the build type; add runtime dependencies `rclpy`, `geometry_msgs`, `python3-yaml`; and test dependencies `python3-pytest`, `ament_flake8`, and `ament_pep257`.

- [ ] **Step 2: Write failing configuration tests**

Create tests using temporary YAML files:

```python
def test_load_destinations_returns_typed_lookup(tmp_path):
    path = tmp_path / 'destinations.yaml'
    path.write_text('destinations:\n  loading_zone:\n    x: 2.5\n    y: -1\n    yaw: 1.57\n')
    result = load_destinations(path)
    assert result['loading_zone'] == Destination('loading_zone', 2.5, -1.0, 1.57)


@pytest.mark.parametrize('body', [
    'destinations: {}',
    'destinations: {dock: {x: .nan, y: 0, yaw: 0}}',
    'destinations: {dock: {x: 1, y: 0}}',
])
def test_load_destinations_rejects_invalid_entries(tmp_path, body):
    path = tmp_path / 'destinations.yaml'
    path.write_text(body)
    with pytest.raises(ConfigError):
        load_destinations(path)
```

Also test positive timeout, nonnegative integer retries, a non-empty model name, a missing file, and malformed YAML.

- [ ] **Step 3: Run the tests to verify they fail**

Run: `pytest -q src/navigation_agent/test/test_config.py`

Expected: FAIL during import because `navigation_agent.config` does not exist.

- [ ] **Step 4: Implement strict configuration loading**

Use frozen dataclasses and reject booleans, missing fields, non-numeric values, infinities, duplicate YAML keys, blank destination names, and an empty destination map. Wrap file, YAML, and type failures in a secret-free `ConfigError`.

```python
@dataclass(frozen=True)
class Destination:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class AgentConfig:
    model: str
    timeout_seconds: float
    max_retries: int
```

Provide initial non-secret configuration:

```yaml
# config/agent.yaml
model: gpt-4.1-mini
timeout_seconds: 20.0
max_retries: 2
```

```yaml
# config/destinations.yaml
destinations:
  loading_zone:
    x: 2.5
    y: -1.0
    yaw: 1.5708
```

The sample name is replaceable site data; document that users must calibrate it against their map before navigation.

- [ ] **Step 5: Run focused tests and package lint**

Run: `pytest -q src/navigation_agent/test/test_config.py`

Expected: all configuration tests PASS.

Run: `python3 -m compileall -q src/navigation_agent/navigation_agent`

Expected: exit code 0.

- [ ] **Step 6: Commit Task 1**

```bash
git add src/navigation_agent
git commit -m "feat: add navigation agent configuration"
```

---

### Task 2: LangChain Structured Command Parser

**Files:**
- Create: `src/navigation_agent/navigation_agent/command_parser.py`
- Create: `src/navigation_agent/test/test_command_parser.py`

**Interfaces:**
- Consumes: `AgentConfig` from Task 1.
- Produces: `NavigationIntent(action: Literal['navigate'], destination: str)`.
- Produces: `CommandParser(config: AgentConfig, api_key: str, base_url: str)`.
- Produces: `CommandParser.parse(text: str, allowed_destinations: Collection[str]) -> NavigationIntent`.
- Raises: `CommandParseError` with a user-safe message.

- [ ] **Step 1: Write parser contract tests with an injected fake chain**

```python
def test_parse_returns_whitelisted_navigation_intent():
    chain = Mock()
    chain.invoke.return_value = NavigationIntent(
        action='navigate', destination='loading_zone')
    parser = CommandParser.for_test(chain)
    intent = parser.parse('请去装卸区', {'loading_zone'})
    assert intent.destination == 'loading_zone'


def test_parse_rejects_destination_outside_whitelist():
    chain = Mock()
    chain.invoke.return_value = NavigationIntent(
        action='navigate', destination='office')
    parser = CommandParser.for_test(chain)
    with pytest.raises(CommandParseError, match='可用地点'):
        parser.parse('去办公室', {'loading_zone'})


def test_parse_wraps_provider_failure_without_secret():
    chain = Mock()
    chain.invoke.side_effect = RuntimeError('Authorization: Bearer secret-value')
    parser = CommandParser.for_test(chain)
    with pytest.raises(CommandParseError) as error:
        parser.parse('前往装卸区', {'loading_zone'})
    assert 'secret-value' not in str(error.value)
```

Also cover blank input, an empty whitelist, malformed chain results, and rejection of extra fields such as `x`, `y`, or `command`.

- [ ] **Step 2: Run parser tests to verify they fail**

Run: `pytest -q src/navigation_agent/test/test_command_parser.py`

Expected: FAIL because `command_parser.py` does not exist.

- [ ] **Step 3: Implement schema-constrained LangChain parsing**

Define a strict Pydantic schema:

```python
class NavigationIntent(BaseModel):
    model_config = ConfigDict(extra='forbid')
    action: Literal['navigate']
    destination: str = Field(min_length=1)
```

Construct `ChatOpenAI` with `model`, `api_key`, `base_url`, `timeout`, `max_retries`, and `temperature=0`. Use `with_structured_output(NavigationIntent)` and a prompt that includes the sorted allowed names and explicitly forbids coordinates and unsupported actions. After invocation, perform a second local membership check; never rely on the prompt for authorization. Convert every provider/schema exception to the fixed message `模型无法解析该指令，请重试。` without interpolating the original exception.

- [ ] **Step 4: Run parser tests**

Run: `pytest -q src/navigation_agent/test/test_command_parser.py`

Expected: all parser tests PASS with zero network calls.

- [ ] **Step 5: Commit Task 2**

```bash
git add src/navigation_agent/navigation_agent/command_parser.py src/navigation_agent/test/test_command_parser.py
git commit -m "feat: parse navigation commands with LangChain"
```

---

### Task 3: Safe ROS Goal Dispatcher

**Files:**
- Create: `src/navigation_agent/navigation_agent/goal_dispatcher.py`
- Create: `src/navigation_agent/test/test_goal_dispatcher.py`

**Interfaces:**
- Consumes: `Destination` from Task 1.
- Produces: `yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]` in `(x, y, z, w)` order.
- Produces: `GoalDispatcher(node: rclpy.node.Node, topic: str = '/goal_pose')`.
- Produces: `GoalDispatcher.publish(destination: Destination) -> PoseStamped`.
- Raises: `GoalDispatchError` when no planner subscribes to `/goal_pose`.

- [ ] **Step 1: Write failing quaternion and publisher tests**

```python
def test_yaw_to_quaternion_for_half_turn():
    x, y, z, w = yaw_to_quaternion(math.pi)
    assert (x, y) == (0.0, 0.0)
    assert z == pytest.approx(1.0)
    assert w == pytest.approx(0.0, abs=1e-7)


def test_publish_builds_map_pose(node):
    dispatcher = GoalDispatcher(node)
    message = dispatcher.publish(Destination('loading_zone', 2.5, -1.0, 1.5708))
    assert message.header.frame_id == 'map'
    assert message.pose.position.x == 2.5
    assert message.pose.position.y == -1.0
    assert message.pose.orientation.z == pytest.approx(math.sin(1.5708 / 2))


def test_publish_fails_when_planner_is_unavailable(node):
    dispatcher = GoalDispatcher(node)
    with pytest.raises(GoalDispatchError, match='导航规划器不可用'):
        dispatcher.publish(Destination('loading_zone', 2.5, -1.0, 1.5708))
```

Use a real test node plus a `/goal_pose` subscriber and a single-threaded executor to verify exactly one message is received and matches the returned message. Wait for ROS discovery before the successful publish assertion.

- [ ] **Step 2: Run dispatcher tests to verify they fail**

Run: `pytest -q src/navigation_agent/test/test_goal_dispatcher.py`

Expected: FAIL because `goal_dispatcher.py` does not exist.

- [ ] **Step 3: Implement message construction and publication**

Create a depth-10 publisher. Before constructing the message, require `publisher.get_subscription_count() > 0`; otherwise raise `GoalDispatchError('导航规划器不可用，未发送目标。')`. Set `header.stamp` from `node.get_clock().now().to_msg()`, frame to `map`, `position.z` to `0.0`, and quaternion from `sin(yaw / 2)` and `cos(yaw / 2)`. Return the published message to make behavior observable without weakening encapsulation.

- [ ] **Step 4: Run dispatcher tests**

Run: `pytest -q src/navigation_agent/test/test_goal_dispatcher.py`

Expected: all tests PASS.

- [ ] **Step 5: Commit Task 3**

```bash
git add src/navigation_agent/navigation_agent/goal_dispatcher.py src/navigation_agent/test/test_goal_dispatcher.py
git commit -m "feat: publish validated navigation goals"
```

---

### Task 4: Terminal Confirmation Workflow and ROS Node

**Files:**
- Create: `src/navigation_agent/navigation_agent/terminal_session.py`
- Create: `src/navigation_agent/navigation_agent/terminal_agent_node.py`
- Create: `src/navigation_agent/test/test_terminal_session.py`
- Create: `src/navigation_agent/test/test_terminal_agent_node.py`

**Interfaces:**
- Consumes: `CommandParser.parse`, destination lookup, and `GoalDispatcher.publish`.
- Produces: `TerminalSession.handle(command: str, confirm: Callable[[str], str]) -> SessionResult`.
- Produces: `SessionResult(status: Literal['published', 'cancelled', 'rejected', 'quit'], message: str)`.
- Produces: console executable `terminal_agent`.

- [ ] **Step 1: Write failing orchestration tests**

```python
def test_confirmed_command_publishes_resolved_destination():
    parser = Mock()
    parser.parse.return_value = NavigationIntent(
        action='navigate', destination='loading_zone')
    dispatcher = Mock()
    session = TerminalSession(parser, {'loading_zone': DESTINATION}, dispatcher)
    result = session.handle('请去装卸区', lambda _: 'y')
    dispatcher.publish.assert_called_once_with(DESTINATION)
    assert result.status == 'published'


@pytest.mark.parametrize('answer', ['', 'n', 'yes', '取消'])
def test_non_y_confirmation_never_publishes(answer):
    session, dispatcher = make_session()
    result = session.handle('前往装卸区', lambda _: answer)
    dispatcher.publish.assert_not_called()
    assert result.status == 'cancelled'
```

Also verify `Y` is accepted; `quit` skips the model and dispatcher; unknown destinations, parser failures, goal-dispatch failures, and confirmation callback failures never publish; and the confirmation prompt contains name, `x`, `y`, and yaw.

- [ ] **Step 2: Run workflow tests to verify they fail**

Run: `pytest -q src/navigation_agent/test/test_terminal_session.py`

Expected: FAIL because `terminal_session.py` does not exist.

- [ ] **Step 3: Implement a pure, fail-closed terminal session**

Keep terminal I/O out of the orchestration class. Normalize only with `strip()` and `lower()`. Return `quit` only for the exact normalized command `quit`; authorize publication only when the normalized confirmation is exactly `y`. Catch `CommandParseError` and return `rejected` with its safe message.

- [ ] **Step 4: Implement the ROS entry point and node test**

`TerminalAgentNode` loads installed YAML paths using `get_package_share_directory('navigation_agent')`, requires both environment variables before constructing the model client, and creates the parser, dispatcher, and session. The `main()` loop is:

```python
while rclpy.ok():
    try:
        command = input('> ')
    except (EOFError, KeyboardInterrupt):
        break
    result = session.handle(command, input)
    print(result.message)
    if result.status == 'quit':
        break
```

Always destroy the node and call `rclpy.shutdown()` in `finally`. Patch configuration, environment, parser, and dispatcher in the node test; verify missing API variables fail before any client is built and the shutdown path runs on EOF.

- [ ] **Step 5: Run terminal and node tests**

Run: `pytest -q src/navigation_agent/test/test_terminal_session.py src/navigation_agent/test/test_terminal_agent_node.py`

Expected: all tests PASS and no network requests occur.

- [ ] **Step 6: Commit Task 4**

```bash
git add src/navigation_agent/navigation_agent/terminal_session.py src/navigation_agent/navigation_agent/terminal_agent_node.py src/navigation_agent/test/test_terminal_session.py src/navigation_agent/test/test_terminal_agent_node.py
git commit -m "feat: add confirmed terminal navigation workflow"
```

---

### Task 5: Lint, Build, Documentation, and Simulation Acceptance

**Files:**
- Create: `src/navigation_agent/test/test_flake8.py`
- Create: `src/navigation_agent/test/test_pep257.py`
- Modify: `README.md`

**Interfaces:**
- Produces: documented installation, environment, destination calibration, launch, and smoke-test workflow.

- [ ] **Step 1: Add standard ament lint tests**

Mirror the repository's existing `neupan_ros2` lint entry points:

```python
@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, 'Found %d errors:\n' % len(errors) + '\n'.join(errors)
```

Use `ament_pep257.main` for the PEP 257 test and keep public modules/classes documented rather than broadly disabling docstring rules.

- [ ] **Step 2: Run the complete package test suite**

Run: `pytest -q src/navigation_agent/test`

Expected: all unit, node, and lint tests PASS with no skips caused by missing model credentials.

- [ ] **Step 3: Build and test through colcon**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select navigation_agent
source install/setup.bash
colcon test --packages-select navigation_agent
colcon test-result --verbose
```

Expected: build exit code 0 and `navigation_agent` reports zero failed tests.

- [ ] **Step 4: Document operator setup in README**

Add a “自然语言导航 Agent” section that documents:

```bash
python3 -m pip install langchain langchain-openai pydantic
export NAV_AGENT_API_KEY='<provider key>'
export NAV_AGENT_BASE_URL='https://provider.example/v1'
ros2 run navigation_agent terminal_agent
```

Explain that operators must edit `src/navigation_agent/config/destinations.yaml`, rebuild/source the workspace, and verify every pose against the active `map` before use. Include the supported interaction (`前往装卸区`, explicit `y`, and `quit`) and state that this feature is not an emergency-stop system.

- [ ] **Step 5: Perform a ROS topic smoke test without the model**

With the package sourced, use a test double or focused node test to verify `/goal_pose` type and contents:

```bash
ros2 topic type /goal_pose
ros2 topic echo /goal_pose --once
```

Expected after an approved request: type `geometry_msgs/msg/PoseStamped`, frame `map`, and pose matching the selected YAML entry.

- [ ] **Step 6: Perform the Gazebo acceptance test**

Start the existing stack:

```bash
bash scripts/nav_hdl_neupan.sh
bash scripts/run_neupan.sh
ros2 run navigation_agent terminal_agent
```

Set the initial pose in RViz, enter `请去装卸区`, verify the displayed coordinates, and enter `y`. Confirm `/goal_pose` is emitted, Hybrid A* publishes `/plan`, and NeuPAN commands the vehicle. Then submit another goal from RViz and confirm manual navigation still works. Record the calibrated destination and observed result in the implementation handoff.

- [ ] **Step 7: Commit Task 5**

```bash
git add src/navigation_agent/test/test_flake8.py src/navigation_agent/test/test_pep257.py README.md
git commit -m "docs: add navigation agent workflow"
```

- [ ] **Step 8: Verify the final diff**

Run:

```bash
git status --short
git log --oneline -5
colcon test-result --verbose
```

Expected: only pre-existing unrelated files remain untracked or modified, the five task commits are present, and the final test summary contains zero failures.
