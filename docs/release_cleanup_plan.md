# roship_io ROS2 Cleanup & Release Plan

This plan defines a practical path to clean up, document, test, and release `roship_io` using ROS2 package release best practices (ament/colcon + bloom-oriented workflow).

## 1) Release goals and scope

### Primary deliverables

1. Production-ready README tailored to `roship_io` (replace template content).
2. Doxygen documentation with API examples (especially Modbus, plus UDP/serial/MQTT paths).
3. Minimal launch and config examples for each supported connection type.
4. Linting integrated into CI and local workflow.
5. Test nodes/scripts for smoke + regression behavior.

### Release success criteria

- Package builds cleanly with `colcon build --symlink-install`.
- `colcon test` passes for lint + unit/integration tests.
- README contains install, quickstart, architecture, topics, parameters, and troubleshooting.
- Doxygen generates class/module pages and a minimal set of executable examples.
- Launch/config examples run without code edits (parameter-only setup).
- Version/tag/changelog/release metadata are ready for bloom release.

## 2) Documentation plan (README + Doxygen)

Use the same information architecture style seen in mature ROS repos (like `echoflow`): concise quickstart first, then deeper operational and API detail.

### README structure to implement

1. **What this package does** (short problem statement + supported transports).
2. **Supported ROS2 distros + dependencies** (system libs: libmodbus, libserialport, mosquitto).
3. **Install/build**
   - apt dependencies
   - workspace setup
   - `colcon build`
4. **Quickstart**
   - Launch UDP connection with example config.
   - Launch MQTT connection with example config.
   - Run minimal publisher/test scripts.
5. **Topics, services, and parameters**
   - `~/to_device`, `~/from_device`, per-transport parameter tables.
6. **Architecture overview**
   - transport layer (`UdpSocket`, `LspSerial`, `MqttClient`)
   - connection nodes/wrappers
   - modbus node abstraction
7. **Troubleshooting** (timeouts, connection drops, parameter mismatches).
8. **Release notes/changelog link**.

### Doxygen work items

- Add module-level pages: `modbus`, `transport`, `connection`.
- Add `@example` blocks for:
  - Modbus register read/write polling loop.
  - UDP raw packet publish/receive.
  - MQTT topic bridge behavior.
  - Serial frame delimiting usage.
- Ensure Doxygen includes `include/` + selected `src/` docs and links to Markdown docs.
- Keep examples minimal and runnable as reference snippets.

## 3) Launch/config examples

Create a minimal example set under `roship_io/launch` and `roship_io/config`:

- `minimal_udp.launch.xml` + `minimal_udp.yaml`
- `minimal_mqtt.launch.xml` + `minimal_mqtt.yaml`
- `minimal_serial.launch.xml` + `minimal_serial.yaml`
- `minimal_modbus.launch.xml` + `minimal_modbus.yaml`

Each should include only required parameters and safe localhost/dev defaults.

## 4) Linting and CI hardening

### Lint baseline

- Keep `ament_lint_auto_find_test_dependencies()` and explicitly add common tools in `package.xml` (cpplint, uncrustify, cppcheck, xmllint as appropriate).
- Add `.clang-format`/`.clang-tidy` if not present.
- Add a local helper script (optional): `scripts/lint.sh` running `colcon test --packages-select roship_io`.

### CI checks (recommended)

1. Build: `colcon build --symlink-install`
2. Lint: `colcon test --ctest-args -R lint`
3. Tests: `colcon test`
4. Docs: Doxygen generation as a separate job

## 5) Test plan (nodes/scripts)

### Unit tests

- `modbus` behavior with mocked transport for read/write error propagation.
- Parameter parsing validation tests (mismatched list sizes, invalid values).
- Serialization/framing tests for serial end-of-frame behavior.

### Integration/smoke tests

- UDP loopback smoke node/script.
- MQTT smoke test against local broker (can be optional in CI with service container).
- Launch tests verifying nodes start with minimal config and expected topics exist.

### Existing scripts to keep/extend

- `scripts/mqtt_test.sh`
- `scripts/replay_pcap.sh`

Add usage notes and expected outputs in README.

## 6) Code audit findings (current repo)

### Potential bugs / defects

1. **Serial framing off-by-one for ASCII delimiter**
   - In `LspSerial::readLoop`, when ASCII delimiter is found, iterator is advanced by delimiter length and then `eof_iter + 1` is used for message slicing. This can include one extra byte or overrun logical boundary.
2. **Serial connection timer does not poll transport** THIS IS FINE, but should be noted as a design choice that relies on background thread for transport updates. If the transport layer (e.g., serial port) has internal state that needs regular polling, this could lead to dead/inconsistent behavior since `spin_once()` is effectively a no-op.
   - `SerialConnection::spin_once()` has the call to `serial_ptr_->spinOnce()` commented out; behavior depends entirely on background thread, making timer path dead/inconsistent.
3. **MQTT global init/cleanup lifecycle risk**
   - `mosquitto_lib_cleanup()` is called in destructor whenever a client is destroyed, which can break multi-instance lifecycle if more than one client exists.
4. **Exception handling by value**
   - Several catches use `catch(std::runtime_error err)` instead of `const std::runtime_error&`, causing copies and potential slicing patterns.

### Easily implemented feature improvements

1. Add parameter validators (ranges for ports, timeout > 0, non-empty topic list).
2. Add runtime parameter update callback for key connection params.
3. Add connection status diagnostics topic (`diagnostic_msgs/DiagnosticArray` or simple status topic).
4. Improve log messages and spelling consistency (e.g., "Connected", "Listening", "Received").

### Hardcoded defaults to parameterize/revisit

1. `ModbusNode` default IP is hardcoded to `192.168.52.209` and should default to localhost/dev-safe value or require explicit override.
2. UDP defaults use fixed ports/hosts (`1234`, `4321`, `127.0.0.1`) — acceptable for examples, but should be clearly documented as defaults and validated.
3. MQTT defaults include hardcoded topic names (`topic1`, `topic2`, `topic3`); better to require explicit user topic list or ship a documented single-topic default.

## 7) ROS2 release process (recommended sequence)

1. **Stabilization branch**
   - Create `release/x.y.z-prep` branch.
2. **Finish deliverables**
   - README, Doxygen examples, launch/config examples, lint/test updates.
3. **Quality gates**
   - Run build/test/lint locally and in CI.
4. **Versioning/changelog**
   - Bump version in `package.xml`, update changelog, verify license and maintainer metadata.
5. **Tag + bloom release**
   - Tag release commit.
   - Run bloom against target distro and rosdistro track.
6. **Post-release verification**
   - Validate binary install path and quickstart from clean environment.

## 8) Suggested implementation timeline

### Phase 1 (1-2 days)
- Replace README template with package-specific docs.
- Add minimal launch/config examples.

### Phase 2 (1-2 days)
- Doxygen API pass + example snippets.
- Fix high-confidence defects (serial framing, exception handling, logging typos).

### Phase 3 (1-2 days)
- Add lint/test coverage and launch smoke tests.
- Final release metadata, version bump, and tag.

## 9) Immediate next actions (high priority)

1. Replace README template content now.
2. Fix serial framing delimiter handling.
3. Add minimal serial/modbus launch+config examples.
4. Add at least one launch smoke test and one transport unit test.
5. Prepare release checklist PR and run full `colcon test`.
