# User Application IDL

The `user_app` Layer provides a semantic abstraction system for the SerialComm middleware to prevent the manually handling of each command.

## Architecture Overview

The middleware architecture is divided into layers:

```text
Application Layer
    ↓
User App IDL
    ↓
Generated Structures
    ↓
Middleware (Events / Requests / Missions)
    ↓
Core Protocol
    ↓
Transport Layer
```

The `user_app` folder belongs to the Application Layer. It defines how the application expects communication data to be structured.

## Directory Structure

The application definitions are organized into folders according to the communication semantic.

```text
user_app/
├── event/
│   └── RobotState.event
│
├── request/
│   └── SetPose.request
│
├── mission/
│   └── PlayMovement.mission
│
├── struct/
│   ├── Pose.struct
│   └── DefaultReturn.struct
│
└── generate.py
```

Note: generated C++ output is **not** written inside `user_app/`. It is written to
`include/serial_comm/generated/` at the component root (see "Code Generation" below),
so the generated headers sit alongside the rest of the public component API and are
reachable through the component's existing `include` directory without any extra
include-path configuration.

## What is a Struct?

A `struct` defines reusable data structures. They are equivalent to shared data types used by:

* events;
* requests;
* missions.

A struct does not define communication behavior. It only defines data layout.

### Struct Example

File:

```text
user_app/struct/Pose.struct
```

Content:

```text
int16[32] joints_positions
```

Generated structure:

```cpp
struct Pose {
    std::array<int16_t, 32> joints_positions;
};
```

---

## What is an Event?

An `event` is an asynchronous publish/subscribe communication. Events are typically used for:

* telemetry;
* sensors;
* state broadcasting;
* notifications.

Events do not expect a reply.

### Event Example

File:

```text
user_app/event/RobotState.event
```

Content:

```text
@id 0x10
@qos best_effort

Pose pose
float32 battery
bool enabled
```

Generated structure:

```cpp
struct RobotState {
    Pose pose;
    float battery;
    bool enabled;
};
```

Example usage:

```cpp
RobotState state;

state.battery = 12.5f;
state.enabled = true;

manager.publish_event(state);
```

## What is a Request?

A `request` defines a request/reply communication. Requests are used when:

* the sender expects a response;
* a command needs confirmation;
* remote procedure execution is required.

A request is divided into:

* Request Payload;
* Response Payload.

The separator between both sections is:

```text
===
```

### Request Example

File:

```text
user_app/request/SetPose.request
```

Content:

```text
@id 0x20
@qos reliable
@timeout 1000

Pose target
float32 duration
bool wait_finish

===

bool success
float32 actual_position
```

Generated structures:

```cpp
struct SetPose_Request {
    Pose target;
    float duration;
    bool wait_finish;
};

struct SetPose_Response {
    bool success;
    float actual_position;
};
```

Example server callback:

```cpp
bool set_pose_callback(
    const SetPose_Request& req,
    SetPose_Response& res
) {

    res.success = true;
    res.actual_position = 42.0f;

    return true;
}
```

Example client usage:

```cpp
SetPose_Request req;
SetPose_Response res;

req.duration = 1.0f;

manager.call_request(req, res);
```

## What is a Mission?

A `mission` defines a long-running asynchronous operation. Missions are used for:

* trajectories;
* robot behaviors;
* motion execution;
* asynchronous tasks;
* operations with progress feedback.

A mission is divided into:

* Goal;
* Result;
* Feedback.

The sections are separated using:

```text
===
```

### Mission Example

File:

```text
user_app/mission/PlayMovement.mission
```

Content:

```text
@id 0x30
@qos reliable

Pose[] poses
float32[] durations

===

bool success

===

float32 progress
uint32 current_pose
```

Generated structures:

```cpp
struct PlayMovement_Goal {
    std::vector<Pose> poses;
    std::vector<float> durations;
};

struct PlayMovement_Result {
    bool success;
};

struct PlayMovement_Feedback {
    float progress;
    uint32_t current_pose;
};
```

Example usage:

```cpp
bool play_movement_callback(
    const PlayMovement_Goal& goal,
    PlayMovement_Result& result
) {

    result.success = true;

    return true;
}
```

## Types to use

### Primitive Types

The following primitive types are supported.

| Type      | Description             |
| --------- | ----------------------- |
| `int8`    | Signed 8-bit integer    |
| `uint8`   | Unsigned 8-bit integer  |
| `int16`   | Signed 16-bit integer   |
| `uint16`  | Unsigned 16-bit integer |
| `int32`   | Signed 32-bit integer   |
| `uint32`  | Unsigned 32-bit integer |
| `float32` | 32-bit floating point   |
| `float64` | 64-bit floating point   |
| `bool`    | Boolean value           |
| `string`  | Dynamic string          |

### Arrays

The IDL supports:

* dynamic arrays;
* fixed arrays;
* bounded arrays.

#### Dynamic Array

```text
float32[] values
```

#### Fixed Array

```text
int16[32] joints
```

## Metadata System

Metadata lines start with `@`.

Metadata allows the middleware behavior to be configured directly from the IDL.

### Command ID

Defines the command identifier used by the protocol.

```text
@id 0x20
```

If no ID is provided, the middleware may dynamically negotiate the ID during discovery.

### QoS Configuration

Defines communication reliability.

Best effort:

```text
@qos best_effort
```

Reliable communication:

```text
@qos reliable
```

### Timeout Configuration

Defines request timeout in milliseconds.

```text
@timeout 1000
```

### Versioning

Defines the endpoint version.

```text
@version 1
```

### Comments

Lines starting with `#` are ignored.

Example:

```text
# This is a comment
float32 battery
```

## Code Generation

The generator parses all IDL files and creates C++ structures automatically.

Generated files are stored in:

```text
include/serial_comm/generated/
```

Because this path already lives under the component's public `include/` directory,
generated headers are reachable as `serial_comm/generated/...` without any extra
include-path configuration.

Generated components include:

* structures, grouped under `struct/`, `event/`, `request/` and `mission/`;
* `SerialCommSerializer<T>` specializations, grouped under `serializers/`;
* an aggregator header (`generated_serializers.hpp`) that includes every
  specialization in dependency order (see "Serializer System" in the
  [component README](../README.md#serializer-system));
* a `manifest.json` listing every generated type with its kind (struct/event/
  request/mission) and packet `id`.

## Running the Generator

`--input` and `--output` are both required (the generator does not assume defaults).
Run it from the component root:

```bash
python user_app/generate.py --input user_app --output include/serial_comm/generated
```

## Build System Integration

The generator already runs automatically as part of this component's build — see the
"CODE GENERATION" block in the component's [`CMakeLists.txt`](../CMakeLists.txt).

In short, the build:

1. creates `include/serial_comm/generated/` (and a scratch dir under
   `${CMAKE_CURRENT_BINARY_DIR}`) if they don't exist;
2. invokes `generate.py --input <component>/user_app --output <component>/include/serial_comm/generated`;
3. touches a `generated.stamp` file so CMake can track the custom command's output and
   skip regeneration when no `.struct`/`.event`/`.request`/`.mission`/`generate.py` file
   changed;
4. registers a `serial_comm_codegen` custom target depended on by the component library,
   so generation always completes before compilation starts.

No `target_include_directories` step is needed: the generated tree lives inside the
`include` directory that `idf_component_register` already exposes.
