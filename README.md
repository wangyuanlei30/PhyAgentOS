<p align="center">
  <img src="oea-logo-v3-full_mmmxy7a5.png" alt="OpenEmbodiedAgent Logo" width="600"/>
</p>

# OpenEmbodiedAgent

OpenEmbodiedAgent is a framework built upon OpenClaw, designed to lower the barrier for users to develop and deploy robot applications easily and efficiently.

---

## Overview

OpenEmbodiedAgent (OEA) is an open-source embodied intelligence framework that bridges large language models (LLMs) with physical robotic hardware and virtual simulation environments. By abstracting the complexity of low-level robotics stacks, OEA allows developers to describe robot behaviors in natural language and have them executed reliably in the real world.

The framework is hardware-agnostic — it supports humanoid robots, robotic arms, quadruped platforms, and drones — and does not require ROS as a dependency, though ROS2 integration is available via the RosClaw bridge.

---

## Architecture

OEA is organized into four core layers:

```
┌─────────────────────────────────────────────────────┐
│                   User / Application                │
│          (Messaging clients, APIs, SDKs)            │
└──────────────────────────┬──────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────┐
│               Cognitive Layer (LLM Brain)           │
│   Task planning · Language grounding · Reasoning    │
│      (Claude / GPT / DeepSeek / custom models)      │
└──────────────────────────┬──────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────┐
│              Policy Boundary (Guardrails)           │
│  Capability tokens · Resource budgets · Audit logs  │
│      Quarantine zones · Approval workflows          │
└──────────────────────────┬──────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────┐
│            Execution Layer (OpenClaw Core)          │
│  Sub-agent orchestration · Tool security auditing   │
│      MCP processing · Plugin management             │
└──────────────────────────┬──────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────┐
│              Perception & Memory Layer              │
│  Spatial Agent Memory · SpatialRAG · Voxel World    │
│  SLAM · Dynamic obstacle avoidance · Sensor fusion  │
└─────────────────────────────────────────────────────┘
```

### Cognitive Layer

The cognitive layer is powered by pluggable LLMs (Claude, GPT, DeepSeek, etc.). It handles natural language understanding, task decomposition, multi-step planning, and tool selection. Developers can swap or fine-tune the underlying model without changing the rest of the stack.

### Policy Boundary

A strict, fully deterministic policy engine sits between the cognitive layer and the execution layer. It enforces:

- **Capability tokens** — explicitly issued, time-bounded, and revocable permissions for each tool
- **Resource budgets** — limits on compute time, memory, network usage, and monetary cost
- **Audit trails** — append-only logs of every action taken by the agent
- **Quarantine zones** — untrusted external data is isolated before influencing trusted memory

No LLM inference can bypass this layer.

### Execution Layer (OpenClaw Core)

The execution layer is provided by OpenClaw — a general-purpose agentic runtime that gives LLMs the ability to take real-world actions. Key components include:

- **Sub-agent orchestration** — spawning and coordinating specialized agents for parallel task execution
- **MCP (Multi-Point Collaboration Protocol)** — structured inter-agent communication
- **Plugin system** — extensible tool registry supporting file systems, browsers, APIs, hardware SDKs, and more
- **RosClaw bridge** — optional ROS2 integration via `rosbridge_server` and DDS for Nav2/MoveIt2-based robots

### Perception & Memory Layer

OEA introduces **Spatial Agent Memory** — a persistent, queryable world model built from sensor streams:

- **Voxelized world representation** — the environment is encoded as spatial cubes (voxels), each carrying a vector embedding and a semantic label
- **SpatialRAG** — retrieval-augmented generation over a multi-dimensional vector store containing objects, rooms, geometry, timestamps, images, and point clouds
- **Sensor fusion** — combines LiDAR, stereo cameras, RGB cameras, and odometry
- **SLAM support** — simultaneous localization and mapping without ROS dependency

This enables robots to answer questions like *"Where did I last see my keys?"* or *"Who entered the room on Monday?"* from long-term memory.

---

## Key Features

| Feature | Description |
|---|---|
| **Hardware-agnostic** | Works with humanoid robots, arms, quadrupeds, drones, and simulated environments |
| **LLM-agnostic** | Pluggable model backends — Claude, GPT, DeepSeek, and more |
| **ROS-optional** | Full SLAM and obstacle avoidance without requiring ROS; ROS2 bridge available |
| **Spatial memory** | Persistent spatiotemporal world model queryable by object, room, geometry, and time |
| **Security-first** | Deterministic policy engine with capability tokens, budgets, and audit logs |
| **Natural language control** | Issue commands like `forward 1m` or `pick up the red cup` via any messaging client |
| **Open source** | Fully open-source and community-driven |

---

## Supported Platforms

- Unitree G1 humanoid robot
- Robotic arms (Python-scripted control via LLM-generated code)
- Quadruped robot dogs
- Drones
- Any hardware exposing a sensor pipeline (LiDAR, stereo camera, RGB-D)

---

## Contributing

Contributions are welcome. Please open an issue or submit a pull request. See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## License

This project is licensed under the Apache 2.0 License. See [LICENSE](LICENSE) for details.
