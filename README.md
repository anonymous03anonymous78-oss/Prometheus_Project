# The Living Map — Spatial Memory for Emergency Robots

TSYP14 · IEEE RAS × IEEE AESS Tunisia Section Chapters Technical Challenge

Robots that work where GPS cannot reach may lose communication or break down before they return, and what they learned is lost with them. **The Living Map** keeps that knowledge inside the hazardous area itself. Writer robots (a slow ground robot that maps and a fast flying one that looks for victims) explore an unknown mine and record what they find in a mesh of radio beacons they deploy as they go. An Executor robot later uses this stored information to reach the victims, even if the Writer that found them is gone. Outside Network Area stations link the mine to the Command Post, and every mission is kept on a server to help plan the next ones.

```
Writers explore → events stored in the beacons (ESP-NOW) → optical fibre → Outside Network Area station
→ Command Post + mission server → Executor briefed → follows the beacon trail to each victim
```

## Repository

| Folder | Contents |
|---|---|
| [Technical Report](Technical%20Report/) | The technical report (PDF): problem, chosen environment (mines and tunnels), technical solution, failure cases, implementation and validation plan |
| [failure-cases](failure-cases/) | The 18 failure cases with their consequences, mitigations and validation tests, and which ones the simulation reproduces |
| [implementation-plan](implementation-plan/) | The implementation and validation plan |
| [technical-architecture-diagrams](technical-architecture-diagrams/) | Diagrams of the system architecture |
| [simulation-demo](simulation-demo/) | The Phase 1 simulation (ROS 2 Humble + Gazebo Fortress): code, worlds, tests and how to run it |

## Key ideas

- **No map, no GPS inside:** SLAM, frontier-based exploration, a modified depth-first search and a utility function choose where to explore next.
- **Memory in the mine:** every beacon stores the events (identifier, type, position, timestamp, confidence, source), so nothing is lost when a robot fails.
- **Global coordinates:** the first beacon of each entrance has a known GPS position, so every event gets real coordinates.
- **Two ways to find victims:** computer vision and Wi-Fi CSI presence sensing (ESP32-S3 nodes + AI), fused into one victim hypothesis.
- **Executor without waiting:** it is sent as soon as a reachable victim is on the map, most confident first, then the nearest.
- **Built to fail gracefully:** no single point of failure; the system degrades step by step instead of stopping.
