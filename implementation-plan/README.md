# Implementation Plan — The Living Map

TSYP14 · IEEE RAS × IEEE AESS Tunisia Section Chapters Technical Challenge

📄 **[Implementation_plan.pdf](Implementation_plan.pdf)** (9 pages) · [open the PDF directly](Implementation_plan.pdf?raw=true) if the preview on GitHub does not show

## Summary

The system is built by **incremental integration**, not all at once: each subsystem is first implemented and tested on its own, then integrated with the others once it meets its requirements. Work goes from simulation and single components to a complete multi-robot demonstration, and every stage produces measurable results that validate the next one.

**Development stages:** environment and simulation → localization and SLAM → autonomous exploration → beacon communication network → persistent spatial memory → computer-vision victim detection → Wi-Fi CSI sensing → multi-modal event detection → Outside Network integration → Executor navigation → multi-robot coordination → failure testing → full-system validation.

## Contents

| Section | Objective | Validated by |
|---|---|---|
| 2 Environment and simulation | A controlled model of the mine before testing hardware: tunnels, intersections, dead ends, entrances, obstacles, victim locations, no GPS, several entrances, communication constraints | Exploration from unknown conditions; comparison with ground truth; explored area, time, localization error, path length |
| 3 Localization and SLAM | Each Writer estimates its position and builds a map without GPS (LiDAR, IMU, wheel encoders, sensor fusion) | Estimated vs reference trajectories; accumulated error in corridors, turns, intersections, loops, uneven ground, sensor degradation |
| 4 Autonomous exploration | Explore an unknown mine without a map: frontiers, graph of regions, modified DFS, utility function (distance, information gain, accessibility, traversal cost, relevance) | Corridors, branches, dead ends, blocked paths; % explored, time, distance, revisits, regions left |
| 5 Beacon communication network | ESP-NOW between Writers, beacons and Executors; compact protocol with beacon and robot identifiers; forwarding; acknowledgements and retransmissions | Range, packet loss, latency, forwarding rate, recovery after a node failure |
| 6 Persistent spatial memory | Information survives the Writer that collected it: records with event ID, type, position, timestamp, confidence, source robot, detection modality | Remove the Writer, then check that another robot retrieves the event at the right place |
| 7 Computer-vision victim detection | Direct visual detection with a confidence, tied to the Writer's SLAM pose and stored as an event record | Lighting, occlusion, orientation, distance; accuracy, false positives and false negatives |
| 8 Wi-Fi CSI presence detection | Level-1 human presence from ESP32-S3 CSI nodes, with Writers or beacons as Wi-Fi sources, kept separate from ESP-NOW | Open space → room → tunnel → obstructed tunnel → mine; detection rate, false positives and negatives, latency, sensitivity to node placement |
| 9 Multi-modal event detection | Combine vision, CSI and other sensors without trusting any single one; merge observations of the same event | Each modality alone, then together: does fusion reduce false detections? |
| 10 Frame translation | Turn the Writer's local position into global coordinates through the entrance/reference beacon | Known reference points; error at different distances and along different paths |
| 11 Outside Network | Stations (computer, parabolic antennas, solar panels, battery, robot charging, optical fibre to the first beacon) bridging the mine and the Command Post | — |
| 12 Command Post and global database | Past missions (results, events, maps, failures, robot performance) improve future ones | — |
| 13 Executor robot | Reach the target from preserved information: briefing, beacon network for localization and navigation | Remove the Writer, deploy the Executor, check that it rebuilds the route |
| 14 Multi-robot integration | Several Writers through different entrances, fast and slow Writers, Executors dispatched by the Command Post | Single vs multiple Writers vs fast + slow: time, area, events, information transferred, Executor success |
| 15 Full-system validation | The whole mission: entrances → Writers → exploration → beacons → events → Outside Network → Executor mission → controlled failures → resilience | — |

## Progress: what the Phase 1 simulation already covers

Stage 1 of the plan is the simulation in [simulation-demo](../simulation-demo/) (ROS 2 Humble + Gazebo Fortress). It already runs a simulated version of most later sections, so they can be checked before the hardware exists:

| Section | In the simulation |
|---|---|
| 2 Environment | A mine with four entrances (A and B clear, C flooded, D collapsed), branches, dead ends, obstacles and four miners; no GPS inside; radio blocked by rock; each robot's estimate shown next to its true position |
| 3 Localization | On-board estimation from odometry, IMU and lidar, its error measured against the true pose; robustness runs with wheel slip, lag, terrain yaw, lidar noise and rough walls |
| 4 Exploration | Frontier detection, modified DFS over the junction graph, utility-based frontier choice, recomputed as the map grows (shown live in the 2D window) |
| 5 Beacon network | ESP-NOW radio model between Writers, beacons and the Executor; 32-byte protocol with beacon and robot identifiers; forwarding; recovery when a beacon is destroyed |
| 6 Spatial memory | Records with event ID, type, position, timestamp, confidence and source stored in the beacons; a lost Writer's finds are still used by the Executor |
| 7, 8, 9 Detection | Simulated camera detections with a confidence; Wi-Fi CSI presence from ESP32-S3 nodes dropped by the Writers; both fused into one victim hypothesis |
| 10 Frame translation | Entrance beacons with surveyed global positions; every record translated to GPS coordinates |
| 11, 12 Outside Network, Command Post | Three stations with solar panels, battery, chargers and fibre; a mission server that stores each mission and passes lessons to the next one |
| 13 Executor | Briefed from stored records, follows the beacon trail, confirms each victim with its camera |
| 14 Multi-robot | Two ground Writers through two entrances, two flying fast Writers, shared junctions, Executor dispatched while exploration continues |
| 15 Full system | The whole sequence in one run, with controlled failures: Writer loss, beacon loss, false alarm |

The hardware-specific measurements (real sensor data, a trained vision model, real CSI captures, radio range and packet-loss measurements) come with the later stages of the plan.

## Related documents

- [Technical Report](../Technical%20Report/): the technical report
- [failure-cases](../failure-cases/): the 18 failure cases and their mitigations
- [technical-architecture-diagrams](../technical-architecture-diagrams/): diagrams of the system architecture
