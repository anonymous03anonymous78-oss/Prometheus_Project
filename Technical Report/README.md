# Technical Report — The Living Map: Spatial Memory for Emergency Robots

TSYP14 · IEEE RAS × IEEE AESS Tunisia Section Chapters Technical Challenge

📄 **[technical report.pdf](technical%20report.pdf)** (5 pages) · [open the PDF directly](technical%20report.pdf?raw=true) if the preview on GitHub does not show

## Summary

Robots that work where GPS cannot reach may lose communication or break down before they return, and what they learned is lost with them. **The Living Map** keeps that knowledge inside the hazardous area itself: Writer robots explore an unknown mine and record what they find (victims, hazards, the layout) in a mesh of radio beacons that they deploy as they go. An Executor robot later uses this stored information to reach the targets, even if the Writer that found them is gone. An Outside Network Area links the mine to the Command Post, and the data of every mission is kept on a global server to help plan the next ones.

The chosen environment is **mines and tunnels**: rock layers cut them off from the outside world, and poisonous gas makes them dangerous for rescuers.

## Contents

| Section | Topics |
|---|---|
| 1 Introduction & System Concept | Problem; selected environment (mines and tunnels) |
| 2 Technical Solution | Writer robots (slow and fast) · autonomous exploration · spatial memory and beacon network · frame translation · victim detection · distributed Wi-Fi CSI infrastructure · victim detection fusion · data flow · Outside Network Area · Executor |
| 3 Failure Cases and Resilience | Writer failure · beacon failure · incomplete exploration |
| 4 Implementation and Validation Plan | 8 stages: simulation → SLAM and autonomy → beacon network → spatial memory → victim detection → integrated system → Executor → failure testing |
| 5 References | ESP-NOW; Wi-Fi sensing and CSI-based presence detection |

## Key ideas

- **Two kinds of Writers.** A slow ground Writer explores systematically, builds the map and deploys the beacons. A fast Writer looks for victims quickly and inspects places that are hard or impossible to reach on the ground. Both detect two event types: hazardous areas and victims.
- **Exploration without a map.** SLAM, frontier-based exploration, a modified depth-first search over the tunnel branches, and a utility function (information gain, distance, energy cost, accessibility, mission relevance) to choose the next frontier.
- **Spatial memory in the beacons.** Each beacon stores event identifier, type, position, timestamp, detection confidence and source. Beacons talk over ESP-NOW and are placed when the link to the last beacon drops significantly or at a relevant event. The information survives the failure of the Writer that produced it.
- **Global coordinates without GPS.** The first beacon of each entrance has a known global position, so the beacon chain turns the Writers' local coordinates into the GPS coordinates the mission needs.
- **Victim detection.** Computer vision with a confidence, plus Wi-Fi CSI (Level-1 presence detection from distributed ESP32-S3 nodes and an AI classifier). The two kinds of evidence are fused into one victim hypothesis.
- **Outside Network Area.** Small stations outside the mine (computer, parabolic antennas, solar panels, battery, optical fibre to the first beacon of each entrance) that bridge the mine and the Command Post and recharge the robots.
- **Executor.** Uses the beacon network for localization, route reconstruction and target identification, and can be sent as soon as enough information exists, without waiting for a complete exploration.

## Implementation

Stage 1 of the validation plan, the simulation (ROS 2 Humble + Gazebo Fortress), is in this repository. Its README maps each section of this report to the code that implements it.
