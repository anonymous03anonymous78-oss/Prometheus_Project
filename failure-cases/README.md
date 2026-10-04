# Failure Cases and Mitigation Plan — The Living Map

TSYP14 · IEEE RAS × IEEE AESS Tunisia Section Chapters Technical Challenge

📄 **[Failure_Cases.pdf](Failure_Cases.pdf)** (7 pages) · [open the PDF directly](Failure_Cases.pdf?raw=true) if the preview on GitHub does not show

## Summary

The Living Map is meant to keep working when parts of it fail. This document goes through **18 failure cases**. For each one it gives the failure, its consequence and the mitigation, and for most of them a validation test. The common principle is to **avoid single points of failure**: information is stored progressively and redundantly in the beacons, sensing, communication and spatial memory are kept separate, and several Writers and entrances are used, so that the system degrades step by step instead of stopping after one failure.

## The 18 failure cases

| # | Failure | Main mitigation |
|---|---|---|
| | **Robots** | |
| 1 | Writer robot failure | Information is passed to nearby beacons during exploration; several Writers; another Writer or the Executor continues from what is preserved |
| 4 | Writer disconnected from the beacon network | Local buffer of unsent events, retransmission when the link returns, a new beacon when link quality drops |
| 14 | Robot battery depletion | Continuous battery monitoring, return cost included in decisions, energy reserved to return, recharging at the Outside Network stations |
| 15 | Partial exploration | All collected information kept; explored and unexplored regions recorded; another Writer continues from the remaining frontier |
| 16 | Executor cannot reach the target | Local re-planning around obstacles; the map is a reference, not immutable; new information kept for later missions |
| 17 | Complete communication loss | Keep working on local information, buffer new events, reconnect to the nearest known beacon, return toward the last communication region if unsafe |
| | **Beacon network** | |
| 2 | Beacon failure | Overlapping coverage, detection of missing acknowledgements, replacement beacon, neighbouring beacons keep the stored information |
| 3 | ESP-NOW communication loss | Link-quality monitoring, acknowledgements and retransmissions, extra beacons, information kept until delivery is confirmed |
| | **Localization** | |
| 5 | SLAM / localization failure | Multi-sensor fusion, localization confidence, beacon references as extra constraints, re-localization |
| 6 | Incorrect frame translation | Carefully surveyed entrance reference, transformation stored in the beacons, checks against known points, uncertainty on coordinates |
| | **Victim detection** | |
| 7 | Computer-vision false detection | Confidence stored, extra observations, fusion with RF sensing, later robots verify uncertain detections |
| 8 | Computer-vision missed detection | Wi-Fi CSI as a complementary modality, RF detections point to regions to investigate, other viewpoints |
| 9 | CSI false positive | Confidence, environmental baselines, extra measurements, camera check, classifier trained on mine data |
| 10 | CSI false negative | Computer vision stays independent, several separated CSI nodes, node placement adjusted, CSI never the only detector |
| 11 | CSI infrastructure failure | Distributed nodes, Writers and beacons as Wi-Fi sources, ESP-NOW independent of CSI, fallback to vision |
| | **Outside infrastructure** | |
| 12 | Outside Network station failure | Several stations, solar + battery power, information kept in the beacon network until the station is back |
| 13 | Optical-fibre failure | Information kept in the beacon network, loss detected, link restored, another entrance/station used |
| | **System** | |
| 18 | Multiple simultaneous failures | No single point of failure, redundant storage, separate functions, several Writers and entrances; validated with progressively harder scenarios |

## Tested in the simulation

The Phase 1 simulation in this repository reproduces several of these cases:

| Failure case | In the simulation |
|---|---|
| 1 Writer failure | Scenario `writer_destroyed`: a roof collapse silences Writer B. Everything it found is already in the beacons; after 3 missed heartbeats the server declares it lost and releases its unexplored branches, which Writer A takes over |
| 2, 3 Beacon failure, communication loss | Scenario `beacon_knockout`: a rockfall destroys a relay beacon on the Executor's route. The neighbour beacon drives back to re-link the chain, reports the lost beacon, and forwards the messages it was holding (store-and-forward) |
| 4 Writer disconnected | A Writer drops a relay beacon when the link to the last beacon weakens, before losing it |
| 5 Localization failure | Robustness runs with wheel slip, lag, terrain yaw and lidar noise, and with rough rock walls; robots re-anchor their position at the beacons |
| 6 Frame translation | Each entrance's first beacon has a surveyed global position; the stations translate every record to GPS coordinates; the 2D window shows each robot's estimate next to its true position |
| 7, 9 False detections | Every detection carries its confidence; the Executor checks each victim hypothesis with its own camera and reports NOT FOUND if nobody is there. Scenario `false_positive` shows the same verification for a gas false alarm |
| 8, 10 Missed detections | Camera and Wi-Fi CSI are independent: a CSI-only detection is a region the Executor searches; a miner who does not move (weak CSI signature) is found by the camera |
| 14 Battery depletion | Battery model on every robot; the frontier utility includes energy and rejects frontiers the battery cannot cover; robots recharge at the stations; the drones fly home before their battery runs out |
| 15 Partial exploration | The Executor is sent before exploration is complete; a hypothesis no beacon trail reaches is reported for the next mission instead of blocking the end |
| 16 Executor cannot reach the target | The Command Post updates the Executor's route when the beacon trail changes |
| 18 Multiple failures | Scenario `all`: Writer loss, beacon loss and a false alarm in the same mission |

Cases 11 (CSI infrastructure), 12 (station) and 13 (optical fibre) are not simulated in Phase 1.
