# The Living Map — Phase 1 simulation (v3.2.1)

TSYP14 IEEE RAS × AESS challenge: a multi-robot spatial-memory system for GPS-denied mines.
v3.2 follows the team's **technical report** section by section (see [Technical report → simulation](#technical-report--simulation)). v3.2.1: shorter Executor stops at the victims, and an unmistakable end of the run (see [How do I know the simulation has ended?](#how-do-i-know-the-simulation-has-ended)).
Two slow ground Writers explore the unknown mine (**frontier-based exploration + modified DFS + a utility function**), each with a fast flying writer (Flix) that also inspects what ground robots cannot reach (the flooded portal C). Victims are detected by **computer vision** and **Wi-Fi CSI** (ESP32-S3 sensing nodes + AI presence classifier), and the two are **fused into victim hypotheses**. Every beacon stores event identifier, type, position, timestamp, detection confidence and source. Three **Outside Network Area stations** (computer, parabolic antenna, solar panels + battery, optical fibre) share one **mission server** that learns from past missions; the Executor is briefed as soon as a victim hypothesis is on the map and visits them **most confident first, then the nearest**.

## Documents

This folder is the simulation. The project's documents are in the other folders of this repository:

| Folder | What it covers |
|---|---|
| [Technical Report](../Technical%20Report/) | The technical report (PDF): the problem and the choice of environment (mines and tunnels), the technical solution, failure cases and resilience, the 8-stage implementation and validation plan |
| [failure-cases](../failure-cases/) | The 18 failure cases with their consequences, mitigations and validation tests, and which ones this simulation reproduces |
| [implementation-plan](../implementation-plan/) | The implementation and validation plan |
| [technical-architecture-diagrams](../technical-architecture-diagrams/) | Diagrams of the system architecture |

### Where this simulation fits in the report

The report's validation plan (section 4) starts with **Stage 1 — Simulation**: model the mine entrances, branches, obstacles and event locations, and validate autonomous exploration and mission allocation. This repository is that stage. It also runs the later stages in simulation, so each one can be checked before the hardware exists:

| Validation plan stage | What the simulation shows |
|---|---|
| 1 Simulation | A mine with four entrances (A and B clear, C flooded, D collapsed), branches, obstacles and four miners; exploration and portal assignment by the Command Post |
| 2 SLAM and autonomy | No GPS inside: the Writers localize with on-board estimation (odometry, IMU, lidar); frontier detection, modified DFS and utility-based frontier choice, shown live in the 2D window |
| 3 Beacon network | ESP-NOW radio model: link quality from distance and rock in the way, a relay dropped when the link to the last beacon weakens, beacon spacing from the radio range, store-and-forward when a link breaks |
| 4 Spatial memory | 32-byte event records (event id, type, position, timestamp, confidence, source) stored in the beacons and read back by the Executor |
| 5 Victim detection | Camera detections with a confidence; Level-1 Wi-Fi CSI presence from ESP32-S3 sensing nodes and an AI classifier; both fused into one victim hypothesis |
| 6 Integrated system | Everything above in one mission, from the hillside sweep to the mission stored on the server |
| 7 Executor | Missions generated from the stored records; the Executor follows the beacon trail to each victim, most confident first, then the nearest |
| 8 Failure testing | Scenarios `writer_destroyed`, `beacon_knockout`, `false_positive` and `all`; robustness runs `LM_PERTURB=1` (wheel slip, lag, terrain yaw, lidar smoke) and `LM_ROUGH=1` (rough rock walls) |

The report is mapped to the code section by section in [Technical report → simulation](#technical-report--simulation).

Stack: **Ubuntu 22.04 · ROS 2 Humble · Gazebo Fortress (ign-gazebo 6)**.

## Quick start

```bash
cd Prometheus_Project/simulation-demo   # this folder (the one with setup.sh)
bash setup.sh            # once: checks versions, installs packages, downloads the SubT tiles, builds, self-tests
bash run.sh all          # Gazebo (left) + 2D dataflow window (right)
```

Scenarios: `nominal`, `writer_destroyed`, `beacon_knockout`, `false_positive`, `all`. Press **Ctrl+C** in the terminal to stop everything. A scenario takes about 11–13 minutes of simulated time.

`run.sh <scenario> [world] [camera] [speed]`:

- **world**: `auto` (default: SubT tiles if they are in the local Fuel cache, otherwise the offline world), `fuel`, `basic`.
- **camera**: `auto` (default: the story camera drives the Gazebo view) or `manual`.
- **speed**: Gazebo real-time factor, e.g. `bash run.sh all auto auto 2` for twice as fast (for recording the demo). Gazebo goes as fast as the PC allows; everything runs on simulated time, so the mission is the same at any speed.

Every run is recorded to `~/living_map_runs/<scenario>_<date>.jsonl.gz`. Every finished mission is also stored in the server's **mission memory** (`~/living_map_runs/mission_memory.json`, see below).

**Robustness runs:** `LM_ROUGH=1` gives the dry-run galleries rough rock walls (bumps 0.2–0.7 m into the gallery every few metres, like the real SubT tiles); `LM_PERTURB=1` adds wheel slip, lag, terrain yaw and lidar smoke. Both are used by `test/test_scenarios.py`.

**Safety net:** `bash dry_run.sh all` runs the same mission nodes and the same 2D window without Gazebo. Add a speed factor to go faster, e.g. `bash dry_run.sh all 3`.

### How do I know the simulation has ended?

The mission is over when the Command Post declares **MISSION COMPLETE**: both Writers are out (parked on their chargers, or lost), and no victim hypothesis is left that the Executor can reach. Three signs, at the same moment:

- **2D window:** a green **✓ MISSION COMPLETE** banner under the header, with the mission time, the victims confirmed by the Executor, the ones reported to the rescue team (no ground access, e.g. the miner beyond the flooded sump), gas events, beacons placed / lost, and "stored in the mission server memory". The last step of the progress bar ("Mission stored") is ticked.
- **Terminal:** a framed `MISSION COMPLETE` block from the `command_post` node with the same summary, ending with *press Ctrl+C in this terminal to stop the simulation*.
- **Gazebo:** ~6 s later the camera cuts to the **cutaway** (hill and tunnel shells removed, top view of the whole mine); the shot's caption in the 2D window reads *MISSION COMPLETE - cutaway … The simulation is over: press Ctrl+C in the terminal to stop it*, and the "3D camera" chip shows *MISSION COMPLETE · cutaway*.

Nothing else happens after that: Gazebo and the nodes keep running (so you can look around the cutaway) until you press **Ctrl+C**.

## The mine and the story

Nothing about the mine's layout is given to the robots. The world has a hillside with **four openings**; only the world builder and the sensor models know them.

1. **Portal discovery.** Flix A and Flix B take off and sweep the hillside face from 3 m away, each covering one half, with their side ToF ranger on the rock.
   - A stretch of "no return" at least 1.5 m wide is an opening. The Flix flies into it with its light (front ToF + camera), reads the portal sign and classifies it.
   - Result: **A clear, B clear, C flooded, D collapsed** (a roof fall 2.5 m inside the old adit D, so it is rejected).
2. **Dispatch.** The Command Post sends the **lessons** of the mission server to the Writers, then:
   - one Writer + its Flix per clear portal: **Writer A + Flix A → portal A**, **Writer B + Flix B → portal B**.
   - **Flooded portal C**: no ground robot can go in. **Flix B** (it found it) first inspects it, and the **ONA-C station crew places the station's own fibered first beacon** at the portal (the report: the fibre connects each station to the first beacon of its entrance).
3. **Flix B over the water.** It flies into C **low over the water of the sump** (water surface 2.5 m below the portal sill, from 10 m to 50 m in; Flix B keeps 0.7 m above the surface measured by its downward ToF, well under the roof), with side-ToF centring, to the air pocket. Its camera finds **the miner in the air pocket**. No Wi-Fi source is in reach there, so it is a camera-only detection. It carries the record out and writes it into ONA-C's beacon as soon as it hears it, flagged **not reachable by ground robots** (the rescue team needs divers or pumping). Then it joins Writer B.
4. **Writers explore** (frontiers + modified DFS + utility function, see below). Each drops a **fibered entrance beacon** at its portal, then **junction beacons (each junction is a mapping event)**, relay beacons (when the link to the last beacon drops significantly, or on the spacing guard set by the ESP-NOW range) and event beacons (victim, hazard). They share one topological map through the beacons, so they never explore the same branch twice. Each carries **10 beacons** and **8 ESP32-S3 Wi-Fi CSI nodes**. When its exploration is complete or the stock is empty, a Writer leaves by the **nearest exit** and parks on its **ONA station's charger**.
5. **Fast writers.** Each Flix flies ahead of its Writer along the gallery the Writer is exploring, never beyond radio reach of a routed beacon.
   - At every junction it spins once: its **camera** looks down every branch, and its ESP32-S3 works as a **flying Wi-Fi CSI node** (CSI of its links to the nearby beacons and its Writer → presence classifier for that region).
   - When its camera sees a person it flies there for a closer look (the detector confidence grows), takes a CSI window, and writes the **VICTIM record** (and PRESENCE if the CSI classifier agrees) into the nearest beacon. The person is on the living map before the slow Writer arrives.
   - It flies home **along the beacon trail's "toward the exit" pointers** before its battery runs out, and recharges on its pad (powered by an ONA station).
6. **Victim detection and fusion.** Camera finds (Writers, Flix: detection confidence) and Wi-Fi CSI presence (aggregator beacons, Flix: region + confidence) are records in the beacons. The Command Post combines the observations of one person into a single **victim hypothesis** (best confidence per modality, the two modalities fused as independent evidence).
7. **Executor.** It is briefed **as soon as a victim hypothesis is on the map and the trail reaches it**, while the Writers are still exploring. It follows the beacon trail (RF gradient + the beacons' coordinates) and re-measures unconfirmed gas events on the way. At each hypothesis its camera identifies the person (a CSI-only hypothesis is a region: it searches it), it confirms the detection and delivers first aid (oxygen self-rescuer + two-way radio, ~5 s). It drives at full speed toward a person its camera sees and only slows down for the last 4 m. Nobody there → NOT FOUND. A person seen from afar by a Flix before any Writer got there is reachable from the nearest beacon in line of sight (≤ 25 m): the Executor covers the last metres on its camera. The Command Post then sends it to the next one: **most confident first (5 % classes), then the nearest**. The next briefing goes out right after the confirmation, so the Executor receives it during the first aid and leaves as soon as the aid is done. If no other hypothesis is reachable yet, it stays with the miner until the Writers put one on the map (or until the mission is complete).
8. **Mission complete**: the mission is stored in the mission server's memory for the next mission, at any ONA. A hypothesis no ground robot can reach (C), one without a beacon trail, or one below the confidence threshold stays on the map and is reported, instead of blocking the end of the mission. The end is shown in the 2D window, the terminal and Gazebo (see [How do I know the simulation has ended?](#how-do-i-know-the-simulation-has-ended)).

The miners (truth): **V1** in the south branch (moving), **V2** in the gas branch (breathing, not moving), **V3** at the end of the east gallery (still: the CSI channel barely changes, so the camera finds him), **V4** in the air pocket beyond the flooded sump. The Command Post numbers hypotheses in the order it hears about them, so its V1, V2 … can differ from these labels.

### Camera sequence (every scenario)

The Gazebo camera follows the story and the 2D window shows a caption for every shot. The 3D view is Fortress's GzScene3D with a follow gain of 0.08 (`<camera_follow><p_gain>` in the world): the camera glides about 1.5 m behind the robot it follows. (The MinimalScene + CameraTracking view has its gain fixed at 0.01 per frame, so the camera fell behind and crept after the robot in small steps.)

1. Intro: the site, the three ONA stations, the relay satellite.
2. Flix B sweeping the hillside (it finds B, the collapsed D and the flooded C).
3. ONA-B: GO and the lessons going out through the dish → satellite link (the beam flashes).
4. Flix B flying low over the water of the flooded portal C (up to ~24 s).
5. Writer B exploring, until the Executor is briefed.
6. The Executor, until the mission is complete. While it stays with a miner for more than 10 s because no other hypothesis is reachable yet, the camera follows the Writer still exploring, and comes back to the Executor when it is briefed again. Event cuts on the way (~7–12 s each):
   - a Flix spotting a person with its camera
   - a Writer dropping its first Wi-Fi CSI node; the first CSI presence detection (cyan rings)
   - Writer A taking a branch at a junction mapped by Writer B
   - a roof collapse (still shot: the Writer falls silent)
   - the server releasing a lost Writer's branches (Writer A takes them over)
   - a rockfall, then the neighbour beacon relocating itself
   - Flix B finding the miner beyond the sump
7. **MISSION COMPLETE → cutaway** (~6 s after the end): hill, tunnel shells and the adit removed, top view of the whole site. This is the last shot.

### Scenarios

| Scenario | What happens |
|---|---|
| `nominal` | The full story above. |
| `writer_destroyed` | A roof collapse hits Writer B in the south branch. It **falls silent**. Everything it found is already in the beacons. After **3 missed heartbeats** (45 s; its last STATUS record gives its last position) the server presumes it lost and **releases the branches Writer B had claimed**; Writer A explores them after its own DFS. |
| `beacon_knockout` | Rockfall destroys a relay beacon on the Executor's route once it enters. The orphaned neighbour re-links to another routed beacon or drives itself back to re-link, reports ROUTE_HEALED and BEACON_LOST, and flushes the messages it held. The trail's "toward the exit" pointers bypass the lost beacon. |
| `false_positive` | A gas-sensor glitch on a Writer creates a hazard event. The Executor re-measures it (highest reading around the spot) and it is logged as FALSE POSITIVE. The server stores it, which raises the learned false-alarm rate. |
| `all` | All three failure cases in one run. |

## Writer autonomy: frontier exploration, modified DFS, utility function

The Writer has no map and no route. It only knows its portal (from Flix) and the miners' last known position (last cap-lamp tag ping, from the Command Post). It localises with SLAM (see [Positioning](#positioning-without-false-information)) and runs the loop of the report, shown live in the 2D window:

**Sense → Localize → Map → Identify frontiers → Evaluate → Select → Navigate**

- **Gallery travel (Navigate)**: it follows the gallery axis, centred by the lidar.
- **Junction (Map, Identify frontiers)**: side openings seen by the lidar for 3 consecutive scans. The Writer creeps to the side gallery's centre line (measured from that gallery's two walls), then reads every exit (forward, left, right):
  - `open` = a **frontier**: unexplored gallery beyond (the lidar sees deeper than 10.4 m, or any doubt: a stub costs ~30 s, a missed branch can cost a miner).
  - `seen`: every lidar ray in a ±12° sector toward it ends on a wall closer than 10.4 m, so it is a short stub explored by sight.
  - The way it came in is `done`.

  It drops a **junction beacon** storing this table: **a junction is a mapping event** (JUNCTION record), not a victim or hazard event.
- **Evaluate + Select (utility function)**: every open exit is scored

  `U = 1.0·gain + 1.5·relevance − 0.8·distance − 0.6·energy + 0.5·accessibility`

  | Term | From |
  |---|---|
  | expected information gain | 0.6 + 0.4 × (lidar depth seen into the exit / 12 m) |
  | mission relevance | toward the last known position (0–1) + the server's survivor prior + **people on the map no Writer has reached yet** (seen only by a Flix or by CSI) |
  | distance | path length to the exit in the junction graph / 50 m (0 at the current junction) |
  | traversal energy cost | (distance + ~20 m of new gallery + way back to the nearest portal) × energy per metre / battery left. Not feasible on the battery → not a candidate |
  | accessibility | exit width from the lidar (a wide footprint passes: 1, narrow: 0.6), −0.3 toward a known hazard |

  The best exit is **claimed** and explored. The 2D window (Writers table) shows every evaluation.
- **Modified DFS**: a **dead end** (rock ahead and no side opening, held for 1 s of scans so dust or smoke can't fake one) marks the branch `done` and the Writer backtracks to the junction. When nothing is open at a junction it backtracks to the previous one (DFS stack). If nothing further down its stack is open, it skips the useless walk back and evaluates the open exits of the whole shared map with the same utility function (now the distance counts).
- **Shared map**: every junction table, with its claims and the links between junctions, is flooded through the mesh. A Writer arriving at a junction found by the other one re-anchors its estimate on that beacon and takes only exits nobody claimed.
- **Beacon budget**: 10 relay beacons. A spacing beacon that is due waits up to 6 m in case a junction comes, whose beacon then does the job. When the stock is empty the Writer leaves by the nearest exit.
- **Wi-Fi CSI nodes**: 8 ESP32-S3 nodes, dropped at each new junction, at each branch end, and every 8 m within 22 m of the last known position (denser sensing where people are expected).

## Victim detection: computer vision, Wi-Fi CSI, fusion

- **Computer vision.** Writers (RGB-D camera, 4 m) and Flix (thermal + RGB camera, 25 m, ±70°) run a person detector. The detection is placed at the robot's SLAM position + range/bearing, and its **confidence** is stored with the event (the score falls with range; the Writer pauses 1 s so several frames are fused, the Flix flies closer).
- **Wi-Fi CSI (Level-1 human presence).** The CSI infrastructure is **independent of the ESP-NOW topology**:
  - Wi-Fi sources: every relay beacon (ESP32) and the Writers' own ESP32-S3.
  - Receivers: the **ESP32-S3 CSI nodes** dropped by the slow Writers (density can be raised locally without densifying the beacon chain), and the Flix while it hovers (flying CSI node).
  - Each node streams its CSI to an **aggregator beacon** (the nearest beacon in Wi-Fi reach) which runs the **AI presence classifier** every 2 s window. Presence in 2 consecutive windows ≥ 60 % → a **PRESENCE record**: a **region** (where the disturbed links are; Phase 1 does not localise the person), its radius, the number of disturbed links and the confidence.
  - Channel model: a link is disturbed by a person within ~2.5 m of it (tunnel multipath), in proportion to how much the body moves (walking ≫ breathing ≫ still), plus noise. A still, unconscious person is barely seen by CSI: the camera finds him.
- **Fusion (Command Post).** Each observation carries event type, region/position, timestamp, confidence and source. Observations of the same person (a camera find inside a CSI region, or two finds within 5 m) are combined into **one victim hypothesis**: best confidence per modality, then `fused = 1 − (1 − camera)(1 − CSI)`. A camera find localises a CSI-only hypothesis. The table in the 2D window shows every observation with its source and confidence.

## Beacon packet (32 bytes, little-endian, packed)

Every record holds what the report asks a beacon to store: **event identifier, event type, position, timestamp, detection confidence, source / Writer identifier**, plus **where to go** (toward the event and toward the exit).

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | beaconID | subject: beacon id (1–49 Writer A, 51–99 Writer B, 100 ONA-C) · CSI nodes 110–149 · robots 0xFE Writer A, 0xF9 Writer B, 0xFD Executor, 0xFC Flix A, 0xF8 Flix B |
| 1 | 1 | eventType | PLACEMENT, VICTIM, HAZARD, ENTRANCE, PRESENCE, EXPLORATION_COMPLETE, VICTIM_CONFIRMED, FALSE_POSITIVE, ROUTE_HEALED, BEACON_LOST, JUNCTION, WRITER_EXIT, STATUS, CSI_NODE |
| 2 | 1 | parentID | radio uplink neighbour (0 = fibre to an ONA station) |
| 3 | 1 | flags | bits 0–3: 0x01 stored-forward, 0x02 relocated, 0x04 not reachable by ground, 0x08 fused · **bits 4–7: source / Writer identifier** (0 beacon network, 1 Writer A, 2 Writer B, 3 Flix A, 4 Flix B, 5 Executor, 6 ONA station) |
| 4 | 4 | localX | float32, m (Writer's SLAM frame, tied to the entrance beacon) |
| 8 | 4 | localY | float32, m |
| 12 | 2 | localZ | int16, cm (altitude for the GPS translation) |
| 14 | 2 | heading | int16, centi-degrees |
| 16 | 1 | valueA | HAZARD: CH₄ in 0.1 % · VICTIM: range to the person, dm · PRESENCE: region radius, m · JUNCTION: exits E,N,W,S (2 bits each) · STATUS: beacons left |
| 17 | 1 | valueB | HAZARD: O₂ in 0.1 % − 150 · VICTIM: camera (1 RGB-D, 2 thermal) · PRESENCE: disturbed CSI links · STATUS: explored / 2 m |
| 18 | 1 | **conf** | **detection confidence 0–100 %** (STATUS: battery %) |
| 19 | 1 | nextIn | **where to go**: next beacon toward the most confident victim deeper in the mine (0 = here) |
| 20 | 2 | bearingIn | int16 centi-degrees toward nextIn / the event |
| 22 | 2 | distIn | uint16 dm, distance along the trail to the event |
| 24 | 1 | nextOut | **escape route**: next beacon toward the exit (0 = portal) |
| 25 | 4 | utc | uint32, UTC seconds (**when** it was written) |
| 29 | 2 | utcMs | uint16, milliseconds |
| 31 | 1 | **eventID** | **event identifier**, numbered by its source: 1–127 the robot's own records, 128–255 the records of its CSI network (aggregator beacons). (source, eventID) identifies an event; re-broadcasts keep it |

```c
typedef struct __attribute__((packed)) {
  uint8_t  beaconID, eventType, parentID, flags;     // who / what (flags high nibble = source)
  float    localX, localY;                           // where it is (m)
  int16_t  localZ_cm, heading_cdeg;
  uint8_t  valueA, valueB, conf;                     // what was found + detection confidence %
  uint8_t  nextIn; int16_t bearingIn_cdeg; uint16_t distIn_dm; uint8_t nextOut;   // where to go
  uint32_t utc; uint16_t utcMs; uint8_t eventID;     // when + event identifier
} BeaconMsg;                                         // sizeof == 32 (ESP-NOW limit 250)
```

- **Spatial memory.** A beacon keeps every record written into it (its own event, a Flix find, a CSI presence from its classifier, the Executor's confirmation) and **re-broadcasts them every 10 s with their original timestamp and event ID** (message aging: receivers compute `age` and `freshness = exp(-age/300 s)`). They stay available if their Writer fails.
- **"Where to go" is maintained by the mesh.** `nextOut` is set by the Writer that drops the beacon: it is the beacon it last passed. The trail tree is searched and every beacon points (`nextIn`) to the child leading to the most confident victim, then hazards, then the nearest. When a beacon is lost, its neighbours bypass it in their `nextOut`.

## Outside Network Areas, satellite and mission server

```
Writer A ─drop─▶ beacons … ▶ entrance beacon A ══fibre══▶ ONA-A ═╗
Writer B ─drop─▶ beacons … ▶ entrance beacon B ══fibre══▶ ONA-B ═╬══ dish ⇄ relay satellite ⇄ CP dish ══▶ Command Post + mission server
Flix A/B ─ESP-NOW─▶ nearest routed beacon (same chain)             ║        (LTE mast = backup)
Flix B (line C) ─ESP-NOW─▶ ONA-C's first beacon (portal C) ═fibre═▶ ONA-C ═╝
CSI nodes ─Wi-Fi CSI─▶ aggregator beacon (AI classifier) ─ESP-NOW─▶ … same chain
Executor ◀──RF── beacon k ◀ … ◀ entrance beacon ◀══fibre══ ONA ◀══ satellite ◀══ Command Post (binary briefing)
         (at the staging area: directly from the ONA's outside radio)
```

- Each **ONA station**: communication computer (control console), parabolic antenna pointed at the relay satellite, hub cabinet, **solar panel array + high-capacity battery cabinet**, a **robot charger**, and the optical-fibre cable to the **first beacon of its portal** (A, B: dropped by the Writers; C: placed by the station crew at the flooded portal). Stations decode the 32 bytes, translate to **GPS lat/lon/altitude** and relay to and from the satellite. There is no direct link between the robots in the mine and the Command Post.
- **Frame translation.** Each Writer runs its SLAM in its own map frame, initialised at the staging area and tied to its **entrance beacon**, whose position in the global frame is surveyed at the portal (RTK-GPS). Every beacon stores the Writer's position along the chain; the station translates every record to global GPS coordinates, and the Executor uses the same reference from the known entrance toward the target.
- **Energy.** The solar panels charge the battery bank (daytime model: Gafsa, October morning), which powers the station and the chargers. Writers park on their station's charger after the mission; the Flix pads are powered by ONA-B. The 2D window shows solar power, battery state and who is charging.
- In Gazebo, every uplink or downlink through an ONA **flashes the beam** dish → satellite → Command Post dish.
- The **mission server** (next to the Command Post) is shared by all ONAs. It stores every finished mission and, before a new one, turns all of them into **lessons**:

| Learned from past missions (every ONA) | Estimate | What changes |
|---|---|---|
| Rockfall zones (beacons lost) | Gamma–Poisson rate per zone | Writers space beacons every 11 m instead of 18 m there |
| Roof-collapse zones (Writers lost) | Gamma–Poisson rate per zone | Writers drop a beacon before the zone; silence watchdog 3 → 2 missed heartbeats (45 → 30 s) |
| CH₄ sensor false alarms | Beta–Binomial P(false alarm) | above 0.15: Writers confirm with a second sample; the Executor re-measures |
| Where survivors were found | frequency per zone | mission relevance term of the utility function |
| Executor speed underground | mean | ETA in every briefing |

The memory file is `~/living_map_runs/mission_memory.json`. If it does not exist, the server starts from `memory_seed.json`, which holds three simulated missions recorded with this package. Use `LM_MEMORY=off` for a run without memory (the test suite does), `LM_MEMORY=seed` to use the seed without writing, or `LM_MEMORY=/path/file.json` for another file.

## Positioning without false information

No robot navigates on simulator truth. Truth is used only for the sensor models listed below, radio physics, and the "error" display in the 2D window. Each robot runs its own EKF.

| Robot | Sensors fused |
|---|---|
| Writers / Executor | wheel encoders (DiffDrive, skid-steer slip rejection) · IMU gyro · visual odometry (model) · 2D lidar scan matching (ICP) · gallery lock (on a gallery of the shared map, between two linked junction beacons, the lock is on the mapped gallery line itself, so heading and cross-gallery position cannot drift) · RTK-GPS outside · own-beacon loop closure · re-anchoring on junction beacons (also the other Writer's: map merge) |
| Flix A/B | gyro (integrated at every sample, bias calibrated on the pad and kept out of position fixes) · optical flow (model, with a scale-drift term; noisier over water) · downward ToF (height above the floor, or above the water surface in the flooded sump) · side ToF against the gallery its Writer mapped (lateral lock, also between two mapped junctions while the Writer backtracks; nose held along the gallery) · on the way home: gallery heading held between two trail beacons + side-ToF lock, RF range fix when a beacon is still metres ahead, positions of beacons it never heard taken from the Writer's shared junction map · beacon fixes at the closest approach (signal peak) when it passes a beacon · if it stalls at a corner it does not expect, a short spiral around the nearest beacon to find its signal peak and re-anchor · RTK-GPS outside |

Robots broadcast their own estimate on the radio: when two ground robots meet in a gallery, both **keep right** to pass each other.

## Package layout

```
src/living_map/
  living_map/config.py          ← geometry (truth), radio, hazards, miners, scenarios (single source of truth)
  living_map/packet.py          ← 32-byte beacon packet, briefing, GPS translation
  living_map/estimation.py      ← EKFs, ICP scan matching, gallery lock
  living_map/geometry.py        ← line of sight, radio model, Wi-Fi CSI channel + classifier, fusion, relocation planner, Dijkstra
  living_map/ground.py          ← shared ground-robot machinery (fusion, lidar steering, keep-right, battery)
  living_map/writer_node.py     ← Writer A / Writer B (frontiers, modified DFS, utility, CSI nodes)  (robot:=writer_a|writer_b)
  living_map/flix_node.py       ← Flix A / Flix B (discovery, flooded portal, fast writer, flying CSI node)  (robot:=flix_a|flix_b)
  living_map/executor_node.py   ← Executor (trail following, target identification, first aid)
  living_map/beacon_network.py  ← beacon firmware + RF medium (stored records, junction tables, where-to-go, CSI aggregation)
  living_map/outside_network.py ← ONA stations A/B/C (ona:=A|B|C): GPS translation, energy, ONA-C first beacon
  living_map/command_post.py    ← Command Post + mission server (memory, lessons, detection fusion, briefings)
  living_map/environment.py     ← mission sensors (gas, camera person detector with confidence) + incidents
  living_map/director_node.py   ← Gazebo camera story + captions + cutaway
  living_map/gz_actions.py      ← spawns/moves/removes models, satellite beams, CSI presence pulses
  living_map/dataflow_view.py + web/index.html   ← 2D window (http://localhost:8765)
  living_map/dry_run.py + fake_ros.py            ← no-Gazebo run / test harness
  living_map/memory_seed.json   ← past missions the server starts from
  launch/sim.launch.py   worlds/mine.sdf (SubT tiles)   worlds/mine_basic.sdf (offline)
  models/                ← robot meshes, hillside, terrain, rocks, beacons, CSI node, NASA dish + TDRS, console, LTE mast
  tools/gen_world.py     ← regenerates worlds + generated models from config.py
  test/test_core.py  test/test_scenarios.py
```

If you change `config.py`, run `python3 src/living_map/tools/gen_world.py`, then `colcon build --symlink-install`.

Tests: `python3 src/living_map/test/test_core.py` (seconds) and `python3 src/living_map/test/test_scenarios.py` (all 5 scenarios end to end on the dry-run stand-in; add a scenario name to run one).

## Credits and licences

- **Robotika X2 and Explorer X1 meshes:** DARPA SubT Challenge virtual models (osrf/subt), Apache-2.0. The SubT sensors were replaced by this project's sensors.
- **Flix frame mesh:** github.com/okalachev/flix. The repository does not state a licence in the files used here, so credit it and check with the author before redistributing.
- **Satellite dish ("Tall Dish") and relay satellite (TDRS):** NASA 3D Resources, github.com/nasa/NASA-3D-Resources. Converted from glTF to OBJ (Z-up, scaled; the dish split into mount + reflector so it can point at the satellite). Use follows the NASA media usage guidelines (no endorsement implied).
- **Control console (ONA PC) and radio tower (LTE mast):** osrf/gazebo_models, Creative Commons Attribution 3.0 (licence file copied into each model folder). Converted from COLLADA to OBJ.
- All the new meshes use the same OBJ layout as the v2 meshes (a position, texture coordinate and normal for every vertex), which is what Gazebo Fortress's Ogre 2 renderer loads reliably. The first v3.0 package crashed Gazebo at start-up while importing one of the new meshes (some had no normals).
- **Tunnel Tiles 1/5/6 and Rescue Randy Sitting:** Open Robotics on Gazebo Fuel (downloaded by setup.sh, not bundled).
- The hillside (with the four openings), terrain, rock meshes and textures are generated by `tools/gen_world.py`.

## Technical report → simulation

| Report | In the simulation |
|---|---|
| 2.1 Writers: slow (systematic mapping, deploys and maintains beacons, CV + RF victim detection), fast (rapid victim detection, regions inaccessible or inefficient for ground, CV + RF); SLAM; two event types; store events in beacons; no direct link to the Outside Network | Writer A/B (X2), Flix A/B; HAZARD + VICTIM/PRESENCE records; everything goes through the beacons; Flix B inspects the flooded portal C |
| 2.2 Unknown environment, SLAM, frontiers, modified DFS, utility function (information gain, distance, energy, accessibility, relevance), exploration loop | `writer_node.py` (utility table + loop stage in the 2D window) |
| 2.3 Persistent spatial memory: event id, type, position, timestamp, confidence, source; ESP-NOW; deployment on link drop or relevant event; spacing from the ESP-NOW range | 32-byte record (`packet.py`), stored + re-broadcast records (`beacon_network.py`), weak-link / spacing-guard / event drops |
| 2.4 Frame translation via the entrance beacon | entrance beacon with surveyed global position, ONA translation to GPS |
| 2.5–2.7 CV + Wi-Fi CSI (Level-1 presence, distributed ESP32-S3 nodes, AI classification, region of the Writer, Writers/beacons as sources, independent of ESP-NOW, local density), fusion into one victim hypothesis | `environment.py` (CV confidence), `geometry.py` + `beacon_network.py` (CSI channel, aggregators, classifier), Flix flying CSI node, `command_post.py` (fusion) |
| 2.8 Data flow | sensors → SLAM → events → 32-byte records → ESP-NOW → fibre → ONA → satellite → Command Post → Executor |
| 2.9 ONA stations: computer, parabolic antennas, solar panels, battery, fibre to the first beacon of each entrance; robot recharging; Command Posts ⇄ global server | ONA-A/B/C (`outside_network.py`, Gazebo stations), chargers, mission server |
| 2.10 Executor: mission from stored information, Writer not needed, localization / route reconstruction / target identification, no complete exploration needed | `executor_node.py`, briefed while the Writers explore |
| 3 Failure cases: Writer failure, beacon failure, incomplete exploration | `writer_destroyed`, `beacon_knockout`, leftover hypotheses reported at the end |
| 4 Stage 8 failure tests: writer failure, beacon failure, communication degradation, incomplete exploration, uncertain detection, partial map | scenarios + `LM_PERTURB` / `LM_ROUGH` runs; NOT FOUND / below-threshold hypotheses; briefing before complete exploration |

**Removed in v3.2** because the report does not have them: the ROV and its acoustic link, FTM triangulation at the stations, the ESP32 Long-Range distress burst, the Flix micro-beacons, the server "sortie", the 60 GHz vital-sign radar and the P1/P2/P3 triage. **Kept** (not in the report, not contradicting it): the relay satellite, the "where to go" fields, the Executor's first aid, the self-moving beacons.

## Please check on the first Gazebo run (v3.2)

The mission logic of every scenario is tested end to end on the dry-run stand-in. These Gazebo parts were checked structurally (world parse, unique names) but could not be run in Gazebo in the build environment:

- **New models:** `csi_node.sdf` (dropped by the Writers), `pulse_csi_*.sdf` (cyan rings where a classifier reports presence), the ONA solar panels / battery cabinet / charger pads, the CSI stack on the X2 and the ESP32-S3 board on the Flix.
- **Flix B over the water:** its downward ToF must see the water surface (the water box now has the lidar visibility bit). If Flix B dives into the water in line C, check that `flooded_sump` has `<visibility_flags>3</visibility_flags>` in the world. It flies 0.7 m above the surface (`FLIX_ALT_WATER`). The water level was lowered from −1.5 m to −2.5 m (`WATER_Z`) so that there is at least ~1.5 m of air between the water and the roof of the flat section, even if the real tunnel tiles are only ~4 m high; if the Flix touches the roof there, lower `WATER_Z` further and run `tools/gen_world.py`.
- **Flix portal discovery / wall lock / DFS on real tiles / camera / load / effects:** as in v3.1 (side ToF on the hill at 3 m; the `5.0` wall distance in `gallery_lock` / `trail_lock`; longest-ray exit reading; `FOLLOW_P_GAIN`; `world basic` if the RTF drops; effects can be switched off in `gz_actions.py`).

## Troubleshooting

- **Gazebo window opens empty, then crashes** with `Ogre::SubMesh::importFromV1 … _arrangeEfficient … __assert_fail` in the terminal: a mesh Ogre 2 can't import. Fixed in v3.0.1 (all new meshes rewritten with normals). If it still happens, run `bash run.sh all 2>&1 | tee ~/lm_gazebo.log` and look for `Unable to`, `mesh` or `Error` lines just before the crash: they name the mesh.

- **Windows not side by side.** Click Gazebo and press Super+←, then click the dataflow window and press Super+→.
- **"Address already in use" on port 8765.** A previous run is still alive. Run `pkill -f living_map` and try again.
- **Gazebo opens but the robots don't move.** Check `ros2 topic echo /clock --once` and `ros2 topic list | grep model`. If they're missing: `sudo apt install ros-humble-ros-gz-bridge`.
- **Gazebo reports it can't find a Fuel model.** Run `bash setup.sh` with internet, or use `bash run.sh all basic`.
- **Gazebo is slow (RTF < 1).** Everything runs on simulated time, so the mission still completes, just slower.
- **The behaviour changed between two runs.** That is the mission memory learning from the previous run. Delete `~/living_map_runs/mission_memory.json` to start again from the seed.
- **A Writer is reported lost while it is fine.** The watchdog presumes a Writer lost after 3 missed heartbeats (45 s with no record from it). If your PC runs Gazebo very slowly, raise `WRITER_SILENCE` in `config.py`.
