# Todo

Open items for nav_robot, from what was tried and not finished (2026-11-13). Tick off only when checked in the sim.

## Bugs
- [ ] Frontier explorer (`scripts/explore.py`, not in the repo) is partly fixed (2026-11-13): it now picks goals and reaches the doorway, but stops with "no frontiers left" after about 5 goals at 17% of the free space explored (SLAM wall precision 100%). Cause so far: right after start slam_toolbox's map is thin rays, so frontier detection is noisy; goal results come back `UNKNOWN`. Try: wait for the map to fill between goals, or score frontiers by unknown area behind them.
- [x] DONE 2026-11-14 (lightly tested): hospital world from rosnav now navigates, goals up to about 20 m succeeded. The earlier "plan has 0 poses" failure went away with UDP-only Fast DDS and was never isolated further.
- [x] DONE 2026-11-13: lidar blindness watchdog (`scripts/scan_watchdog.py`, started by `nav.launch.py`; quiet on a healthy sim, fires on an all-inf scan) and the patrol now prints the distance driven and warns under 1 m (a real patrol drove 33.2 m). The cause of the sim going blind is still unknown.
- [x] DONE 2026-11-14 (one clean run): lifecycle stalls and lost goal responses stopped after forcing UDP-only Fast DDS (`config/fastdds_udp.xml`). Confirm over several more runs; if it holds, set the profile automatically in the launch files.
- [x] DONE 2026-11-13: SLAM mode no longer logs "Filter mask was not received" (the generated params drop the zone filters when mapping; 0 warnings in a fresh run).

## Verify
- [ ] Drive the robot through the keepout and slow zones with the patrol and check the logs, not just the planner and speed samples.
- [ ] Obstacle avoidance: run the patrol with the unmapped box in the robot's path and check it routes around it rather than relying on recovery.
- [ ] Run the web dashboard in a browser (the API was checked with curl only).

## Docs and release
- [ ] Re-record a full patrol GIF including the dock return (the Gazebo GIF is partial).
- [ ] Add RViz screenshots to the README.
- [ ] Cut v0.2.0 once the explorer and the zone checks are done.
- [ ] CI: build the package and run `gen_world.py` plus the launch file syntax check on every push.

## Features
- [ ] Automatic exploration that saves a map (needs the explorer fixed).
- [ ] Mobile UR3: put the arm on the base, navigate to a table, then run the peg-in-hole task. Scope still to confirm.
- [ ] Docking with `opennav_docking` instead of driving to a point.
- [ ] Make the zone masks from the world file automatically for any world.
- [ ] Jazzy support (the launch files use Humble and Gazebo Harmonic naming).
