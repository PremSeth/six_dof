# New wrist wiring mapping — 2026-09-29

## Both motor directions identified

Viewpoint: behind robot, wrist pointing away. A is viewer's left motor.
A STEP4/DIR5: DIR0 gives UP + RIGHT twist; encoder mux5 decreases.
B STEP6/DIR7: DIR0 gives DOWN + RIGHT twist; encoder mux4 decreases.
Latest B2667pulse run: mux4-14.15039deg, mux5unchanged; preceding B run
mux4-12.30469deg,mux5+.08789deg. Associations are A->5,B->4, NOT pin-order.

Expected coordinated signs (not yet physically tested): UP A0/B1; DOWN A1/B0;
RIGHT twist A0/B0; LEFT twist A1/B1. Equal intended gearbox displacement,
not merely equal elapsed time; axis scale and backlash remain uncalibrated.
Do not use old wrist_position_test.py: its pins, ports and viewpoint differ.

## Observed A mapping

User viewing from behind robot, wrist facing away: motor A is on viewer's LEFT.
A=STEP4/DIR5,DIR0 moved wrist UP and twisted RIGHT from that viewpoint.
User-run2667pulse result: mux4+0.08789deg(one count),mux5-15.73242deg.
This identifies A with encoder5;DIR0 decreases that encoder. Do not label
clockwise/counterclockwise without defining the viewing axis. Motor B mapping
is recorded above; coordinated differential motions remain unverified.
Measured15.73deg differs from nominal30.00deg; do not change10:1/3200ppr
calibration from one test—backlash, step loss or mechanical scaling may contribute.

User confirmed motor A STEP4/DIR5, motor B STEP6/DIR7, encoders mux4/5.
Both3200ppr,10:1. Encoder-to-motor association and physical direction UNKNOWN.
Do not reuse old wrist controller mapping. Firmware only permits finite bursts
of1–2667pulses at50–200Hz, one motor at a time. At3200ppr/10:1,2667pulses
is30.00375gearboxdegrees, taking13.335seconds at200Hz. This is commanded
gearbox travel, not guaranteed wrist travel. Default remains400; request the
larger burst explicitly with--pulses2667. User approved this increased limit.
No encoder-based pause during burst; encoder readings happen at rest before/after.
1.5sec communication watchdog; count completion and STOP remain active.
Keep wrist clear/support arm. Firmware replaces J2 firmware; only one client.

On Pi from ~/six_dof/teensy_wrist_mapping:
```
~/.local/share/mamba/envs/six_dof/bin/python map_wrist.py
~/.local/share/mamba/envs/six_dof/bin/python map_wrist.py --move --motor A --pulses 2667 --dir 0
```
First command reads only. Second moves; B selects STEP6/DIR7. Results append
mapping_results.jsonl; human observation needed for up/down/twist direction.
