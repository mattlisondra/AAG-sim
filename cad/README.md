# D435i wrist adapter CAD

`d435i_yam_wrist_adapter.py` generates two mirrored research mounts plus a
hole-pitch fit coupon. The confirmed camera interfaces are:

- existing D405 bracket: two M3 positions, 20 mm pitch;
- D435i rear mount: two M3 threaded positions, 45 mm pitch, 3 mm maximum
  insertion into the camera;
- D435i envelope: 90 × 25 × 25.05 mm, mass approximately 75 g.

The arm-side shape surrounding the I2RT bracket is not published. Print the
coupon, inspect the bracket, and do the cardboard/slow-motion clearance checks
in [`docs/D435I_WRIST_ADAPTER.md`](../docs/D435I_WRIST_ADAPTER.md) before the
full parts.

Regenerate the checked-in STL and STEP files with:

```bash
uv run --with cadquery python cad/d435i_yam_wrist_adapter.py
```

The default mount targets a D435i RGB lens-to-grasp distance of 166.72 mm. If
factory intrinsics produce a different distance in `aag-yam wrist-match-plan`,
regenerate both parts with `--match-distance-mm` set to that result.

The camera carrier is 5 mm thick with 1 mm bosses. Start with M3 × 8 camera
screws and 0.5–1.0 mm washers, then verify the measured stack leaves no more
than 3 mm of insertion into the camera. The arm side uses nominal 4.6 × 5 mm
heat-set-insert pockets; tune them for the insert and printer actually used.

Recommended starting print: PA-CF or PETG, 0.2 mm layers, five walls, six top
and bottom layers, at least 40% gyroid infill. Orient the part on a lateral
side and use supports where your slicer finds the carrier bridge unsupported.
The small coupon may be printed in PLA.
