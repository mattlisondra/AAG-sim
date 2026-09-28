# D435i wrist adapter CAD

`d435i_yam_wrist_adapter.py` generates two side-specific research mounts plus
two unambiguous, two-hole pitch gauges. The confirmed interfaces are:

- existing YAM bracket camera face: two 3.4 mm clearance holes, 20 mm pitch;
- existing YAM bracket arm side: two 3.4 mm clearance holes, 40 mm pitch (not
  used by this adapter);
- D435i rear mount: two M3 threaded positions, 45 mm pitch, 3 mm maximum
  insertion into the camera;
- D435i envelope: 90 × 25 × 25.05 mm, mass approximately 75 g.

The support routing was checked against i2rt robotics' published
[YAM D405 bracket](https://makerworld.com/en/models/2994377-d405-camera-bracket-yam-arm-d405-camera-bracket-fo#profileId-3361179).
The reference is CC BY-NC-SA 4.0 and is not redistributed here. The adapter
replaces the D405 on that bracket; it does not replace the bracket itself.

Regenerate the checked-in STL and STEP files with:

```bash
uv run --with cadquery python cad/d435i_yam_wrist_adapter.py
```

This creates:

- `yam_d405_camera_interface_20mm_coupon` (one edge notch);
- `d435i_camera_interface_45mm_coupon` (two edge notches);
- `yam_d435i_left` and `yam_d435i_right`.

Check the meshes and regenerate the PNG previews with:

```bash
uv run --with vtk python cad/render_previews.py
```

Check the two adapters and approximate D435i housing against a locally
downloaded official bracket STL with:

```bash
uv run --with trimesh --with manifold3d --with scipy --with networkx \
  python cad/check_official_bracket_fit.py /absolute/path/to/the-bracket.stl
```

The default mount targets a D435i RGB lens-to-grasp distance of 166.72 mm. It
raises the housing to 24.0 mm image-up for bracket clearance and applies a
2.3116° pitch correction to retain the target bearing. If factory intrinsics
produce a different distance in `aag-yam wrist-match-plan`, regenerate both
parts with `--match-distance-mm` set to that result, then rerun the collision
checker.

The camera carrier is 5 mm thick with 1 mm bosses. Start with M3 × 8 camera
screws and 0.5–1.0 mm washers, then verify the measured stack leaves no more
than 3 mm of insertion into the camera. The arm side uses nominal 4.6 × 5 mm
heat-set-insert pockets; tune them for the insert and printer actually used.

Recommended starting print: PA-CF or PETG, 0.2 mm layers, five walls, six top
and bottom layers, at least 40% gyroid infill. Orient the part on a lateral
side and use supports where your slicer finds the carrier bridge unsupported.
The small gauges may be printed in PLA. Still do the stationary fit and
slow-motion clearance procedure in
[`docs/D435I_WRIST_ADAPTER.md`](../docs/D435I_WRIST_ADAPTER.md) before motion.
