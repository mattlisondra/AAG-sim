# D435i full replacement wrist mount

The preferred hardware is a one-piece replacement for the YAM D405 wrist
bracket. It preserves the supplied bracket's original two 40 mm-pitch
through-holes for mounting to the arm, then fuses on a carrier with two
complete 45 mm-pitch passages for M3 screws into the D435i.

Confirmed interfaces:

- YAM bracket-to-arm: two 3.4 mm clearance holes, 40 mm pitch;
- original D405 camera face: two 3.4 mm holes, 20 mm pitch;
- D435i rear mount: two M3 threaded positions, 45 mm pitch, 3 mm maximum screw
  insertion;
- D435i envelope: 90 × 25 × 25.05 mm, approximately 75 g.

The source bracket is i2rt robotics'
[D405 camera bracket – YAM arm](https://makerworld.com/en/models/2994377-d405-camera-bracket-yam-arm-d405-camera-bracket-fo#profileId-3361179).
Its SHA-256 must be
`c912eb55577ce157fb8b2cc11cb7baf350383baba564007fec4ce0a7b8ac0de2`.

## Build the preferred full mounts

```bash
uv run --with cadquery --with trimesh --with manifold3d \
  --with scipy --with networkx \
  python cad/build_full_d435i_wrist_mount.py /absolute/path/to/the-bracket.stl
```

This creates two mesh-only replacement parts under
`cad/generated/official-bracket-derived`:

- `yam_d435i_full_wrist_mount_left.stl`;
- `yam_d435i_full_wrist_mount_right.stl`.

The builder refuses an unknown source hash. It also verifies that each result
is one watertight component, that all four screw passages are open, and that
the approximate D435i housing clears the result.

The full mounts incorporate the official mesh and are derivative works under
CC BY-NC-SA 4.0, not Apache-2.0. See the attribution and license in their
output directory. The raw source STL is not redistributed.

Render and validate the final meshes:

```bash
uv run --with vtk python cad/render_previews.py \
  cad/generated/official-bracket-derived
```

## Camera geometry

The default mount targets a 166.72 mm D435i RGB lens-to-grasp distance. It:

- places the D435i body 23.5 mm outboard to align the RGB optical axis;
- raises the screw row to 24.0 mm for bracket clearance;
- applies a 2.3116° pitch correction to retain the nominal target bearing;
- routes both structural rails on the side opposite the wide D435i housing.

If exact factory intrinsics produce a different distance through
`aag-yam wrist-match-plan`, pass `--match-distance-mm` to the full-mount
builder and repeat every clearance check.

## Screws

- Arm side: reuse the original/vendor-specified bracket screws through the two
  preserved 40 mm-pitch holes. The STL does not establish the arm's thread or
  safe engagement depth.
- Camera side: start with two M3 × 8 screws and 0.5–1.0 mm washers. The carrier
  plus boss is 6 mm thick; verify the actual stack leaves no more than 3 mm
  insertion into the D435i.

## Optional bolt-on adapter and gauges

`d435i_yam_wrist_adapter.py` also generates an optional bolt-on adapter, plus
20 mm and 45 mm overlay gauges:

```bash
uv run --with cadquery python cad/d435i_yam_wrist_adapter.py
```

The bolt-on adapter deliberately has blind heat-set-insert pockets because it
attaches to the camera face of an already-installed D405 bracket. It is not the
preferred arm-mount part. The full replacement mount needs no heat-set inserts.

Recommended starting print: PA-CF or PETG, 0.2 mm layers, five walls, six top
and bottom layers, and at least 40% gyroid infill. Print the small 45 mm gauge
first. Follow the powered-down fit and slow-motion clearance procedure in
[`docs/D435I_WRIST_ADAPTER.md`](../docs/D435I_WRIST_ADAPTER.md).
