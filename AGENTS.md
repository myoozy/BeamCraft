# Repository instructions for coding agents

**Read `ARCHITECTURE.md` first.** It is the repository's architecture map,
written for human maintainers and agents alike: the source-set split, the client
vehicle lifecycle, the physics step (asynchronous, and split so the physics
thread never touches the Minecraft world), the JBeam pipeline, the powertrain and
materials subsystems, and asset/config discovery.

The rules below are about provenance and licensing.

## Provenance and licensing

- Read `SOURCE_PROVENANCE.md` before changing JBeam parsing, wheel generation,
  adaptive dampers, or powertrain code.
- Prefer BeamNG's public documentation for compatibility work. If BeamNG Lua is
  consulted, verify the license header of each upstream file before using it.
- Any code adapted, translated, or structurally ported from bCDDL source must
  retain the bCDDL file header, identify the upstream path and contributor, and
  be added to `SOURCE_PROVENANCE.md`.
- Do not claim clean-room implementation when BeamNG source was consulted. Do
  not make correct compatibility behavior intentionally worse for avoidance.
- Loading user-supplied compatible data does not authorize bundling or
  redistributing BeamNG assets.
- AI-generated or AI-assisted code follows the provenance and license of the
  material used to produce it; model output is not a license reset.
