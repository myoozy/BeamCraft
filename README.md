# BeamCraft

BeamCraft is an experimental Minecraft mod exploring soft-body vehicle simulation inspired by BeamNG-style physics.

## Videos and Channels

- [YouTube playlist](https://youtube.com/playlist?list=PLKse2v6xW8Dc&si=kp0lr9YGZrn9Fw5Q)
- [Bilibili channel](https://space.bilibili.com/270425369?spm_id_from=333.788.upinfo.head.click)

⚠️ Status: early-stage prototype. Not a playable game system.

---

## Installation

### 1. User-provided vehicle data

BeamCraft does not include or distribute vehicle data. Compatibility testing
requires user-provided JBeam, DAE, material, and texture data that the user is
legally entitled to use.

Do not redistribute third-party game assets with BeamCraft.

---

### 2. Asset placement

BeamCraft reads its asset locations from `config/beamcraft.json` in the game
directory — in a development client that is `run/config/beamcraft.json`. The
file is created automatically on first launch if it does not exist.

The `assetRoots` field is a list of load paths, scanned in order. Relative
paths are resolved against the game directory (the default entry preserves the
classic `mods/beamcraft/vehicles` location), and absolute paths are accepted.
Add as many roots as you like:

```json
{
  "assetRoots": [
    "mods/beamcraft/vehicles",
    "D:/MyVehicles",
    "E:/MyVehicleData"
  ],
  "conflict": {
    "notify": false,
    "strategy": "later-root"
  }
}
```

Each root is a directory whose direct children are vehicle containers —
folders or `.zip` archives. A container's outer name is arbitrary; the real
vehicle name is the inner `vehicles/<name>/` path segment, matched
segment-boundary-aware so `vehicles/sunburst2/` never matches namespace
`sunburst`. If the same logical path appears in several
roots, `conflict.strategy` (`newer`, `later-root`, or `earlier-root`) decides
which version wins and optionally reports the conflict in chat
(`conflict.notify`).

Other settings (`input.*`) are merged into the file on the first run; they
rebind the driving controls listed under Usage.

#### Shared data (`common`)

Many vehicle data sets reference shared definitions and assets that live in
the `common` namespace. BeamCraft resolves it from either a `vehicles/common/`
folder inside a vehicle container, or a container literally named `common` or
`common.zip` placed directly under an asset root — so no extraction or special
placement is needed.

BeamCraft does not provide `common` or any other game asset. If you already
own the data, load it in place: add an asset root that points at a folder you
own which contains such a container, or point one directly at the `common`
container itself. Keep game assets for your own use — do not redistribute or
republish them.

---

## Usage

### Spawn a vehicle

`/spawnvehicle <name> <pcFile>`

- `<name>`: vehicle identifier from the compatible vehicle data
- `<pcFile>`: vehicle configuration / preset file

---

### Enter, drive, and exit

Right-click (the vanilla use key) a spawned vehicle to get in. Driving uses
the following defaults:

| Control | Default key |
|---|---|
| Steer | ← / → |
| Throttle | ↑ |
| Brake | ↓ |
| Clutch | C |
| Starter (ignition) | V |
| Shift up / down | X / Z |
| Reset vehicle to player | G |
| Exit vehicle | Left Shift |

Each control can be rebound by filling in the matching `input.*` entry in
`config/beamcraft.json` with a GLFW translation key such as
`key.keyboard.up` (an empty string keeps the built-in default above).

---

### Remove spawned vehicles

`/kill @e[type=beamcraft:physics_vehicle]`

---

## Features

Working so far:

1. Node–beam soft-body physics simulation
2. Constraint types: beams, torsion bars, sliders, hydro actuators
3. Collision with the Minecraft voxel world and soft-body ↔ soft-body interaction
4. Wheels with inflatable tires and compliant braking (non-destructible)
5. Partial part damage and breakage: breakable triangles, beam break-groups
   (cascading part loss), and beam plastic deformation
6. A simple powertrain:
   - naturally aspirated combustion engine (torque curve, idle control)
   - manual gearbox with a friction clutch
   - open differential
   - other drivetrain and gearbox types are not simulated yet
7. A snapshot-based electric bus that carries player input and control signals
   from the render thread into the physics solvers
8. Get in / get out of a vehicle, with the player riding the entity; the
   server entity position is kept in sync for multiplayer visibility
9. Configurable driving controls (see Usage)
10. GPU-accelerated skinned mesh rendering:
   - COLLADA (`.dae`) meshes loaded via Assimp, split by material
   - textures and basic material effects loaded from vehicle data
   - partial lighting support; some culling and overlay quirks remain
11. Physics runs asynchronously on a background thread, decoupled from the
    render timeline (the renderer consumes published snapshots)
12. Multi-root asset discovery and conflict resolution via `config/beamcraft.json`

---

## Missing Features

1. Sound (engine, tire, and collision audio)
2. Electronic controls and driver aids (ABS, traction control, etc.)
3. An event system for interactive parts — doors cannot yet be opened/closed,
   and windows cannot yet switch to their shattered model
4. Some vehicle models are not implemented (e.g. the steering wheel)
5. Full powertrain coverage (automatic, sequential, and DCT gearboxes, etc.)
6. Interaction with other Minecraft entities
7. Aerodynamics (drag, lift, downforce)
8. Gameplay systems

---

## Limitations

- Not a playable game system
- Driving is experimental and largely unpolished
- Rendering bugs may occur
- No gameplay loop
- Performance not optimized

---

## Compatibility

Minecraft 1.21 Fabric

Other versions not tested.

---

## Development Notes

- Some code is AI-assisted
- Many comments and logs are in Chinese
- Debug output may include Chinese text

---

## Asset Notice

This project does not include or distribute any assets from BeamNG.drive.

BeamCraft is an independent interoperability experiment. Users are responsible
for ensuring that they have the right to use any data they load.

BeamNG.drive is a product of BeamNG GmbH. This project is not affiliated with or endorsed by BeamNG GmbH.

---

## License

Copyright (C) 2026 M1AO.

BeamCraft's original code is licensed under the [MIT License](LICENSE).

Portions derived from the Fabric Example Mod remain available under CC0. Third-party components retain their respective licenses.
