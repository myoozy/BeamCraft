# BeamCraft

BeamCraft is an experimental Minecraft mod exploring node-and-beam soft-body
vehicle simulation.

## Videos and Channels

- [YouTube playlist](https://youtube.com/playlist?list=PLKse2v6xW8Dc&si=kp0lr9YGZrn9Fw5Q)
- [Bilibili channel](https://space.bilibili.com/270425369?spm_id_from=333.788.upinfo.head.click)

⚠️ Status: early-stage prototype. Not a playable game system.

---

## Installation

### 1. User-provided vehicle data

BeamCraft does not include or distribute vehicle data. Testing it needs JBeam,
DAE, material and texture data that you supply and place under an asset root.

---

### 2. Asset placement

Vehicle data is loaded from one or more **asset roots**. A root is a directory
whose direct children are vehicle containers — folders or `.zip` archives. A
container's outer name is arbitrary; the real vehicle name is the inner
`vehicles/<name>/` path segment, matched segment-boundary-aware so
`vehicles/sunburst2/` never matches namespace `sunburst`. A container holds the
files a vehicle is built from:

- `vehicles/<name>/*.jbeam` — node, beam and triangle definitions (JSON with
  relaxed syntax)
- `vehicles/<name>/<name>.pc` — the part config, selecting which parts are loaded
- `vehicles/<name>/<name>.dae` — the COLLADA mesh

Roots, conflict resolution and control bindings all live in a single file,
`config/beamcraft.json` — see **Configuration** below. The default root is the
historical `mods/beamcraft/vehicles`.

#### Shared data (`common`)

Many vehicle data sets reference shared definitions and assets that live in
the `common` namespace. BeamCraft resolves it from either a `vehicles/common/`
folder inside a vehicle container, or a container literally named `common` or
`common.zip` placed directly under an asset root — so no extraction or special
placement is needed.

BeamCraft does not provide `common` or any other third-party game asset; it
loads what you point it at. Either add an asset root for the folder that holds a
`common` container, or point a root directly at that container.

---

## Configuration

BeamCraft is configured by a single JSON file in the game directory — in a
development client that is `run/config/beamcraft.json`. It is created
automatically on first launch if absent; on later launches any missing default
section is merged back in without discarding keys that are not recognised. The
file has three sections:

```json
{
  "assetRoots": [
    "mods/beamcraft/vehicles",
    "D:/MyVehicles"
  ],
  "conflict": {
    "notify": false,
    "strategy": "later-root"
  },
  "input": {
    "throttle": {
      "keys": [{ "key": "key.keyboard.up", "value": 1.0 }],
      "riseTime": 0.15,
      "fallTime": 0.25
    }
  }
}
```

### `assetRoots` — where vehicles are loaded from

An ordered list of roots, scanned in order. Relative paths are resolved against
the game directory (the default entry preserves the classic
`mods/beamcraft/vehicles` location) and absolute paths are used as they are, so
an installed vehicle data set can be pointed at directly:

```json
"assetRoots": [
  "mods/beamcraft/vehicles",
  "D:/MyVehicles",
  "E:/MyVehicleData"
]
```

Add as many roots as you like. Each one is scanned for the containers described
under **Asset placement**; a root that is missing is simply empty, not an error.

### `conflict` — which copy of an asset wins

The same logical path can exist in several roots. `conflict.strategy` picks the
winner:

| Value | Winning source |
|---|---|
| `later-root` (default) | the root listed last |
| `earlier-root` | the root listed first |
| `newer` | the newest file; a `.zip` entry with no usable timestamp counts as oldest |

`conflict.notify` (default `false`) additionally posts a one-line in-game chat
message naming the conflicting sources. Conflicts are logged either way.

### `input` — driving controls

Every action takes a **list** of keys, so one control can be driven by several
physical keys. Keys are GLFW translation keys — `key.keyboard.up`,
`key.keyboard.left.shift`, `key.mouse.left` — and an action that is absent or
empty keeps the default listed below. Blank or unparseable entries are ignored
with a warning in the log rather than breaking the section.

| Action | Kind | Default |
|---|---|---|
| `exitVehicle` | key | Left Shift |
| `starter` | key | V |
| `shiftUp` / `shiftDown` | key | X / Z |
| `rangeBoxToggle` | key | B |
| `resetVehicle` | key | G |
| `steering` | axis | ← / → |
| `throttle` | axis | ↑ |
| `brake` | axis | ↓ |
| `clutch` | axis | C |

The four continuous actions are **axes**: each key carries a signed `value` in
`-1..1`, so a single axis can be driven in both directions and a key can supply
a partial input (the defaults read `left = -1`, `right = +1`). They also accept
optional `riseTime` / `fallTime` in seconds — the axis ramps linearly toward its
target instead of snapping, which is what turns a keyboard's on/off into a
usable pedal:

```json
"clutch": {
  "keys": [
    { "key": "key.keyboard.c", "value": 1.0 },
    { "key": "key.mouse.right", "value": 1.0 }
  ],
  "riseTime": 0.10,
  "fallTime": 0.10
}
```

Defaults are deliberately never written out, so a fresh config carries an empty
`input` section; an entry overriding one field inherits the rest. If the section
is malformed it is reset on its own, leaving the asset settings usable.

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
| Range box toggle | B |
| Reset vehicle to player | G |
| Exit vehicle | Left Shift |

Every one of these is rebindable from the `input` section of
`config/beamcraft.json`, which also controls how the continuous pedals ramp —
see **Configuration** above.

---

### Remove spawned vehicles

`/kill @e[type=beamcraft:physics_vehicle]`

---

## Features

Working so far:

1. Node–beam soft-body physics simulation
2. Constraint types: beams, torsion bars, sliders, couplers, hydro actuators
3. Collision with the Minecraft voxel world and soft-body ↔ soft-body interaction
4. Wheels with inflatable tires and compliant braking (non-destructible)
5. Partial part damage and breakage: breakable triangles, beam break-groups
   (cascading part loss), deform groups (a damaged part switches to its damaged
   material), and beam plastic deformation
6. A powertrain assembled from the vehicle's own authored devices:
   - naturally aspirated combustion engine (torque curve, idle control)
   - turbocharger (prototype) and supercharger
   - manual and dual-clutch gearboxes, friction clutch, torque converter
   - range box, split shafts and nested split couplings
   - open differential and the torsion reactor
   - per-wheel and reactor torque-reaction paths
7. Beam tuning taken from the vehicle data: bounded-beam transitions and
   separate fast/rebound damping channels with a velocity split
8. Torsion hydro steering
9. Get in / get out of a vehicle, with the player riding the entity; the
   server entity position is kept in sync for multiplayer visibility
10. Configurable driving controls: several keys per action, signed analog axes
    and linear ramp times (see Configuration)
11. GPU-accelerated skinned mesh rendering:
    - COLLADA (`.dae`) meshes loaded via Assimp, split by material
    - textures and material effects loaded from vehicle data
    - partial lighting support; some culling and overlay quirks remain
12. Physics runs asynchronously on a background thread, decoupled from the
    render timeline (the renderer consumes published snapshots)
13. A snapshot-based electric bus that carries player input and control signals
    from the render thread into the physics solvers
14. Multi-root asset discovery and conflict resolution via `config/beamcraft.json`

---

## Missing Features

1. Sound (engine, tire, and collision audio)
2. Electronic controls and driver aids (ABS, traction control, etc.)
3. An event system for interactive parts — doors cannot yet be opened or closed
4. Some vehicle models are not implemented (e.g. the steering wheel)
5. Full powertrain coverage: automatic and sequential gearbox shift logic, and
   the driveline devices that are still unhandled
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
- Many code comments are in Chinese; log and error output is in English
- Some compatibility code is adapted from BeamNG.drive bCDDL 1.1 Lua source;
  see [Source provenance](SOURCE_PROVENANCE.md)

---

## Asset Notice

This project does not include or distribute any assets from BeamNG.drive®.

BeamCraft is an independent, unofficial Minecraft mod and interoperability
experiment. It is not approved by, endorsed by, associated with, supported by,
or connected to Mojang, Microsoft, or BeamNG GmbH. Users are responsible for
ensuring that they have the right to use any data they load.

Minecraft is a trademark of the Microsoft group of companies. BeamNG.drive® is
a registered trademark of BeamNG GmbH. All other trademarks belong to their
respective owners.

---

## License

Copyright (C) 2026 M1AO.

BeamCraft's original code is licensed under the [MIT License](LICENSE).

Files identified as adaptations of BeamNG.drive Lua source are licensed under
the [bCDDL 1.1](LICENSES/bCDDL-1.1.txt), not MIT. See
[Source provenance](SOURCE_PROVENANCE.md) for the file-level mapping.

Portions derived from the Fabric Example Mod remain available under CC0 1.0.
Bundled third-party components retain their respective licenses; see
[Third-Party Notices](THIRD_PARTY_NOTICES.md) for copyright and license details.
