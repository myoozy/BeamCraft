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

For the current prototype, place compatible vehicle folders or archives into:

`run/mods/beamcraft/vehicles`

---

### Shared data

Some vehicle data sets also require their corresponding `common.zip` for
shared definitions and assets. BeamCraft does not provide this file.

---

## Usage

### Spawn a vehicle

`/spawnvehicle <name> <pcFile>`

- `<name>`: vehicle identifier from the compatible vehicle data
- `<pcFile>`: vehicle configuration / preset file

---

### Remove spawned vehicles

`/kill @e[type=beamcraft:physics_vehicle]`

---

## Features

1. Node–beam soft-body physics simulation
2. Torsion bar and slider constraints
3. Collision with Minecraft world and soft-body interaction
4. Inflatable tire simulation (non-destructible)
5. Basic skinned rendering system
   - partial lighting support
   - no textures yet
   - known rendering culling issues

---

## Missing Features

1. Damage / breakable parts system
2. Aerodynamics (drag, lift, downforce)
3. Powertrain system
4. Vehicle control logic
5. Texture and audio system
6. Interaction with Minecraft entities
7. Gameplay systems

---

## Limitations

- Not a playable game system
- Vehicles can only be spawned and observed
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
