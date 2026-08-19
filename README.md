# BeamCraft

BeamCraft is an experimental Minecraft mod exploring soft-body vehicle simulation inspired by BeamNG-style physics.

## Videos and Channels

- [YouTube playlist](https://youtube.com/playlist?list=PLKse2v6xW8Dc&si=kp0lr9YGZrn9Fw5Q)
- [Bilibili channel](https://space.bilibili.com/270425369?spm_id_from=333.788.upinfo.head.click)

鈿狅笍 Status: early-stage prototype. Not a playable game system.

---

## Installation

### 1. Required external assets (BeamNG.drive)

BeamCraft requires vehicle data from BeamNG.drive.

You must provide your own locally installed copy of BeamNG.drive vehicle files.

Typical source location:

steamapps/common/BeamNG.drive/content/vehicles

---

### 2. Asset placement

Place vehicle folders or archives into:

run/mods/beamcraft/vehicles

---

### 鈿狅笍 Required file

The mod also requires `common.zip` from the BeamNG vehicle content directory.

It must be present for correct loading of shared assets.

---

## Usage

### Spawn a vehicle

`/spawnvehicle <name> <pcFile>`

- `<name>`: vehicle identifier (folder name from BeamNG content)
- `<pcFile>`: vehicle configuration / preset file

---

### Remove spawned vehicles

`/kill @e[type=beamcraft:physics_vehicle]`

---

## Features

1. Node鈥揵eam soft-body physics simulation
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

Users must provide their own locally installed BeamNG.drive vehicle data.

BeamNG.drive is a product of BeamNG GmbH. This project is not affiliated with or endorsed by BeamNG GmbH.

---

## License

Copyright (C) 2026 M1AO.

BeamCraft's original code is licensed under the [GNU General Public License v3.0 or later](LICENSE). If you distribute a modified version, you must make its corresponding source code available under the same license.

Portions derived from the Fabric Example Mod remain available under CC0. Third-party components retain their respective licenses.
