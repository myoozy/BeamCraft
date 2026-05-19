# BeamCraft

BeamCraft is an experimental Minecraft mod exploring soft-body vehicle simulation inspired by BeamNG-style physics.

⚠️ Status: early-stage prototype. Not a playable game system.

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

### ⚠️ Required file

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

Users must provide their own locally installed BeamNG.drive vehicle data.

BeamNG.drive is a product of BeamNG GmbH. This project is not affiliated with or endorsed by BeamNG GmbH.

---

## License

Based on Fabric Example Mod (CC0).

Original code licensed under MIT unless otherwise stated.