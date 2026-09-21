# Architecture

The architecture map for BeamCraft, written for maintainers and coding agents alike.

Licensing and upstream-source provenance are documented in `SOURCE_PROVENANCE.md`;
the coding-agent rules for when it must be consulted are in `AGENTS.md`.

## Build / Test / Run

```bash
./gradlew build          # full build (includes datagen)
./gradlew runClient      # launch Minecraft client with the mod
./gradlew runServer      # launch dedicated server
```

Gradle wrapper scripts are checked in — use `gradlew` (not a system Gradle).
`build.gradle` sets `release`/`targetCompatibility` to Java 21.

**No CI runs here.** The GitHub Actions workflow exists but is inert: its file
sits in `.github/workflows (disabled)/`, a directory GitHub does not read, and
the repository's Actions switch is off as well. Verify changes locally.

## Project Overview

BeamCraft is a Minecraft 1.21 Fabric mod that implements real-time **soft-body vehicle physics**, inspired by the BeamNG.drive JBeam format. Vehicles are defined by node-and-beam structures loaded from the filesystem, simulated client-side, and rendered via GPU-accelerated mesh skinning.

- **Mod ID**: `beamcraft`
- **Package**: `me.mzy.beamcraft`
- **Minecraft**: 1.21 (Yarn mappings `1.21+build.9`)
- **Java**: 21
- **Key dependency**: LWJGL Assimp 3.3.3 (bundled via `include` — ships inside the mod JAR)

## Architecture

### Source split (Fabric split sources)

The project uses `loom.splitEnvironmentSourceSets()` — common code lives in `src/main/`, client-only code in `src/client/`. The client source set has access to main classes, but not vice versa.

### Entry points (defined in `fabric.mod.json`)

| Entry point | Class |
|---|---|
| `main` | `BeamCraft` — registers entity type, network payloads, spawn command |
| `client` | `BeamCraftClient` — initializes physics world, renderer, HUD, input hooks |
| `fabric-datagen` | `BeamCraftDataGenerator` |

### Server-side (`src/main/java/me/mzy/beamcraft/`)

**`BeamCraft.java`** — Mod initializer. Registers:
- `PhysicsVehicleEntity` as a custom entity type (`SpawnGroup.MISC`, fire-immune)
- C2S payload codecs `VehicleSyncPayload` and `VehicleRidePayload`
- Receivers: `VehicleSyncPayload` writes position/yaw onto the entity, but only while the vehicle is passenger-free or the sender is the passenger; `VehicleRidePayload` mounts/dismounts the sender within a 6-block range
- `/spawnvehicle <name> <pcFile>` command (spawns a vehicle entity at the player's position)

**`entity/PhysicsVehicleEntity.java`** — Lightweight, rideable entity. Only holds two synced data-tracker strings (`rootPartName`, `pcFileName`). All physics and rendering are client-side; the server entity is essentially a world anchor that gets position updates from the client via `VehicleSyncPayload`. Overrides `canHit()` so the crosshair yields an `EntityHitResult` (without it right-click entry can never trigger). On client, `updateTrackedPositionAndAngles` is a no-op to prevent server position interpolation from overriding client physics.

**`network/`** — Record-based Fabric C2S payloads. `VehicleSyncPayload`: `(entityId, x, y, z, yaw)`, sent each physics tick so the server entity stays roughly in sync for multiplayer visibility. `VehicleRidePayload`: `(entityId, mount)` for entering and leaving a vehicle.

**`texture/`** — `TextureDecoder`, `DdsDecoder` (+ `DdsFormat`, `DdsDecodeException`, `UnsupportedDdsFormatException`), `TextureCompositor`, `DecodedTextureCache`, `TextureOwnership`: DDS/BCn block decoding, format fallback and composition. Deliberately in the common source set so it is unit-testable without the client.

### Client vehicle lifecycle (`src/client/java/me/mzy/beamcraft/client/`)

**`ClientVehicleManager.java`** — Singleton that maps entity IDs → `SoftBodyVehicle` instances. Each client tick it scans world entities for `PhysicsVehicleEntity` instances, creates vehicles on first sight (loading JBeam + meshes + materials), updates entity bounding boxes, and cleans up removed entities. `VehicleLoadFailureCache` keeps a vehicle whose load already failed from being retried every tick. Also owns shared interpolation arrays used during GPU skinning — they are safe to share because each vehicle's upload completes before the next one starts.

**`BeamCraftClient.java`** — Wires everything together:
1. **Config**: loads `config/beamcraft.json` via `BeamCraftConfigManager`, configures `AssetScanner` with the conflict policy, and builds the `VehicleInputHandler`.
2. **Physics tick**: one fixed physics step per game tick. `AsyncPhysicsScheduler` keeps exactly one step in flight — the preceding step is committed at the tick boundary (`finishPreviousStep`), then `prepareStep` (world access, client thread) and `simulatePreparedStep` (pure physics, worker pool). `DELTA_TIME = 0.05` at a 2000 Hz substep rate → 100 substeps per step. A step exceeding the 50 ms budget is logged and reported in chat at most once per 5 s; a worker failure stops the simulation and is reported once.
3. **Render hand-off**: each vehicle publishes node positions into its `PhysicsRenderTimeline`; the renderer samples that timeline against wall-clock time instead of reading live physics arrays.
4. **HUD**: physics step timing (red if >10 ms, green otherwise), powertrain diagnostics, and the body's pitch/roll.
5. **Debug rendering**: when `DEBUG_DRAW = true`, renders beams, triangles, and torsion bars as colored lines in world space.
6. **Lifecycle**: registers `PhysicsVehicleRenderer` for the custom entity type, and closes the scheduler and texture uploader when the client stops.

**`input/`** — `VehicleInputHandler` resolves the config's bindings and pushes them onto each vehicle's `ElectricBus` (edge events are counted, continuous axes are values). `DriverInputFilter` applies the configured per-substep ramp times via `LinearRamp` and latches edge-triggered input.

**`mixin/`** — `VehicleUseMixin` (vehicle entry follows the vanilla Use action rather than a configurable binding), `VehicleKeyboardInputMixin` (suppresses vanilla riding movement), `VehicleCameraMixin` and `VehicleRiderRenderMixin` (camera and rider rendering), plus the template's `ExampleClientMixin`.

### Physics engine (`src/client/java/me/mzy/beamcraft/client/physics/`)

**`PhysicsWorld.java`** — Core simulation controller. Owns the vehicle list and the globally shared `VoxelSnapshot`, `SoftBodyCollisionManager` and `CollisionPipeline`. Vehicle-to-vehicle broadphase currently uses stable, independently partitioned node chunks and whole-triangle meshlets. Their swept bounds are refitted every ten substeps; a small SAP over node chunks rejects coarse pairs, and a second SAP over the at-most-16 nodes in each surviving chunk rejects most fine pairs before the complete three-axis AABB test. The local SAP maintains X/Y/Z orders and each overlapping meshlet/chunk pair chooses the order with the smallest predicted scan. Spatially oversized triangle outliers receive singleton meshlets so they cannot inflate an otherwise local meshlet. `DynamicAxisSweep` remains available for the previous whole-vehicle SAP path and focused tests while the chunk prototype is measured. Cached point-triangle pairs that are geometrically separated retain a conservative clearance certificate in triangle-relative coordinates; unchanged relative geometry skips repeated narrow-phase work, while actual relative motion or deformation consumes the clearance and re-enables the full test. The optional `PhysicsEventTrace` manually records the interval between two key presses into a ten-second ring and formats/writes it to a standalone CSV on a background daemon, without consuming HUD space or dumping the capture through the console. Key constants: `invPhysicsDT = 2000`, `RENDER_SNAPSHOT_SUBSTEP_INTERVAL = 10` (snapshots at 200 Hz of simulated time), `ELECTRIC_SNAPSHOT_SUBSTEP_INTERVAL = 10`. A step is split in two so that **no Minecraft world access happens on the physics thread**: `prepareStep` captures the voxel snapshot and other world-derived state on the client thread, `simulatePreparedStep` runs the substep loop.

**`AsyncPhysicsScheduler.java`** — keeps exactly one physics step in flight on a dedicated `ForkJoinPool`, with a 50 ms tick budget and failure propagation.

**`PhysicsRenderTimeline.java`** — per-vehicle single-producer/single-consumer timeline of node positions. The physics worker publishes snapshots in increasing time order; the render thread samples them against wall-clock time. Snapshot arrays are retained and reused, so the 200 Hz publish rate creates no GC pressure.

**`CollisionPipeline.java`** — the shared collision algorithms: soft-body contact candidate generation (triangle vs node, fed by the shared SAP), batched resolution of cached soft-body contacts, and Minecraft environment collision from a `VoxelSnapshot`. Point-triangle contacts use the actual previous/current node positions for side selection and test a moving/deforming triangle crossing when the current point has already left its tight AABB. Contact batches use conflict-free greedy colors plus an explicitly serial overflow batch. It is called from `PhysicsWorld`'s per-vehicle parallel phases, so it must not keep per-vehicle scratch state (the per-vehicle sweep buffer stays on `SoftBodyVehicle`).

**`VehicleInternalForceSolver.java`** — one substep's internal-force body for a single vehicle: tire pressure, every beam constraint family (normal / support / bounded / LBeam / anisotropic), torsion bars, slide nodes, plastic deformation with hardening and relaxation, node integration under gravity, the node velocity sanitizer, and the fixed call order of the hydro, powertrain, brake and coupler updates. Owned by one vehicle, mutates only that vehicle's containers, allocates nothing per substep.

**`SoftBodyVehicle.java`** — One vehicle instance. Owns all physics data in Structure-of-Arrays (SoA) containers and the container lifecycle (`add`/`reset`/`clear`/`finalize`), break-group coordination and the public `solveInternalForces` entry points, which delegate to `VehicleInternalForceSolver`. Containers: `NodeContainer`; two `BeamContainer`s (normal + support); `BoundedBeamContainer`, `LBeamContainer`, `AnisotropicBeamContainer`, `CouplerContainer`, `HydroContainer`, `TorsionHydroContainer`, `TorsionBarContainer`, `SlideNodeContainer`, `TriangleContainer`, `WheelContainer`, `FlexbodyContainer`; plus `PowertrainSystem`, `AdaptiveDamperActuators`, `DriverInputFilter`, `VehicleCameraData`. Also caches per-part bounding boxes so inactive sub-assemblies can be culled.

Break group system: beams can be grouped; when enough beams in a group break, all remaining beams in that group break too.

**`DirectionalStabilityLimiter.java`** — direction-aware per-node budget clamp. The reduction is shared across a node's constraints weighted by how much each fills the node, so a damper pays for its own share while a small contributor is effectively protected.

**Actuators and other constraint systems**: `AdaptiveDamperActuators`/`AdaptiveDamperController`/`AdaptiveDamperParser` (BeamNG adaptive damper actuators — the JBeam interface is parsed and kept, but the mode-switching logic is not implemented), `HydroActuatorController` + `HydroContainer` (hydro actuators), `TorsionHydroContainer` (torsion hydro steering), `CouplerContainer`/`CouplerRegistry` (modern coupler system), `TorqueReactionSolver`.

**JBeam format pipeline**:
- `JBeamLoader` — Reads `.jbeam` files (JSON with C-style comments, auto-inserts missing commas). Delegates discovery to `AssetScanner`, which scans every configured asset root for the vehicle's `.jbeam` files and the `.pc` config. A named-but-missing `.pc` is reported rather than silently falling back to an all-default build.
- `RelaxedJson` (in `client/material/`) — the relaxed-JSON cleaner used by both JBeam and material parsing.
- `JBeamParser` — Parses cleaned JSON into raw data maps, with `$=...` expression evaluation via `JBeamExpressionEvaluator`/`BeamExpressionContext`.
- `JBeamAssembler` — Two-pass assembler with hierarchical transform context. Pass 1: register all nodes. Pass 2: build beams, triangles, and constraints. Handles `nodeRotate`, `nodeOffset`, `nodeMove` transforms with mirroring support.
- `JBeamPartMerger` — unifies the active part configuration into the vehicle-wide section view the other parsers consume.
- Specialized parsers: `JBeamPressureWheelsParser` (generates the rim/tire beam families), `JBeamCameraParser`, `powertrain/JBeamPowertrainParser`.

**`powertrain/`** — `PowertrainSystem` (per-vehicle runtime, driven from the internal-force substep), `PowertrainCompiler` with `PowertrainSpecs`/`PowertrainSpecNormalizer`, the device containers (combustion engine, friction clutch, torque converter, manual and DCT gearboxes, range box, shaft and split shaft, differential, torsion reactor, torque reaction, turbocharger, supercharger), and the coupling solvers (`ImplicitCouplingSolver`, `DctCouplingSolver`, `TorqueConverterSolver`, `SplitShaftSolver`).

**`electrics/`** — `ElectricBus` + `ElectricSignals` + `ElectricSnapshot`: the snapshot-based signal bus that carries player input and control signals from the client thread into the physics solvers. Refreshed every 10 substeps (200 Hz).

### Rendering (`src/client/java/me/mzy/beamcraft/client/render/`)

**`PhysicsVehicleRenderer.java`** — Entity renderer. On each frame: gets the `SoftBodyVehicle` from `ClientVehicleManager`, ensures flexbody binding is done, then draws using Minecraft's `RenderSystem` with the GPU-skinned VBO. Selects a part's deform material (base vs damaged) by its deform-group state.

**`ComputeSkinningPipeline.java`** — GPU skinning via **OpenGL 3.2 Transform Feedback**, which it uses *deliberately* instead of an OpenGL 4.3 compute shader for broader GPU compatibility. Uploads per-vertex rig data (node indices + weights) as TBO textures alongside the physics node positions, runs a vertex shader under rasterizer discard, and captures the transformed positions/normals into a dynamic VBO. The Minecraft renderer then draws this VBO as a regular entity.

**`TransformFeedbackShaderLoader.java`** — Compiles the skinning vertex shader and links a transform-feedback program with `(tfPosition, tfNormal)` varyings.

**`VehicleTextureUploader.java`** / **`VehicleCameraController.java`** — texture upload lifetime, and the vehicle camera driven by `VehicleCameraData` (filled by `JBeamCameraParser`).

**`softbody_transform.vsh`** (GLSL 150 core) — The skinning kernel. Supports two modes per vertex:
- **Deform basis**: Vertex animated by a center node + two direction nodes forming a local basis (vx, vy, cross-product normal)
- **Static offset**: Vertex rigidly attached to a single node with a static offset

### Materials (`src/client/java/me/mzy/beamcraft/client/material/`)

**`MaterialLibrary.java`** — loads material definitions from the vehicle data via `AssetScanner`, resolves textures with `TextureResourceLocator`, and harvests `glowMap` aliases (`GlowMapAliasExtractor`). **`MaterialRenderPlanner`** / **`MaterialRenderPlan`** / **`MaterialDefinition`** decide how a material is drawn: the declared `doubleSided` flag, the `translucentBlendOp` (premultiplied alpha), and cutout masks selected by the vehicle's own `alphaRef`.

> **Known limitation**: `TextureResourceLocator` still resolves textures by first-registered source, independent of the conflict strategy (see the `TODO` in `MaterialLibrary.scanVehicle`). This only diverges when two registered containers share a texture path with different content *and* both hold a winning entry (partial vehicle overlap across roots); a full override is consistent because the shadowed root is never registered.

### Model loading (`src/client/java/me/mzy/beamcraft/client/model/`)

**`DaeMeshLoader.java`** — Uses LWJGL Assimp to load COLLADA (`.dae`) files. Caches parsed `RawGeometry` by filename with ref-counting. Supports zip bundles and a "common" shared mesh library. A mesh is resolved **by name on demand**: the flexbody table names a mesh but not the file it lives in, so candidate DAE files are scanned for node and geometry names (cached per file) and only the providers are imported; the vehicle's own namespace is still imported whole, and a name no file declares is reported rather than chased.

**`FlexbodyBindingUtil.java`** — Binds loaded mesh vertices to physics nodes. For each mesh vertex, finds the nearest N physics nodes (by position) and computes skinning weights + the local offset basis. Uploads the result as TBO textures for the GPU skinning pipeline.

### Asset discovery & configuration (`src/client/java/me/mzy/beamcraft/client/assets/`, `.../config/`)

**`BeamCraftConfig.java`** — Loads `config/beamcraft.json` (auto-created with defaults). Holds the `assetRoots` list (default `mods/beamcraft/vehicles`, resolved against the game dir; absolute paths accepted), a `conflict` policy (`strategy` = `newer` | `later-root` | `earlier-root`, plus `notify`), optional `diagnostics` switches, and `input` bindings (per-action key *lists*; continuous actions take signed axis values and optional `riseTime`/`fallTime` ramps). Missing default sections are merged back in on load without discarding unknown keys, and defaults are never written out for `input`. `BeamCraftConfigManager` performs the one-time load and exposes the resolved roots.

**`AssetScanner.java`** — The single shared discovery engine used by `JBeamLoader`, `DaeMeshLoader` and `MaterialLibrary`. For each asset root it scans direct-child containers (folders or `.zip`s). The outer container name is **arbitrary**; the real vehicle name is the inner `vehicles/<name>/` path segment (segment-boundary aware and case-insensitive, so `vehicles/sunburst2/` never matches `sunburst`). Entries are grouped by logical path; when one path exists in several sources the configured `ConflictStrategy` picks a winner and `ConflictReporter` logs (and optionally notifies). Containers are deduplicated by canonical path. `NamespaceScan.sources()` only returns containers of winning entries, so a shadowed root is never registered with the texture locator.

**`ConflictReporter.java`** — Always `LOGGER.warn`s; optionally posts a deduplicated in-game chat message listing the conflicting source addresses.

### Shaders

- `src/main/resources/assets/beamcraft/shaders/softbody_transform.vsh` — OpenGL 3.2 vertex shader used as a GPU compute kernel via transform feedback. Reads 4 TBO samplers (`uRigWeights`, `uRigNormals`, `uRigOffsets`, `uPhysicsNodes`) and outputs `tfPosition`/`tfNormal`.

### Vehicle data format

By default vehicles live in `<gameDir>/mods/beamcraft/vehicles/`, but the asset
roots are configurable via `config/beamcraft.json` (`assetRoots` list; the
default entry is the historical `mods/beamcraft/vehicles` path). Each asset
root is a directory of containers (folders or `.zip` archives). The outer
container name is arbitrary; the real vehicle name is the inner
`vehicles/<name>/` path segment (BeamNG mod convention):

- `vehicles/<name>/*.jbeam` — node/beam/triangle definitions (JSON with relaxed syntax)
- `vehicles/<name>/<name>.pc` — part config (references which JBeam parts to load and in what order)
- `vehicles/<name>/<name>.dae` — COLLADA mesh with vertex positions matching the physics nodes
- A shared `vehicles/common/` library holds assets shared across vehicle variants (a container literally named `common`/`common.zip` is accepted in full)

## Key patterns

- **SoA everywhere**: Physics data uses Structure-of-Arrays with primitive `float[]`/`int[]` fields (not objects) for cache efficiency and parallel stream compatibility.
- **Manual array growth**: Arrays start at `INIT_NODE_CAP` (128) and double on overflow via `Utility.expand()`.
- **Parallel per-vehicle phases**: Internal force solving and the collision phases run as parallel phases over the vehicle list; per-vehicle solvers therefore must not hold shared scratch state.
- **No world access on the physics thread**: `prepareStep` captures everything the substep loop needs (`VoxelSnapshot`, electric snapshot) so that `simulatePreparedStep` runs with no Minecraft objects in play.
- **Snapshot hand-off to the render thread**: the renderer never reads live physics arrays; it samples `PhysicsRenderTimeline`, which was filled at 200 Hz.
- **Client-authoritative physics**: The server entity is passive — the client runs all physics and pushes position via custom payloads.
- **Comments are often Chinese; log and error output is English.**
