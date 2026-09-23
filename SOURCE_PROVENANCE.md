# Source provenance

BeamCraft is a mixed-license project. This inventory records the best current
understanding of how source code was produced. For licensing simplicity,
BeamCraft conservatively distributes an entire Java file under bCDDL 1.1 when
that file contains a BeamNG source-derived portion, unless the derived portion
is separated into another file.

## BeamNG bCDDL 1.1 adaptations

The files below adapt or translate behavior from the named BeamNG.drive Lua
sources. They are distributed under the bCDDL 1.1 in
`LICENSES/bCDDL-1.1.txt`. Java adaptations and later modifications were
contributed by M1AO and BeamCraft contributors, with substantial AI assistance
under the maintainer's direction. The September 2026 audit checked the upstream
paths against an installed BeamNG.drive 0.38.5 tree and verified that each named
Lua source carries a bCDDL 1.1 notice.

| BeamCraft files | Upstream BeamNG source |
| --- | --- |
| `JBeamExpressionEvaluator.java`, `BeamExpressionContext.java`, `JBeamParser.java` | `lua/common/jbeam/expressionParser.lua`, `lua/common/jbeam/variables.lua`, plus localized defaults from `lua/common/jbeam/loader.lua` and `lua/vehicle/jbeam/stage2.lua` |
| `JBeamPartMerger.java` | `lua/common/jbeam/slotSystem.lua` (`unifyParts` semantics) |
| `WheelContainer.java`, `JBeamPressureWheelsParser.java` | `lua/common/jbeam/sections/wheels.lua`, especially pressure-wheel construction |
| `AdaptiveDamperActuators.java`, `AdaptiveDamperMode.java` | `lua/vehicle/controller/drivingDynamics/actuators/adaptiveDampers.lua` |
| `JBeamPowertrainParser.java`, `PowertrainSpecNormalizer.java`, `PowertrainCompiler.java`, `PowertrainSystem.java`, `PowertrainSpecs.java`, `PowertrainTopologyContainer.java`, `DrivenWheelPathContainer.java`, `TorqueReactionContainer.java` | `lua/vehicle/powertrain.lua` and the device files below; parsing, topology, ports, traversal, inertia reflection and torque-reaction semantics |
| `CombustionEngineContainer.java`, `TurbochargerContainer.java`, `SuperchargerContainer.java` | `lua/vehicle/powertrain/combustionEngine.lua`, `turbocharger.lua`, `supercharger.lua` |
| `ClutchlikeContainer.java`, `FrictionClutchContainer.java`, `TorqueConverterContainer.java`, `DctGearboxContainer.java` | `lua/vehicle/powertrain/frictionClutch.lua`, `torqueConverter.lua`, `dctGearbox.lua` |
| `GearboxContainer.java`, `RangeBoxContainer.java`, `ShaftContainer.java`, `SplitShaftContainer.java`, `DifferentialContainer.java`, `TorsionReactorContainer.java` | `lua/vehicle/powertrain/manualGearbox.lua`, `automaticGearbox.lua`, `sequentialGearbox.lua`, `rangeBox.lua`, `shaft.lua`, `splitShaft.lua`, `differential.lua`, `torsionReactor.lua` |
| `DifferentialSolver.java`, `DifferentialSolverTest.java` | `lua/vehicle/powertrain/differential.lua`; passive LSD, viscous, locked and active-lock constitutive behavior |

`DifferentialSolver.java` also uses the no-slip impulse bound and reduced-inertia
idea from the maintainer's MIT-licensed KinetiForgeVehicles
`Source/KinetiForge/Private/VehicleDifferentialComponent.cpp` (copyright Zhengyi
Miao). BeamCraft combines that stability bound with the bCDDL differential
behavior instead of copying KinetiForge's direct synchronization model.

The following powertrain files are kept as independently implemented BeamCraft
numerical/structural work under MIT: `ImplicitCouplingSolver.java`,
`DctCouplingSolver.java`, `SplitShaftSolver.java`, `TorqueConverterSolver.java`,
and `PowertrainData.java`. Their presence in the same package does not make them
translations of BeamNG code. The implicit spring/coupling integration and the
two-inertia formulation were developed for BeamCraft and informed by the
maintainer's separate KinetiForgeVehicles work. Where these files consume
BeamNG-compatible parameters, that is interface compatibility rather than a
claim that the upstream implementation is identical.

## Source-consulted compatibility facts

These areas use documented formats, observed data semantics, or small
compatibility facts. No source-code adaptation was identified in the September
2026 audit, so their BeamCraft implementation remains MIT unless later evidence
is recorded:

- JBeam assembly/loading outside the bCDDL-marked parser and merger files.
- Node-and-beam soft-body dynamics, stability/stiffness limiting, collision and
  damage logic.
- Flexbody binding and DAE loading.
- Rendering, material interpretation and GPU skinning.
- Rigs of Rods GPLv3 `source/main/physics/flex/FlexBody.cpp`, `FlexBody.h`, and
  `Locator_t.h` were consulted in September 2026 as a comparative reference for
  three-node flexbody locators. No RoR source was copied or structurally ported;
  BeamCraft's binding changes use BeamNG's public flexbody documentation and
  independently derived distance/conditioning math.
- Minecraft entity, networking and gameplay integration.
- `SoftBodyVehicle` body-axis convention, which was checked against
  `lua/ge/extensions/core/cameraModes/autopoint.lua` but was not translated.
- `AdaptiveDamperParser.java`, `AdaptiveDamperController.java` and
  `AdaptiveDamperSpec.java`, which provide BeamCraft's parser, immutable data and
  thread-safe controller infrastructure around the separately bCDDL-marked
  actuator behavior.
- `TorqueReactionSolver.java`, which independently distributes a requested
  torque as zero-net-force node forces using a mass centroid and inertia tensor.
- Test sources, which contain independently written compatibility assertions;
  no copied BeamNG Lua or bundled BeamNG asset fixture was identified.

This classification means "no adaptation identified", not a claim that nobody
ever viewed a BeamNG source file.

## Other influences and tools

- KinetiForgeVehicles supplied independent design experience for implicit
  rotational coupling and one-dimensional driveline modeling. That project was
  itself influenced by publicly available Unreal Engine vehicle-physics
  tutorials; no tutorial source code has been identified in BeamCraft.
- Gemini, ChatGPT, DeepSeek, Codex and Claude Code were used as development
  tools. AI assistance does not erase upstream provenance or change the license
  of source material used to produce an adaptation.
- Fabric Example Mod-derived template portions are CC0 1.0. Bundled libraries
  retain the licenses recorded in `THIRD_PARTY_NOTICES.md`.

## Maintenance rule

When a contributor or coding agent consults BeamNG source code, it must first
check the header of every source file used. If implementation code is adapted,
translated, or structurally ported from a bCDDL file, the contributor must:

1. retain a bCDDL notice in every affected source file;
2. identify the BeamNG source path and the BeamCraft contributor;
3. update this inventory; and
4. keep the covered source available under bCDDL 1.1.

Behavior implemented only from public documentation, interoperability
requirements, mathematical facts, or independently designed tests may remain
MIT. Do not deliberately degrade compatibility merely to make an implementation
look different, and do not describe source-consulted work as clean-room work.
