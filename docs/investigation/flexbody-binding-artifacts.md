# Flexbody binding artifacts

This note records the investigation behind the topology-aware flexbody binding
introduced in September 2026. The main reproduction vehicle was the stock
BeamNG ETK800 configuration `846x_ttsport_plus_DCT`.

## Symptoms

- Attached front bumpers produced black spikes, suspended fragments and sharp
  folds after an impact. Removing the bumper before the impact made the rest of
  the front end look substantially better.
- Tightening the locator limit could suppress deformation entirely and leave
  vertices that no longer followed vehicle rotation.
- Applying the safe three-node representation to every mesh improved damaged
  body panels, but made tire tread move in visible steps while steering.

The problem was therefore not simply "four nodes are bad". Volumetric meshes
such as generated tires benefit from a real fourth locator, while thin body
panels are often more stable with a normal derived from a three-node plane.
Mixing both representations arbitrarily within one continuous mesh can itself
create discontinuities.

## Binding model

Every deforming vertex uses a center node plus two planar locator nodes. Its
third deformation direction is one of:

1. an explicit VZ vector from the center to a fourth node; or
2. the normalized cross product of the two planar axes.

The explicit form follows a genuinely volumetric node cage and behaves well for
tires. It becomes dangerous when the fourth node belongs to a nearby but
independently moving bumper, lamp, liner or detachable structure. The cross
form avoids that dependency, but cannot reproduce all volumetric motion and
becomes undefined when its planar triangle collapses.

## Structural graph

`BeamGraph` is a reusable immutable adjacency view built when vehicle assembly
finishes. It exposes both complete adjacency and a narrower *cohesive* graph.
The cohesive graph excludes support beams and beams assigned to a break group:
those constraints do not reliably mean that two nodes will remain part of the
same local structure after damage.

An explicit VZ candidate must:

- be directly connected to at least one of the three planar cage nodes;
- reach all three within the configured cohesive hop radius;
- form a sufficiently non-planar basis; and
- keep its affine node gain within the safety bound.

This is a structural test, not a mesh-name, part-name or wheel special case.

## Mesh-level decision

Binding is performed in two stages:

1. `CONFIDENCE_PROBE` conservatively tests each vertex. In addition to the
   structural requirements, explicit VZ must improve displacement sensitivity
   enough to justify the extra node dependency.
2. If at least `MIN_EXPLICIT_Z_MESH_COVERAGE` of the mesh passes, the complete
   mesh is rebound in `STRUCTURAL` mode. Otherwise its explicit vertices are
   rebound in `DISABLED` mode and the mesh consistently uses cross-derived Z.

This prevents a continuous surface from being split between deformation models
while allowing a strongly volumetric structure to use four nodes throughout.
The policy constants live together at the top of `FlexbodyBindingUtil`; they
should be changed with corpus audit evidence rather than adjusted per vehicle.

## Collapsed cross basis

Previously, a collapsed planar basis fell back to a world-up direction while
retaining the full normal offset. That could launch vertices away from the
damaged panel and produce a spike.

Each cross-bound vertex now stores the rest-pose cross magnitude. The transform
shader compares the current magnitude with that rest value and smoothly fades
the thickness offset as the triangle collapses. An inversion therefore passes
through a flat state instead of switching to an unrelated world-space axis.
The fade limits are named shader constants.

## ETK800 audit result

The local corpus audit after the change produced these representative results:

| Mesh | Explicit VZ | Cross-derived Z | Reason |
| --- | ---: | ---: | --- |
| `etk800_bumper_F_sport` | 0 / 8555 | 8555 | Thin detachable surface |
| `etk800_duct_F` | 0 / 280 | 280 | Shares the bumper structure |
| `etk800_hood_sport` | 0 / 1738 | 1738 | Thin panel |
| `tire_265_35_19_sport` | 1078 / 1078 | 0 | Volumetric generated wheel cage |
| `etk_brakedisc_*_carbon` | 178 / 178 | 0 | Cohesive rotating structure |
| front sport brake ducts | 195 / 195 | 0 | Strong local structural consensus |

The audit also checks that brake calipers do not bind to detachable wheel-axis
nodes, bumper/duct locator cages remain local, explicit bindings respect the
affine-gain limit, and tires use one continuous four-node representation.

Run the focused tests with:

```powershell
.\gradlew.bat test --tests me.mzy.beamcraft.client.model.FlexbodyBindingUtilTest --tests me.mzy.beamcraft.client.physics.BeamGraphTest
```

Run the stock ETK800 corpus audit with:

```powershell
$env:BEAMCRAFT_JBEAM_CORPUS='E:\Games\Steam\steamapps\common\BeamNG.drive\content\vehicles'
.\gradlew.bat test --tests me.mzy.beamcraft.client.model.Etk800FlexbodyAuditTest --info
```

The corpus test is intentionally opt-in because BeamNG assets are user-supplied
and must not be bundled with BeamCraft.

## Future investigation

- Test other vehicles whose flexbodies contain multiple mechanically distinct
  regions in one render mesh. Mesh-wide consistency may eventually need
  connected render-surface regions rather than one decision for the entire DAE
  geometry.
- Keep the collapse fade relative to rest-pose area. Absolute world-unit limits
  would regress differently scaled vehicles.
- When a new artifact appears, first log the selected locator node names,
  cohesive connections, affine gain and explicit/cross coverage. Raising a
  global limit without that evidence can trade spikes for detached vertices.
