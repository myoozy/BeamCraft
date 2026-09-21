# Vehicle-to-Vehicle Collision Broadphase Notes

This document records the September 2026 design discussion around BeamCraft's
vehicle-to-vehicle collision broadphase. It is a design note, not a description
of code that has already been implemented. The purpose is to preserve the
measurements, failed experiments, constraints, and promising directions so a
future implementation does not have to rediscover them.

## Goals

The target is not mathematically perfect continuous collision detection at any
speed. The desired trade-off is:

- avoid most tunnelling at ordinary and moderately high relative speeds;
- retain stable contact response during deformation and fracture;
- keep the no-contact and ordinary-driving cost low;
- keep a two-vehicle collision comfortably inside the physics budget;
- avoid designs whose worst-case bookkeeping freezes the client.

Minecraft block collision is intentionally outside this investigation. Its
simple per-node response is cheap, sufficiently accurate for the game, and has
the desirable property that a node spawned inside a block remains trapped
instead of being teleported out.

## Current pipeline

The current vehicle-to-vehicle path is node versus triangle:

1. Every collidable node receives a swept AABB.
2. `DynamicAxisSweep` sorts nodes and answers triangle-AABB queries.
3. Every collidable, unbroken triangle queries the node SAP.
4. Candidate contacts are greedily colored so contacts in one normal batch do
   not write the same node concurrently. The last batch is a serial overflow.
5. Cached candidates are tested every substep. Separation certificates skip
   most pairs whose relative geometry has not consumed their known clearance.
6. The narrow phase performs position correction, normal impulse, and friction.

The broadphase is rebuilt every ten 2000 Hz substeps, or approximately every
5 ms. Node and triangle swept bounds include previous/current positions and a
linear future prediction over that interval.

Relevant classes:

- `src/client/java/me/mzy/beamcraft/client/physics/PhysicsWorld.java`
- `src/client/java/me/mzy/beamcraft/client/physics/DynamicAxisSweep.java`
- `src/client/java/me/mzy/beamcraft/client/physics/CollisionPipeline.java`
- `src/client/java/me/mzy/beamcraft/client/physics/SoftBodyCollisionManager.java`

## Measured two-vehicle baseline

A 2026-09-15 test on `codex/collision-separation-cache` reported the following
rolling averages. The total average was **23.73 ms**, not 29 ms.

| Stage | Average |
| --- | ---: |
| Total physics | 23.73 ms |
| Internal forces | 11.87 ms |
| Global node SAP | 1.37 ms |
| Candidate generation wall time | 4.62 ms |
| Contact coloring | 0.12 ms |
| Soft-body contact solve | 3.58 ms |
| Minecraft collision | 1.85 ms |

The vehicle-to-vehicle collision stages therefore consumed about 9.69 ms:

```text
global SAP 1.37 + candidate/color 4.74 + soft collision 3.58 = 9.69 ms
```

The same capture showed:

```text
raw SAP hits                         4263
stored candidates                    1138
dropped candidates                      0
narrow checks over the physics step 113340
separation-certificate skips         97552
current-AABB passes                  15650 (13.8%)
resolved contacts                     1500
```

Important conclusions from this capture:

- Candidate capacity was not the problem in this test (`dropped = 0`).
- Approximately 86% of repeated narrow checks were cheaply rejected by the
  separation cache.
- Only a small fraction of cached candidates became real contacts.
- Candidate generation was expensive even while the vehicles were nearly
  stationary and overlapping. High-speed prediction is therefore not the only,
  or even the first, explanation for the measured candidate cost.
- Internal force and Minecraft collision phases scale well across vehicles
  because they are per-vehicle parallel phases. The major second-vehicle cost is
  the shared vehicle-to-vehicle collision path.

## Why a triangle query is not O(1)

Sorting the nodes does not make one triangle query constant time. A query
currently performs work resembling:

```text
for every vehicle segment
    inspect vehicle/part gates
    binary-search the sorted node segment       O(log N)
    scan every possible 1D overlap              O(k)
    reject non-overlap on the other two axes
    apply topology/self-collision filters
```

For `T` triangles the total candidate-generation shape is closer to:

```text
O(T * (vehicle/part gating + log N + k))
```

where `k` grows when swept intervals become long or geometry overlaps heavily.
At 100 substeps per Minecraft tick and one rebuild every ten substeps, every
collidable triangle performs this query ten times per tick.

Primitive `Arrays.sort(long[])` can be faster than this repeated query work in
practice because it is optimized and accesses contiguous memory. The observed
cost does not imply that sorting itself is the dominant operation.

## Current branch versus `dev`

Measurements indicate that `dev` spends roughly one third as much time in its
node SAP, but more time in candidate generation. The total physics cost is
similar.

The current branch performs additional SAP-side work:

- separate sorted node segments and active axes per vehicle;
- vehicle and authored-part swept bounds;
- a derived self-collision-only sorted view;
- normal and self-collision prefix maxima;
- previous/current/future node bounds;
- repeated `findOrCreatePart` work while inserting nodes.

This moves some work from candidate generation into SAP construction. It also
means there are low-risk optimizations worth trying before replacing the whole
broadphase:

- precompute the static node-to-SAP-part mapping instead of rediscovering it on
  every rebuild;
- use the already-computed vehicle AABB as an early reject before scanning part
  bounds;
- build self-collision data only for vehicles that actually have self-collision
  nodes;
- precompute vehicle/part-pair gates once per broadphase instead of repeating
  the same authored-part loop for every triangle.

Authored-part gates reduce candidates, but during an actual vehicle overlap a
boolean "some part overlaps" gate may still lead to a query of the entire
vehicle node segment.

## Persistent SAP experiment

The branch `codex/persistent-sap-authoritative` (notably commits `2f126ab` and
`77ea60b`) explored a tick-local persistent node-and-triangle SAP.

At substep zero it:

- assigned proxies to all collidable nodes and triangles;
- rebuilt their three-axis AABBs;
- sorted three endpoint arrays;
- rebuilt an active node-triangle-pair hash table.

For each of the remaining substeps it still:

- scanned all nodes;
- scanned all triangles and their vertices;
- refreshed every endpoint key on all three axes;
- insertion-repaired all three sorted arrays;
- updated and purged the active-pair hash table;
- serially solved the active pairs in the authoritative path.

Few endpoint swaps did not make this cheap: the implementation had already paid
the full `O(nodes + triangles)` bound-refresh cost and touched several large
arrays before repairing order. Its active path also lost the current colored
contact batching.

The useful idea from this experiment is not necessarily the complete persistent
data structure. It is the guarantee that a pair can become active from actual
substep motion rather than only from a longer prediction made several substeps
earlier.

Persistent SAP may still be appropriate at a much coarser level with tens of
proxies, but it was too expensive over all nodes and triangles.

## Prediction, absolute speed, and relative speed

Long prediction horizons cause broadphase inflation. A 5 ms horizon extends a
bound by:

| Speed | Distance |
| ---: | ---: |
| 20 m/s | 0.10 m |
| 50 m/s | 0.25 m |
| 100 m/s | 0.50 m |

This can create many false candidates. It is especially wasteful when two
objects share the same large world-space velocity: their relative configuration
does not change, but both world-space swept AABBs become long.

The intuition "same direction and same speed probably will not collide" is
quantifiable as relative motion. At a coarse vehicle/group level, velocity
ranges and a swept-AABB slab test can reject pairs whose three-axis overlap-time
intervals never intersect. For deforming groups, minimum/maximum member velocity
per axis gives a conservative range.

Prediction horizon should ideally depend on relative motion and a maximum sweep
distance, not absolute vehicle speed alone. A fast-closing pair may use a one- or
two-substep horizon while a same-speed convoy can use a much longer horizon.

No predictor is perfect during impulses, fracture, and rapidly changing
acceleration. A system must choose between more rebuilds, larger conservative
bounds, or occasional missed candidates.

## Fat-bound validity instead of trusting prediction

One possible hybrid is to treat a predicted/fat bound as a certificate:

1. At fine-broadphase rebuild time, store each node's fat bound for the chosen
   horizon.
2. During the existing node integration loop, check whether the actual node
   remains inside that bound.
3. If all nodes remain inside, triangle bounds formed from those nodes remain
   conservatively contained too, so the cached broadphase is still valid.
4. If a node escapes, mark the relevant collision cache dirty and rebuild before
   relying on it again.

The simplest implementation would rebuild everything and can thrash during
rapidly changing motion. A better implementation invalidates only vehicle-pair
or coarse-group-pair caches involving the dirty vehicle/group. Repeated failures
can temporarily shorten the horizon, enlarge the observed prediction-error
margin, or fall back to per-substep rebuilding for the active collision island.

This does not remove the fundamental trade-off, but it makes missed predictions
detectable instead of silently trusting them.

## Per-substep authored-part AABBs

Part AABBs are cheap to update while the node loop is already running. Updating
and comparing them every substep is logically reasonable as a frequent, cheap
coarse broadphase above a less frequent fine broadphase.

They can:

- reject completely separated part pairs;
- detect the substep in which a part pair first becomes overlapping;
- trigger local fine-candidate generation without a long coarse prediction.

They cannot discover a new node-triangle interaction inside a part pair that
has remained overlapping for many substeps. Such a pair still needs a fine
cache-validity mechanism or periodic/local rebuilding.

Authored parts are also weak spatial proxies:

- parts are configuration/topology concepts and may overlap heavily by design;
- a broken part can contain two fragments moving apart, stretching one part
  AABB across a large empty volume.

For these reasons authored parts are useful gates and initialization boundaries,
but should not automatically be the final spatial acceleration units.

## Fracture and connectivity

Recomputing a complete connected-components graph every substep is unnecessary
and potentially expensive. Beam breaks are events, so connectivity work can be
event-driven and batched:

1. Commit all breaks for a substep.
2. Mark affected collision groups dirty.
3. Process each dirty group at most once after the break batch.

If physical connectivity is required, a dirty component can be locally searched
through remaining structural links. However, collision acceleration groups do
not have to equal physical connected components. A spatially stretched group
may be split for broadphase quality even while alternate beams still connect its
two sides.

Useful rebuild triggers include excessive current/rest AABB diagonal, surface
area, or aspect-ratio growth. Rebuilding only the affected small group is more
attractive than maintaining an exact whole-vehicle decremental connectivity
structure at 2000 Hz.

## Collision chunks are not Minecraft-style voxels

A previous experiment inserted every node and triangle AABB into a world-space
voxel hash and was unusably slow despite avoiding allocation. Plausible causes
include:

- Minecraft-sized cells containing hundreds of dense primitives and degenerating
  into local all-pairs tests;
- smaller cells duplicating swept triangle AABBs across many buckets;
- duplicate node-triangle pairs produced by several cells;
- hash probes, bucket writes, clearing, and poor memory locality;
- higher speed increasing the number of cells touched by swept bounds.

Avoiding allocation does not avoid memory bandwidth, hash, branch, and
combinatorial candidate costs.

A collision chunk/meshlet is different. It is a stable group of primitives in a
vehicle, not exclusive ownership of a world-space cell. Chunks may overlap in
space, and their bounds move with the vehicle.

## Choosing chunk size

Do not choose one fixed size in meters or Minecraft blocks for every vehicle.
Instead fix a computational budget and let physical size adapt to local density.
Possible starting constraints for experimentation are:

- a maximum number of unique nodes per node chunk;
- a maximum number of triangles and unique referenced nodes per triangle
  meshlet;
- a maximum rest-space aspect ratio or spatial cost;
- splitting when a surface-area-style cost improves sufficiently.

For example, an initial experiment might explore leaves around 32--64 unique
nodes and 64--128 triangles, but these are test ranges, not selected production
constants.

A dense wheel then produces physically smaller leaves, while sparse geometry
produces larger leaves. Elongated groups split even if their primitive count is
low. Triangle adjacency or centroid ordering can keep groups compact. The total
number of unique-node references across meshlets must be monitored so boundary
sharing does not recreate a three-reads-per-triangle scan.

## Cross-chunk triangles

Space must not be divided into mutually exclusive cells. Nodes and triangles
need separate groupings:

- every collision node can have one primary node chunk;
- every complete triangle belongs to one triangle chunk/meshlet;
- a triangle is never physically split merely because its vertices belong to
  different node chunks;
- boundary nodes may be referenced by several triangle meshlets when updating
  their bounds, while the physical node itself remains unique.

A triangle chunk's AABB includes all three vertices of every triangle it owns.
It is therefore correct even if a triangle's three vertices belong to three
different node chunks or its interior spans several spatial regions. The cost is
reduced pruning, not missed collision.

An unusually large triangle should normally receive its own leaf so it does not
inflate a regular meshlet. If real vehicle data shows that a few oversized
triangles dominate candidate cost, they can be subdivided only for broadphase
into virtual barycentric patches. These patches add no physical nodes, mass, or
beams; a patch hit maps back to the original triangle and its three physical
nodes. This is an advanced fallback, not a default for every triangle.

## Coarse chunk broadphase options

If collision chunks are introduced, start with the simplest method:

```text
reject by vehicle AABB
then compare node-chunk AABBs with triangle-chunk AABBs directly
then run fine node-triangle broadphase only for overlapping chunk pairs
```

With a small number of chunks, contiguous all-pairs AABB checks may be cheaper
and easier to validate than another complex structure. If chunk count grows,
options include:

1. Re-sort chunk endpoints every substep with primitive `Arrays.sort`.
2. Maintain a persistent SAP over only tens or hundreds of chunk proxies.
3. Refit a BVH over chunks.

A refitted BVH does not retain initialization-time split planes. Every substep
updates leaf AABBs and then replaces every parent AABB with the union of its
current children. Rotation, translation, and changing leaf sizes therefore do
not make it incorrect; severe deformation only reduces pruning quality. A poor
subtree can be rebuilt when a quality metric degrades. Nevertheless, BVH
complexity is not justified until direct chunk-pair checks or chunk SAP are
measured to be too expensive.

## Narrow-phase tunnelling gaps

Broadphase completeness alone does not prevent tunnelling. The current swept
point-triangle CCD is only attempted when the final node lies outside the
triangle's current tight AABB. Crossings whose endpoints remain inside that AABB,
or whose endpoint projection has moved outside the triangle, may bypass CCD.

A more complete but still selective trigger is:

1. Compute previous/current oriented-volume signs for a cached candidate.
2. If they differ, run moving/deforming triangle TOI regardless of the final
   current-AABB result.
3. Use fewer robust bisection iterations (for example eight) unless evidence
   requires more.
4. Preserve the original side and ensure CCD correction cannot leave the node
   on the crossed side merely because a fixed positional push cap was reached.

Node-triangle collision also cannot detect a pure edge-edge crossing where no
vertex crosses the opposing face. Full edge-edge CCD is likely too expensive as
a first response. Possible later mitigations include collision radius around
nodes/triangle features or selected boundary/sharp-edge tests.

## Benchmarking constraints

Do not run two complete broadphases as a live shadow test in the current
two-vehicle scene. With a 23.73 ms average and peaks above 27 ms, the shadow work
would distort the measurement and risk making the client unusable.

Prefer one of:

- separate branches/configuration modes run against the same repeatable scene;
- capture one broadphase state and replay algorithms outside the live physics
  tick with JIT warm-up;
- add cheap counters first and compare separate runs.

Useful counters include:

- bounds construction, sorting, gating, query/sweep, coloring, and narrow times;
- raw one-axis overlaps, 3D overlaps, topology-filtered candidates, and drops;
- candidate lifetime and actual resolve count;
- per-vehicle/part/chunk-pair candidate counts;
- fat-bound invalidations and early rebuild count;
- chunk membership duplication and current/rest bound inflation.

## Recommended investigation order

The discussion converged on the following staged approach rather than an
immediate rewrite:

1. Remove avoidable fixed overhead from the current SAP: static node/part
   mapping, vehicle early reject, conditional self data, and cached coarse gates.
2. Improve the current CCD trigger so broadphase candidates that genuinely cross
   a face are not lost in narrow phase.
3. Re-measure the exact two-vehicle scene in a separate run.
4. If repeated per-triangle queries still dominate, test a from-scratch joint
   node/triangle sweep at each broadphase rebuild, using optimized primitive
   sorting and no persistent pair hash.
5. Separately prototype coarse collision chunks/triangle meshlets with direct
   chunk-pair checks. Keep nodes and whole triangles in independent groupings.
6. Add per-substep chunk maintenance only if it demonstrably reduces fine
   candidate scans. Use chunk SAP or BVH only after direct checks are measured.
7. Add local/event-driven regrouping for fracture-induced bound inflation.
8. Consider virtual subdivision of only demonstrably oversized triangles or
   selected edge collision only after real data shows they are necessary.

## Open questions

- How much of the current 4.62 ms candidate wall is repeated authored-part
  gating versus binary query/scanning versus atomic candidate insertion?
- How many self-collision nodes and candidates exist in representative vehicles?
- What is the distribution of collision triangle size, aspect ratio, and unique
  node reuse, especially for pressure wheels?
- What chunk primitive budgets minimize total coarse-update plus fine-candidate
  time on real vehicles?
- How often would fat bounds invalidate during ordinary driving, suspension
  movement, wheel rotation, and actual crashes?
- Does a joint rebuild sweep outperform repeated triangle queries after the
  current separation and part-gating improvements?
- How frequently are observed tunnelling events broadphase misses, narrow CCD
  misses, edge-edge cases, or contact-capacity/order effects?

These questions should be answered with separate-run or replay measurements
before selecting a final broadphase architecture.
