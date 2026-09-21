# Vehicle-to-Vehicle Collision Broadphase Notes

This document records the September 2026 design discussion and experiments
around BeamCraft's vehicle-to-vehicle collision broadphase. It includes both
design proposals and measured implementations. The purpose is to preserve the
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

The current prototype vehicle-to-vehicle path is node versus triangle through
stable node chunks and whole-triangle meshlets:

1. Static collision membership is partitioned into node chunks and triangle
   meshlets. Complete triangles are never split between meshlets.
2. Every collidable node and triangle receives a swept AABB. Chunk and meshlet
   AABBs are refitted from those primitive bounds.
3. A node-chunk SAP rejects coarse meshlet/chunk pairs.
4. Each surviving node chunk maintains X/Y/Z node orders. A meshlet/chunk pair
   selects the order with the smallest predicted scan, then each triangle runs
   a local one-dimensional query followed by the complete three-axis AABB test.
5. Candidate contacts are greedily colored so contacts in one normal batch do
   not write the same node concurrently. The last batch is a serial overflow.
6. Cached candidates are tested every substep. Separation certificates skip
   most pairs whose relative geometry has not consumed their known clearance.
7. The narrow phase performs position correction, normal impulse, and friction.

The broadphase is rebuilt every ten 2000 Hz substeps, or approximately every
5 ms. Node and triangle swept bounds include previous/current positions and a
linear future prediction over that interval.

Relevant classes:

- `src/client/java/me/mzy/beamcraft/client/physics/PhysicsWorld.java`
- `src/client/java/me/mzy/beamcraft/client/physics/CollisionChunkIndex.java`
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

## Implemented chunk experiments

The prototype was implemented on `codex/collision-chunk-broadphase`. All times
below are rolling in-game averages from two deliberately overlapping copies of
the same vehicle. Later parameter-grid runs used matching vehicle orientation
near a worst-case 45-degree horizontal angle. They are useful comparative
measurements, not a deterministic benchmark: vehicle pose, solver state, JIT,
and especially internal-force time varied between captures.

The preceding whole-vehicle SAP capture used for the closest comparison showed
approximately:

| Stage | Average |
| --- | ---: |
| Total physics | 24.00 ms |
| Global node SAP | 1.49 ms |
| Candidate wall | 4.77 ms |
| Global SAP + candidate wall | 6.26 ms |

### Direct chunk Cartesian product

The first implementation compared every node-chunk AABB with every triangle
meshlet AABB and ran a direct node/triangle loop inside overlapping pairs. A
96-triangle meshlet limit was clearly too coarse: candidate wall reached about
25.14 ms and total physics about 43.27 ms. Reducing the triangle limit to 16
improved candidate wall to about 12.27 ms and total physics to 31.50 ms. This
confirmed that large meshlet AABBs were retaining too much empty space.

A controlled size grid then produced:

| Node/triangle limit | Refit | Candidate wall | Chunk overlap/tested | Fine tested | Total physics |
| --- | ---: | ---: | ---: | ---: | ---: |
| 16 / 8 | 0.45 ms | 8.35 ms | 3,013 / 29,980 | 81,810 | 26.80 ms |
| 16 / 4 | 0.44 ms | 8.43 ms | 2,463 / 46,116 | 65,342 | 27.67 ms |
| 8 / 8 | 0.45 ms | 8.67 ms | 2,452 / 55,454 | 54,944 | 26.38 ms |
| 8 / 4 | 0.46 ms | 9.25 ms | 2,914 / 85,270 | 43,352 | 27.53 ms |

Smaller leaves reduced fine tests but increased chunk-pair bookkeeping enough
to lose overall. The selected working limit remained 16 nodes / 8 triangles.

### Hierarchical local SAP

A node-chunk SAP replaced the coarse Cartesian product, and each overlapping
chunk pair used a small node SAP before the full three-axis AABB test:

| Node/triangle limit | Refit | Candidate wall | Chunk overlap/tested | Fine passed/tested | Total physics |
| --- | ---: | ---: | ---: | ---: | ---: |
| 16 / 8 | 0.77 ms | 6.61 ms | 1,996 / 9,941 | 4,292 / 25,060 | 24.60 ms |
| 32 / 16 | 0.73 ms | 6.98 ms | 1,068 / 3,261 | 4,298 / 35,768 | 26.16 ms |

The larger leaves reduced coarse pair count but increased fine scans, so 16/8
remained better. The approximately 4,300 final AABB passes stayed stable across
correct implementations and are an important lower-bound signal in this fully
overlapped scene.

A joint local endpoint sweep was also tested. It pre-sorted triangle endpoints,
merged four endpoint streams, wrote pair bitmasks, and consumed them in a second
pass. It did not reduce the final scan counts but raised refit to 1.44 ms and
candidate wall to 7.46 ms (about 27.01 ms total), so it was discarded.

### Adaptive local axes and SAH membership

Maintaining X/Y/Z node orders per chunk and selecting the narrowest predicted
scan for each meshlet/chunk pair reduced fine tests from 25,060 to 13,664:

| Variant | Refit | Candidate wall | Chunk overlap/tested | Fine passed/tested | Total physics |
| --- | ---: | ---: | ---: | ---: | ---: |
| Fixed local axis | 0.77 ms | 6.61 ms | 1,996 / 9,941 | 4,292 / 25,060 | 24.60 ms |
| Adaptive X/Y/Z | 1.10 ms | 5.73 ms | 1,961 / 10,546 | 4,306 / 13,664 | 25.56 ms |
| Adaptive + bounded SAH | 1.34 ms | 5.12 ms | 1,372 / 9,140 | 4,309 / 11,846 | 24.54 ms |

The adaptive orders saved 0.88 ms of candidate time but added 0.33 ms of refit
time. Bounded build-time SAH then replaced longest-axis median membership. It
evaluates complete primitive bounds on all axes and prevents children smaller
than half a full leaf. SAH reduced chunk overlaps by about 30% relative to the
adaptive median split, but increased refit cost because the resulting leaf
population and distribution changed. Its net `refit + candidate wall` result
was about 6.46 ms, close to but still above the old whole-vehicle SAP's 6.26 ms.

The SAH capture added a productive-pair counter. Of 1,372 overlapping chunk
pairs, 874 (63.70%) produced at least one complete node/triangle AABB pass.
Consequently 36.30% of coarse overlaps were empty, but most overlaps represented
real fine candidates. Better grouping still has room to help, but cannot remove
the approximately 4,300 irreducible AABB candidates in this test.

Refit instrumentation then separated node bounds, node-chunk reduction, coarse
SAP, local SAP, triangle bounds, and meshlet reduction. Local SAP was initially
the largest component at about 0.71 ms of a 1.27 ms refit. Its own breakdown was
approximately 0.13 ms generating keys, 0.45 ms sorting, and 0.14 ms rebuilding
prefix maxima.

The implementation had been regenerating keys in static membership order before
every `Arrays.sort`, discarding temporal coherence. Retaining the previous
sorted node IDs, updating their coordinate keys in place, and still calling the
JDK sort reduced the measured sort average from about 0.45 ms to 0.06 ms. Local
SAP fell to about 0.31 ms and complete chunk refit to about 0.65 ms. This avoided
a custom insertion sort while still giving the JDK an almost-sorted primitive
array. The corresponding capture showed approximately 0.11 / 0.06 / 0.14 ms for
key generation / sort / prefix maxima. Prefix construction, not sorting, is now
the largest of those three local operations.

### Conclusions from the chunk prototype

- Primitive budget alone is not enough: oversized or spatially incoherent
  meshlets can dominate candidate cost.
- Smaller chunks are not monotonically faster because chunk-pair dispatch and
  local query overhead eventually exceed the saved fine AABB tests.
- SAH membership and adaptive axes both work, but their gains are modest after
  refit overhead is included.
- A custom joint sweep added machinery without reducing candidates and should
  remain discarded.
- Total-physics averages must not be compared without stage times; internal
  force varied by around a millisecond between several captures.
- The current implementation still rebuilds the broadphase every ten substeps.
  It has not yet implemented the original proposal to check chunk AABBs every
  substep.
- Merely moving all primitive maintenance to every substep is unlikely to work.
  The earlier persistent SAP showed that insertion sorting does not eliminate
  the cost of scanning every node, triangle, and triangle vertex.
- A future per-substep design should update only coarse chunk bounds frequently
  and reuse conservative fine-candidate supersets until a chunk pair is new or
  its fat/swept validity bound is escaped.

## Proposed per-substep two-rate pipeline

The original goal was to compare collision-chunk AABBs every substep. That does
not require rebuilding every primitive-level SAP and candidate list every
substep. The proposed design separates cheap discovery from expensive fine
candidate generation.

### Static data built or rebuilt on topology changes

- bounded-SAH node-chunk and triangle-meshlet membership;
- one primary node chunk for every collidable node;
- the unique referenced-node list for every triangle meshlet;
- node-to-meshlet reverse adjacency, used to invalidate only meshlets affected
  by an escaping node or broken triangle;
- self-collision/topology metadata and oversized-triangle isolation;
- stable identifiers for directed meshlet/node-chunk pairs.

Meshlet coarse bounds should be reduced directly from their unique referenced
nodes. Scanning every triangle and its three vertices merely to discover coarse
meshlet overlap would repeat the main failure mode of the full persistent SAP.
Tight individual triangle bounds are needed only when a fine cache is rebuilt.

### Work performed every substep

1. Integrate/update nodes as today.
2. While node positions are already hot, compute previous-to-current swept node
   bounds, reduce node-chunk bounds, and check containment in each node's cached
   fat validity bound.
3. Reduce coarse triangle-meshlet bounds from precomputed unique referenced-node
   lists. A node that escapes its fat bound marks its node chunk and referencing
   meshlets dirty through the reverse adjacency.
4. Compare coarse meshlet/node-chunk swept AABBs for relevant vehicle pairs.
   Retain pair-state bits so new overlap, continuing overlap, and separation are
   distinguishable without reconstructing a hash table.
5. Remove caches for separated pairs. Rebuild only newly overlapping or dirty
   pair caches. Continuing valid pairs reuse their conservative candidate
   supersets.
6. Present the active cached candidates to the existing coloring and narrow
   phase. The point-triangle CCD still operates on actual substep motion.

The coarse bound must cover previous-to-current motion so a pair that crosses
entirely during one substep is still discovered. Longer absolute-velocity
prediction is unnecessary for this frequent pass. A relative-motion slab gate
can later reject pairs whose conservative chunk velocity ranges cannot close,
but it is an optional optimization rather than a correctness dependency.

### Fine-candidate cache validity

When rebuilding one meshlet/node-chunk pair, generate candidates from fat node
bounds and fat triangle bounds covering a chosen horizon. The resulting list is
a conservative superset. It remains complete while every contributing node
stays inside its stored fat bound. Because triangle vertices are nodes, vertex
containment also conservatively contains the triangle bound.

A cache is invalidated by:

- any contributing node escaping its fat bound;
- the pair becoming newly overlapping after separation;
- a triangle break or collision/topology flag change;
- local regrouping after fracture;
- an explicit maximum-age fallback while the scheme is being validated.

Start with whole-pair rebuild on invalidation. Only introduce partial primitive
updates if measurements show pair rebuilds are frequent and expensive. Track
cache rebuild count, average lifetime, escape reason, candidate count, and time
spent in coarse versus fine work.

This design preserves the current useful behavior: once a conservative contact
candidate exists, separation certificates can cheaply reject it on later
substeps. The new cache controls when the candidate set itself must change.

## Candidate-generation cost and parallelism

The SAH/order-reuse capture still showed about 5.36 ms candidate wall time for
roughly 8,712 tested chunk pairs, 1,456 overlaps, 12,608 complete fine AABB
tests, and 4,457 passes. Candidate generation is therefore the dominant
broadphase stage even after refit sorting became cheap.

The current code calls `activeVehicles.parallelStream()` and assigns one
candidate-generation task per triangle vehicle. With two vehicles this exposes
only two large tasks. Every accepted candidate also calls
`SoftBodyCollisionManager.addContact`, whose shared `AtomicInteger` reserves a
global output slot. Splitting the same loop into more tasks without changing
output ownership would increase atomic contention and scheduling overhead.

The preferred parallel form is deterministic buffered generation:

1. Build tasks from directed vehicle-pair meshlet ranges, or from dirty
   meshlet/node-chunk pair ranges in the two-rate design.
2. Each task writes contacts and counters into an exclusive reusable buffer;
   it performs no atomic increment per contact.
3. After the parallel join, compute a prefix sum of task counts, clamp once to
   global capacity, and copy buffers into the manager's SoA arrays.
4. Merge tasks in stable range order so overflow behavior and solver ordering
   remain reproducible.
5. Run small workloads sequentially; create multiple tasks only above a measured
   meshlet/pair threshold.

This can use more cores than vehicle-level parallelism and removes the current
per-contact atomic. It needs a bounded, allocation-free buffer strategy before
implementation. A task must not reserve `MAX_CONTACTS` independently. Practical
options are preallocated worker slices plus an overflow path, or a two-pass
count/fill scheme if repeating the cheap portion proves faster than oversized
buffers.

Reducing computation is preferable to parallelizing it. Priorities are:

1. Reuse valid fine-candidate supersets so most substeps do no local SAP or fine
   candidate generation.
2. Build coarse meshlet bounds from unique referenced nodes rather than scanning
   all triangle vertices every substep.
3. Generate fine candidates only for new/dirty chunk pairs.
4. Buffer contact output to remove atomics and then parallelize dirty pair or
   meshlet ranges.
5. Instrument coarse traversal, local query, full AABB, topology filtering, and
   output/merge separately before attempting lower-level loop rewrites.

The two directed cross-vehicle passes are both required: nodes of A against
triangles of B and nodes of B against triangles of A detect different contacts.
They are not duplicate work that can simply be removed.

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

The direct chunk, hierarchical SAP, joint-sweep, adaptive-axis, bounded-SAH,
and refit-order experiments have now been measured. The next work should avoid
another broad rewrite and proceed in this order:

1. Instrument candidate generation by coarse traversal, local query/full AABB,
   topology filtering, and contact output/merge. Refit is no longer the largest
   unknown after temporal ordering reduced it to roughly 0.65 ms.
2. Precompute unique referenced-node lists per meshlet and node-to-meshlet
   reverse adjacency, prerequisites for cheap coarse bounds and local cache
   invalidation.
3. Measure a coarse-only per-substep path separately. It should update chunk
   bounds and detect new chunk-pair overlap without rebuilding all fine
   node/triangle candidates.
4. Cache a conservative fine-candidate superset per chunk pair. Rebuild only
   when the pair is new, a primitive escapes its fat/swept validity bounds, or a
   fracture/regroup event invalidates membership.
5. If dirty-pair rebuild wall time remains material, replace per-vehicle tasks
   and atomic contact insertion with stable meshlet/pair-range tasks, reusable
   local buffers, a prefix sum, and one deterministic merge.
6. Improve the CCD trigger so cached broadphase candidates that genuinely cross
   a face are not lost in narrow phase.
7. Add local/event-driven regrouping for fracture-induced bound inflation.
8. Consider virtual subdivision only for measured oversized-triangle hot spots,
   and selected edge collision only after data shows it is necessary.

Do not retry the full primitive persistent SAP or the joint local endpoint
sweep without materially different evidence. Both already paid substantial
linear scan/bookkeeping cost without improving the measured result.

## Open questions

- How much of the remaining candidate wall is local binary querying and full
  AABB work versus topology filtering, atomic output, and task imbalance?
- How many self-collision nodes and candidates exist in representative vehicles?
- What is the distribution of collision triangle size, aspect ratio, and unique
  node reuse, especially for pressure wheels?
- Do vehicles other than the tested Bastion favor the current 16/8 budgets and
  bounded SAH membership?
- How often would fat bounds invalidate during ordinary driving, suspension
  movement, wheel rotation, and actual crashes?
- Can a per-substep coarse chunk pass plus cached fine-candidate supersets avoid
  tunnelling without repeating the full primitive scan?
- How frequently are observed tunnelling events broadphase misses, narrow CCD
  misses, edge-edge cases, or contact-capacity/order effects?

These questions should be answered with separate-run or replay measurements
before selecting a final broadphase architecture.
