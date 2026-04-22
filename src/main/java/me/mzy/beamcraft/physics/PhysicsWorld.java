package me.mzy.beamcraft.physics;

import net.minecraft.world.World;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import java.util.List;
import java.util.Arrays;

/**
 * Core physical world controller for beam-based vehicle simulation
 * Manages nodes, beams, collision caching and physics integration
 */
public class PhysicsWorld {
    public static final double GRAVITY = -9.81;
    public static final double SOUND_SPEED = 340;
    public static final double BLOCK_REBOUND = 0.0;
    public static final double METAL_PLASTIC_FLOW_RATE = 10.0;
    public static final double KINDA_SMALL_NUMBER = 1e-8;
    public static final double KINDA_BIG_NUMBER = 1e8;
    public static final int MAX_AABB_SIZE = 30;
    public final VoxelSnapshot snapshot = new VoxelSnapshot();

    public final NodeContainer nodes = new NodeContainer();
    public final BeamContainer beams = new BeamContainer();
    public final TriangleContainer triangles = new TriangleContainer();
    public final TorsionBarContainer torsionbars = new TorsionBarContainer();
    public final SlideNodeContainer slidenodes = new SlideNodeContainer();

    // Bounding box cache array for independent part culling
    private int maxTrackedPartId = -1;
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    public PhysicsWorld() {
        // Empty constructor, data will be injected by JBeam parser
    }

    /**
     * Expand array capacity to avoid index out of bounds for new part id
     */
    private void ensurePartCapacity(int maxId) {
        if (maxId >= partMinX.length) {
            int newSize = maxId * 2;
            partMinX = Arrays.copyOf(partMinX, newSize);
            partMinY = Arrays.copyOf(partMinY, newSize);
            partMinZ = Arrays.copyOf(partMinZ, newSize);
            partMaxX = Arrays.copyOf(partMaxX, newSize);
            partMaxY = Arrays.copyOf(partMaxY, newSize);
            partMaxZ = Arrays.copyOf(partMaxZ, newSize);
            partActive = Arrays.copyOf(partActive, newSize);
        }
    }

    /**
     * Register node into physics world and expand part bounding box cache
     */
    public void addNode(String name, double x, double y, double z, double nodeMass, double friction, int partId, boolean collision) {
        nodes.addNode(name, x, y, z, nodeMass, friction, partId, collision);

        // Calculate current maximum part id and expand buffer
        int currentMaxPartId = -1;
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.partId[i] > currentMaxPartId) {
                currentMaxPartId = nodes.partId[i];
            }
        }
        maxTrackedPartId = currentMaxPartId;
        ensurePartCapacity(currentMaxPartId);
    }

    /**
     * Create physical beam constraint between two existing nodes
     */
    public void addBeam(String name1, String name2, double spring, double damp, double deform, double strength,
                        int type, double precomp, double shortBound, double longBound,
                        double shortBoundRange, double longBoundRange,
                        double limitSpring, double limitDamp) {

        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];

            double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);

            beams.addBeam(n1, n2, dist,spring,damp,deform,strength,type,precomp,shortBound,longBound,shortBoundRange,longBoundRange,limitSpring,limitDamp);
        }
    }

    /**
     * Register collision triangle face composed of three nodes
     */
    public void addTriangle(String name1, String name2, String name3, int triPartId) {
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2) && nodes.nameToIndex.containsKey(name3)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            int n3 = nodes.nameToIndex.get(name3);

            triangles.addTriangle(n1, n2, n3, triPartId);
        }
    }

    /**
     * Spawn torsion bar joint with four control nodes and physical properties
     */
    public void addTorsionBar(String name1, String name2, String name3, String name4,
                              double spring, double damp, double deform, double strength) {

        // Verify all node exists
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2) &&
                nodes.nameToIndex.containsKey(name3) && nodes.nameToIndex.containsKey(name4)) {

            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            int n3 = nodes.nameToIndex.get(name3);
            int n4 = nodes.nameToIndex.get(name4);

            double px1 = nodes.posX[n1]; double py1 = nodes.posY[n1]; double pz1 = nodes.posZ[n1];
            double px2 = nodes.posX[n2]; double py2 = nodes.posY[n2]; double pz2 = nodes.posZ[n2];
            double px3 = nodes.posX[n3]; double py3 = nodes.posY[n3]; double pz3 = nodes.posZ[n3];
            double px4 = nodes.posX[n4]; double py4 = nodes.posY[n4]; double pz4 = nodes.posZ[n4];

            torsionbars.addTorsionBar(n1, n2, n3, n4,
                    px1, px2, px3, px4,
                    py1, py2, py3, py4,
                    pz1, pz2, pz3, pz4,
                    spring, damp, deform, strength);
        }
    }

    /**
     * Calculate closest rail segment and add sliding node constraint
     */
    public void addSlidenode(String node, String[] railNodes, double spring, double damp) {
        if (!nodes.nameToIndex.containsKey(node)) return;
        int nId = nodes.nameToIndex.get(node);

        int bestA = -1;
        int bestB = -1;
        double minDist = Double.MAX_VALUE;
        double bestRestDist = 0;

        // Geometry pre-calculate: find nearest rail segment
        for (int i = 0; i < railNodes.length - 1; i++) {
            if (!nodes.nameToIndex.containsKey(railNodes[i]) || !nodes.nameToIndex.containsKey(railNodes[i+1])) continue;
            int aId = nodes.nameToIndex.get(railNodes[i]);
            int bId = nodes.nameToIndex.get(railNodes[i+1]);

            double nx = nodes.posX[nId], ny = nodes.posY[nId], nz = nodes.posZ[nId];
            double ax = nodes.posX[aId], ay = nodes.posY[aId], az = nodes.posZ[aId];
            double bx = nodes.posX[bId], by = nodes.posY[bId], bz = nodes.posZ[bId];

            double abx = bx - ax, aby = by - ay, abz = bz - az;
            double anx = nx - ax, any = ny - ay, anz = nz - az;
            double ab_sq = abx*abx + aby*aby + abz*abz;
            double dist = 0.0;

            if (ab_sq > 1e-8) {
                double t = (anx*abx + any*aby + anz*abz) / ab_sq;
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;
                double px = ax + t * abx;
                double py = ay + t * aby;
                double pz = az + t * abz;
                double pnx = nx - px, pny = ny - py, pnz = nz - pz;
                dist = Math.sqrt(pnx*pnx + pny*pny + pnz*pnz);
            } else {
                dist = Math.sqrt(anx*anx + any*any + anz*anz);
            }

            if (dist < minDist) {
                minDist = dist;
                bestA = aId;
                bestB = bId;
                bestRestDist = dist;
            }
        }

        // Pass calculated index data to slide node container
        if (bestA != -1 && bestB != -1) {
            slidenodes.addSlideNode(nId, bestA, bestB, spring, damp, bestRestDist);
        }
    }

    /**
     * Spawn assembled vehicle at target coordinate, reset velocity and deformation state
     */
    public void spawnAt(double startX, double startY, double startZ) {
        for(int i = 0; i < nodes.count; i++) {
            nodes.posX[i] = nodes.baseX[i] + startX;
            nodes.posY[i] = nodes.baseY[i] + startY;
            nodes.posZ[i] = nodes.baseZ[i] + startZ;

            nodes.velX[i] = 0; nodes.velY[i] = 0; nodes.velZ[i] = 0;
            nodes.forceX[i] = 0; nodes.forceY[i] = 0; nodes.forceZ[i] = 0;
        }

        for(int i = 0; i < beams.count; i++) {
            beams.restLength[i] = beams.baseRestLength[i];
            beams.broken[i] = false;
        }

        System.out.println("Vehicle spawned at coordinate: " + startX + ", " + startY + ", " + startZ);
    }

    /**
     * Clear all physics container data and reset simulation world
     */
    public void clear() {
        nodes.clear();
        beams.clear();
        triangles.clear();
        torsionbars.clear();
        slidenodes.clear();

        System.out.println("🧹 Physics world data cleared and reset");
    }

    /**
     * Pre-update step: cache block collision around vehicle for performance
     * Generate per-part bounding box and sample Minecraft voxel data
     */
    public void preStep(World mcWorld, double dt) {
        snapshot.clear();

        // Initialize bounding box min/max value for current tick
        for (int p = 0; p <= maxTrackedPartId; p++) {
            partMinX[p] = Double.MAX_VALUE; partMinY[p] = Double.MAX_VALUE; partMinZ[p] = Double.MAX_VALUE;
            partMaxX[p] = -Double.MAX_VALUE; partMaxY[p] = -Double.MAX_VALUE; partMaxZ[p] = -Double.MAX_VALUE;
            partActive[p] = false;
        }

        // Put node current and predicted position into corresponding part bounding box
        for (int i = 0; i < nodes.count; i++) {
            int p = nodes.partId[i];
            partActive[p] = true;

            double px = nodes.posX[i]; double py = nodes.posY[i]; double pz = nodes.posZ[i];
            double nx = px + nodes.velX[i] * dt;
            double ny = py + nodes.velY[i] * dt;
            double nz = pz + nodes.velZ[i] * dt;

            if (px < partMinX[p]) partMinX[p] = px; if (nx < partMinX[p]) partMinX[p] = nx;
            if (px > partMaxX[p]) partMaxX[p] = px; if (nx > partMaxX[p]) partMaxX[p] = nx;

            if (py < partMinY[p]) partMinY[p] = py; if (ny < partMinY[p]) partMinY[p] = ny;
            if (py > partMaxY[p]) partMaxY[p] = py; if (ny > partMaxY[p]) partMaxY[p] = ny;

            if (pz < partMinZ[p]) partMinZ[p] = pz; if (nz < partMinZ[p]) partMinZ[p] = nz;
            if (pz > partMaxZ[p]) partMaxZ[p] = pz; if (nz > partMaxZ[p]) partMaxZ[p] = nz;
        }

        BlockPos.Mutable mutablePos = new BlockPos.Mutable();

        // Iterate every active part and scan surrounding blocks
        for (int p = 0; p <= maxTrackedPartId; p++) {
            if (!partActive[p]) continue;

            double sizeX = partMaxX[p] - partMinX[p];
            double sizeY = partMaxY[p] - partMinY[p];
            double sizeZ = partMaxZ[p] - partMinZ[p];

            // Over-stretched part protection: shrink oversized bounding box
            if (sizeX > MAX_AABB_SIZE || sizeY > MAX_AABB_SIZE || sizeZ > MAX_AABB_SIZE) {
                double cx = (partMinX[p] + partMaxX[p]) * 0.5;
                double cy = (partMinY[p] + partMaxY[p]) * 0.5;
                double cz = (partMinZ[p] + partMaxZ[p]) * 0.5;
                double half = MAX_AABB_SIZE * 0.5;

                partMinX[p] = cx - half; partMaxX[p] = cx + half;
                partMinY[p] = cy - half; partMaxY[p] = cy + half;
                partMinZ[p] = cz - half; partMaxZ[p] = cz + half;
            }

            // Expand bounding box with 1 block safe margin
            int bMinX = (int)Math.floor(partMinX[p]) - 1;
            int bMaxX = (int)Math.ceil(partMaxX[p]) + 1;
            int bMinY = (int)Math.floor(partMinY[p]) - 1;
            int bMaxY = (int)Math.ceil(partMaxY[p]) + 1;
            int bMinZ = (int)Math.floor(partMinZ[p]) - 1;
            int bMaxZ = (int)Math.ceil(partMaxZ[p]) + 1;

            // Scan and cache block voxel data
            for (int x = bMinX; x <= bMaxX; x++) {
                for (int y = bMinY; y <= bMaxY; y++) {
                    for (int z = bMinZ; z <= bMaxZ; z++) {
                        long posLong = VoxelSnapshot.asLong(x, y, z);

                        if (snapshot.hasCache(posLong)) continue;

                        mutablePos.set(x, y, z);
                        VoxelShape shape = mcWorld.getBlockState(mutablePos).getCollisionShape(mcWorld, mutablePos);

                        if (shape.isEmpty()) {
                            snapshot.cacheBlock(posLong, VoxelSnapshot.TYPE_AIR, null);
                        } else {
                            List<Box> boxes = shape.getBoundingBoxes();
                            boolean isFull = boxes.size() == 1 &&
                                    boxes.get(0).minX <= 0.01 && boxes.get(0).minY <= 0.01 && boxes.get(0).minZ <= 0.01 &&
                                    boxes.get(0).maxX >= 0.99 && boxes.get(0).maxY >= 0.99 && boxes.get(0).maxZ >= 0.99;

                            if (isFull) {
                                snapshot.cacheBlock(posLong, VoxelSnapshot.TYPE_FULL, null);
                            } else {
                                float[] aabbs = new float[boxes.size() * 6];
                                int ptr = 0;
                                for (Box b : boxes) {
                                    aabbs[ptr++] = (float)b.minX; aabbs[ptr++] = (float)b.minY; aabbs[ptr++] = (float)b.minZ;
                                    aabbs[ptr++] = (float)b.maxX; aabbs[ptr++] = (float)b.maxY; aabbs[ptr++] = (float)b.maxZ;
                                }
                                snapshot.cacheBlock(posLong, VoxelSnapshot.TYPE_COMPLEX, aabbs);
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Main physics update loop, high substep count for stable simulation
     */
    public void step(double dt) {
        int subSteps = 100;
        double subDt = dt / subSteps;
        double invSubDt = 1.0 / subDt;

        for (int s = 0; s < subSteps; s++) {
            // Reset node force accumulation every substep
            for (int i = 0; i < nodes.count; i++) {
                nodes.forceX[i] = 0;
                nodes.forceY[i] = 0;
                nodes.forceZ[i] = 0;
            }

            // 1. Calculate spring and damping force for all beams
            for (int i = 0; i < beams.count; i++) {
                if (beams.broken[i]) continue;

                int n1 = beams.node1[i];
                int n2 = beams.node2[i];
                double m1 = nodes.mass[n1];
                double m2 = nodes.mass[n2];
                if (m1 < KINDA_SMALL_NUMBER || m2 < KINDA_SMALL_NUMBER) continue;

                double dx = nodes.posX[n2] - nodes.posX[n1];
                double dy = nodes.posY[n2] - nodes.posY[n1];
                double dz = nodes.posZ[n2] - nodes.posZ[n1];

                double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                double invDist = 1.0 / dist;

                int type = beams.type[i];

                // Support beam logic: only resist compression, ignore tension
                if (type == BeamContainer.BEAM_SUPPORT && dist > beams.restLength[i]) {
                    continue;
                }

                // Calculate safe limited stiffness for explicit euler
                double reducedMass = (m1 * m2) / (m1 + m2);
                double maxSafeSpring = getMaxSafeSpring(reducedMass, subDt);

                double activeSpring = Math.min(beams.spring[i], maxSafeSpring);

                double activeDamp = beams.damp[i];
                double springForce = activeSpring * (dist - beams.restLength[i]);

                // Bounded beam limit logic
                if (type == BeamContainer.BEAM_BOUNDED) {
                    double shortBoundary = beams.restLength[i] * (1.0 - beams.shortBound[i]);
                    double longBoundary = beams.restLength[i] * (1.0 + beams.longBound[i]);

                    double limitSpring = Math.min(beams.limitSpring[i], maxSafeSpring);

                    if (dist < shortBoundary) {
                        springForce += limitSpring * (dist - shortBoundary);
                        activeDamp = Math.max(activeDamp, beams.limitDamp[i]);
                    } else if (dist > longBoundary) {
                        springForce += limitSpring * (dist - longBoundary);
                        activeDamp = Math.max(activeDamp, beams.limitDamp[i]);
                    }
                }

                // Relative velocity and damping calculation
                double vx = nodes.velX[n2] - nodes.velX[n1];
                double vy = nodes.velY[n2] - nodes.velY[n1];
                double vz = nodes.velZ[n2] - nodes.velZ[n1];
                double relVel = (vx*dx + vy*dy + vz*dz) * invDist;
                double absRelVel = Math.abs(relVel);

                // Anti-explosion damping clamp
                double maxDampForce = (reducedMass * absRelVel) * invSubDt;
                double actualDampForce = activeDamp * absRelVel;
                actualDampForce = Math.min(actualDampForce, maxDampForce);
                double dampForce = actualDampForce * Math.signum(relVel);

                double totalForce = springForce + dampForce;

                // Break beam if over strength limit
                if (Math.abs(totalForce) > beams.strength[i]) {
                    beams.broken[i] = true;
                    continue;
                }

                // Quadratic plastic deformation model
                if (Math.abs(totalForce) > beams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                    double overForce = Math.abs(totalForce) - beams.deform[i];
                    double flowRate = (overForce * overForce) / (beams.deform[i] * activeSpring);
                    double deformAmount = flowRate * METAL_PLASTIC_FLOW_RATE * subDt;

                    if (dist > beams.restLength[i]) {
                        beams.restLength[i] += deformAmount;
                    } else {
                        beams.restLength[i] -= deformAmount;
                        if (beams.restLength[i] < KINDA_SMALL_NUMBER) {
                            beams.restLength[i] = KINDA_SMALL_NUMBER;
                        }
                    }
                }

                // Split and apply force to two nodes
                double fx = totalForce * dx * invDist;
                double fy = totalForce * dy * invDist;
                double fz = totalForce * dz * invDist;

                nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
                nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
            }

            // Sliding node rail constraint solver
            for (int i = 0; i < slidenodes.count; i++) {
                int nId = slidenodes.nodeId[i];
                int aId = slidenodes.railA[i];
                int bId = slidenodes.railB[i];

                double nx = nodes.posX[nId], ny = nodes.posY[nId], nz = nodes.posZ[nId];
                double ax = nodes.posX[aId], ay = nodes.posY[aId], az = nodes.posZ[aId];
                double bx = nodes.posX[bId], by = nodes.posY[bId], bz = nodes.posZ[bId];

                double abx = bx - ax, aby = by - ay, abz = bz - az;
                double anx = nx - ax, any = ny - ay, anz = nz - az;

                double ab_sq = abx*abx + aby*aby + abz*abz;
                if (ab_sq < KINDA_SMALL_NUMBER) continue;

                double t = (anx*abx + any*aby + anz*abz) / ab_sq;
                if (t < 0.0) t = 0.0;
                if (t > 1.0) t = 1.0;

                double px = ax + t * abx;
                double py = ay + t * aby;
                double pz = az + t * abz;

                double pnx = nx - px, pny = ny - py, pnz = nz - pz;
                double dist = Math.sqrt(pnx*pnx + pny*pny + pnz*pnz);

                // Anti zero-divide protection
                if (dist < KINDA_SMALL_NUMBER) { pnx = KINDA_SMALL_NUMBER; dist = KINDA_SMALL_NUMBER; }

                double invDist = 1.0 / dist;
                double nDirX = pnx * invDist, nDirY = pny * invDist, nDirZ = pnz * invDist;

                double mN = nodes.mass[nId];
                double mRail = nodes.mass[aId] + nodes.mass[bId];
                if (mN < KINDA_SMALL_NUMBER ||  mRail < KINDA_SMALL_NUMBER) continue;

                double reducedMass = (mN * mRail) / (mN + mRail);
                double maxSafeSpring = getMaxSafeSpring(reducedMass, subDt);

                double activeSpring = slidenodes.spring[i];
                if (Math.abs(activeSpring) > maxSafeSpring) activeSpring = maxSafeSpring;

                // Keep original rest offset, no forced snap
                double springForce = activeSpring * (dist - slidenodes.restDist[i]);

                // Rail point velocity interpolation
                double vpx = nodes.velX[aId] * (1 - t) + nodes.velX[bId] * t;
                double vpy = nodes.velY[aId] * (1 - t) + nodes.velY[bId] * t;
                double vpz = nodes.velZ[aId] * (1 - t) + nodes.velZ[bId] * t;

                double relVel = (nodes.velX[nId] - vpx) * nDirX + (nodes.velY[nId] - vpy) * nDirY + (nodes.velZ[nId] - vpz) * nDirZ;
                double dampForce = slidenodes.damp[i] * relVel;

                double maxDamp = (reducedMass * Math.abs(relVel)) / subDt;
                if (Math.abs(dampForce) > maxDamp) dampForce = maxDamp * Math.signum(relVel);

                // Apply slide constraint force
                double fx = (springForce + dampForce) * nDirX;
                double fy = (springForce + dampForce) * nDirY;
                double fz = (springForce + dampForce) * nDirZ;

                nodes.forceX[nId] -= fx; nodes.forceY[nId] -= fy; nodes.forceZ[nId] -= fz;
                nodes.forceX[aId] += fx * (1 - t); nodes.forceY[aId] += fy * (1 - t); nodes.forceZ[aId] += fz * (1 - t);
                nodes.forceX[bId] += fx * t;       nodes.forceY[bId] += fy * t;       nodes.forceZ[bId] += fz * t;
            }

            // 2. Node velocity integration & block collision response
            for (int i = 0; i < nodes.count; i++) {
                if (nodes.mass[i] < KINDA_SMALL_NUMBER)continue;

                // Apply gravity force
                nodes.forceY[i] += GRAVITY * nodes.mass[i];

                double invMass = 1.0 / nodes.mass[i];
                nodes.velX[i] += (nodes.forceX[i] * invMass) * subDt;
                nodes.velY[i] += (nodes.forceY[i] * invMass) * subDt;
                nodes.velZ[i] += (nodes.forceZ[i] * invMass) * subDt;

                // Speed limit to prevent infinite acceleration
                double speedSq = nodes.velX[i]*nodes.velX[i] + nodes.velY[i]*nodes.velY[i] + nodes.velZ[i]*nodes.velZ[i];
                if (speedSq > SOUND_SPEED * SOUND_SPEED) {
                    double speed = Math.sqrt(speedSq);
                    nodes.velX[i] = (nodes.velX[i] / speed) * SOUND_SPEED;
                    nodes.velY[i] = (nodes.velY[i] / speed) * SOUND_SPEED;
                    nodes.velZ[i] = (nodes.velZ[i] / speed) * SOUND_SPEED;
                }

                double nextX = nodes.posX[i] + nodes.velX[i] * subDt;
                double nextY = nodes.posY[i] + nodes.velY[i] * subDt;
                double nextZ = nodes.posZ[i] + nodes.velZ[i] * subDt;

                // Collision check with height range filter
                if (nodes.collision[i] && nextY < 320 && nextY > -70 && snapshot.isSolid(nextX, nextY, nextZ)) {

                    boolean hitX = snapshot.isSolid(nextX, nodes.posY[i], nodes.posZ[i]);
                    boolean hitY = snapshot.isSolid(nodes.posX[i], nextY, nodes.posZ[i]);
                    boolean hitZ = snapshot.isSolid(nodes.posX[i], nodes.posY[i], nextZ);

                    double friction = Math.max(0, 1.0 - nodes.friction[i]);

                    if (hitY) {
                        nodes.velY[i] *= BLOCK_REBOUND;
                        nodes.velX[i] *= friction; nodes.velZ[i] *= friction;
                        nextY = nodes.posY[i];
                    }
                    if (hitX) {
                        nodes.velX[i] *= BLOCK_REBOUND;
                        nodes.velY[i] *= friction; nodes.velZ[i] *= friction;
                        nextX = nodes.posX[i];
                    }
                    if (hitZ) {
                        nodes.velZ[i] *= BLOCK_REBOUND;
                        nodes.velX[i] *= friction; nodes.velY[i] *= friction;
                        nextZ = nodes.posZ[i];
                    }
                    if (!hitX && !hitY && !hitZ) {
                        nodes.velX[i] *= BLOCK_REBOUND; nodes.velY[i] *= BLOCK_REBOUND; nodes.velZ[i] *= BLOCK_REBOUND;
                        nextX = nodes.posX[i]; nextY = nodes.posY[i]; nextZ = nodes.posZ[i];
                    }
                }

                nodes.posX[i] = nextX;
                nodes.posY[i] = nextY;
                nodes.posZ[i] = nextZ;
            }
        }
    }

    /**
     * Calculate maximum stable spring stiffness for current timestep
     */
    public static double getMaxSafeSpring(double mass, double dt){
        return (dt > KINDA_SMALL_NUMBER) ? mass / (dt * dt) : 0.0;
    }
}