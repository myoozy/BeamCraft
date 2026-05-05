package me.mzy.beamcraft.physics;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.entity.Entity;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.World;

import java.util.Arrays;
import java.util.List;

public class SoftBodyVehicle {
    public static final double KINDA_SMALL_NUMBER = PhysicsWorld.KINDA_SMALL_NUMBER;
    public static final int MAX_AABB_SIZE = PhysicsWorld.MAX_AABB_SIZE;

    public final PhysicsVehicleEntity parentEntity; // 绑定的实体
    public final double[] localCOM = new double[3];
    public int vehicleId = -1; // 物理世界给它分配的顺序 ID (0, 1, 2...)
    public int globalNodeOffset = 0; // 全局节点偏移量

    public final NodeContainer nodes = new NodeContainer();
    public final BeamContainer normalBeams = new BeamContainer();
    public final BeamContainer supportBeams = new BeamContainer();
    public final BoundedBeamContainer boundedBeams = new BoundedBeamContainer();
    public final TriangleContainer triangles = new TriangleContainer();
    public final TorsionBarContainer torsionbars = new TorsionBarContainer();
    public final SlideNodeContainer slidenodes = new SlideNodeContainer();
    public final WheelContainer wheels = new WheelContainer(this);

    // Bounding box cache array for independent part culling
    private int maxTrackedPartId = -1;
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    private final SweepResultBuffer sweepResultBuffer = new SweepResultBuffer();

    // 获取实体当前的世界坐标作为锚点
    double entityX = 0.0;
    double entityY = 0.0;
    double entityZ = 0.0;

    public SoftBodyVehicle(PhysicsVehicleEntity parentEntity) {
        this.parentEntity = parentEntity;
        cacheEntityLocation();
    }

    public void cacheEntityLocation() {
        if (this.parentEntity == null) return;
        entityX = this.parentEntity.getX();
        entityY = this.parentEntity.getY();
        entityZ = this.parentEntity.getZ();
    }

    public void updateEntityLocation() {
        nodes.getCenterOfMass(localCOM);

        this.parentEntity.setVelocity(0, 0, 0);

        double newEntityX = entityX + localCOM[0];
        double newEntityY = entityY + localCOM[1];
        double newEntityZ = entityZ + localCOM[2];
        this.parentEntity.setPos(newEntityX,  newEntityY, newEntityZ);
        nodes.moveNodes(-localCOM[0], -localCOM[1], -localCOM[2]);
    }

    public void updateBeamPrecompression(double dt) {
        normalBeams.updatePrecompression(dt);
        supportBeams.updatePrecompression(dt);
        boundedBeams.updatePrecompression(dt);
    }

    /**
     * Expand array capacity to avoid index out of bounds for new part id
     */
    private void ensurePartCapacity(int maxId) {
        if (maxId >= partMinX.length) {
            // 不要用 maxId * 2，如果 maxId 是 0，0*2 还是 0，会导致严重崩溃！
            // 用 Math.max 确保它至少比 maxId 大 1
            int newSize = Math.max(maxId + 1, partMinX.length * 2);

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
    public void addNode(String name, double x, double y, double z, double nodeMass, double friction, double slidingFriction, int partId, boolean collision, boolean selfCollision) {
        nodes.addNode(name, x, y, z, nodeMass, friction, slidingFriction, partId, collision, selfCollision);

        // Calculate current maximum part id and expand buffer
        int currentMaxPartId = -1;
        for (int i = 0; i < nodes.count; i++) {
            if (nodes.partId[i] > currentMaxPartId) {
                currentMaxPartId = nodes.partId[i];
            }
        }
        maxTrackedPartId = currentMaxPartId;
        ensurePartCapacity(maxTrackedPartId);
    }

    /**
     * Create physical beam constraint between two existing nodes
     */
    public void addBeam(int type,
                        String name1, String name2,
                        double spring, double damp,
                        double deform, double strength,
                        double precomp, double precompRange, double precompTime,
                        double shortBound, double longBound,
                        double shortBoundRange, double longBoundRange,
                        double limitSpring, double limitDamp,
                        double dampVelSplit, double dampFast,
                        double dampRebound, double dampReboundFast) {
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);

            nodes.degree[n1]++;
            nodes.degree[n2]++;

            if (type == BeamContainer.BEAM_SUPPORT) {
                supportBeams.addBeam(n1, n2, dist, spring, damp,
                        deform, strength, precomp, precompRange, precompTime);
            } else if (type == BeamContainer.BEAM_BOUNDED) {
                boundedBeams.addBeam(n1, n2, dist,
                        spring, damp, deform, strength,
                        precomp, precompRange, precompTime,
                        shortBound, longBound,
                        shortBoundRange, longBoundRange,
                        limitSpring, limitDamp,
                        dampVelSplit, dampFast,
                        dampRebound, dampReboundFast);
            } else {
                normalBeams.addBeam(n1, n2, dist, spring, damp,
                        deform, strength, precomp, precompRange, precompTime);
            }
        }
    }

    /**
     * Register collision triangle face composed of three nodes
     */
    public void addTriangle(String name1, String name2, String name3, int triPartId, boolean collision) {
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2) && nodes.nameToIndex.containsKey(name3)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            int n3 = nodes.nameToIndex.get(name3);

            triangles.addTriangle(n1, n2, n3, triPartId, collision);
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
    public void addSlideNode(String node, String[] railNodes, double spring, double damp) {
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

    public void finalizePhysicsSetup() {
        double invDt = PhysicsWorld.invPhysicsDT;
        double safeFractionSpring = 0.95;
        double safeFractionDamp = 0.95;
        double avgCosSq = 1.0;

        // ==========================================
        // 1. 处理普通梁 (Normal Beams)
        // ==========================================
        for (int i = 0; i < normalBeams.count; i++) {
            int n1 = normalBeams.node1[i];
            int n2 = normalBeams.node2[i];

            // --- A. 刚度质量 (Scaled by Degree) ---
            double effM1 = nodes.mass[n1] / Math.max(1.0, nodes.degree[n1] * avgCosSq);
            double effM2 = nodes.mass[n2] / Math.max(1.0, nodes.degree[n2] * avgCosSq);
            double effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            // --- B. 阻尼质量 (Unscaled) ---
            double realM1 = nodes.mass[n1];
            double realM2 = nodes.mass[n2];
            double unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            // 弹簧截断：使用带 degree 惩罚的质量，乘以 4.0 的绝对极限
            double maxSafeSpring = 4.0 * effReducedMass * invDt * invDt * safeFractionSpring;
            normalBeams.spring[i] = Math.min(normalBeams.spring[i], maxSafeSpring);

            // 阻尼截断：使用你推导出的物理公式 (Unscaled Mass * invDt)
            double maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            normalBeams.damp[i] = Math.min(normalBeams.damp[i], maxSafeDamp);
        }

        // ==========================================
        // 2. 处理支撑梁 (Support Beams)
        // ==========================================
        for (int i = 0; i < supportBeams.count; i++) {
            int n1 = supportBeams.node1[i];
            int n2 = supportBeams.node2[i];

            double effM1 = nodes.mass[n1] / Math.max(1.0, nodes.degree[n1] * avgCosSq);
            double effM2 = nodes.mass[n2] / Math.max(1.0, nodes.degree[n2] * avgCosSq);
            double effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            double realM1 = nodes.mass[n1];
            double realM2 = nodes.mass[n2];
            double unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            double maxSafeSpring = 4.0 * effReducedMass * invDt * invDt * safeFractionSpring;
            supportBeams.spring[i] = Math.min(supportBeams.spring[i], maxSafeSpring);

            double maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            supportBeams.damp[i] = Math.min(supportBeams.damp[i], maxSafeDamp);
        }

        // ==========================================
        // 3. 处理限界梁 (Bounded Beams)
        // ==========================================
        for (int i = 0; i < boundedBeams.count; i++) {
            int n1 = boundedBeams.node1[i];
            int n2 = boundedBeams.node2[i];

            double effM1 = nodes.mass[n1] / Math.max(1.0, nodes.degree[n1] * avgCosSq);
            double effM2 = nodes.mass[n2] / Math.max(1.0, nodes.degree[n2] * avgCosSq);
            double effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            double realM1 = nodes.mass[n1];
            double realM2 = nodes.mass[n2];
            double unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            double maxSafeSpring = 4.0 * effReducedMass * invDt * invDt * safeFractionSpring;
            boundedBeams.spring[i] = Math.min(boundedBeams.spring[i], maxSafeSpring);
            boundedBeams.limitSpring[i] = Math.min(boundedBeams.limitSpring[i], maxSafeSpring);

            double maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            boundedBeams.damp[i] = Math.min(boundedBeams.damp[i], maxSafeDamp);
            boundedBeams.limitDamp[i] = Math.min(boundedBeams.limitDamp[i], maxSafeDamp);
            boundedBeams.dampFast[i] = Math.min(boundedBeams.dampFast[i], maxSafeDamp);
            boundedBeams.dampRebound[i] = Math.min(boundedBeams.dampRebound[i], maxSafeDamp);
            boundedBeams.dampReboundFast[i] = Math.min(boundedBeams.dampReboundFast[i], maxSafeDamp);
        }
    }

    /**
     * Sreset velocity and deformation state
     */
    public void reset() {
        nodes.reset();
        normalBeams.reset();
        supportBeams.reset();
        boundedBeams.reset();
        torsionbars.reset();
        System.out.println("Vehicle reset.");
    }

    /**
     * Clear all physics container data and reset simulation world
     */
    public void clear() {
        nodes.clear();
        normalBeams.clear();
        supportBeams.clear();
        boundedBeams.clear();
        triangles.clear();
        torsionbars.clear();
        slidenodes.clear();

        System.out.println("🧹 Vehicle data cleared and reset");
    }

    public void updateVoxelSnapshot(World mcWorld, VoxelSnapshot snapshot, BlockPos.Mutable mutablePos, double dt) {

        if (nodes.count == 0) return;

        // Initialize bounding box min/max value for current tick
        for (int p = 0; p <= maxTrackedPartId; p++) {
            partMinX[p] = Double.MAX_VALUE;
            partMinY[p] = Double.MAX_VALUE;
            partMinZ[p] = Double.MAX_VALUE;
            partMaxX[p] = -Double.MAX_VALUE;
            partMaxY[p] = -Double.MAX_VALUE;
            partMaxZ[p] = -Double.MAX_VALUE;
            partActive[p] = false;
        }

        // Put node current and predicted position into corresponding part bounding box
        for (int i = 0; i < nodes.count; i++) {
            int p = nodes.partId[i];
            partActive[p] = true;

            double px = entityX + nodes.posX[i]; // to world coordinate！
            double py = entityY + nodes.posY[i];
            double pz = entityZ + nodes.posZ[i];
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

    public void solveInternalForces(double dt){
        double invDt = 1.0 / dt;

        // 初始化物理状态
        for (int i = 0; i < nodes.count; i++) {
            nodes.forceX[i] = 0;
            nodes.forceY[i] = 0;
            nodes.forceZ[i] = 0;

            nodes.prevPosX[i] = nodes.posX[i];
            nodes.prevPosY[i] = nodes.posY[i];
            nodes.prevPosZ[i] = nodes.posZ[i];
        }

        // ==========================================
        // 🛡️ 梁计算 (Beams)
        // ==========================================

        // ========== 1. 处理普通梁（NORMAL） ==========
        for (int i = 0; i < normalBeams.count; i++) {
            if (normalBeams.broken[i]) continue;

            int n1 = normalBeams.node1[i];
            int n2 = normalBeams.node2[i];

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            double dist = Math.sqrt(distSq);
            double invDist = 1.0 / dist;

            double restL = normalBeams.restLength[i];
            double activeSpring = normalBeams.spring[i];
            double springForce = normalBeams.spring[i] * (dist - restL);

            double vx = nodes.velX[n2] - nodes.velX[n1];
            double vy = nodes.velY[n2] - nodes.velY[n1];
            double vz = nodes.velZ[n2] - nodes.velZ[n1];
            double relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            double activeDamp = normalBeams.damp[i];
            double dampForce = activeDamp * relVel;

            double totalForce = springForce + dampForce;
            double absTotalForce = Math.abs(totalForce);

            if (absTotalForce > normalBeams.strength[i]) {
                normalBeams.broken[i] = true;
                continue;
            }

            if (absTotalForce > normalBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - normalBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (normalBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) normalBeams.restLength[i] += deformAmount;
                else normalBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            double fx = totalForce * dx * invDist;
            double fy = totalForce * dy * invDist;
            double fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }

        // ========== 2. 处理支撑梁（SUPPORT，仅压缩） ==========
        for (int i = 0; i < supportBeams.count; i++) {
            if (supportBeams.broken[i]) continue;

            int n1 = supportBeams.node1[i];
            int n2 = supportBeams.node2[i];

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            double dist = Math.sqrt(distSq);
            double invDist = 1.0 / dist;

            double restL = supportBeams.restLength[i];

            // 支撑梁：只抗压，不抗拉（拉伸时跳过）
            if (dist > restL) continue;

            double activeSpring = supportBeams.spring[i];
            double springForce = activeSpring * (dist - restL);  // dist <= restL，力为负（压力）

            double vx = nodes.velX[n2] - nodes.velX[n1];
            double vy = nodes.velY[n2] - nodes.velY[n1];
            double vz = nodes.velZ[n2] - nodes.velZ[n1];
            double relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            double activeDamp = supportBeams.damp[i];
            double dampForce = activeDamp * relVel;

            double totalForce = springForce + dampForce;
            double absTotalForce = Math.abs(totalForce);

            if (absTotalForce > supportBeams.strength[i]) {
                supportBeams.broken[i] = true;
                continue;
            }

            if (absTotalForce > supportBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - supportBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (supportBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) supportBeams.restLength[i] += deformAmount;
                else supportBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            double fx = totalForce * dx * invDist;
            double fy = totalForce * dy * invDist;
            double fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }

        // ========== 3. 处理限界梁（BOUNDED，含复杂阻尼和限位） ==========
        for (int i = 0; i < boundedBeams.count; i++) {
            if (boundedBeams.broken[i]) continue;

            int n1 = boundedBeams.node1[i];
            int n2 = boundedBeams.node2[i];

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            double dist = Math.sqrt(distSq);
            double invDist = 1.0 / dist;

            double restL = boundedBeams.restLength[i];
            double activeSpring = boundedBeams.spring[i];
            double springForce = activeSpring * (dist - restL);

            double vx = nodes.velX[n2] - nodes.velX[n1];
            double vy = nodes.velY[n2] - nodes.velY[n1];
            double vz = nodes.velZ[n2] - nodes.velZ[n1];
            double relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            // ----- 复杂阻尼（仅限界梁拥有）-----
            double activeDamp = boundedBeams.damp[i];
            double split = boundedBeams.dampVelocitySplit[i];
            boolean isRebound = relVel > 0;
            boolean isFast = Math.abs(relVel) > split;
            if (isRebound) {
                activeDamp = isFast ? boundedBeams.dampReboundFast[i] : boundedBeams.dampRebound[i];
            } else {
                if (isFast) activeDamp = boundedBeams.dampFast[i];
                // else 保持 activeDamp 不变（原逻辑：isFast? dampFast : activeDamp）
            }

            // ----- 限位逻辑 -----
            double shortBoundary, longBoundary;

            // 短边界：如果指定了 Range（绝对米），就用 restL 减去它；否则用比例直接乘。
            if (boundedBeams.shortBoundRange[i] >= 0) {
                shortBoundary = restL - boundedBeams.shortBoundRange[i];
            } else {
                shortBoundary = restL * (1.0 - boundedBeams.shortBound[i]);
            }

            // 长边界：如果指定了 Range（绝对米），就用 restL 加上它；否则用比例直接乘。
            if (boundedBeams.longBoundRange[i] >= 0) {
                longBoundary = restL + boundedBeams.longBoundRange[i];
            } else {
                longBoundary = restL * (1.0 + boundedBeams.longBound[i]);
            }

            double limitSpring = boundedBeams.limitSpring[i];

            if (dist < shortBoundary) {
                springForce += limitSpring * (dist - shortBoundary);
                activeDamp = boundedBeams.limitDamp[i];
            } else if (dist > longBoundary) {
                springForce += limitSpring * (dist - longBoundary);
                activeDamp = boundedBeams.limitDamp[i];
            }

            double totalForce = springForce + (relVel * activeDamp);
            double absTotalForce = Math.abs(totalForce);

            if (absTotalForce > boundedBeams.strength[i]) {
                boundedBeams.broken[i] = true;
                continue;
            }

            if (absTotalForce > boundedBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - boundedBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (boundedBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) boundedBeams.restLength[i] += deformAmount;
                else boundedBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            double fx = totalForce * dx * invDist;
            double fy = totalForce * dy * invDist;
            double fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }

        // ==========================================
        // 🛡️ 扭杆计算 (Torsionbars)
        // ==========================================
        for (int i = 0; i < torsionbars.count; i++) {
            if (torsionbars.broken[i]) continue;

            int n1 = torsionbars.node1[i], n2 = torsionbars.node2[i], n3 = torsionbars.node3[i], n4 = torsionbars.node4[i];

            if (nodes.mass[n1] < KINDA_SMALL_NUMBER ||
                nodes.mass[n2] < KINDA_SMALL_NUMBER ||
                nodes.mass[n3] < KINDA_SMALL_NUMBER ||
                nodes.mass[n4] < KINDA_SMALL_NUMBER) {
                torsionbars.broken[i] = true;
                continue;
            }

            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];
            double x4 = nodes.posX[n4], y4 = nodes.posY[n4], z4 = nodes.posZ[n4];

            double b1x = x2 - x1, b1y = y2 - y1, b1z = z2 - z1;
            double b2x = x3 - x2, b2y = y3 - y2, b2z = z3 - z2;
            double b3x = x4 - x3, b3y = y4 - y3, b3z = z4 - z3;

            double c1x = b1y * b2z - b1z * b2y;
            double c1y = b1z * b2x - b1x * b2z;
            double c1z = b1x * b2y - b1y * b2x;

            double c2x = b2y * b3z - b2z * b3y;
            double c2y = b2z * b3x - b2x * b3z;
            double c2z = b2x * b3y - b2y * b3x;

            double c1_sq = c1x*c1x + c1y*c1y + c1z*c1z;
            double c2_sq = c2x*c2x + c2y*c2y + c2z*c2z;
            double b2_sq = b2x*b2x + b2y*b2y + b2z*b2z;

            // 如果有NaN，直接判定为断裂
            if (Double.isNaN(c1_sq) || Double.isNaN(c2_sq) ||  Double.isNaN(b2_sq)) {
                torsionbars.broken[i] = true;
                continue;
            }

            double b2_mag = Math.sqrt(b2_sq);

            double c1Xc2_x = c1y * c2z - c1z * c2y;
            double c1Xc2_y = c1z * c2x - c1x * c2z;
            double c1Xc2_z = c1x * c2y - c1y * c2x;

            double dot1 = (c1Xc2_x * b2x + c1Xc2_y * b2y + c1Xc2_z * b2z) / b2_mag;
            double dot2 = c1x * c2x + c1y * c2y + c1z * c2z;
            double currentAngle = Math.atan2(dot1, dot2);

            double deltaAngle = currentAngle - torsionbars.restAngle[i];
            while (deltaAngle > Math.PI) deltaAngle -= Math.PI * 2;
            while (deltaAngle < -Math.PI) deltaAngle += Math.PI * 2;

            // 🚨🚨🚨 注意梯度符号
            double g1_factor = b2_mag / c1_sq;
            double g4_factor = -b2_mag / c2_sq;

            double g1x = g1_factor * c1x, g1y = g1_factor * c1y, g1z = g1_factor * c1z;
            double g4x = g4_factor * c2x, g4y = g4_factor * c2y, g4z = g4_factor * c2z;

            double b1_dot_b2_div_sq = (b1x*b2x + b1y*b2y + b1z*b2z) / b2_sq;
            double b3_dot_b2_div_sq = (b3x*b2x + b3y*b2y + b3z*b2z) / b2_sq;

            // 注意这里的符号
            double g2x = -g1x * b1_dot_b2_div_sq + g4x * b3_dot_b2_div_sq - g1x;
            double g2y = -g1y * b1_dot_b2_div_sq + g4y * b3_dot_b2_div_sq - g1y;
            double g2z = -g1z * b1_dot_b2_div_sq + g4z * b3_dot_b2_div_sq - g1z;

            double g3x = -g1x - g2x - g4x;
            double g3y = -g1y - g2y - g4y;
            double g3z = -g1z - g2z - g4z;

            // 广义质量
            double g1_sq_val = g1x*g1x + g1y*g1y + g1z*g1z;
            double g2_sq_val = g2x*g2x + g2y*g2y + g2z*g2z;
            double g3_sq_val = g3x*g3x + g3y*g3y + g3z*g3z;
            double g4_sq_val = g4x*g4x + g4y*g4y + g4z*g4z;

            // 因为之前检查过mass，所以invGenMass基本不会是NaN
            double invGenMass = (g1_sq_val / nodes.mass[n1]) + (g2_sq_val / nodes.mass[n2]) +
                    (g3_sq_val / nodes.mass[n3]) + (g4_sq_val / nodes.mass[n4]);

            double genMass = 1.0 / invGenMass;

            double maxSafeSpring = genMass * invDt * invDt;
            double maxSafeDamp = genMass * invDt;

            double activeSpring = Math.min(torsionbars.spring[i], maxSafeSpring);
            double activeDamp = Math.min(torsionbars.damp[i], maxSafeDamp);

            // 💥 最终受力输出
            double omega = (g1x*nodes.velX[n1] + g1y*nodes.velY[n1] + g1z*nodes.velZ[n1]) +
                    (g2x*nodes.velX[n2] + g2y*nodes.velY[n2] + g2z*nodes.velZ[n2]) +
                    (g3x*nodes.velX[n3] + g3y*nodes.velY[n3] + g3z*nodes.velZ[n3]) +
                    (g4x*nodes.velX[n4] + g4y*nodes.velY[n4] + g4z*nodes.velZ[n4]);

            double torque = (activeSpring * deltaAngle) - (activeDamp * omega);

            double absTorque = Math.abs(torque);
            if (Double.isNaN(torque) || absTorque > torsionbars.strength[i]) {
                torsionbars.broken[i] = true;
                continue;
            }
            if (absTorque > torsionbars.deform[i] && torsionbars.spring[i] > KINDA_SMALL_NUMBER) {
                double overTorque = absTorque - torsionbars.deform[i];
                double flowRate = (overTorque * overTorque) / (torsionbars.deform[i] * torsionbars.spring[i]);
                double deformAmount = flowRate * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;

                torsionbars.restAngle[i] += Math.signum(deltaAngle) * deformAmount;
                while (torsionbars.restAngle[i] > Math.PI) torsionbars.restAngle[i] -= Math.PI * 2;
                while (torsionbars.restAngle[i] < -Math.PI) torsionbars.restAngle[i] += Math.PI * 2;
            }

            nodes.forceX[n1] += torque * g1x; nodes.forceY[n1] += torque * g1y; nodes.forceZ[n1] += torque * g1z;
            nodes.forceX[n2] += torque * g2x; nodes.forceY[n2] += torque * g2y; nodes.forceZ[n2] += torque * g2z;
            nodes.forceX[n3] += torque * g3x; nodes.forceY[n3] += torque * g3y; nodes.forceZ[n3] += torque * g3z;
            nodes.forceX[n4] += torque * g4x; nodes.forceY[n4] += torque * g4y; nodes.forceZ[n4] += torque * g4z;
        }

        // ==========================================
        // 🛡️ 计算滑块 (slidenodes)
        // ==========================================
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
            if (dist < KINDA_SMALL_NUMBER) {
                dist = KINDA_SMALL_NUMBER;
            }

            double invDist = 1.0 / dist;
            double nDirX = pnx * invDist, nDirY = pny * invDist, nDirZ = pnz * invDist;

            double mN = nodes.mass[nId];
            double mRail = nodes.mass[aId] + nodes.mass[bId];
            if (mN < KINDA_SMALL_NUMBER ||  mRail < KINDA_SMALL_NUMBER) continue;

            double reducedMass = (mN * mRail) / (mN + mRail);
            double maxSafeSpring = reducedMass * invDt * invDt;
            double activeSpring = Math.min(slidenodes.spring[i], maxSafeSpring);

            // Keep original rest offset, no forced snap
            double springForce = activeSpring * (dist - slidenodes.restDist[i]);

            // Rail point velocity interpolation
            double vpx = nodes.velX[aId] * (1 - t) + nodes.velX[bId] * t;
            double vpy = nodes.velY[aId] * (1 - t) + nodes.velY[bId] * t;
            double vpz = nodes.velZ[aId] * (1 - t) + nodes.velZ[bId] * t;

            double relVel = (nodes.velX[nId] - vpx) * nDirX + (nodes.velY[nId] - vpy) * nDirY + (nodes.velZ[nId] - vpz) * nDirZ;

            double activeDamp = Math.min(slidenodes.damp[i], reducedMass * invDt);
            double dampForce = activeDamp * relVel;

            // Apply slide constraint force
            double fx = (springForce + dampForce) * nDirX;
            double fy = (springForce + dampForce) * nDirY;
            double fz = (springForce + dampForce) * nDirZ;

            nodes.forceX[nId] -= fx; nodes.forceY[nId] -= fy; nodes.forceZ[nId] -= fz;
            nodes.forceX[aId] += fx * (1 - t); nodes.forceY[aId] += fy * (1 - t); nodes.forceZ[aId] += fz * (1 - t);
            nodes.forceX[bId] += fx * t;       nodes.forceY[bId] += fy * t;       nodes.forceZ[bId] += fz * t;
        }

        // ==========================================
        // 🛡️ 积分速度和位置（预测）。整个tick只有这一次积分
        // ==========================================
        for (int i = 0; i < nodes.count; i++) {

            if (nodes.mass[i] < PhysicsWorld.KINDA_SMALL_NUMBER) continue;
            // 加重力
            nodes.forceY[i] += PhysicsWorld.GRAVITY * nodes.mass[i];

            double invMass = 1.0 / nodes.mass[i];
            nodes.velX[i] += (nodes.forceX[i] * invMass) * dt;
            nodes.velY[i] += (nodes.forceY[i] * invMass) * dt;
            nodes.velZ[i] += (nodes.forceZ[i] * invMass) * dt;

            // 算出当前节点的速度大小
            double speedSq = nodes.velX[i]*nodes.velX[i] + nodes.velY[i]*nodes.velY[i] + nodes.velZ[i]*nodes.velZ[i];

            // 施加一个高速时增长极快的魔法阻力，防止节点炸飞
            final double K_V4 = 1.2e-7;   // 防飞出系数，根据最高期望速度调
            double v4 = speedSq * speedSq;
            double factor = 1.0 / (1.0 + K_V4 * v4 * dt);
            nodes.velX[i] *= factor;
            nodes.velY[i] *= factor;
            nodes.velZ[i] *= factor;

            // 清洗极端的 NaN (应对除以0等极端异常)
            if (Double.isNaN(nodes.velX[i]) || Double.isNaN(nodes.velY[i]) || Double.isNaN(nodes.velZ[i])) {
                nodes.velX[i] = 0; nodes.velY[i] = 0; nodes.velZ[i] = 0;
            }

            // 局部预测坐标 (直接覆盖 posX，因为 prevPos 已经存好了)
            nodes.posX[i] += nodes.velX[i] * dt;
            nodes.posY[i] += nodes.velY[i] * dt;
            nodes.posZ[i] += nodes.velZ[i] * dt;
        }
    }

    /**
     * 宽阶段扫描：每隔 N 个子步调用一次，预测未来的位移，收集可能碰撞的对子
     * @param sap 全局 SAP 加速结构
     * @param manager 我们的碰撞调度中心
     * @param dtPredict 预测时间 (例如 10 个子步的时间总和)
     */
    public void generateCollisionCandidates(DynamicAxisSweep sap, SoftBodyCollisionManager manager, double dtPredict) {
        double eX = entityX, eY = entityY, eZ = entityZ;

        // 铁皮的物理厚度常数：2厘米。不再承担防穿透的任务，只负责防浮点误差！
        float BASE_MARGIN = 0.01f;

        for (int i = 0; i < triangles.count; i++) {
            if (!triangles.collision[i]) continue;

            int nA = triangles.node1[i];
            int nB = triangles.node2[i];
            int nC = triangles.node3[i];

            double ax = eX + nodes.posX[nA], ay = eY + nodes.posY[nA], az = eZ + nodes.posZ[nA];
            double bx = eX + nodes.posX[nB], by = eY + nodes.posY[nB], bz = eZ + nodes.posZ[nB];
            double cx = eX + nodes.posX[nC], cy = eY + nodes.posY[nC], cz = eZ + nodes.posZ[nC];

            // 1. 算出当前帧最紧凑的基准 AABB
            float minX = (float) Math.min(ax, Math.min(bx, cx)) - BASE_MARGIN;
            float maxX = (float) Math.max(ax, Math.max(bx, cx)) + BASE_MARGIN;
            float minY = (float) Math.min(ay, Math.min(by, cy)) - BASE_MARGIN;
            float maxY = (float) Math.max(ay, Math.max(by, cy)) + BASE_MARGIN;
            float minZ = (float) Math.min(az, Math.min(bz, cz)) - BASE_MARGIN;
            float maxZ = (float) Math.max(az, Math.max(bz, cz)) + BASE_MARGIN;

            // 2. 算物理：计算这个三角形的平均速度
            double triVx = (nodes.velX[nA] + nodes.velX[nB] + nodes.velX[nC]) * 0.3333333333;
            double triVy = (nodes.velY[nA] + nodes.velY[nB] + nodes.velY[nC]) * 0.3333333333;
            double triVz = (nodes.velZ[nA] + nodes.velZ[nB] + nodes.velZ[nC]) * 0.3333333333;

            // 3. 算预判位移向量：速度 * 预判时间
            float dx = (float) (triVx * dtPredict);
            float dy = (float) (triVy * dtPredict);
            float dz = (float) (triVz * dtPredict);

            // 4. 定向拉伸：只在运动方向上延伸包围盒！(Swept AABB)
            if (dx > 0) maxX += dx; else minX += dx;
            if (dy > 0) maxY += dy; else minY += dy;
            if (dz > 0) maxZ += dz; else minZ += dz;

            sweepResultBuffer.clear();
            sap.queryNodesInAABB(minX, minY, minZ, maxX, maxY, maxZ, sweepResultBuffer);

            for (int k = 0; k < sweepResultBuffer.count; k++) {
                SoftBodyVehicle hitVeh = sweepResultBuffer.vehicles[k];
                int hitNodeId = sweepResultBuffer.nodeIds[k];

                // 排除不需要计算自碰撞的点，以及三角形自己的顶点
                if (hitVeh == this && !nodes.selfCollision[hitNodeId]) continue;
                if (hitVeh == this && (hitNodeId == nA || hitNodeId == nB || hitNodeId == nC)) continue;

                // 找到嫌疑人了！不用算物理，直接交给调度中心！
                manager.addContact(hitVeh, hitNodeId, this, nA, nB, nC);
            }
        }
    }

    public void applyPositionAndVelocityDeltaUnSafe(int nodeId, double dPx, double dPy, double dPz,
                                                  double  dVx, double dVy, double dVz) {
        nodes.posX[nodeId] += dPx;
        nodes.posY[nodeId] += dPy;
        nodes.posZ[nodeId] += dPz;
        nodes.velX[nodeId] += dVx;
        nodes.velY[nodeId] += dVy;
        nodes.velZ[nodeId] += dVz;
    }

    public void solveEnvironmentCollisions(VoxelSnapshot snapshot, double dt) {
        for (int i = 0; i < nodes.count; i++) {
            if (!nodes.collision[i]) continue;

            double worldX = entityX + nodes.posX[i];
            double worldY = entityY + nodes.posY[i];
            double worldZ = entityZ + nodes.posZ[i];

            if (worldY < 320 && worldY > -70 && snapshot.isSolid(worldX, worldY, worldZ)) {

                // 回到上一步的安全坐标！
                double oldLocalX = nodes.prevPosX[i];
                double oldLocalY = nodes.prevPosY[i];
                double oldLocalZ = nodes.prevPosZ[i];

                double oldWorldX = entityX + oldLocalX;
                double oldWorldY = entityY + oldLocalY;
                double oldWorldZ = entityZ + oldLocalZ;

                boolean hitX = snapshot.isSolid(worldX, oldWorldY, oldWorldZ);
                boolean hitY = snapshot.isSolid(oldWorldX, worldY, oldWorldZ);
                boolean hitZ = snapshot.isSolid(oldWorldX, oldWorldY, worldZ);

                double friction = Math.max(0, 1.0 - nodes.friction[i]);

                // 撞了哪面墙，就把坐标回退，并彻底削减那个方向的速度！
                if (hitY) {
                    nodes.velY[i] *= -PhysicsWorld.BLOCK_REBOUND;
                    nodes.velX[i] *= friction; nodes.velZ[i] *= friction;
                    nodes.posY[i] = oldLocalY;
                }
                if (hitX) {
                    nodes.velX[i] *= -PhysicsWorld.BLOCK_REBOUND;
                    nodes.velY[i] *= friction; nodes.velZ[i] *= friction;
                    nodes.posX[i] = oldLocalX;
                }
                if (hitZ) {
                    nodes.velZ[i] *= -PhysicsWorld.BLOCK_REBOUND;
                    nodes.velX[i] *= friction; nodes.velY[i] *= friction;
                    nodes.posZ[i] = oldLocalZ;
                }
            }
        }
    }
}