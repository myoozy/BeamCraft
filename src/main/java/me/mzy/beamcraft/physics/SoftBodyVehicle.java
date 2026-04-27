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
    public final BeamContainer beams = new BeamContainer();
    public final TriangleContainer triangles = new TriangleContainer();
    public final TorsionBarContainer torsionbars = new TorsionBarContainer();
    public final SlideNodeContainer slidenodes = new SlideNodeContainer();

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
    public void addNode(String name, double x, double y, double z, double nodeMass, double friction, int partId, boolean collision, boolean selfCollision) {
        nodes.addNode(name, x, y, z, nodeMass, friction, partId, collision, selfCollision);

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
    public void addBeam(String name1, String name2, double spring, double damp, double deform, double strength,
                        int type, double precomp, double shortBound, double longBound,
                        double shortBoundRange, double longBoundRange,
                        double limitSpring, double limitDamp,
                        double dampVelSplit, double dampFast, double dampRebound, double dampReboundFast) {
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);

            beams.addBeam(n1, n2, dist, spring, damp, deform, strength, type, precomp,
                    shortBound, longBound, shortBoundRange, longBoundRange, limitSpring, limitDamp,
                    dampVelSplit, dampFast, dampRebound, dampReboundFast);
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

    /**
     * Sreset velocity and deformation state
     */
    public void reset() {
        nodes.reset();
        beams.reset();
        torsionbars.reset();
        System.out.println("Vehicle reset.");
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

        for (int i = 0; i < beams.count; i++) {
            if (beams.broken[i]) continue;

            int n1 = beams.node1[i];
            int n2 = beams.node2[i];
            double m1 = nodes.mass[n1];
            double m2 = nodes.mass[n2];

            if (m1 * m2 < KINDA_SMALL_NUMBER) continue;

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];

            double distSq = dx*dx + dy*dy + dz*dz;
            double dist = Math.sqrt(distSq);
            if (dist < KINDA_SMALL_NUMBER) dist = KINDA_SMALL_NUMBER;
            double invDist = 1.0 / dist;

            // 🚀 读取合并后的状态，并瞬间解包
            int rawType = beams.type[i];
            int type = rawType & BeamContainer.MASK_TYPE;

            double restL = beams.restLength[i];

            if (type == BeamContainer.BEAM_SUPPORT && dist > restL) continue;

            double reducedMass = (m1 * m2) / (m1 + m2);
            double maxSafeSpring = getMaxSafeSpring(reducedMass, dt);

            double activeSpring = Math.min(beams.spring[i], maxSafeSpring);
            double springForce = activeSpring * (dist - restL);

            double activeDamp = beams.damp[i];

            double vx = nodes.velX[n2] - nodes.velX[n1];
            double vy = nodes.velY[n2] - nodes.velY[n1];
            double vz = nodes.velZ[n2] - nodes.velZ[n1];
            double relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            // ==========================================
            // 🔥 位运算门控 1：复杂阻尼（位与运算只要 1 个 CPU 时钟周期）
            // ==========================================
            if ((rawType & BeamContainer.FLAG_HAS_COMPLEX_DAMP) != 0) {
                double split = beams.dampVelocitySplit[i];
                boolean isRebound = relVel > 0;
                boolean isFast = Math.abs(relVel) > split;
                activeDamp = isRebound ?
                        (isFast ? beams.dampReboundFast[i] : beams.dampRebound[i]) :
                        (isFast ? beams.dampFast[i] : activeDamp);
            }

            // ==========================================
            // 🔥 位运算门控 2：限位计算
            // ==========================================
            if ((rawType & BeamContainer.FLAG_HAS_BOUND) != 0) {
                double shortBoundary = restL * (1.0 - beams.shortBound[i]);
                double longBoundary  = restL * (1.0 + beams.longBound[i]);
                double limitSpring = Math.min(beams.limitSpring[i], maxSafeSpring);

                if (dist < shortBoundary) {
                    springForce += limitSpring * (shortBoundary - dist);
                    double lDamp = beams.limitDamp[i];
                    if (lDamp > activeDamp) activeDamp = lDamp;
                } else if (dist > longBoundary) {
                    springForce += limitSpring * (dist - longBoundary);
                    double lDamp = beams.limitDamp[i];
                    if (lDamp > activeDamp) activeDamp = lDamp;
                }
            }

            // ... (下面保持不变)
            double maxDamp = reducedMass * invDt;
            if (activeDamp > maxDamp) activeDamp = maxDamp;

            double totalForce = springForce + (relVel * activeDamp);

            double absTotalForce = Math.abs(totalForce);
            if (absTotalForce > beams.strength[i]) {
                beams.broken[i] = true;
                continue;
            }

            if (absTotalForce > beams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - beams.deform[i];
                double deformAmount = ((overForce * overForce) / (beams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;

                if (dist > restL) beams.restLength[i] += deformAmount;
                else beams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
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

            // 限制极小值，防除零崩溃 (1e-8 已经足够)
            double c1_sq_safe = Math.max(c1_sq, 1e-8);
            double c2_sq_safe = Math.max(c2_sq, 1e-8);
            double b2_sq_safe = Math.max(b2_sq, 1e-8);
            double b2_mag = Math.sqrt(b2_sq_safe);

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
            double g1_factor = b2_mag / c1_sq_safe;
            double g4_factor = -b2_mag / c2_sq_safe;

            double g1x = g1_factor * c1x, g1y = g1_factor * c1y, g1z = g1_factor * c1z;
            double g4x = g4_factor * c2x, g4y = g4_factor * c2y, g4z = g4_factor * c2z;

            double b1_dot_b2_div_sq = (b1x*b2x + b1y*b2y + b1z*b2z) / b2_sq_safe;
            double b3_dot_b2_div_sq = (b3x*b2x + b3y*b2y + b3z*b2z) / b2_sq_safe;

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

            double invGenMass = (g1_sq_val / nodes.mass[n1]) + (g2_sq_val / nodes.mass[n2]) +
                    (g3_sq_val / nodes.mass[n3]) + (g4_sq_val / nodes.mass[n4]);

            double genMass = 1.0 / Math.max(invGenMass, 1e-12);

            double maxSafeSpring = getMaxSafeSpring (genMass, dt);
            double maxSafeDamp = genMass * invDt;

            double activeSpring = Math.min(torsionbars.spring[i], maxSafeSpring);
            double activeDamp = Math.min(torsionbars.damp[i], maxSafeDamp);

            // 💥 最终受力输出
            double omega = (g1x*nodes.velX[n1] + g1y*nodes.velY[n1] + g1z*nodes.velZ[n1]) +
                    (g2x*nodes.velX[n2] + g2y*nodes.velY[n2] + g2z*nodes.velZ[n2]) +
                    (g3x*nodes.velX[n3] + g3y*nodes.velY[n3] + g3z*nodes.velZ[n3]) +
                    (g4x*nodes.velX[n4] + g4y*nodes.velY[n4] + g4z*nodes.velZ[n4]);

            // 纯净的扭矩，不需要复杂的正负号判断
            double torque = (activeSpring * deltaAngle) - (activeDamp * omega);

            double absTorque = Math.abs(torque);
            if (absTorque > torsionbars.strength[i]) {
                torsionbars.broken[i] = true;
                continue;
            }
            if (absTorque > torsionbars.deform[i] && torsionbars.spring[i] > 1e-8) {
                double overTorque = absTorque - torsionbars.deform[i];
                double flowRate = (overTorque * overTorque) / (torsionbars.deform[i] * torsionbars.spring[i]);
                double deformAmount = flowRate * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;

                torsionbars.restAngle[i] += Math.signum(deltaAngle) * deformAmount;
                while (torsionbars.restAngle[i] > Math.PI) torsionbars.restAngle[i] -= Math.PI * 2;
                while (torsionbars.restAngle[i] < -Math.PI) torsionbars.restAngle[i] += Math.PI * 2;

                torque = Math.signum(torque) * torsionbars.deform[i];
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
            if (dist < KINDA_SMALL_NUMBER) { pnx = KINDA_SMALL_NUMBER; dist = KINDA_SMALL_NUMBER; }

            double invDist = 1.0 / dist;
            double nDirX = pnx * invDist, nDirY = pny * invDist, nDirZ = pnz * invDist;

            double mN = nodes.mass[nId];
            double mRail = nodes.mass[aId] + nodes.mass[bId];
            if (mN < KINDA_SMALL_NUMBER ||  mRail < KINDA_SMALL_NUMBER) continue;

            double reducedMass = (mN * mRail) / (mN + mRail);
            double maxSafeSpring = getMaxSafeSpring(reducedMass, dt);

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

            double maxDamp = (reducedMass * Math.abs(relVel)) * invDt;
            if (Math.abs(dampForce) > maxDamp) dampForce = maxDamp * Math.signum(relVel);

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

            if (speedSq > 1.0) { // 速度太小就忽略空气阻力，省性能
                double speed = Math.sqrt(speedSq);
                double mach = speed / PhysicsWorld.SOUND_SPEED; // 计算马赫数

                // 1. 基础空气阻力 (与 v^2 成正比，这里简化为一个极小的常数系数)
                double baseDrag = 0.0005 * speed;

                // 2. 激波阻力模拟！(魔法就在这里：马赫数的 6 次方)
                // 速度慢时几乎为 0；接近 Mach 1 时暴增；超过 340m/s 时变成一面绝望的墙
                double shockDrag = 0.0;
                if (mach > 0.5) { // 0.5马赫(170m/s)以下没激波
                    shockDrag = 2.0 * Math.pow(mach, 6.0);
                }

                // 计算本子步的总速度衰减率 (dt = subDt)
                double totalDamping = (baseDrag + shockDrag) * dt;

                // 防止衰减超过 100% 导致速度反向
                double velocityMultiplier = Math.max(0.0, 1.0 - totalDamping);

                // 应用空气动力学衰减！
                nodes.velX[i] *= velocityMultiplier;
                nodes.velY[i] *= velocityMultiplier;
                nodes.velZ[i] *= velocityMultiplier;
            }

            // 清洗极端的 NaN (应对除以0等极端异常)
            if (Double.isNaN(nodes.velX[i])) {
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

    /**
     * Calculate maximum stable spring stiffness for current timestep
     */
    public static double getMaxSafeSpring(double mass, double dt){
        return (dt > KINDA_SMALL_NUMBER) ? mass / (dt * dt) : 0.0;
    }
}