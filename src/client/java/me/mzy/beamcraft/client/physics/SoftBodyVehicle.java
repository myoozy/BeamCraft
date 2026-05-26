package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
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
    public final LBeamContainer lBeams = new LBeamContainer();
    public final AnisotropicBeamContainer anisotropicBeams = new AnisotropicBeamContainer();
    public final TriangleContainer triangles = new TriangleContainer();
    public final TorsionBarContainer torsionbars = new TorsionBarContainer();
    public final SlideNodeContainer slidenodes = new SlideNodeContainer();
    public final WheelContainer wheels = new WheelContainer(this);
    public final FlexbodyContainer flexbodies = new FlexbodyContainer();

    // Bounding box cache array for independent part culling
    private int maxTrackedPartId = -1; // 必须在reset时重置
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    // 存储扁平化的 2D 矩阵: nodeInPart[nodeId * (maxPartId + 1) + partId]
    public boolean[] nodeInPartMatrix;
    public int matrixPartStride; // 矩阵的列数 (maxTrackedPartId + 1)

    public java.util.Map<String, List<BeamPointer>> breakGroupMap = new java.util.HashMap<>();
    private final java.util.Set<String> triggeredBreakGroups = new java.util.HashSet<>();

    private final SweepResultBuffer sweepResultBuffer = new SweepResultBuffer();

    // 获取实体当前的世界坐标作为锚点
    double entityX = 0.0;
    double entityY = 0.0;
    double entityZ = 0.0;

    public SoftBodyVehicle(PhysicsVehicleEntity parentEntity) {
        this.parentEntity = parentEntity;
        this.flexbodies.vehicleNamespace = parentEntity.getRootPartName();
        cacheEntityLocation();
    }

    public void cacheEntityLocation() {
        if (this.parentEntity == null) return;
        entityX = this.parentEntity.getX();
        entityY = this.parentEntity.getY();
        entityZ = this.parentEntity.getZ();
    }

    /*
    Must call updateEntityLocation after
     */
    public void updateLocalCOMCache() {
        nodes.getCenterOfMass(localCOM);
        nodes.moveNodes(-localCOM[0], -localCOM[1], -localCOM[2]);
    }

    /*
    Must call updateLocalCOMCache before
     */
    public void updateEntityLocation() {
        this.parentEntity.setVelocity(0, 0, 0);

        double newEntityX = entityX + localCOM[0];
        double newEntityY = entityY + localCOM[1];
        double newEntityZ = entityZ + localCOM[2];
        this.parentEntity.setPos(newEntityX,  newEntityY, newEntityZ);
    }

    public void updateBeamPrecompression(double dt) {
        normalBeams.updatePrecompression(dt);
        supportBeams.updatePrecompression(dt);
        boundedBeams.updatePrecompression(dt);
        lBeams.updatePrecompression(dt);
        anisotropicBeams.updatePrecompression(dt);
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
    public void addNode(String name, double x, double y, double z, double nodeMass,
                        double friction, double slidingFriction, int partId,
                        boolean collision, boolean selfCollision, java.util.List<String> groups) {
        nodes.addNode(name, x, y, z, nodeMass, friction, slidingFriction, partId, collision, selfCollision, groups);


        // ==========================================
        // ⭐追踪所有的part
        // 用来优化碰撞 (和minecraft世界 & 和softbody)
        // ==========================================
        if (partId > maxTrackedPartId) {
            maxTrackedPartId = partId;
            ensurePartCapacity(maxTrackedPartId);
        }
    }

    /**
     * Create physical beam constraint between two existing nodes
     */
    public void addBeam(int type,
                        String name1, String name2, String name3,
                        java.util.List<String> breakGroups, int breakGroupType,
                        double spring, double damp,
                        double deform, double strength,
                        double precomp, double precompRange, double precompTime,
                        double shortBound, double longBound,
                        double shortBoundRange, double longBoundRange,
                        double limitSpring, double limitDamp,
                        double dampVelSplit, double dampFast,
                        double dampRebound, double dampReboundFast,
                        double springExpansion, double dampExpansion, double transitionZone) {
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);

            nodes.degree[n1]++;
            nodes.degree[n2]++;

            BeamContainer container;
            int beamIdx;

            if (type == BeamContainer.BEAM_SUPPORT) {

                beamIdx = supportBeams.addBeam(breakGroups, breakGroupType,
                        n1, n2, dist, spring, damp,
                        deform, strength, precomp, precompRange, precompTime);
                container = supportBeams;

            } else if (type == BeamContainer.BEAM_BOUNDED) {

                beamIdx = boundedBeams.addBeam(breakGroups, breakGroupType,
                        n1, n2, dist,
                        spring, damp, deform, strength,
                        precomp, precompRange, precompTime,
                        shortBound, longBound,
                        shortBoundRange, longBoundRange,
                        limitSpring, limitDamp,
                        dampVelSplit, dampFast,
                        dampRebound, dampReboundFast);
                container = boundedBeams;

            } else if (type == BeamContainer.BEAM_LBEAM && nodes.nameToIndex.containsKey(name3)) {

                int n3 = nodes.nameToIndex.get(name3);
                nodes.degree[n3]++;
                double node12Dist = dist;
                dx = nodes.posX[n3] - nodes.posX[n1];
                dy = nodes.posY[n3] - nodes.posY[n1];
                dz = nodes.posZ[n3] - nodes.posZ[n1];
                double node13Dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                dx = nodes.posX[n3] - nodes.posX[n2];
                dy = nodes.posY[n3] - nodes.posY[n2];
                dz = nodes.posZ[n3] - nodes.posZ[n2];
                double node23Dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                beamIdx = lBeams.addBeam(breakGroups, breakGroupType,
                        n1, n2, n3, node12Dist, node13Dist, node23Dist,
                        spring, damp, deform, strength, precomp, precompRange, precompTime);
                container = lBeams;

            } else if (type == BeamContainer.BEAM_ANISOTROPIC) {

                beamIdx = anisotropicBeams.addBeam(breakGroups, breakGroupType,
                        n1, n2, dist, spring, damp,
                        deform, strength, precomp, precompRange, precompTime,
                        springExpansion, dampExpansion, transitionZone);
                container = anisotropicBeams;

            } else {

                beamIdx = normalBeams.addBeam(breakGroups, breakGroupType, n1, n2, dist, spring, damp,
                        deform, strength, precomp, precompRange, precompTime);
                container = normalBeams;

            }

            if (breakGroups != null && !breakGroups.isEmpty()) {
                for (String bg : breakGroups) {
                    this.breakGroupMap
                            .computeIfAbsent(bg, k -> new java.util.ArrayList<>())
                            .add(new BeamPointer(container, beamIdx));
                }
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


        // ==========================================
        // ⭐处理FlexBody的所有group
        // ==========================================
        flexbodies.compileGroupsCSR(nodes);

        // ==========================================
        // ⭐相同零件碰撞剔除
        // ==========================================

        // 1. 初始化矩阵大小
        matrixPartStride = maxTrackedPartId + 1;
        nodeInPartMatrix = new boolean[nodes.count * matrixPartStride];

        // 2. 基础传染：节点自己的原籍 Part
        for (int i = 0; i < nodes.count; i++) {
            int originalPart = nodes.partId[i];
            if (originalPart >= 0 && originalPart < matrixPartStride) {
                nodeInPartMatrix[i * matrixPartStride + originalPart] = true;
            }
        }

        // 3. 三角形传染：三角形所在的 Part，其三个顶点也默认从属于该 Part
        for (int i = 0; i < triangles.count; i++) {
            int tPart = triangles.partId[i]; // 你原来代码里有 triPartId，这里假设存为了 partId[i]
            if (tPart >= 0 && tPart < matrixPartStride) {
                nodeInPartMatrix[triangles.node1[i] * matrixPartStride + tPart] = true;
                nodeInPartMatrix[triangles.node2[i] * matrixPartStride + tPart] = true;
                nodeInPartMatrix[triangles.node3[i] * matrixPartStride + tPart] = true;
            }
        }

        // ==========================================
        // ⭐梁刚度钳制
        // ==========================================

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

        // ==========================================
        // 4. 处理角阻抗梁 (L-Beams)
        // ==========================================
        for (int i = 0; i < lBeams.count; i++) {
            int n1 = lBeams.node1[i];
            int n2 = lBeams.node2[i];
            int n3 = lBeams.node3[i];

            // 读取真实节点质量
            double m1 = nodes.mass[n1];
            double m2 = nodes.mass[n2];
            double m3 = nodes.mass[n3];

            // 计算 L-Beam 的广义反质量 (拐点 n3 承受两侧反力，权重取 2.0)
            double wTotal = (1.0 / m1) + (1.0 / m2) + (2.0 / m3);
            double genMass = 1.0 / wTotal;

            // 刚度安全截断
            double maxSafeSpring = 4.0 * genMass * invDt * invDt * safeFractionSpring;
            lBeams.spring[i] = Math.min(lBeams.spring[i], maxSafeSpring);

            // 阻尼安全截断 (极其关键！将强制把 180 截断到安全的 47.5 以内，彻底消灭 -2.6 倍数爆炸)
            double maxSafeDamp = genMass * invDt * safeFractionDamp;
            lBeams.damp[i] = Math.min(lBeams.damp[i], maxSafeDamp);
        }

        // ==========================================================
        // 5. 处理各向异性梁 (Anisotropic Beams) 安全截断
        // ==========================================================
        for (int i = 0; i < anisotropicBeams.count; i++) {
            int n1 = anisotropicBeams.node1[i];
            int n2 = anisotropicBeams.node2[i];

            // 计算质量乘子 (与普通梁逻辑保持一致)
            double effM1 = nodes.mass[n1] / Math.max(1.0, nodes.degree[n1] * avgCosSq);
            double effM2 = nodes.mass[n2] / Math.max(1.0, nodes.degree[n2] * avgCosSq);
            double effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            double realM1 = nodes.mass[n1];
            double realM2 = nodes.mass[n2];
            double unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            // 基础刚度与阻尼压制
            double maxSafeSpring = 4.0 * effReducedMass * invDt * invDt * safeFractionSpring;
            anisotropicBeams.spring[i] = Math.min(anisotropicBeams.spring[i], maxSafeSpring);

            double maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            anisotropicBeams.damp[i] = Math.min(anisotropicBeams.damp[i], maxSafeDamp);

            // ⚠️ 极其关键：对爆炸级的 Expansion 参数同步应用物理边界拦截！
            anisotropicBeams.springExpansion[i] = Math.min(anisotropicBeams.springExpansion[i], maxSafeSpring);
            anisotropicBeams.dampExpansion[i]   = Math.min(anisotropicBeams.dampExpansion[i],   maxSafeDamp);
        }
    }

    /**
     * Sreset velocity and deformation state
     */
    public void reset() {
        triggeredBreakGroups.clear();
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
        lBeams.clear();
        triangles.clear();
        torsionbars.clear();
        slidenodes.clear();
        wheels.clear();
        flexbodies.clear();
        triggeredBreakGroups.clear();
        maxTrackedPartId = -1;

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

    /**
     * 使用散度定理
     */
    private void solveTirePressure() {
        for (int w = 0; w < wheels.count; w++) {
            if (wheels.isDeflated[w]) continue;

            int start = wheels.tireTriangleIdxStart[w];
            int end = wheels.tireTriangleIdxEnd[w];
            if (start >= end || start == 0) continue;

            // 1. 直接读取【上一子步】缓存的静止体积与自适应符号，当场算出压强
            // 0.0005秒的反馈延迟对流体体积而言完全可以忽略不计，绝对稳定
            double currentVolume = wheels.prevVolume[w];
            if (currentVolume < KINDA_SMALL_NUMBER) continue;

            double p0_Pa = wheels.pressurePSI[w] * 6894.76;
            double absP0_Pa = p0_Pa + 101325.0;
            double currentAbsPressurePa = absP0_Pa * (wheels.initialVolume[w] / currentVolume);
            double pressureDiffPa = currentAbsPressurePa - 101325.0;

            // 均摊乘子 (结合上一子步提取的网格自适应朝向符号)
            double forceMultiplier = (pressureDiffPa * wheels.normalSign[w]) / 6.0;

            double nextVolumeSum = 0.0;

            // 2. 读取一次节点坐标，同时完成【推力施加】与【下步体积积分】！
            for (int i = start; i <= end; i++) {
                int nA = triangles.node1[i];
                int nB = triangles.node2[i];
                int nC = triangles.node3[i];

                double ax = nodes.posX[nA], ay = nodes.posY[nA], az = nodes.posZ[nA];
                double bx = nodes.posX[nB], by = nodes.posY[nB], bz = nodes.posZ[nB];
                double cx = nodes.posX[nC], cy = nodes.posY[nC], cz = nodes.posZ[nC];

                double abx = bx - ax, aby = by - ay, abz = bz - az;
                double acx = cx - ax, acy = cy - ay, acz = cz - az;

                // 算叉乘 (天然包含 2 倍面积与法线方向)
                double nx = aby * acz - abz * acy;
                double ny = abz * acx - abx * acz;
                double nz = abx * acy - aby * acx;

                // --- A. 施加真实的气压外推力 ---
                double fx = nx * forceMultiplier;
                double fy = ny * forceMultiplier;
                double fz = nz * forceMultiplier;

                nodes.forceX[nA] += fx; nodes.forceY[nA] += fy; nodes.forceZ[nA] += fz;
                nodes.forceX[nB] += fx; nodes.forceY[nB] += fy; nodes.forceZ[nB] += fz;
                nodes.forceX[nC] += fx; nodes.forceY[nC] += fy; nodes.forceZ[nC] += fz;

                // --- B. 顺手使用标量三重积累加当前网格体积 (供下一子步解算使用) ---
                // 完美复用已加载的寄存器数据
                nextVolumeSum += (ax * nx + ay * ny + az * nz);
            }

            // 3. 更新黑板缓存
            wheels.normalSign[w] = (nextVolumeSum < 0.0) ? -1.0 : 1.0;
            wheels.prevVolume[w] = Math.abs(nextVolumeSum / 6.0);
        }
    }

    public void triggerBreakGroup(String groupName) {
        // 使用 Set.add() 充当安全闸
        // 如果 add 返回 false，说明这个组之前已经触发过了，直接拦截，彻底切断死循环
        if (!triggeredBreakGroups.add(groupName)) {
            return;
        }

        // 只读读取，不破坏结构
        List<BeamPointer> linkedBeams = breakGroupMap.get(groupName);
        if (linkedBeams == null) return;

        for (BeamPointer ptr : linkedBeams) {
            breakBeamAt(ptr.container, ptr.index);
        }
    }

    private void breakBeamAt(BeamContainer container, int idx) {
        container.broken[idx] = true;
        if (container.breakGroupType[idx] == 0) {
            if (container.assignedBreakGroups != null && container.assignedBreakGroups[idx] != null) {
                for (String bg : container.assignedBreakGroups[idx]) {
                    this.triggerBreakGroup(bg); // 抛给车辆的只读状态机处理
                }
            }
            int wheelIdx = container.wheelId[idx];
            wheels.deflateWheel(wheelIdx);
        }
    }

    private void solveNormalBeams(double dt, double invDt) {
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
                breakBeamAt(normalBeams, i);
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
    }

    private void solveSupportBeams(double dt, double invDt) {
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
                breakBeamAt(supportBeams, i);
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
    }

    private void solveBoundedBeams(double dt, double invDt) {
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
                breakBeamAt(boundedBeams, i);
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
    }

    private void solveLBeams(double dt, double invDt) {
        for (int i = 0; i < lBeams.count; i++) {
            if (lBeams.broken[i]) continue;

            int n1 = lBeams.node1[i]; // 端点 1 (例如 hInCur)
            int n2 = lBeams.node2[i]; // 端点 2 (例如 tOutCur)
            int n3 = lBeams.node3[i]; // 共享拐点 3 (例如 tInCur)

            // 1. 读取三点实时坐标
            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];

            // 2. 计算三边向量与长度
            // 臂 1-3
            double dx13 = x1 - x3, dy13 = y1 - y3, dz13 = z1 - z3;
            double l1Sq = dx13*dx13 + dy13*dy13 + dz13*dz13;

            // 臂 2-3
            double dx23 = x2 - x3, dy23 = y2 - y3, dz23 = z2 - z3;
            double l2Sq = dx23*dx23 + dy23*dy23 + dz23*dz23;

            // 对角线 1-2
            double dx12 = x2 - x1, dy12 = y2 - y1, dz12 = z2 - z1;
            double distSq = dx12*dx12 + dy12*dy12 + dz12*dz12;

            // 防御性拦截极小距离，防止 NaN 传染
            if (l1Sq < KINDA_SMALL_NUMBER || l2Sq < KINDA_SMALL_NUMBER || distSq < KINDA_SMALL_NUMBER) continue;

            double l1 = Math.sqrt(l1Sq);
            double l2 = Math.sqrt(l2Sq);
            double dist = Math.sqrt(distSq);

            double invL1 = 1.0 / l1;
            double invL2 = 1.0 / l2;
            double invDist = 1.0 / dist;

            // 3. 计算动态目标对角线长度 D_target
            double cosTheta0 = lBeams.restCosTheta[i];
            double targetDistSq = l1Sq + l2Sq - 2.0 * l1 * l2 * cosTheta0;
            if (targetDistSq < KINDA_SMALL_NUMBER) continue;
            double targetDist = Math.sqrt(targetDistSq);
            double invTargetDist = 1.0 / targetDist;

            // 4. 计算链式求导放大因子
            double g1 = (l1 - l2 * cosTheta0) * invTargetDist;
            double g2 = (l2 - l1 * cosTheta0) * invTargetDist;

            // 5. 计算真实的相对阻尼速率 (彻底消灭垂直压缩抖动)
            double vx1 = nodes.velX[n1], vy1 = nodes.velY[n1], vz1 = nodes.velZ[n1];
            double vx2 = nodes.velX[n2], vy2 = nodes.velY[n2], vz2 = nodes.velZ[n2];
            double vx3 = nodes.velX[n3], vy3 = nodes.velY[n3], vz3 = nodes.velZ[n3];

            // 臂 1-3 的伸缩速率
            double v13x = vx1 - vx3, v13y = vy1 - vy3, v13z = vz1 - vz3;
            double l1Dot = (v13x*dx13 + v13y*dy13 + v13z*dz13) * invL1;

            // 臂 2-3 的伸缩速率
            double v23x = vx2 - vx3, v23y = vy2 - vy3, v23z = vz2 - vz3;
            double l2Dot = (v23x*dx23 + v23y*dy23 + v23z*dz23) * invL2;

            // 目标对角线长度随外挂臂形变产生的理论收缩速率
            double targetDistDot = g1 * l1Dot + g2 * l2Dot;

            // 物理对角线的实际接近速率
            double v12x = vx2 - vx1, v12y = vy2 - vy1, v12z = vz2 - vz1;
            double distDot = (v12x*dx12 + v12y*dy12 + v12z*dz12) * invDist;

            // 真正的弹性相对速率 = 实际速率 - 理论速率
            double dampVel = distDot - targetDistDot;

            // 6. 标量合力计算
            double activeSpring = lBeams.spring[i];
            double springForce = activeSpring * (dist - targetDist);
            double dampForce = lBeams.damp[i] * dampVel;
            double totalForce = springForce + dampForce;

            double absTotalForce = Math.abs(totalForce);
            if (absTotalForce > lBeams.strength[i]) {
                breakBeamAt(lBeams, i);
                continue;
            }

            // 塑性形变逻辑兜底：通过微调常驻角度吸收冲击
            if (absTotalForce > lBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - lBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (lBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                double sign = Math.signum(dist - targetDist);
                lBeams.restCosTheta[i] -= sign * deformAmount * invL1 * invL2;
                // 限制余弦范围防止崩坏
                if (lBeams.restCosTheta[i] > 1.0) lBeams.restCosTheta[i] = 1.0;
                if (lBeams.restCosTheta[i] < -1.0) lBeams.restCosTheta[i] = -1.0;
            }

            // 7. 🚀 三点梯度力分配
            // 各边的单位向量
            double u13x = dx13 * invL1,   u13y = dy13 * invL1,   u13z = dz13 * invL1;
            double u23x = dx23 * invL2,   u23y = dy23 * invL2,   u23z = dz23 * invL2;
            double u12x = dx12 * invDist, u12y = dy12 * invDist, u12z = dz12 * invDist;

            // 施加给 端点 1 的力 (注意是 + g1)
            double f1x = totalForce * (u12x + g1 * u13x);
            double f1y = totalForce * (u12y + g1 * u13y);
            double f1z = totalForce * (u12z + g1 * u13z);

            // 施加给 端点 2 的力 (注意是 + g2)
            double f2x = totalForce * (-u12x + g2 * u23x);
            double f2y = totalForce * (-u12y + g2 * u23y);
            double f2z = totalForce * (-u12z + g2 * u23z);

            // 施加给 拐点 3 的反作用力 (注意是全部取负！完美抵消 f1 和 f2 附加的额外分量)
            double f3x = totalForce * (-g1 * u13x - g2 * u23x);
            double f3y = totalForce * (-g1 * u13y - g2 * u23y);
            double f3z = totalForce * (-g1 * u13z - g2 * u23z);

            nodes.forceX[n1] += f1x; nodes.forceY[n1] += f1y; nodes.forceZ[n1] += f1z;
            nodes.forceX[n2] += f2x; nodes.forceY[n2] += f2y; nodes.forceZ[n2] += f2z;
            nodes.forceX[n3] += f3x; nodes.forceY[n3] += f3y; nodes.forceZ[n3] += f3z;
        }
    }

    private void solveAnisotropicBeams(double dt, double invDt)  {
        for (int i = 0; i < anisotropicBeams.count; i++) {
            if (anisotropicBeams.broken[i]) continue;

            int n1 = anisotropicBeams.node1[i];
            int n2 = anisotropicBeams.node2[i];

            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            double dist = Math.sqrt(distSq);
            double invDist = 1.0 / dist;

            // 初始生成原长 (Spawned Length)
            double restL = anisotropicBeams.restLength[i];

            // 默认输出基础刚度和阻尼 (适用于 压缩区 dist <= restL)
            double activeSpring = anisotropicBeams.spring[i];
            double activeDamp   = anisotropicBeams.damp[i];

            // 🚀 仅在 Expansion (拉伸区 dist > restL) 触发高级逻辑
            if (dist > restL) {
                double expSpring = anisotropicBeams.springExpansion[i];
                double expDamp   = anisotropicBeams.dampExpansion[i];
                double tZoneRatio = anisotropicBeams.transitionZone[i];

                if (tZoneRatio > KINDA_SMALL_NUMBER) {
                    // 1. 算出绝对过渡区长度 (比例 × 原长)
                    double absoluteTZone = tZoneRatio * restL;
                    // 2. 算出当前拉伸量
                    double stretch = dist - restL;

                    if (stretch >= absoluteTZone) {
                        // 彻底越过过渡区，完全使用 Expansion 属性
                        activeSpring = expSpring;
                        activeDamp   = expDamp;
                    } else {
                        // 处于过渡区斜坡内部，进行线性插值 (Lerp)
                        double factor = stretch / absoluteTZone;
                        activeSpring += (expSpring - activeSpring) * factor;
                        activeDamp   += (expDamp   - activeDamp)   * factor;
                    }
                } else {
                    // 默认情况 (transitionZone == 0)，瞬间越变
                    activeSpring = expSpring;
                    activeDamp   = expDamp;
                }
            }

            // 计算弹簧力
            double springForce = activeSpring * (dist - restL);

            // 计算阻尼力
            double vx = nodes.velX[n2] - nodes.velX[n1];
            double vy = nodes.velY[n2] - nodes.velY[n1];
            double vz = nodes.velZ[n2] - nodes.velZ[n1];
            double relVel = (vx*dx + vy*dy + vz*dz) * invDist;
            double dampForce = activeDamp * relVel;

            double totalForce = springForce + dampForce;
            double absTotalForce = Math.abs(totalForce);

            // 断裂判定
            if (absTotalForce > anisotropicBeams.strength[i]) {
                breakBeamAt(anisotropicBeams, i);
                continue;
            }

            // 塑性形变
            if (absTotalForce > anisotropicBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - anisotropicBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (anisotropicBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) anisotropicBeams.restLength[i] += deformAmount;
                else anisotropicBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            // 施加力
            double fx = totalForce * dx * invDist;
            double fy = totalForce * dy * invDist;
            double fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveTorsionBars(double dt, double invDt) {
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
    }

    private void solveSlideNodes(double dt, double invDt) {
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
        // 🛡️ 轮胎气压计算 (Pressure Wheels)
        // ==========================================
        solveTirePressure();

        // ==========================================
        // 🛡️ 梁计算 (Beams)
        // ==========================================

        // ========== 1. 处理普通梁（NORMAL） ==========
        solveNormalBeams(dt, invDt);

        // ========== 2. 处理支撑梁（SUPPORT，仅压缩） ==========
        solveSupportBeams(dt, invDt);

        // ========== 3. 处理限界梁（BOUNDED，含复杂阻尼和限位） ==========
        solveBoundedBeams(dt, invDt);

        // ========== 4. LBeams ==========
        solveLBeams(dt, invDt);

        // ========== 5. 各向异性梁计算 (Anisotropic Beams) ==========
        solveAnisotropicBeams(dt, invDt);

        // ==========================================
        // 🛡️ 扭杆计算 (Torsionbars)
        // ==========================================
        solveTorsionBars(dt, invDt);

        // ==========================================
        // 🛡️ 计算滑块 (slidenodes)
        // ==========================================
        solveSlideNodes(dt, invDt);

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

                if (hitVeh == this) {
                    int triPartId = triangles.partId[i];
                    if (triPartId >= 0 && triPartId < matrixPartStride) {
                        // 只要这个被撞的节点，从属于这个三角形所在的 Part，直接无视！
                        if (nodeInPartMatrix[hitNodeId * matrixPartStride + triPartId]) {
                            continue;
                        }
                    }
                }

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

                if (hitY || hitX || hitZ) {
                    double invDt = 1.0 / dt;

                    // TODO: 从缓存中读取动态的摩擦系数和回弹系数？但是回弹系数即便是0也很弹，因为beam有弹性。
                    double reboundCoef = PhysicsWorld.BLOCK_REBOUND;
                    double blockFriction = PhysicsWorld.BLOCK_FRICTION;

                    // 提取常驻重力在一小段时间内向下施加的冲量大小标量
                    double gravityImpulse = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY) * dt;

                    // 1. 仅当该轴发生碰撞时，才计入位置修正量，利用三元运算符避免跳转开销
                    double pushX = hitX ? Math.abs(nodes.posX[i] - oldLocalX) : 0.0;
                    double pushY = hitY ? Math.abs(nodes.posY[i] - oldLocalY) : 0.0;
                    double pushZ = hitZ ? Math.abs(nodes.posZ[i] - oldLocalZ) : 0.0;

                    // 真实的立体合成法向挤压距离 (欧几里得长度)
                    double totalNormalPush = Math.sqrt(pushX*pushX + pushY*pushY + pushZ*pushZ);

                    // 2. 计算统一的等效立体载荷力 Fn (牛顿)
                    double equivalentLoadN = (nodes.mass[i] * totalNormalPush) * (invDt * invDt);
                    double minGravityLoad = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY);
                    if (equivalentLoadN < minGravityLoad) equivalentLoadN = minGravityLoad;

                    // 提取基础摩擦
                    double mu_s = nodes.friction[i] * blockFriction;
                    double mu_k = nodes.slidingFriction[i] * blockFriction;

                    // 3. 针对轮胎节点计算统一的高级载荷衰减与 Stribeck 乘子
                    int wIdx = nodes.wheelId[i];
                    if (0 <= wIdx && wIdx < wheels.count) {
                        double staticBase  = wheels.frictionCoef[wIdx];
                        double slidingBase = wheels.slidingFrictionCoef[wIdx];
                        double noLoad      = wheels.noLoadCoef[wIdx];
                        double fullLoad    = wheels.fullLoadCoef[wIdx];
                        double slope       = wheels.loadSensitivitySlope[wIdx];
                        double treadCoef   = wheels.treadCoef[wIdx];

                        // 载荷衰减 (此时使用完美兼容多面的 equivalentLoadN)
                        double loadFactor = noLoad - (slope * equivalentLoadN);
                        if (loadFactor < fullLoad) loadFactor = fullLoad;

                        // 计算 3D 真实的切向滑移速率 (剔除已碰撞轴向的法向分量)
                        // 哪面墙撞了，那个轴的速度就是法向分量，剩余轴的速度平方和即为切向滑移速率
                        double vx = nodes.velX[i], vy = nodes.velY[i], vz = nodes.velZ[i];
                        double tVelSq = (hitX ? 0 : vx*vx) + (hitY ? 0 : vy*vy) + (hitZ ? 0 : vz*vz);
                        double vtLen = Math.sqrt(tVelSq);

                        // Stribeck 曲线过渡
                        double stribeckVel = wheels.stribeckVelMult[wIdx];
                        double exponent    = wheels.stribeckExponent[wIdx];
                        double speedFactor = 1.0;
                        if (vtLen > 1e-4 && stribeckVel > 1e-4) {
                            double velRatio = vtLen / stribeckVel;
                            speedFactor = Math.exp(-Math.pow(velRatio, exponent));
                        }

                        double dynamicMuMultiplier = slidingBase + (staticBase - slidingBase) * speedFactor;
                        mu_s = (staticBase * loadFactor * treadCoef)  * blockFriction;
                        mu_k = (dynamicMuMultiplier * loadFactor * treadCoef) * blockFriction;
                    }

                    if (hitY) {
                        //修正：真实的法向总冲量 = 反弹动量差值 + 重力压迫冲量
                        // J = m * |v| * (1 + e) + m * |g| * dt
                        double jn = nodes.mass[i] * Math.abs(nodes.velY[i]) * (1.0 + reboundCoef) + gravityImpulse;

                        // 执行垂直反弹与位置回退
                        nodes.velY[i] *= -reboundCoef;
                        nodes.posY[i] = oldLocalY;

                        // --- 解耦层 1：速度层真实冲量衰减 ---
                        double vx = nodes.velX[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vx*vx + vz*vz);
                        double jtReq = vtLen * nodes.mass[i]; // 完全制停所需的真实动量

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0; nodes.velZ[i] = 0.0; // 静摩擦咬死
                            } else {
                                // 动摩擦恒定阻力剥离
                                double frictionImpulse = mu_k * jn;
                                velKeepRatio = Math.max(0.0, 1.0 - (frictionImpulse / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        // --- 解耦层 2：PBD 几何位置层蠕动约束 ---
                        // 位置的拉扯极限同样由当前接触面的库仑上限决定
                        double creepX = nodes.posX[i] - oldLocalX, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepX*creepX + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            // 几何位置完全锁死 (不发生蠕动)
                            nodes.posX[i] = oldLocalX; nodes.posZ[i] = oldLocalZ;
                        } else {
                            // 发生微观打滑，位置蠕动按速度层相同的比例或动摩擦比例衰减
                            // 采用 velKeepRatio 能够保证位置滑动与速度滑动在视觉上绝对同步
                            nodes.posX[i] = oldLocalX + creepX * velKeepRatio;
                            nodes.posZ[i] = oldLocalZ + creepZ * velKeepRatio;
                        }
                    }

                    if (hitX) {
                        double jn = nodes.mass[i] * Math.abs(nodes.velX[i]) * (1.0 + reboundCoef) + gravityImpulse;
                        nodes.velX[i] *= -reboundCoef;
                        nodes.posX[i] = oldLocalX;

                        double vy = nodes.velY[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vy*vy + vz*vz);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velY[i] = 0.0; nodes.velZ[i] = 0.0;
                            } else {
                                velKeepRatio = Math.max(0.0, 1.0 - ((mu_k * jn) / jtReq));
                                nodes.velY[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        double creepY = nodes.posY[i] - oldLocalY, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepY*creepY + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posY[i] = oldLocalY; nodes.posZ[i] = oldLocalZ;
                        } else {
                            nodes.posY[i] = oldLocalY + creepY * velKeepRatio;
                            nodes.posZ[i] = oldLocalZ + creepZ * velKeepRatio;
                        }
                    }

                    if (hitZ) {
                        double jn = nodes.mass[i] * Math.abs(nodes.velZ[i]) * (1.0 + reboundCoef) + gravityImpulse;
                        nodes.velZ[i] *= -reboundCoef;
                        nodes.posZ[i] = oldLocalZ;

                        double vx = nodes.velX[i], vy = nodes.velY[i];
                        double vtLen = Math.sqrt(vx*vx + vy*vy);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0; nodes.velY[i] = 0.0;
                            } else {
                                velKeepRatio = Math.max(0.0, 1.0 - ((mu_k * jn) / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velY[i] *= velKeepRatio;
                            }
                        }

                        double creepX = nodes.posX[i] - oldLocalX, creepY = nodes.posY[i] - oldLocalY;
                        double creepLen = Math.sqrt(creepX*creepX + creepY*creepY);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posX[i] = oldLocalX; nodes.posY[i] = oldLocalY;
                        } else {
                            nodes.posX[i] = oldLocalX + creepX * velKeepRatio;
                            nodes.posY[i] = oldLocalY + creepY * velKeepRatio;
                        }
                    }
                }
            }
        }
    }
}