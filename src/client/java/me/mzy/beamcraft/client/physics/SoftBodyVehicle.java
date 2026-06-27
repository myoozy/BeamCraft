package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.World;

import java.util.Arrays;
import java.util.List;

public class SoftBodyVehicle {
    public static final float KINDA_SMALL_NUMBER = PhysicsWorld.KINDA_SMALL_NUMBER;
    public static final int MAX_AABB_SIZE = PhysicsWorld.MAX_AABB_SIZE;

    public final PhysicsVehicleEntity parentEntity; // 缁戝畾鐨勫疄浣?
    public final float[] localCOM = new float[3];
    public int vehicleId = -1; // 鐗╃悊涓栫晫缁欏畠鍒嗛厤鐨勯『搴?ID (0, 1, 2...)
    public int globalNodeOffset = 0; // 鍏ㄥ眬鑺傜偣鍋忕Щ閲?

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
    private int maxTrackedPartId = -1; // 蹇呴』鍦╮eset鏃堕噸缃?
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    // 瀛樺偍鎵佸钩鍖栫殑 2D 鐭╅樀: nodeInPart[nodeId * (maxPartId + 1) + partId]
    public boolean[] nodeInPartMatrix;
    public int matrixPartStride; // 鐭╅樀鐨勫垪鏁?(maxTrackedPartId + 1)

    public java.util.Map<String, List<BeamPointer>> breakGroupMap = new java.util.HashMap<>();
    private final java.util.Set<String> triggeredBreakGroups = new java.util.HashSet<>();

    private final SweepResultBuffer sweepResultBuffer = new SweepResultBuffer();

    // 鑾峰彇瀹炰綋褰撳墠鐨勪笘鐣屽潗鏍囦綔涓洪敋鐐?
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
        float fDt = (float) dt;
        normalBeams.updatePrecompression(fDt);
        supportBeams.updatePrecompression(fDt);
        boundedBeams.updatePrecompression(fDt);
        lBeams.updatePrecompression(fDt);
        anisotropicBeams.updatePrecompression(fDt);
    }

    /**
     * Expand array capacity to avoid index out of bounds for new part id
     */
    private void ensurePartCapacity(int maxId) {
        if (maxId >= partMinX.length) {
            // 涓嶈鐢?maxId * 2锛屽鏋?maxId 鏄?0锛?*2 杩樻槸 0锛屼細瀵艰嚧涓ラ噸宕╂簝锛?
            // 鐢?Math.max 纭繚瀹冭嚦灏戞瘮 maxId 澶?1
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
        // 猸愯拷韪墍鏈夌殑part
        // 鐢ㄦ潵浼樺寲纰版挒 (鍜宮inecraft涓栫晫 & 鍜宻oftbody)
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
        // 猸愬鐞咶lexBody鐨勬墍鏈塯roup
        // ==========================================
        flexbodies.compileGroupsCSR(nodes);

        // ==========================================
        // 猸愮浉鍚岄浂浠剁鎾炲墧闄?
        // ==========================================

        // 1. 鍒濆鍖栫煩闃靛ぇ灏?
        matrixPartStride = maxTrackedPartId + 1;
        nodeInPartMatrix = new boolean[nodes.count * matrixPartStride];

        // 2. 鍩虹浼犳煋锛氳妭鐐硅嚜宸辩殑鍘熺睄 Part
        for (int i = 0; i < nodes.count; i++) {
            int originalPart = nodes.partId[i];
            if (originalPart >= 0 && originalPart < matrixPartStride) {
                nodeInPartMatrix[i * matrixPartStride + originalPart] = true;
            }
        }

        // 3. 涓夎褰紶鏌擄細涓夎褰㈡墍鍦ㄧ殑 Part锛屽叾涓変釜椤剁偣涔熼粯璁や粠灞炰簬璇?Part
        for (int i = 0; i < triangles.count; i++) {
            int tPart = triangles.partId[i]; // 浣犲師鏉ヤ唬鐮侀噷鏈?triPartId锛岃繖閲屽亣璁惧瓨涓轰簡 partId[i]
            if (tPart >= 0 && tPart < matrixPartStride) {
                nodeInPartMatrix[triangles.node1[i] * matrixPartStride + tPart] = true;
                nodeInPartMatrix[triangles.node2[i] * matrixPartStride + tPart] = true;
                nodeInPartMatrix[triangles.node3[i] * matrixPartStride + tPart] = true;
            }
        }

        // ==========================================
        // 猸愭鍒氬害閽冲埗
        // ==========================================

        float invDt = PhysicsWorld.invPhysicsDT;
        float safeFractionSpring = 0.95f;
        float safeFractionDamp = 0.95f;
        float avgCosSq = 1.0f;

        // ==========================================
        // 1. 澶勭悊鏅€氭 (Normal Beams)
        // ==========================================
        for (int i = 0; i < normalBeams.count; i++) {
            int n1 = normalBeams.node1[i];
            int n2 = normalBeams.node2[i];

            // --- A. 鍒氬害璐ㄩ噺 (Scaled by Degree) ---
            float effM1 = nodes.mass[n1] / Math.max(1.0f, nodes.degree[n1] * avgCosSq);
            float effM2 = nodes.mass[n2] / Math.max(1.0f, nodes.degree[n2] * avgCosSq);
            float effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            // --- B. 闃诲凹璐ㄩ噺 (Unscaled) ---
            float realM1 = nodes.mass[n1];
            float realM2 = nodes.mass[n2];
            float unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            // 寮圭哀鎴柇锛氫娇鐢ㄥ甫 degree 鎯╃綒鐨勮川閲忥紝涔樹互 4.0 鐨勭粷瀵规瀬闄?
            float maxSafeSpring = 4.0f * effReducedMass * invDt * invDt * safeFractionSpring;
            normalBeams.spring[i] = Math.min(normalBeams.spring[i], maxSafeSpring);

            // 闃诲凹鎴柇锛氫娇鐢ㄤ綘鎺ㄥ鍑虹殑鐗╃悊鍏紡 (Unscaled Mass * invDt)
            float maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            normalBeams.damp[i] = Math.min(normalBeams.damp[i], maxSafeDamp);
        }

        // ==========================================
        // 2. 澶勭悊鏀拺姊?(Support Beams)
        // ==========================================
        for (int i = 0; i < supportBeams.count; i++) {
            int n1 = supportBeams.node1[i];
            int n2 = supportBeams.node2[i];

            float effM1 = nodes.mass[n1] / Math.max(1.0f, nodes.degree[n1] * avgCosSq);
            float effM2 = nodes.mass[n2] / Math.max(1.0f, nodes.degree[n2] * avgCosSq);
            float effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            float realM1 = nodes.mass[n1];
            float realM2 = nodes.mass[n2];
            float unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            float maxSafeSpring = 4.0f * effReducedMass * invDt * invDt * safeFractionSpring;
            supportBeams.spring[i] = Math.min(supportBeams.spring[i], maxSafeSpring);

            float maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            supportBeams.damp[i] = Math.min(supportBeams.damp[i], maxSafeDamp);
        }

        // ==========================================
        // 3. 澶勭悊闄愮晫姊?(Bounded Beams)
        // ==========================================
        for (int i = 0; i < boundedBeams.count; i++) {
            int n1 = boundedBeams.node1[i];
            int n2 = boundedBeams.node2[i];

            float effM1 = nodes.mass[n1] / Math.max(1.0f, nodes.degree[n1] * avgCosSq);
            float effM2 = nodes.mass[n2] / Math.max(1.0f, nodes.degree[n2] * avgCosSq);
            float effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            float realM1 = nodes.mass[n1];
            float realM2 = nodes.mass[n2];
            float unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            float maxSafeSpring = 4.0f * effReducedMass * invDt * invDt * safeFractionSpring;
            boundedBeams.spring[i] = Math.min(boundedBeams.spring[i], maxSafeSpring);
            boundedBeams.limitSpring[i] = Math.min(boundedBeams.limitSpring[i], maxSafeSpring);

            float maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            boundedBeams.damp[i] = Math.min(boundedBeams.damp[i], maxSafeDamp);
            boundedBeams.limitDamp[i] = Math.min(boundedBeams.limitDamp[i], maxSafeDamp);
            boundedBeams.dampFast[i] = Math.min(boundedBeams.dampFast[i], maxSafeDamp);
            boundedBeams.dampRebound[i] = Math.min(boundedBeams.dampRebound[i], maxSafeDamp);
            boundedBeams.dampReboundFast[i] = Math.min(boundedBeams.dampReboundFast[i], maxSafeDamp);
        }

        // ==========================================
        // 4. 澶勭悊瑙掗樆鎶楁 (L-Beams)
        // ==========================================
        for (int i = 0; i < lBeams.count; i++) {
            int n1 = lBeams.node1[i];
            int n2 = lBeams.node2[i];
            int n3 = lBeams.node3[i];

            // 璇诲彇鐪熷疄鑺傜偣璐ㄩ噺
            float m1 = nodes.mass[n1];
            float m2 = nodes.mass[n2];
            float m3 = nodes.mass[n3];

            // 璁＄畻 L-Beam 鐨勫箍涔夊弽璐ㄩ噺 (鎷愮偣 n3 鎵垮彈涓や晶鍙嶅姏锛屾潈閲嶅彇 2.0)
            float wTotal = (1.0f / m1) + (1.0f / m2) + (2.0f / m3);
            float genMass = 1.0f / wTotal;

            // 鍒氬害瀹夊叏鎴柇
            float maxSafeSpring = 4.0f * genMass * invDt * invDt * safeFractionSpring;
            lBeams.spring[i] = Math.min(lBeams.spring[i], maxSafeSpring);

            // 闃诲凹瀹夊叏鎴柇 (鏋佸叾鍏抽敭锛佸皢寮哄埗鎶?180 鎴柇鍒板畨鍏ㄧ殑 47.5 浠ュ唴锛屽交搴曟秷鐏?-2.6 鍊嶆暟鐖嗙偢)
            float maxSafeDamp = genMass * invDt * safeFractionDamp;
            lBeams.damp[i] = Math.min(lBeams.damp[i], maxSafeDamp);
        }

        // ==========================================================
        // 5. 澶勭悊鍚勫悜寮傛€ф (Anisotropic Beams) 瀹夊叏鎴柇
        // ==========================================================
        for (int i = 0; i < anisotropicBeams.count; i++) {
            int n1 = anisotropicBeams.node1[i];
            int n2 = anisotropicBeams.node2[i];

            // 璁＄畻璐ㄩ噺涔樺瓙 (涓庢櫘閫氭閫昏緫淇濇寔涓€鑷?
            float effM1 = nodes.mass[n1] / Math.max(1.0f, nodes.degree[n1] * avgCosSq);
            float effM2 = nodes.mass[n2] / Math.max(1.0f, nodes.degree[n2] * avgCosSq);
            float effReducedMass = (effM1 * effM2) / (effM1 + effM2);

            float realM1 = nodes.mass[n1];
            float realM2 = nodes.mass[n2];
            float unscaledReducedMass = (realM1 * realM2) / (realM1 + realM2);

            // 鍩虹鍒氬害涓庨樆灏煎帇鍒?
            float maxSafeSpring = 4.0f * effReducedMass * invDt * invDt * safeFractionSpring;
            anisotropicBeams.spring[i] = Math.min(anisotropicBeams.spring[i], maxSafeSpring);

            float maxSafeDamp = unscaledReducedMass * invDt * safeFractionDamp;
            anisotropicBeams.damp[i] = Math.min(anisotropicBeams.damp[i], maxSafeDamp);

            // 鈿狅笍 鏋佸叾鍏抽敭锛氬鐖嗙偢绾х殑 Expansion 鍙傛暟鍚屾搴旂敤鐗╃悊杈圭晫鎷︽埅锛?
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

        System.out.println("馃Ч Vehicle data cleared and reset");
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

            double px = entityX + nodes.posX[i]; // to world coordinate锛?
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
     * 浣跨敤鏁ｅ害瀹氱悊
     */
    private void solveTirePressure() {
        for (int w = 0; w < wheels.count; w++) {
            if (wheels.isDeflated[w]) continue;

            int start = wheels.tireTriangleIdxStart[w];
            int end = wheels.tireTriangleIdxEnd[w];
            if (start >= end || start == 0) continue;

            // 1. 鐩存帴璇诲彇銆愪笂涓€瀛愭銆戠紦瀛樼殑闈欐浣撶Н涓庤嚜閫傚簲绗﹀彿锛屽綋鍦虹畻鍑哄帇寮?
            // 0.0005绉掔殑鍙嶉寤惰繜瀵规祦浣撲綋绉€岃█瀹屽叏鍙互蹇界暐涓嶈锛岀粷瀵圭ǔ瀹?
            double currentVolume = wheels.prevVolume[w];
            if (currentVolume < KINDA_SMALL_NUMBER) continue;

            double p0_Pa = wheels.pressurePSI[w] * 6894.76;
            double absP0_Pa = p0_Pa + 101325.0;
            double currentAbsPressurePa = absP0_Pa * (wheels.initialVolume[w] / currentVolume);
            double pressureDiffPa = currentAbsPressurePa - 101325.0;

            // 鍧囨憡涔樺瓙 (缁撳悎涓婁竴瀛愭鎻愬彇鐨勭綉鏍艰嚜閫傚簲鏈濆悜绗﹀彿)
            double forceMultiplier = (pressureDiffPa * wheels.normalSign[w]) / 6.0;

            double nextVolumeSum = 0.0;

            // 2. 璇诲彇涓€娆¤妭鐐瑰潗鏍囷紝鍚屾椂瀹屾垚銆愭帹鍔涙柦鍔犮€戜笌銆愪笅姝ヤ綋绉Н鍒嗐€戯紒
            for (int i = start; i <= end; i++) {
                int nA = triangles.node1[i];
                int nB = triangles.node2[i];
                int nC = triangles.node3[i];

                double ax = nodes.posX[nA], ay = nodes.posY[nA], az = nodes.posZ[nA];
                double bx = nodes.posX[nB], by = nodes.posY[nB], bz = nodes.posZ[nB];
                double cx = nodes.posX[nC], cy = nodes.posY[nC], cz = nodes.posZ[nC];

                double abx = bx - ax, aby = by - ay, abz = bz - az;
                double acx = cx - ax, acy = cy - ay, acz = cz - az;

                // 绠楀弶涔?(澶╃劧鍖呭惈 2 鍊嶉潰绉笌娉曠嚎鏂瑰悜)
                double nx = aby * acz - abz * acy;
                double ny = abz * acx - abx * acz;
                double nz = abx * acy - aby * acx;

                // --- A. 鏂藉姞鐪熷疄鐨勬皵鍘嬪鎺ㄥ姏 ---
                double fx = nx * forceMultiplier;
                double fy = ny * forceMultiplier;
                double fz = nz * forceMultiplier;

                nodes.forceX[nA] += fx; nodes.forceY[nA] += fy; nodes.forceZ[nA] += fz;
                nodes.forceX[nB] += fx; nodes.forceY[nB] += fy; nodes.forceZ[nB] += fz;
                nodes.forceX[nC] += fx; nodes.forceY[nC] += fy; nodes.forceZ[nC] += fz;

                // --- B. 椤烘墜浣跨敤鏍囬噺涓夐噸绉疮鍔犲綋鍓嶇綉鏍间綋绉?(渚涗笅涓€瀛愭瑙ｇ畻浣跨敤) ---
                // 瀹岀編澶嶇敤宸插姞杞界殑瀵勫瓨鍣ㄦ暟鎹?
                nextVolumeSum += (ax * nx + ay * ny + az * nz);
            }

            // 3. 鏇存柊榛戞澘缂撳瓨
            wheels.normalSign[w] = (nextVolumeSum < 0.0) ? -1.0f : 1.0f;
            wheels.prevVolume[w] = (float) Math.abs(nextVolumeSum / 6.0);
        }
    }

    public void triggerBreakGroup(String groupName) {
        // 浣跨敤 Set.add() 鍏呭綋瀹夊叏闂?
        // 濡傛灉 add 杩斿洖 false锛岃鏄庤繖涓粍涔嬪墠宸茬粡瑙﹀彂杩囦簡锛岀洿鎺ユ嫤鎴紝褰诲簳鍒囨柇姝诲惊鐜?
        if (!triggeredBreakGroups.add(groupName)) {
            return;
        }

        // 鍙璇诲彇锛屼笉鐮村潖缁撴瀯
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
                    this.triggerBreakGroup(bg); // 鎶涚粰杞﹁締鐨勫彧璇荤姸鎬佹満澶勭悊
                }
            }
            int wheelIdx = container.wheelId[idx];
            wheels.deflateWheel(wheelIdx);
        }
    }

    private void solveNormalBeams(float dt, float invDt) {
        for (int i = 0; i < normalBeams.count; i++) {
            if (normalBeams.broken[i]) continue;

            int n1 = normalBeams.node1[i];
            int n2 = normalBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = normalBeams.restLength[i];
            float activeSpring = normalBeams.spring[i];
            float springForce = normalBeams.spring[i] * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = normalBeams.damp[i];
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > normalBeams.strength[i]) {
                breakBeamAt(normalBeams, i);
                continue;
            }

            if (absTotalForce > normalBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                float overForce = absTotalForce - normalBeams.deform[i];
                float deformAmount = ((overForce * overForce) / (normalBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) normalBeams.restLength[i] += deformAmount;
                else normalBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveSupportBeams(float dt, float invDt) {
        for (int i = 0; i < supportBeams.count; i++) {
            if (supportBeams.broken[i]) continue;

            int n1 = supportBeams.node1[i];
            int n2 = supportBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = supportBeams.restLength[i];

            // 鏀拺姊侊細鍙姉鍘嬶紝涓嶆姉鎷夛紙鎷変几鏃惰烦杩囷級
            if (dist > restL) continue;

            float activeSpring = supportBeams.spring[i];
            float springForce = activeSpring * (dist - restL);  // dist <= restL锛屽姏涓鸿礋锛堝帇鍔涳級

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = supportBeams.damp[i];
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > supportBeams.strength[i]) {
                breakBeamAt(supportBeams, i);
                continue;
            }

            if (absTotalForce > supportBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                float overForce = absTotalForce - supportBeams.deform[i];
                float deformAmount = ((overForce * overForce) / (supportBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) supportBeams.restLength[i] += deformAmount;
                else supportBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveBoundedBeams(float dt, float invDt) {
        for (int i = 0; i < boundedBeams.count; i++) {
            if (boundedBeams.broken[i]) continue;

            int n1 = boundedBeams.node1[i];
            int n2 = boundedBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            float restL = boundedBeams.restLength[i];
            float activeSpring = boundedBeams.spring[i];
            float springForce = activeSpring * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            // ----- 澶嶆潅闃诲凹锛堜粎闄愮晫姊佹嫢鏈夛級-----
            float activeDamp = boundedBeams.damp[i];
            float split = boundedBeams.dampVelocitySplit[i];
            boolean isRebound = relVel > 0;
            boolean isFast = Math.abs(relVel) > split;
            if (isRebound) {
                activeDamp = isFast ? boundedBeams.dampReboundFast[i] : boundedBeams.dampRebound[i];
            } else {
                if (isFast) activeDamp = boundedBeams.dampFast[i];
                // else 淇濇寔 activeDamp 涓嶅彉锛堝師閫昏緫锛歩sFast? dampFast : activeDamp锛?
            }

            // ----- 闄愪綅閫昏緫 -----
            float shortBoundary, longBoundary;

            // 鐭竟鐣岋細濡傛灉鎸囧畾浜?Range锛堢粷瀵圭背锛夛紝灏辩敤 restL 鍑忓幓瀹冿紱鍚﹀垯鐢ㄦ瘮渚嬬洿鎺ヤ箻銆?
            if (boundedBeams.shortBoundRange[i] >= 0) {
                shortBoundary = restL - boundedBeams.shortBoundRange[i];
            } else {
                shortBoundary = restL * (1.0f - boundedBeams.shortBound[i]);
            }

            // 闀胯竟鐣岋細濡傛灉鎸囧畾浜?Range锛堢粷瀵圭背锛夛紝灏辩敤 restL 鍔犱笂瀹冿紱鍚﹀垯鐢ㄦ瘮渚嬬洿鎺ヤ箻銆?
            if (boundedBeams.longBoundRange[i] >= 0) {
                longBoundary = restL + boundedBeams.longBoundRange[i];
            } else {
                longBoundary = restL * (1.0f + boundedBeams.longBound[i]);
            }

            float limitSpring = boundedBeams.limitSpring[i];

            if (dist < shortBoundary) {
                springForce += limitSpring * (dist - shortBoundary);
                activeDamp = boundedBeams.limitDamp[i];
            } else if (dist > longBoundary) {
                springForce += limitSpring * (dist - longBoundary);
                activeDamp = boundedBeams.limitDamp[i];
            }

            float totalForce = springForce + (relVel * activeDamp);
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > boundedBeams.strength[i]) {
                breakBeamAt(boundedBeams, i);
                continue;
            }

            if (absTotalForce > boundedBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                float overForce = absTotalForce - boundedBeams.deform[i];
                float deformAmount = ((overForce * overForce) / (boundedBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) boundedBeams.restLength[i] += deformAmount;
                else boundedBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveLBeams(float dt, float invDt) {
        for (int i = 0; i < lBeams.count; i++) {
            if (lBeams.broken[i]) continue;

            int n1 = lBeams.node1[i]; // 绔偣 1 (渚嬪 hInCur)
            int n2 = lBeams.node2[i]; // 绔偣 2 (渚嬪 tOutCur)
            int n3 = lBeams.node3[i]; // 鍏变韩鎷愮偣 3 (渚嬪 tInCur)

            // 1. 璇诲彇涓夌偣瀹炴椂鍧愭爣
            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];

            // 2. 璁＄畻涓夎竟鍚戦噺涓庨暱搴?
            // 鑷?1-3
            double dx13 = x1 - x3, dy13 = y1 - y3, dz13 = z1 - z3;
            double l1Sq = dx13*dx13 + dy13*dy13 + dz13*dz13;

            // 鑷?2-3
            double dx23 = x2 - x3, dy23 = y2 - y3, dz23 = z2 - z3;
            double l2Sq = dx23*dx23 + dy23*dy23 + dz23*dz23;

            // 瀵硅绾?1-2
            double dx12 = x2 - x1, dy12 = y2 - y1, dz12 = z2 - z1;
            double distSq = dx12*dx12 + dy12*dy12 + dz12*dz12;

            // 闃插尽鎬ф嫤鎴瀬灏忚窛绂伙紝闃叉 NaN 浼犳煋
            if (l1Sq < KINDA_SMALL_NUMBER || l2Sq < KINDA_SMALL_NUMBER || distSq < KINDA_SMALL_NUMBER) continue;

            double l1 = Math.sqrt(l1Sq);
            double l2 = Math.sqrt(l2Sq);
            double dist = Math.sqrt(distSq);

            double invL1 = 1.0 / l1;
            double invL2 = 1.0 / l2;
            double invDist = 1.0 / dist;

            // 3. 璁＄畻鍔ㄦ€佺洰鏍囧瑙掔嚎闀垮害 D_target
            double cosTheta0 = lBeams.restCosTheta[i];
            double targetDistSq = l1Sq + l2Sq - 2.0 * l1 * l2 * cosTheta0;
            if (targetDistSq < KINDA_SMALL_NUMBER) continue;
            double targetDist = Math.sqrt(targetDistSq);
            double invTargetDist = 1.0 / targetDist;

            // 4. 璁＄畻閾惧紡姹傚鏀惧ぇ鍥犲瓙
            double g1 = (l1 - l2 * cosTheta0) * invTargetDist;
            double g2 = (l2 - l1 * cosTheta0) * invTargetDist;

            // 5. 璁＄畻鐪熷疄鐨勭浉瀵归樆灏奸€熺巼 (褰诲簳娑堢伃鍨傜洿鍘嬬缉鎶栧姩)
            double vx1 = nodes.velX[n1], vy1 = nodes.velY[n1], vz1 = nodes.velZ[n1];
            double vx2 = nodes.velX[n2], vy2 = nodes.velY[n2], vz2 = nodes.velZ[n2];
            double vx3 = nodes.velX[n3], vy3 = nodes.velY[n3], vz3 = nodes.velZ[n3];

            // 鑷?1-3 鐨勪几缂╅€熺巼
            double v13x = vx1 - vx3, v13y = vy1 - vy3, v13z = vz1 - vz3;
            double l1Dot = (v13x*dx13 + v13y*dy13 + v13z*dz13) * invL1;

            // 鑷?2-3 鐨勪几缂╅€熺巼
            double v23x = vx2 - vx3, v23y = vy2 - vy3, v23z = vz2 - vz3;
            double l2Dot = (v23x*dx23 + v23y*dy23 + v23z*dz23) * invL2;

            // 鐩爣瀵硅绾块暱搴﹂殢澶栨寕鑷傚舰鍙樹骇鐢熺殑鐞嗚鏀剁缉閫熺巼
            double targetDistDot = g1 * l1Dot + g2 * l2Dot;

            // 鐗╃悊瀵硅绾跨殑瀹為檯鎺ヨ繎閫熺巼
            double v12x = vx2 - vx1, v12y = vy2 - vy1, v12z = vz2 - vz1;
            double distDot = (v12x*dx12 + v12y*dy12 + v12z*dz12) * invDist;

            // 鐪熸鐨勫脊鎬х浉瀵归€熺巼 = 瀹為檯閫熺巼 - 鐞嗚閫熺巼
            double dampVel = distDot - targetDistDot;

            // 6. 鏍囬噺鍚堝姏璁＄畻
            double activeSpring = lBeams.spring[i];
            double springForce = activeSpring * (dist - targetDist);
            double dampForce = lBeams.damp[i] * dampVel;
            double totalForce = springForce + dampForce;

            double absTotalForce = Math.abs(totalForce);
            if (absTotalForce > lBeams.strength[i]) {
                breakBeamAt(lBeams, i);
                continue;
            }

            // 濉戞€у舰鍙橀€昏緫鍏滃簳锛氶€氳繃寰皟甯搁┗瑙掑害鍚告敹鍐插嚮
            if (absTotalForce > lBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                double overForce = absTotalForce - lBeams.deform[i];
                double deformAmount = ((overForce * overForce) / (lBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                double sign = Math.signum(dist - targetDist);
                lBeams.restCosTheta[i] -= sign * deformAmount * invL1 * invL2;
                // 闄愬埗浣欏鸡鑼冨洿闃叉宕╁潖
                if (lBeams.restCosTheta[i] > 1.0f) lBeams.restCosTheta[i] = 1.0f;
                if (lBeams.restCosTheta[i] < -1.0f) lBeams.restCosTheta[i] = -1.0f;
            }

            // 7. 馃殌 涓夌偣姊害鍔涘垎閰?
            // 鍚勮竟鐨勫崟浣嶅悜閲?
            double u13x = dx13 * invL1,   u13y = dy13 * invL1,   u13z = dz13 * invL1;
            double u23x = dx23 * invL2,   u23y = dy23 * invL2,   u23z = dz23 * invL2;
            double u12x = dx12 * invDist, u12y = dy12 * invDist, u12z = dz12 * invDist;

            // 鏂藉姞缁?绔偣 1 鐨勫姏 (娉ㄦ剰鏄?+ g1)
            double f1x = totalForce * (u12x + g1 * u13x);
            double f1y = totalForce * (u12y + g1 * u13y);
            double f1z = totalForce * (u12z + g1 * u13z);

            // 鏂藉姞缁?绔偣 2 鐨勫姏 (娉ㄦ剰鏄?+ g2)
            double f2x = totalForce * (-u12x + g2 * u23x);
            double f2y = totalForce * (-u12y + g2 * u23y);
            double f2z = totalForce * (-u12z + g2 * u23z);

            // 鏂藉姞缁?鎷愮偣 3 鐨勫弽浣滅敤鍔?(娉ㄦ剰鏄叏閮ㄥ彇璐燂紒瀹岀編鎶垫秷 f1 鍜?f2 闄勫姞鐨勯澶栧垎閲?
            double f3x = totalForce * (-g1 * u13x - g2 * u23x);
            double f3y = totalForce * (-g1 * u13y - g2 * u23y);
            double f3z = totalForce * (-g1 * u13z - g2 * u23z);

            nodes.forceX[n1] += f1x; nodes.forceY[n1] += f1y; nodes.forceZ[n1] += f1z;
            nodes.forceX[n2] += f2x; nodes.forceY[n2] += f2y; nodes.forceZ[n2] += f2z;
            nodes.forceX[n3] += f3x; nodes.forceY[n3] += f3y; nodes.forceZ[n3] += f3z;
        }
    }

    private void solveAnisotropicBeams(float dt, float invDt)  {
        for (int i = 0; i < anisotropicBeams.count; i++) {
            if (anisotropicBeams.broken[i]) continue;

            int n1 = anisotropicBeams.node1[i];
            int n2 = anisotropicBeams.node2[i];

            float dx = nodes.posX[n2] - nodes.posX[n1];
            float dy = nodes.posY[n2] - nodes.posY[n1];
            float dz = nodes.posZ[n2] - nodes.posZ[n1];
            float distSq = dx*dx + dy*dy + dz*dz;
            if (distSq < KINDA_SMALL_NUMBER) continue;
            float dist = (float) Math.sqrt(distSq);
            float invDist = 1.0f / dist;

            // 鍒濆鐢熸垚鍘熼暱 (Spawned Length)
            float restL = anisotropicBeams.restLength[i];

            // 榛樿杈撳嚭鍩虹鍒氬害鍜岄樆灏?(閫傜敤浜?鍘嬬缉鍖?dist <= restL)
            float activeSpring = anisotropicBeams.spring[i];
            float activeDamp   = anisotropicBeams.damp[i];

            // 馃殌 浠呭湪 Expansion (鎷変几鍖?dist > restL) 瑙﹀彂楂樼骇閫昏緫
            if (dist > restL) {
                float expSpring = anisotropicBeams.springExpansion[i];
                float expDamp   = anisotropicBeams.dampExpansion[i];
                float tZoneRatio = anisotropicBeams.transitionZone[i];

                if (tZoneRatio > KINDA_SMALL_NUMBER) {
                    // 1. 绠楀嚭缁濆杩囨浮鍖洪暱搴?(姣斾緥 脳 鍘熼暱)
                    float absoluteTZone = tZoneRatio * restL;
                    // 2. 绠楀嚭褰撳墠鎷変几閲?
                    float stretch = dist - restL;

                    if (stretch >= absoluteTZone) {
                        // 褰诲簳瓒婅繃杩囨浮鍖猴紝瀹屽叏浣跨敤 Expansion 灞炴€?
                        activeSpring = expSpring;
                        activeDamp   = expDamp;
                    } else {
                        // 澶勪簬杩囨浮鍖烘枩鍧″唴閮紝杩涜绾挎€ф彃鍊?(Lerp)
                        float factor = stretch / absoluteTZone;
                        activeSpring += (expSpring - activeSpring) * factor;
                        activeDamp   += (expDamp   - activeDamp)   * factor;
                    }
                } else {
                    // 榛樿鎯呭喌 (transitionZone == 0)锛岀灛闂磋秺鍙?
                    activeSpring = expSpring;
                    activeDamp   = expDamp;
                }
            }

            // 璁＄畻寮圭哀鍔?
            float springForce = activeSpring * (dist - restL);

            // 璁＄畻闃诲凹鍔?
            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            // 鏂鍒ゅ畾
            if (absTotalForce > anisotropicBeams.strength[i]) {
                breakBeamAt(anisotropicBeams, i);
                continue;
            }

            // 濉戞€у舰鍙?
            if (absTotalForce > anisotropicBeams.deform[i] && activeSpring > KINDA_SMALL_NUMBER) {
                float overForce = absTotalForce - anisotropicBeams.deform[i];
                float deformAmount = ((overForce * overForce) / (anisotropicBeams.deform[i] * activeSpring)) * PhysicsWorld.METAL_PLASTIC_FLOW_RATE * dt;
                if (dist > restL) anisotropicBeams.restLength[i] += deformAmount;
                else anisotropicBeams.restLength[i] = Math.max(KINDA_SMALL_NUMBER, restL - deformAmount);
            }

            // 鏂藉姞鍔?
            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveTorsionBars(float dt, float invDt) {
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

            // 濡傛灉鏈塏aN锛岀洿鎺ュ垽瀹氫负鏂
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

            // 馃毃馃毃馃毃 娉ㄦ剰姊害绗﹀彿
            double g1_factor = b2_mag / c1_sq;
            double g4_factor = -b2_mag / c2_sq;

            double g1x = g1_factor * c1x, g1y = g1_factor * c1y, g1z = g1_factor * c1z;
            double g4x = g4_factor * c2x, g4y = g4_factor * c2y, g4z = g4_factor * c2z;

            double b1_dot_b2_div_sq = (b1x*b2x + b1y*b2y + b1z*b2z) / b2_sq;
            double b3_dot_b2_div_sq = (b3x*b2x + b3y*b2y + b3z*b2z) / b2_sq;

            // 娉ㄦ剰杩欓噷鐨勭鍙?
            double g2x = -g1x * b1_dot_b2_div_sq + g4x * b3_dot_b2_div_sq - g1x;
            double g2y = -g1y * b1_dot_b2_div_sq + g4y * b3_dot_b2_div_sq - g1y;
            double g2z = -g1z * b1_dot_b2_div_sq + g4z * b3_dot_b2_div_sq - g1z;

            double g3x = -g1x - g2x - g4x;
            double g3y = -g1y - g2y - g4y;
            double g3z = -g1z - g2z - g4z;

            // 骞夸箟璐ㄩ噺
            double g1_sq_val = g1x*g1x + g1y*g1y + g1z*g1z;
            double g2_sq_val = g2x*g2x + g2y*g2y + g2z*g2z;
            double g3_sq_val = g3x*g3x + g3y*g3y + g3z*g3z;
            double g4_sq_val = g4x*g4x + g4y*g4y + g4z*g4z;

            // 鍥犱负涔嬪墠妫€鏌ヨ繃mass锛屾墍浠nvGenMass鍩烘湰涓嶄細鏄疦aN
            double invGenMass = (g1_sq_val / nodes.mass[n1]) + (g2_sq_val / nodes.mass[n2]) +
                    (g3_sq_val / nodes.mass[n3]) + (g4_sq_val / nodes.mass[n4]);

            double genMass = 1.0 / invGenMass;

            double maxSafeSpring = genMass * invDt * invDt;
            double maxSafeDamp = genMass * invDt;

            double activeSpring = Math.min(torsionbars.spring[i], maxSafeSpring);
            double activeDamp = Math.min(torsionbars.damp[i], maxSafeDamp);

            // 馃挜 鏈€缁堝彈鍔涜緭鍑?
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

    private void solveSlideNodes(float dt, float invDt) {
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

    public void solveInternalForces(float dt){
        float invDt = 1.0f / dt;

        // 鍒濆鍖栫墿鐞嗙姸鎬?
        for (int i = 0; i < nodes.count; i++) {
            nodes.forceX[i] = 0.0f;
            nodes.forceY[i] = 0.0f;
            nodes.forceZ[i] = 0.0f;

            nodes.prevPosX[i] = nodes.posX[i];
            nodes.prevPosY[i] = nodes.posY[i];
            nodes.prevPosZ[i] = nodes.posZ[i];
        }

        // ==========================================
        // 馃洝锔?杞儙姘斿帇璁＄畻 (Pressure Wheels)
        // ==========================================
        solveTirePressure();

        // ==========================================
        // 馃洝锔?姊佽绠?(Beams)
        // ==========================================

        // ========== 1. 澶勭悊鏅€氭锛圢ORMAL锛?==========
        solveNormalBeams(dt, invDt);

        // ========== 2. 澶勭悊鏀拺姊侊紙SUPPORT锛屼粎鍘嬬缉锛?==========
        solveSupportBeams(dt, invDt);

        // ========== 3. 澶勭悊闄愮晫姊侊紙BOUNDED锛屽惈澶嶆潅闃诲凹鍜岄檺浣嶏級 ==========
        solveBoundedBeams(dt, invDt);

        // ========== 4. LBeams ==========
        solveLBeams(dt, invDt);

        // ========== 5. 鍚勫悜寮傛€ф璁＄畻 (Anisotropic Beams) ==========
        solveAnisotropicBeams(dt, invDt);

        // ==========================================
        // 馃洝锔?鎵潌璁＄畻 (Torsionbars)
        // ==========================================
        solveTorsionBars(dt, invDt);

        // ==========================================
        // 馃洝锔?璁＄畻婊戝潡 (slidenodes)
        // ==========================================
        solveSlideNodes(dt, invDt);

        // ==========================================
        // 馃洝锔?绉垎閫熷害鍜屼綅缃紙棰勬祴锛夈€傛暣涓猼ick鍙湁杩欎竴娆＄Н鍒?
        // ==========================================
        for (int i = 0; i < nodes.count; i++) {

            if (nodes.mass[i] < PhysicsWorld.KINDA_SMALL_NUMBER) continue;
            // 鍔犻噸鍔?
            nodes.forceY[i] += PhysicsWorld.GRAVITY * nodes.mass[i];

            float invMass = 1.0f / nodes.mass[i];
            nodes.velX[i] += (nodes.forceX[i] * invMass) * dt;
            nodes.velY[i] += (nodes.forceY[i] * invMass) * dt;
            nodes.velZ[i] += (nodes.forceZ[i] * invMass) * dt;

            // 绠楀嚭褰撳墠鑺傜偣鐨勯€熷害澶у皬
            float speedSq = nodes.velX[i]*nodes.velX[i] + nodes.velY[i]*nodes.velY[i] + nodes.velZ[i]*nodes.velZ[i];

            // 鏂藉姞涓€涓珮閫熸椂澧為暱鏋佸揩鐨勯瓟娉曢樆鍔涳紝闃叉鑺傜偣鐐搁
            final float K_V4 = 1.2e-7f;   // 闃查鍑虹郴鏁帮紝鏍规嵁鏈€楂樻湡鏈涢€熷害璋?
            float v4 = speedSq * speedSq;
            float factor = 1.0f / (1.0f + K_V4 * v4 * dt);
            nodes.velX[i] *= factor;
            nodes.velY[i] *= factor;
            nodes.velZ[i] *= factor;

            // 娓呮礂鏋佺鐨?NaN (搴斿闄や互0绛夋瀬绔紓甯?
            if (Float.isNaN(nodes.velX[i]) || Float.isNaN(nodes.velY[i]) || Float.isNaN(nodes.velZ[i])) {
                nodes.velX[i] = 0.0f; nodes.velY[i] = 0.0f; nodes.velZ[i] = 0.0f;
            }

            // 灞€閮ㄩ娴嬪潗鏍?(鐩存帴瑕嗙洊 posX锛屽洜涓?prevPos 宸茬粡瀛樺ソ浜?
            nodes.posX[i] += nodes.velX[i] * dt;
            nodes.posY[i] += nodes.velY[i] * dt;
            nodes.posZ[i] += nodes.velZ[i] * dt;
        }
    }

    /**
     * 瀹介樁娈垫壂鎻忥細姣忛殧 N 涓瓙姝ヨ皟鐢ㄤ竴娆★紝棰勬祴鏈潵鐨勪綅绉伙紝鏀堕泦鍙兘纰版挒鐨勫瀛?
     * @param sap 鍏ㄥ眬 SAP 鍔犻€熺粨鏋?
     * @param manager 鎴戜滑鐨勭鎾炶皟搴︿腑蹇?
     * @param dtPredict 棰勬祴鏃堕棿 (渚嬪 10 涓瓙姝ョ殑鏃堕棿鎬诲拰)
     */
    public void generateCollisionCandidates(DynamicAxisSweep sap, SoftBodyCollisionManager manager, double dtPredict) {
        double eX = entityX, eY = entityY, eZ = entityZ;

        // 閾佺毊鐨勭墿鐞嗗帤搴﹀父鏁帮細2鍘樼背銆備笉鍐嶆壙鎷呴槻绌块€忕殑浠诲姟锛屽彧璐熻矗闃叉诞鐐硅宸紒
        double BASE_MARGIN = 0.01;

        for (int i = 0; i < triangles.count; i++) {
            if (!triangles.collision[i]) continue;

            int nA = triangles.node1[i];
            int nB = triangles.node2[i];
            int nC = triangles.node3[i];

            double ax = eX + nodes.posX[nA], ay = eY + nodes.posY[nA], az = eZ + nodes.posZ[nA];
            double bx = eX + nodes.posX[nB], by = eY + nodes.posY[nB], bz = eZ + nodes.posZ[nB];
            double cx = eX + nodes.posX[nC], cy = eY + nodes.posY[nC], cz = eZ + nodes.posZ[nC];

            // 1. 绠楀嚭褰撳墠甯ф渶绱у噾鐨勫熀鍑?AABB
            double minX = Math.min(ax, Math.min(bx, cx)) - BASE_MARGIN;
            double maxX = Math.max(ax, Math.max(bx, cx)) + BASE_MARGIN;
            double minY = Math.min(ay, Math.min(by, cy)) - BASE_MARGIN;
            double maxY = Math.max(ay, Math.max(by, cy)) + BASE_MARGIN;
            double minZ = Math.min(az, Math.min(bz, cz)) - BASE_MARGIN;
            double maxZ = Math.max(az, Math.max(bz, cz)) + BASE_MARGIN;

            // 2. 绠楃墿鐞嗭細璁＄畻杩欎釜涓夎褰㈢殑骞冲潎閫熷害
            double triVx = (nodes.velX[nA] + nodes.velX[nB] + nodes.velX[nC]) * 0.3333333333;
            double triVy = (nodes.velY[nA] + nodes.velY[nB] + nodes.velY[nC]) * 0.3333333333;
            double triVz = (nodes.velZ[nA] + nodes.velZ[nB] + nodes.velZ[nC]) * 0.3333333333;

            // 3. 绠楅鍒や綅绉诲悜閲忥細閫熷害 * 棰勫垽鏃堕棿
            double dx = triVx * dtPredict;
            double dy = triVy * dtPredict;
            double dz = triVz * dtPredict;

            // 4. 瀹氬悜鎷変几锛氬彧鍦ㄨ繍鍔ㄦ柟鍚戜笂寤朵几鍖呭洿鐩掞紒(Swept AABB)
            if (dx > 0) maxX += dx; else minX += dx;
            if (dy > 0) maxY += dy; else minY += dy;
            if (dz > 0) maxZ += dz; else minZ += dz;

            sweepResultBuffer.clear();
            sap.queryNodesInAABB(minX, minY, minZ, maxX, maxY, maxZ, sweepResultBuffer);

            for (int k = 0; k < sweepResultBuffer.count; k++) {
                SoftBodyVehicle hitVeh = sweepResultBuffer.vehicles[k];
                int hitNodeId = sweepResultBuffer.nodeIds[k];

                // 鎺掗櫎涓嶉渶瑕佽绠楄嚜纰版挒鐨勭偣锛屼互鍙婁笁瑙掑舰鑷繁鐨勯《鐐?
                if (hitVeh == this && !nodes.selfCollision[hitNodeId]) continue;
                if (hitVeh == this && (hitNodeId == nA || hitNodeId == nB || hitNodeId == nC)) continue;

                if (hitVeh == this) {
                    int triPartId = triangles.partId[i];
                    if (triPartId >= 0 && triPartId < matrixPartStride) {
                        // 鍙杩欎釜琚挒鐨勮妭鐐癸紝浠庡睘浜庤繖涓笁瑙掑舰鎵€鍦ㄧ殑 Part锛岀洿鎺ユ棤瑙嗭紒
                        if (nodeInPartMatrix[hitNodeId * matrixPartStride + triPartId]) {
                            continue;
                        }
                    }
                }

                // 鎵惧埌瀚岀枒浜轰簡锛佷笉鐢ㄧ畻鐗╃悊锛岀洿鎺ヤ氦缁欒皟搴︿腑蹇冿紒
                manager.addContact(hitVeh, hitNodeId, this, nA, nB, nC);
            }
        }
    }

    public void applyPositionAndVelocityDeltaUnSafe(int nodeId, float dPx, float dPy, float dPz,
                                                  float  dVx, float dVy, float dVz) {
        nodes.posX[nodeId] += dPx;
        nodes.posY[nodeId] += dPy;
        nodes.posZ[nodeId] += dPz;
        nodes.velX[nodeId] += dVx;
        nodes.velY[nodeId] += dVy;
        nodes.velZ[nodeId] += dVz;
    }

    public void solveEnvironmentCollisions(VoxelSnapshot snapshot, float dt) {
        for (int i = 0; i < nodes.count; i++) {
            if (!nodes.collision[i]) continue;

            double worldX = entityX + nodes.posX[i];
            double worldY = entityY + nodes.posY[i];
            double worldZ = entityZ + nodes.posZ[i];

            if (worldY < 320 && worldY > -70 && snapshot.isSolid(worldX, worldY, worldZ)) {

                // 鍥炲埌涓婁竴姝ョ殑瀹夊叏鍧愭爣锛?
                float oldLocalX = nodes.prevPosX[i];
                float oldLocalY = nodes.prevPosY[i];
                float oldLocalZ = nodes.prevPosZ[i];

                double oldWorldX = entityX + oldLocalX;
                double oldWorldY = entityY + oldLocalY;
                double oldWorldZ = entityZ + oldLocalZ;

                boolean hitX = snapshot.isSolid(worldX, oldWorldY, oldWorldZ);
                boolean hitY = snapshot.isSolid(oldWorldX, worldY, oldWorldZ);
                boolean hitZ = snapshot.isSolid(oldWorldX, oldWorldY, worldZ);

                if (hitY || hitX || hitZ) {
                    double invDt = 1.0 / dt;

                    // TODO: 浠庣紦瀛樹腑璇诲彇鍔ㄦ€佺殑鎽╂摝绯绘暟鍜屽洖寮圭郴鏁帮紵浣嗘槸鍥炲脊绯绘暟鍗充究鏄?涔熷緢寮癸紝鍥犱负beam鏈夊脊鎬с€?
                    double reboundCoef = PhysicsWorld.BLOCK_REBOUND;
                    double blockFriction = PhysicsWorld.BLOCK_FRICTION;

                    // 鎻愬彇甯搁┗閲嶅姏鍦ㄤ竴灏忔鏃堕棿鍐呭悜涓嬫柦鍔犵殑鍐查噺澶у皬鏍囬噺
                    double gravityImpulse = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY) * dt;

                    // 1. 浠呭綋璇ヨ酱鍙戠敓纰版挒鏃讹紝鎵嶈鍏ヤ綅缃慨姝ｉ噺锛屽埄鐢ㄤ笁鍏冭繍绠楃閬垮厤璺宠浆寮€閿€
                    double pushX = hitX ? Math.abs(nodes.posX[i] - oldLocalX) : 0.0;
                    double pushY = hitY ? Math.abs(nodes.posY[i] - oldLocalY) : 0.0;
                    double pushZ = hitZ ? Math.abs(nodes.posZ[i] - oldLocalZ) : 0.0;

                    // 鐪熷疄鐨勭珛浣撳悎鎴愭硶鍚戞尋鍘嬭窛绂?(娆у嚑閲屽緱闀垮害)
                    double totalNormalPush = Math.sqrt(pushX*pushX + pushY*pushY + pushZ*pushZ);

                    // 2. 璁＄畻缁熶竴鐨勭瓑鏁堢珛浣撹浇鑽峰姏 Fn (鐗涢】)
                    double equivalentLoadN = (nodes.mass[i] * totalNormalPush) * (invDt * invDt);
                    double minGravityLoad = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY);
                    if (equivalentLoadN < minGravityLoad) equivalentLoadN = minGravityLoad;

                    // 鎻愬彇鍩虹鎽╂摝
                    double mu_s = nodes.friction[i] * blockFriction;
                    double mu_k = nodes.slidingFriction[i] * blockFriction;

                    // 3. 閽堝杞儙鑺傜偣璁＄畻缁熶竴鐨勯珮绾ц浇鑽疯“鍑忎笌 Stribeck 涔樺瓙
                    int wIdx = nodes.wheelId[i];
                    if (0 <= wIdx && wIdx < wheels.count) {
                        double staticBase  = wheels.frictionCoef[wIdx];
                        double slidingBase = wheels.slidingFrictionCoef[wIdx];
                        double noLoad      = wheels.noLoadCoef[wIdx];
                        double fullLoad    = wheels.fullLoadCoef[wIdx];
                        double slope       = wheels.loadSensitivitySlope[wIdx];
                        double treadCoef   = wheels.treadCoef[wIdx];

                        // 杞借嵎琛板噺 (姝ゆ椂浣跨敤瀹岀編鍏煎澶氶潰鐨?equivalentLoadN)
                        double loadFactor = noLoad - (slope * equivalentLoadN);
                        if (loadFactor < fullLoad) loadFactor = fullLoad;

                        // 璁＄畻 3D 鐪熷疄鐨勫垏鍚戞粦绉婚€熺巼 (鍓旈櫎宸茬鎾炶酱鍚戠殑娉曞悜鍒嗛噺)
                        // 鍝潰澧欐挒浜嗭紝閭ｄ釜杞寸殑閫熷害灏辨槸娉曞悜鍒嗛噺锛屽墿浣欒酱鐨勯€熷害骞虫柟鍜屽嵆涓哄垏鍚戞粦绉婚€熺巼
                        double vx = nodes.velX[i], vy = nodes.velY[i], vz = nodes.velZ[i];
                        double tVelSq = (hitX ? 0 : vx*vx) + (hitY ? 0 : vy*vy) + (hitZ ? 0 : vz*vz);
                        double vtLen = Math.sqrt(tVelSq);

                        // Stribeck 鏇茬嚎杩囨浮
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
                        //淇锛氱湡瀹炵殑娉曞悜鎬诲啿閲?= 鍙嶅脊鍔ㄩ噺宸€?+ 閲嶅姏鍘嬭揩鍐查噺
                        // J = m * |v| * (1 + e) + m * |g| * dt
                        double jn = nodes.mass[i] * Math.abs(nodes.velY[i]) * (1.0 + reboundCoef) + gravityImpulse;

                        // 鎵ц鍨傜洿鍙嶅脊涓庝綅缃洖閫€
                        nodes.velY[i] *= -reboundCoef;
                        nodes.posY[i] = oldLocalY;

                        // --- 瑙ｈ€﹀眰 1锛氶€熷害灞傜湡瀹炲啿閲忚“鍑?---
                        double vx = nodes.velX[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vx*vx + vz*vz);
                        double jtReq = vtLen * nodes.mass[i]; // 瀹屽叏鍒跺仠鎵€闇€鐨勭湡瀹炲姩閲?

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0f; nodes.velZ[i] = 0.0f; // 闈欐懇鎿﹀挰姝?                            } else {
                                // 鍔ㄦ懇鎿︽亽瀹氶樆鍔涘墺绂?
                                double frictionImpulse = mu_k * jn;
                                velKeepRatio = Math.max(0.0, 1.0 - (frictionImpulse / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        // --- 瑙ｈ€﹀眰 2锛歅BD 鍑犱綍浣嶇疆灞傝爼鍔ㄧ害鏉?---
                        // 浣嶇疆鐨勬媺鎵瀬闄愬悓鏍风敱褰撳墠鎺ヨЕ闈㈢殑搴撲粦涓婇檺鍐冲畾
                        double creepX = nodes.posX[i] - oldLocalX, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepX*creepX + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            // 鍑犱綍浣嶇疆瀹屽叏閿佹 (涓嶅彂鐢熻爼鍔?
                            nodes.posX[i] = oldLocalX; nodes.posZ[i] = oldLocalZ;
                        } else {
                            // 鍙戠敓寰鎵撴粦锛屼綅缃爼鍔ㄦ寜閫熷害灞傜浉鍚岀殑姣斾緥鎴栧姩鎽╂摝姣斾緥琛板噺
                            // 閲囩敤 velKeepRatio 鑳藉淇濊瘉浣嶇疆婊戝姩涓庨€熷害婊戝姩鍦ㄨ瑙変笂缁濆鍚屾
                            nodes.posX[i] = (float) (oldLocalX + creepX * velKeepRatio);
                            nodes.posZ[i] = (float) (oldLocalZ + creepZ * velKeepRatio);
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
                                nodes.velY[i] = 0.0f; nodes.velZ[i] = 0.0f;
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
                            nodes.posY[i] = (float) (oldLocalY + creepY * velKeepRatio);
                            nodes.posZ[i] = (float) (oldLocalZ + creepZ * velKeepRatio);
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
                                nodes.velX[i] = 0.0f; nodes.velY[i] = 0.0f;
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
                            nodes.posX[i] = (float) (oldLocalX + creepX * velKeepRatio);
                            nodes.posY[i] = (float) (oldLocalY + creepY * velKeepRatio);
                        }
                    }
                }
            }
        }
    }
}
