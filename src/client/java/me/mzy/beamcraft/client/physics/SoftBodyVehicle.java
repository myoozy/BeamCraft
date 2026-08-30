package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.World;

import java.util.Arrays;
import java.util.List;

public class SoftBodyVehicle {
    public static final float KINDA_SMALL_NUMBER = PhysicsWorld.KINDA_SMALL_NUMBER;
    public static final int MAX_AABB_SIZE = PhysicsWorld.MAX_AABB_SIZE;

    public final PhysicsVehicleEntity parentEntity;
    public final float[] localCOM = new float[3];
    public int vehicleId = -1;
    public int globalNodeOffset = 0;

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
    private int maxTrackedPartId = -1;
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    public boolean[] nodeInPartMatrix;
    public int matrixPartStride;

    public java.util.Map<String, List<BeamPointer>> breakGroupMap = new java.util.HashMap<>();
    private final java.util.Set<String> triggeredBreakGroups = new java.util.HashSet<>();

    private final SweepResultBuffer sweepResultBuffer = new SweepResultBuffer();

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
    public void addNode(PhysicsSpecs.NodeSpec spec) {
        nodes.addNode(spec);


        // ==========================================
        // ==========================================
        if (spec.partId() > maxTrackedPartId) {
            maxTrackedPartId = spec.partId();
            ensurePartCapacity(maxTrackedPartId);
        }
    }

    /**
     * Create physical beam constraint between two existing nodes
     */
    public void addBeam(PhysicsSpecs.BeamSpec spec) {
        String name1 = spec.name1();
        String name2 = spec.name2();
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

            if (spec.type() == BeamContainer.BEAM_SUPPORT) {

                beamIdx = supportBeams.addBeam(spec, n1, n2, (float) dist);
                container = supportBeams;

            } else if (spec.type() == BeamContainer.BEAM_BOUNDED) {

                beamIdx = boundedBeams.addBeam(spec, n1, n2, (float) dist);
                container = boundedBeams;

            } else if (spec.type() == BeamContainer.BEAM_LBEAM && nodes.nameToIndex.containsKey(spec.name3())) {

                int n3 = nodes.nameToIndex.get(spec.name3());
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
                beamIdx = lBeams.addBeam(spec, n1, n2, n3,
                        (float) node12Dist, (float) node13Dist, (float) node23Dist);
                container = lBeams;

            } else if (spec.type() == BeamContainer.BEAM_ANISOTROPIC) {

                beamIdx = anisotropicBeams.addBeam(spec, n1, n2, (float) dist);
                container = anisotropicBeams;

            } else {

                beamIdx = normalBeams.addBeam(spec, n1, n2, (float) dist);
                container = normalBeams;

            }

            if (spec.breakGroups() != null && !spec.breakGroups().isEmpty()) {
                for (String bg : spec.breakGroups()) {
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
    public void addTriangle(PhysicsSpecs.TriangleSpec spec) {
        String name1 = spec.name1();
        String name2 = spec.name2();
        String name3 = spec.name3();
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2) && nodes.nameToIndex.containsKey(name3)) {
            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            int n3 = nodes.nameToIndex.get(name3);

            triangles.addTriangle(spec, n1, n2, n3);
        }
    }

    /**
     * Spawn torsion bar joint with four control nodes and physical properties
     */
    public void addTorsionBar(PhysicsSpecs.TorsionBarSpec spec) {
        String name1 = spec.name1();
        String name2 = spec.name2();
        String name3 = spec.name3();
        String name4 = spec.name4();

        // Verify all node exists
        if (nodes.nameToIndex.containsKey(name1) && nodes.nameToIndex.containsKey(name2) &&
                nodes.nameToIndex.containsKey(name3) && nodes.nameToIndex.containsKey(name4)) {

            int n1 = nodes.nameToIndex.get(name1);
            int n2 = nodes.nameToIndex.get(name2);
            int n3 = nodes.nameToIndex.get(name3);
            int n4 = nodes.nameToIndex.get(name4);

            torsionbars.addTorsionBar(spec, n1, n2, n3, n4, nodes);
        }
    }

    /**
     * Calculate closest rail segment and add sliding node constraint
     */
    public void addSlideNode(PhysicsSpecs.SlideNodeSpec spec) {
        String node = spec.node();
        String[] railNodes = spec.railNodes();
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
            slidenodes.addSlideNode(spec, nId, bestA, bestB, (float) bestRestDist);
        }
    }

    public void finalizePhysicsSetup() {
        flexbodies.compileGroupsCSR(nodes);
        triangles.buildBreakIndices();

        matrixPartStride = maxTrackedPartId + 1;
        nodeInPartMatrix = new boolean[nodes.count * matrixPartStride];
        for (int i = 0; i < nodes.count; i++) {
            int originalPart = nodes.partId[i];
            if (originalPart >= 0 && originalPart < matrixPartStride) {
                nodeInPartMatrix[i * matrixPartStride + originalPart] = true;
            }
        }
        for (int i = 0; i < triangles.count; i++) {
            int part = triangles.partId[i];
            if (part >= 0 && part < matrixPartStride) {
                nodeInPartMatrix[triangles.node1[i] * matrixPartStride + part] = true;
                nodeInPartMatrix[triangles.node2[i] * matrixPartStride + part] = true;
                nodeInPartMatrix[triangles.node3[i] * matrixPartStride + part] = true;
            }
        }

        limitConstraintStiffnessAndDamping(PhysicsWorld.invPhysicsDT, 0.90f);
    }

    private void limitConstraintStiffnessAndDamping(float invDt, float safetyFraction) {
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(nodes.count, nodes.mass, invDt, safetyFraction);

        int[] normalIds = addAxialConstraints(limiter, normalBeams, normalBeams.spring, normalBeams.damp, invDt);
        int[] supportIds = addAxialConstraints(limiter, supportBeams, supportBeams.spring, supportBeams.damp, invDt);

        int[] boundedIds = new int[boundedBeams.count];
        for (int i = 0; i < boundedBeams.count; i++) {
            float stiffness = Utility.positive(boundedBeams.spring[i])
                    + Utility.positive(boundedBeams.limitSpring[i]);
            float damping = Utility.maxPositive(
                    boundedBeams.damp[i], boundedBeams.limitDamp[i], boundedBeams.dampFast[i],
                    boundedBeams.dampRebound[i], boundedBeams.dampReboundFast[i]);
            boundedIds[i] = addAxialConstraint(limiter, boundedBeams, i, stiffness,
                    Math.min(damping, axialDampingCeiling(boundedBeams, i, invDt)));
        }

        int[] lBeamIds = new int[lBeams.count];
        float[] lBeamDampingCeilings = new float[lBeams.count];
        for (int i = 0; i < lBeams.count; i++) {
            lBeamIds[i] = addLBeamConstraint(limiter, i, invDt, lBeamDampingCeilings);
        }

        int[] anisotropicIds = new int[anisotropicBeams.count];
        for (int i = 0; i < anisotropicBeams.count; i++) {
            float stiffness = Math.max(Utility.positive(anisotropicBeams.spring[i]),
                    Utility.positive(anisotropicBeams.springExpansion[i]));
            float damping = Math.max(Utility.positive(anisotropicBeams.damp[i]),
                    Utility.positive(anisotropicBeams.dampExpansion[i]));
            anisotropicIds[i] = addAxialConstraint(limiter, anisotropicBeams, i, stiffness,
                    Math.min(damping, axialDampingCeiling(anisotropicBeams, i, invDt)));
        }

        limiter.solve();

        allocateAxialBeams(normalBeams, normalIds, limiter, invDt);
        allocateAxialBeams(supportBeams, supportIds, limiter, invDt);
        for (int i = 0; i < boundedBeams.count; i++) {
            float dampingCeiling = axialDampingCeiling(boundedBeams, i, invDt);
            float stiffness = Utility.positive(boundedBeams.spring[i])
                    + Utility.positive(boundedBeams.limitSpring[i]);
            float damping = Utility.maxPositive(
                    boundedBeams.damp[i], boundedBeams.limitDamp[i], boundedBeams.dampFast[i],
                    boundedBeams.dampRebound[i], boundedBeams.dampReboundFast[i]);
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    boundedIds[i], stiffness, damping, dampingCeiling);
            Utility.FloatPair springs = Utility.capPairToSum(
                    boundedBeams.spring[i], boundedBeams.limitSpring[i], ceilings.maxStiffness());
            boundedBeams.spring[i] = springs.first();
            boundedBeams.limitSpring[i] = springs.second();
            boundedBeams.damp[i] = Math.min(boundedBeams.damp[i], ceilings.maxDamping());
            boundedBeams.limitDamp[i] = Math.min(boundedBeams.limitDamp[i], ceilings.maxDamping());
            boundedBeams.dampFast[i] = Math.min(boundedBeams.dampFast[i], ceilings.maxDamping());
            boundedBeams.dampRebound[i] = Math.min(boundedBeams.dampRebound[i], ceilings.maxDamping());
            boundedBeams.dampReboundFast[i] = Math.min(
                    boundedBeams.dampReboundFast[i], ceilings.maxDamping());
        }
        for (int i = 0; i < lBeams.count; i++) {
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    lBeamIds[i], lBeams.spring[i], lBeams.damp[i], lBeamDampingCeilings[i]);
            lBeams.spring[i] = Math.min(lBeams.spring[i], ceilings.maxStiffness());
            lBeams.damp[i] = Math.min(lBeams.damp[i], ceilings.maxDamping());
        }
        for (int i = 0; i < anisotropicBeams.count; i++) {
            float dampingCeiling = axialDampingCeiling(anisotropicBeams, i, invDt);
            float stiffness = Math.max(Utility.positive(anisotropicBeams.spring[i]),
                    Utility.positive(anisotropicBeams.springExpansion[i]));
            float damping = Math.max(Utility.positive(anisotropicBeams.damp[i]),
                    Utility.positive(anisotropicBeams.dampExpansion[i]));
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    anisotropicIds[i], stiffness, damping, dampingCeiling);
            anisotropicBeams.spring[i] = Math.min(anisotropicBeams.spring[i], ceilings.maxStiffness());
            anisotropicBeams.springExpansion[i] = Math.min(
                    anisotropicBeams.springExpansion[i], ceilings.maxStiffness());
            anisotropicBeams.damp[i] = Math.min(anisotropicBeams.damp[i], ceilings.maxDamping());
            anisotropicBeams.dampExpansion[i] = Math.min(
                    anisotropicBeams.dampExpansion[i], ceilings.maxDamping());
        }
    }

    private int[] addAxialConstraints(DirectionalStabilityLimiter limiter, BeamContainer beams,
                                      float[] stiffness, float[] damping, float invDt) {
        int[] ids = new int[beams.count];
        for (int i = 0; i < beams.count; i++) {
            ids[i] = addAxialConstraint(limiter, beams, i, stiffness[i],
                    Math.min(damping[i], axialDampingCeiling(beams, i, invDt)));
        }
        return ids;
    }

    private int addAxialConstraint(DirectionalStabilityLimiter limiter, BeamContainer beams, int i,
                                   float stiffness, float damping) {
        int n1 = beams.node1[i];
        int n2 = beams.node2[i];
        double dx = nodes.posX[n2] - nodes.posX[n1];
        double dy = nodes.posY[n2] - nodes.posY[n1];
        double dz = nodes.posZ[n2] - nodes.posZ[n1];
        double length = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (length < KINDA_SMALL_NUMBER) {
            return limiter.addIsotropicTwoNode(n1, n2, stiffness, damping);
        }
        return limiter.addTwoNode(n1, n2, dx / length, dy / length, dz / length, stiffness, damping);
    }

    private int addLBeamConstraint(DirectionalStabilityLimiter limiter, int i, float invDt,
                                   float[] dampingCeilings) {
        int n1 = lBeams.node1[i];
        int n2 = lBeams.node2[i];
        int n3 = lBeams.node3[i];

        double dx13 = nodes.posX[n1] - nodes.posX[n3];
        double dy13 = nodes.posY[n1] - nodes.posY[n3];
        double dz13 = nodes.posZ[n1] - nodes.posZ[n3];
        double dx23 = nodes.posX[n2] - nodes.posX[n3];
        double dy23 = nodes.posY[n2] - nodes.posY[n3];
        double dz23 = nodes.posZ[n2] - nodes.posZ[n3];
        double dx12 = nodes.posX[n2] - nodes.posX[n1];
        double dy12 = nodes.posY[n2] - nodes.posY[n1];
        double dz12 = nodes.posZ[n2] - nodes.posZ[n1];

        double l1 = Math.sqrt(dx13 * dx13 + dy13 * dy13 + dz13 * dz13);
        double l2 = Math.sqrt(dx23 * dx23 + dy23 * dy23 + dz23 * dz23);
        double dist = Math.sqrt(dx12 * dx12 + dy12 * dy12 + dz12 * dz12);
        double targetDistSq = l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * lBeams.restCosTheta[i];
        if (l1 < KINDA_SMALL_NUMBER || l2 < KINDA_SMALL_NUMBER || dist < KINDA_SMALL_NUMBER
                || targetDistSq < KINDA_SMALL_NUMBER
                || nodes.mass[n1] <= KINDA_SMALL_NUMBER
                || nodes.mass[n2] <= KINDA_SMALL_NUMBER
                || nodes.mass[n3] <= KINDA_SMALL_NUMBER) {
            dampingCeilings[i] = 0.0f;
            return limiter.addThreeNode(n1, 0, 0, 0, n2, 0, 0, 0, n3, 0, 0, 0, 0, 0);
        }

        double targetDist = Math.sqrt(targetDistSq);
        double g1 = (l1 - l2 * lBeams.restCosTheta[i]) / targetDist;
        double g2 = (l2 - l1 * lBeams.restCosTheta[i]) / targetDist;
        double u13x = dx13 / l1, u13y = dy13 / l1, u13z = dz13 / l1;
        double u23x = dx23 / l2, u23y = dy23 / l2, u23z = dz23 / l2;
        double u12x = dx12 / dist, u12y = dy12 / dist, u12z = dz12 / dist;

        double g1x = u12x + g1 * u13x, g1y = u12y + g1 * u13y, g1z = u12z + g1 * u13z;
        double g2x = -u12x + g2 * u23x, g2y = -u12y + g2 * u23y, g2z = -u12z + g2 * u23z;
        double g3x = -g1 * u13x - g2 * u23x;
        double g3y = -g1 * u13y - g2 * u23y;
        double g3z = -g1 * u13z - g2 * u23z;
        double inverseGeneralizedMass = (g1x * g1x + g1y * g1y + g1z * g1z) / nodes.mass[n1]
                + (g2x * g2x + g2y * g2y + g2z * g2z) / nodes.mass[n2]
                + (g3x * g3x + g3y * g3y + g3z * g3z) / nodes.mass[n3];
        dampingCeilings[i] = inverseGeneralizedMass > KINDA_SMALL_NUMBER
                ? (float) ((1.0 / inverseGeneralizedMass) * invDt * 0.95)
                : 0.0f;

        return limiter.addThreeNode(
                n1, g1x, g1y, g1z,
                n2, g2x, g2y, g2z,
                n3, g3x, g3y, g3z,
                lBeams.spring[i], Math.min(lBeams.damp[i], dampingCeilings[i]));
    }

    private void allocateAxialBeams(BeamContainer beams, int[] ids,
                                    DirectionalStabilityLimiter limiter, float invDt) {
        for (int i = 0; i < beams.count; i++) {
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    ids[i], beams.spring[i], beams.damp[i], axialDampingCeiling(beams, i, invDt));
            beams.spring[i] = Math.min(beams.spring[i], ceilings.maxStiffness());
            beams.damp[i] = Math.min(beams.damp[i], ceilings.maxDamping());
        }
    }

    private float axialDampingCeiling(BeamContainer beams, int i, float invDt) {
        float m1 = nodes.mass[beams.node1[i]];
        float m2 = nodes.mass[beams.node2[i]];
        if (m1 <= KINDA_SMALL_NUMBER || m2 <= KINDA_SMALL_NUMBER) return 0.0f;
        return Utility.reducedMass(m1, m2) * invDt * 0.95f;
    }

    /**
     * Reset velocity and deformation state.
     */
    public void reset() {
        triggeredBreakGroups.clear();
        nodes.reset();
        normalBeams.reset();
        supportBeams.reset();
        boundedBeams.reset();
        lBeams.reset();
        anisotropicBeams.reset();
        triangles.reset();
        torsionbars.reset();
        wheels.reset();
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
        anisotropicBeams.clear();
        triangles.clear();
        torsionbars.clear();
        slidenodes.clear();
        wheels.clear();
        flexbodies.clear();
        breakGroupMap.clear();
        triggeredBreakGroups.clear();
        maxTrackedPartId = -1;

        System.out.println("Vehicle data cleared and reset");
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

            double px = entityX + nodes.posX[i];
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
     *
     */
    private void solveTirePressure() {
        for (int w = 0; w < wheels.count; w++) {
            if (wheels.isDeflated[w]) continue;

            int start = wheels.tireTriangleIdxStart[w];
            int end = wheels.tireTriangleIdxEnd[w];
            if (start >= end || start == 0) continue;

            double currentVolume = wheels.prevVolume[w];
            if (currentVolume < KINDA_SMALL_NUMBER) continue;

            double p0_Pa = wheels.pressurePSI[w] * 6894.76;
            double absP0_Pa = p0_Pa + 101325.0;
            double currentAbsPressurePa = absP0_Pa * (wheels.initialVolume[w] / currentVolume);
            double pressureDiffPa = currentAbsPressurePa - 101325.0;

            double forceMultiplier = (pressureDiffPa * wheels.normalSign[w]) / 6.0;

            double nextVolumeSum = 0.0;

            for (int i = start; i <= end; i++) {
                int nA = triangles.node1[i];
                int nB = triangles.node2[i];
                int nC = triangles.node3[i];

                double ax = nodes.posX[nA], ay = nodes.posY[nA], az = nodes.posZ[nA];
                double bx = nodes.posX[nB], by = nodes.posY[nB], bz = nodes.posZ[nB];
                double cx = nodes.posX[nC], cy = nodes.posY[nC], cz = nodes.posZ[nC];

                double abx = bx - ax, aby = by - ay, abz = bz - az;
                double acx = cx - ax, acy = cy - ay, acz = cz - az;

                double nx = aby * acz - abz * acy;
                double ny = abz * acx - abx * acz;
                double nz = abx * acy - aby * acx;

                double fx = nx * forceMultiplier;
                double fy = ny * forceMultiplier;
                double fz = nz * forceMultiplier;

                nodes.forceX[nA] += fx; nodes.forceY[nA] += fy; nodes.forceZ[nA] += fz;
                nodes.forceX[nB] += fx; nodes.forceY[nB] += fy; nodes.forceZ[nB] += fz;
                nodes.forceX[nC] += fx; nodes.forceY[nC] += fy; nodes.forceZ[nC] += fz;

                nextVolumeSum += (ax * nx + ay * ny + az * nz);
            }

            wheels.normalSign[w] = (nextVolumeSum < 0.0) ? -1.0f : 1.0f;
            wheels.prevVolume[w] = (float) Math.abs(nextVolumeSum / 6.0);
        }
    }

    public void triggerBreakGroup(String groupName) {
        if (!triggeredBreakGroups.add(groupName)) {
            return;
        }

        triangles.breakByGroup(groupName);

        List<BeamPointer> linkedBeams = breakGroupMap.get(groupName);
        if (linkedBeams == null) return;

        for (BeamPointer ptr : linkedBeams) {
            breakBeamAt(ptr.container, ptr.index);
        }
    }

    private void breakBeamAt(BeamContainer container, int idx) {
        if (container.broken[idx]) return;
        container.broken[idx] = true;
        if (!container.disableTriangleBreaking[idx]) {
            triangles.breakByEdge(container.node1[idx], container.node2[idx]);
        }
        if (container.breakGroupType[idx] == 0) {
            if (container.assignedBreakGroups != null && container.assignedBreakGroups[idx] != null) {
                for (String bg : container.assignedBreakGroups[idx]) {
                    this.triggerBreakGroup(bg);
                }
            }
            int wheelIdx = container.wheelId[idx];
            wheels.deflateWheel(wheelIdx);
        }
    }

    /**
     * Returns the permanent rest-state change for an elastic load beyond its yield load.
     * The exponential relaxation is timestep independent and never overshoots the
     * current hardening surface for a single solve.
     */
    private static float plasticDeltaFromExcess(float excessLoad, float yieldLoad, float maxYieldLoad,
                                                float stiffness, float relaxation) {
        float invStiffness = 1.0f / stiffness;
        float remainingHardening = Math.max(0.0f, maxYieldLoad - yieldLoad);
        float targetDelta;
        if (remainingHardening > 0.0f) {
            if (excessLoad <= 2.0f * remainingHardening) {
                targetDelta = 0.5f * excessLoad * invStiffness;
            } else {
                targetDelta = (excessLoad - remainingHardening) * invStiffness;
            }
        } else {
            targetDelta = excessLoad * invStiffness;
        }
        return relaxation * targetDelta;
    }

    private static double plasticDeltaFromExcess(double excessLoad, double yieldLoad, double maxYieldLoad,
                                                 double stiffness, double relaxation) {
        double invStiffness = 1.0 / stiffness;
        double remainingHardening = Math.max(0.0, maxYieldLoad - yieldLoad);
        double targetDelta;
        if (remainingHardening > 0.0) {
            if (excessLoad <= 2.0 * remainingHardening) {
                targetDelta = 0.5 * excessLoad * invStiffness;
            } else {
                targetDelta = (excessLoad - remainingHardening) * invStiffness;
            }
        } else {
            targetDelta = excessLoad * invStiffness;
        }
        return relaxation * targetDelta;
    }

    private static float hardenDeform(float currentDeform, float maxDeform, float stiffness, float permanentDelta) {
        float remainingHardening = Math.max(0.0f, maxDeform - currentDeform);
        return currentDeform + Math.min(remainingHardening, stiffness * permanentDelta);
    }

    private static double hardenDeform(double currentDeform, double maxDeform,
                                       double stiffness, double permanentDelta) {
        double remainingHardening = Math.max(0.0, maxDeform - currentDeform);
        return currentDeform + Math.min(remainingHardening, stiffness * permanentDelta);
    }

    private void solveNormalBeams(float plasticRelaxation, float invDt) {
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

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > normalBeams.deform[i]) {
                float maxDeform = normalBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - normalBeams.deform[i],
                        normalBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    normalBeams.restLength[i] = newRestLength;
                    normalBeams.deform[i] = hardenDeform(normalBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveSupportBeams(float plasticRelaxation, float invDt) {
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

            if (dist > restL) continue;

            float activeSpring = supportBeams.spring[i];
            float springForce = activeSpring * (dist - restL);

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

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > supportBeams.deform[i]) {
                float maxDeform = supportBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - supportBeams.deform[i],
                        supportBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    supportBeams.restLength[i] = newRestLength;
                    supportBeams.deform[i] = hardenDeform(supportBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveBoundedBeams(float plasticRelaxation, float invDt) {
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
            float plasticStiffness = activeSpring;

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;

            float activeDamp = boundedBeams.damp[i];
            float split = boundedBeams.dampVelocitySplit[i];
            boolean isRebound = relVel > 0;
            boolean isFast = Math.abs(relVel) > split;
            if (isRebound) {
                activeDamp = isFast ? boundedBeams.dampReboundFast[i] : boundedBeams.dampRebound[i];
            } else {
                if (isFast) activeDamp = boundedBeams.dampFast[i];
            }

            float shortBoundary, longBoundary;

            if (boundedBeams.shortBoundRange[i] >= 0) {
                shortBoundary = restL - boundedBeams.shortBoundRange[i];
            } else {
                shortBoundary = restL * (1.0f - boundedBeams.shortBound[i]);
            }

            if (boundedBeams.longBoundRange[i] >= 0) {
                longBoundary = restL + boundedBeams.longBoundRange[i];
            } else {
                longBoundary = restL * (1.0f + boundedBeams.longBound[i]);
            }

            float limitSpring = boundedBeams.limitSpring[i];

            if (dist < shortBoundary) {
                springForce += limitSpring * (dist - shortBoundary);
                plasticStiffness += limitSpring * (boundedBeams.shortBoundRange[i] >= 0
                        ? 1.0f : 1.0f - boundedBeams.shortBound[i]);
                activeDamp = boundedBeams.limitDamp[i];
            } else if (dist > longBoundary) {
                springForce += limitSpring * (dist - longBoundary);
                plasticStiffness += limitSpring * (boundedBeams.longBoundRange[i] >= 0
                        ? 1.0f : 1.0f + boundedBeams.longBound[i]);
                activeDamp = boundedBeams.limitDamp[i];
            }

            float totalForce = springForce + (relVel * activeDamp);
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > boundedBeams.strength[i]) {
                breakBeamAt(boundedBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (plasticStiffness > KINDA_SMALL_NUMBER && absSpringForce > boundedBeams.deform[i]) {
                float maxDeform = boundedBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - boundedBeams.deform[i],
                        boundedBeams.deform[i], maxDeform,
                        plasticStiffness, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    boundedBeams.restLength[i] = newRestLength;
                    boundedBeams.deform[i] = hardenDeform(boundedBeams.deform[i], maxDeform,
                            plasticStiffness, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveLBeams(float plasticRelaxation, float invDt) {
        for (int i = 0; i < lBeams.count; i++) {
            if (lBeams.broken[i]) continue;

            int n1 = lBeams.node1[i];
            int n2 = lBeams.node2[i];
            int n3 = lBeams.node3[i];

            double x1 = nodes.posX[n1], y1 = nodes.posY[n1], z1 = nodes.posZ[n1];
            double x2 = nodes.posX[n2], y2 = nodes.posY[n2], z2 = nodes.posZ[n2];
            double x3 = nodes.posX[n3], y3 = nodes.posY[n3], z3 = nodes.posZ[n3];

            double dx13 = x1 - x3, dy13 = y1 - y3, dz13 = z1 - z3;
            double l1Sq = dx13*dx13 + dy13*dy13 + dz13*dz13;

            double dx23 = x2 - x3, dy23 = y2 - y3, dz23 = z2 - z3;
            double l2Sq = dx23*dx23 + dy23*dy23 + dz23*dz23;

            double dx12 = x2 - x1, dy12 = y2 - y1, dz12 = z2 - z1;
            double distSq = dx12*dx12 + dy12*dy12 + dz12*dz12;

            if (l1Sq < KINDA_SMALL_NUMBER || l2Sq < KINDA_SMALL_NUMBER || distSq < KINDA_SMALL_NUMBER) continue;

            double l1 = Math.sqrt(l1Sq);
            double l2 = Math.sqrt(l2Sq);
            double dist = Math.sqrt(distSq);

            double invL1 = 1.0 / l1;
            double invL2 = 1.0 / l2;
            double invDist = 1.0 / dist;

            double cosTheta0 = lBeams.restCosTheta[i];
            double targetDistSq = l1Sq + l2Sq - 2.0 * l1 * l2 * cosTheta0;
            if (targetDistSq < KINDA_SMALL_NUMBER) continue;
            double targetDist = Math.sqrt(targetDistSq);
            double invTargetDist = 1.0 / targetDist;

            double g1 = (l1 - l2 * cosTheta0) * invTargetDist;
            double g2 = (l2 - l1 * cosTheta0) * invTargetDist;

            double vx1 = nodes.velX[n1], vy1 = nodes.velY[n1], vz1 = nodes.velZ[n1];
            double vx2 = nodes.velX[n2], vy2 = nodes.velY[n2], vz2 = nodes.velZ[n2];
            double vx3 = nodes.velX[n3], vy3 = nodes.velY[n3], vz3 = nodes.velZ[n3];

            double v13x = vx1 - vx3, v13y = vy1 - vy3, v13z = vz1 - vz3;
            double l1Dot = (v13x*dx13 + v13y*dy13 + v13z*dz13) * invL1;

            double v23x = vx2 - vx3, v23y = vy2 - vy3, v23z = vz2 - vz3;
            double l2Dot = (v23x*dx23 + v23y*dy23 + v23z*dz23) * invL2;

            double targetDistDot = g1 * l1Dot + g2 * l2Dot;

            double v12x = vx2 - vx1, v12y = vy2 - vy1, v12z = vz2 - vz1;
            double distDot = (v12x*dx12 + v12y*dy12 + v12z*dz12) * invDist;

            double dampVel = distDot - targetDistDot;

            double activeSpring = lBeams.spring[i];
            double springForce = activeSpring * (dist - targetDist);
            double dampForce = lBeams.damp[i] * dampVel;
            double totalForce = springForce + dampForce;

            double absTotalForce = Math.abs(totalForce);
            if (absTotalForce > lBeams.strength[i]) {
                breakBeamAt(lBeams, i);
                continue;
            }

            double absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > lBeams.deform[i]) {
                float maxDeform = lBeams.maxDeform[i];
                double permanentDelta = plasticDeltaFromExcess(absSpringForce - lBeams.deform[i],
                        lBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0) {
                    double newTargetDist = targetDist + Math.signum(springForce) * permanentDelta;
                    newTargetDist = Math.clamp(newTargetDist, Math.abs(l1 - l2), l1 + l2);
                    double appliedDelta = Math.abs(newTargetDist - targetDist);
                    double newRestCos = (l1Sq + l2Sq - newTargetDist * newTargetDist) / (2.0 * l1 * l2);
                    lBeams.restCosTheta[i] = (float) Math.clamp(newRestCos, -1.0, 1.0);
                    lBeams.deform[i] = (float) hardenDeform(lBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            double u13x = dx13 * invL1,   u13y = dy13 * invL1,   u13z = dz13 * invL1;
            double u23x = dx23 * invL2,   u23y = dy23 * invL2,   u23z = dz23 * invL2;
            double u12x = dx12 * invDist, u12y = dy12 * invDist, u12z = dz12 * invDist;

            double f1x = totalForce * (u12x + g1 * u13x);
            double f1y = totalForce * (u12y + g1 * u13y);
            double f1z = totalForce * (u12z + g1 * u13z);

            double f2x = totalForce * (-u12x + g2 * u23x);
            double f2y = totalForce * (-u12y + g2 * u23y);
            double f2z = totalForce * (-u12z + g2 * u23z);

            double f3x = totalForce * (-g1 * u13x - g2 * u23x);
            double f3y = totalForce * (-g1 * u13y - g2 * u23y);
            double f3z = totalForce * (-g1 * u13z - g2 * u23z);

            nodes.forceX[n1] += f1x; nodes.forceY[n1] += f1y; nodes.forceZ[n1] += f1z;
            nodes.forceX[n2] += f2x; nodes.forceY[n2] += f2y; nodes.forceZ[n2] += f2z;
            nodes.forceX[n3] += f3x; nodes.forceY[n3] += f3y; nodes.forceZ[n3] += f3z;
        }
    }

    private void solveAnisotropicBeams(float plasticRelaxation, float invDt)  {
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

            float restL = anisotropicBeams.restLength[i];

            float activeSpring = anisotropicBeams.spring[i];
            float activeDamp   = anisotropicBeams.damp[i];

            if (dist > restL) {
                float expSpring = anisotropicBeams.springExpansion[i];
                float expDamp   = anisotropicBeams.dampExpansion[i];
                float tZoneRatio = anisotropicBeams.transitionZone[i];

                if (tZoneRatio > KINDA_SMALL_NUMBER) {
                    float absoluteTZone = tZoneRatio * restL;
                    float stretch = dist - restL;

                    if (stretch >= absoluteTZone) {
                        activeSpring = expSpring;
                        activeDamp   = expDamp;
                    } else {
                        float factor = stretch / absoluteTZone;
                        activeSpring += (expSpring - activeSpring) * factor;
                        activeDamp   += (expDamp   - activeDamp)   * factor;
                    }
                } else {
                    activeSpring = expSpring;
                    activeDamp   = expDamp;
                }
            }

            float springForce = activeSpring * (dist - restL);

            float vx = nodes.velX[n2] - nodes.velX[n1];
            float vy = nodes.velY[n2] - nodes.velY[n1];
            float vz = nodes.velZ[n2] - nodes.velZ[n1];
            float relVel = (vx*dx + vy*dy + vz*dz) * invDist;
            float dampForce = activeDamp * relVel;

            float totalForce = springForce + dampForce;
            float absTotalForce = Math.abs(totalForce);

            if (absTotalForce > anisotropicBeams.strength[i]) {
                breakBeamAt(anisotropicBeams, i);
                continue;
            }

            float absSpringForce = Math.abs(springForce);
            if (activeSpring > KINDA_SMALL_NUMBER && absSpringForce > anisotropicBeams.deform[i]) {
                float maxDeform = anisotropicBeams.maxDeform[i];
                float permanentDelta = plasticDeltaFromExcess(absSpringForce - anisotropicBeams.deform[i],
                        anisotropicBeams.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0f) {
                    float newRestLength = Math.max(KINDA_SMALL_NUMBER,
                            restL + Math.signum(springForce) * permanentDelta);
                    float appliedDelta = Math.abs(newRestLength - restL);
                    anisotropicBeams.restLength[i] = newRestLength;
                    anisotropicBeams.deform[i] = hardenDeform(anisotropicBeams.deform[i], maxDeform,
                            activeSpring, appliedDelta);
                }
            }

            float fx = totalForce * dx * invDist;
            float fy = totalForce * dy * invDist;
            float fz = totalForce * dz * invDist;

            nodes.forceX[n1] += fx; nodes.forceY[n1] += fy; nodes.forceZ[n1] += fz;
            nodes.forceX[n2] -= fx; nodes.forceY[n2] -= fy; nodes.forceZ[n2] -= fz;
        }
    }

    private void solveTorsionBars(float plasticRelaxation, float invDt) {
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

            double g1_factor = b2_mag / c1_sq;
            double g4_factor = -b2_mag / c2_sq;

            double g1x = g1_factor * c1x, g1y = g1_factor * c1y, g1z = g1_factor * c1z;
            double g4x = g4_factor * c2x, g4y = g4_factor * c2y, g4z = g4_factor * c2z;

            double b1_dot_b2_div_sq = (b1x*b2x + b1y*b2y + b1z*b2z) / b2_sq;
            double b3_dot_b2_div_sq = (b3x*b2x + b3y*b2y + b3z*b2z) / b2_sq;

            double g2x = -g1x * b1_dot_b2_div_sq + g4x * b3_dot_b2_div_sq - g1x;
            double g2y = -g1y * b1_dot_b2_div_sq + g4y * b3_dot_b2_div_sq - g1y;
            double g2z = -g1z * b1_dot_b2_div_sq + g4z * b3_dot_b2_div_sq - g1z;

            double g3x = -g1x - g2x - g4x;
            double g3y = -g1y - g2y - g4y;
            double g3z = -g1z - g2z - g4z;

            double g1_sq_val = g1x*g1x + g1y*g1y + g1z*g1z;
            double g2_sq_val = g2x*g2x + g2y*g2y + g2z*g2z;
            double g3_sq_val = g3x*g3x + g3y*g3y + g3z*g3z;
            double g4_sq_val = g4x*g4x + g4y*g4y + g4z*g4z;

            double invGenMass = (g1_sq_val / nodes.mass[n1]) + (g2_sq_val / nodes.mass[n2]) +
                    (g3_sq_val / nodes.mass[n3]) + (g4_sq_val / nodes.mass[n4]);

            double genMass = 1.0 / invGenMass;

            double maxSafeSpring = genMass * invDt * invDt;
            double maxSafeDamp = genMass * invDt;

            double activeSpring = Math.min(torsionbars.spring[i], maxSafeSpring);
            double activeDamp = Math.min(torsionbars.damp[i], maxSafeDamp);

            double omega = (g1x*nodes.velX[n1] + g1y*nodes.velY[n1] + g1z*nodes.velZ[n1]) +
                    (g2x*nodes.velX[n2] + g2y*nodes.velY[n2] + g2z*nodes.velZ[n2]) +
                    (g3x*nodes.velX[n3] + g3y*nodes.velY[n3] + g3z*nodes.velZ[n3]) +
                    (g4x*nodes.velX[n4] + g4y*nodes.velY[n4] + g4z*nodes.velZ[n4]);

            double elasticTorque = activeSpring * deltaAngle;
            double torque = elasticTorque - (activeDamp * omega);

            double absTorque = Math.abs(torque);
            if (Double.isNaN(torque) || absTorque > torsionbars.strength[i]) {
                torsionbars.broken[i] = true;
                continue;
            }
            double absElasticTorque = Math.abs(elasticTorque);
            if (activeSpring > KINDA_SMALL_NUMBER && absElasticTorque > torsionbars.deform[i]) {
                double maxDeform = torsionbars.strength[i];
                double permanentDelta = plasticDeltaFromExcess(absElasticTorque - torsionbars.deform[i],
                        torsionbars.deform[i], maxDeform,
                        activeSpring, plasticRelaxation);
                if (permanentDelta > 0.0) {
                    torsionbars.restAngle[i] += Math.signum(elasticTorque) * permanentDelta;
                    torsionbars.deform[i] = (float) hardenDeform(torsionbars.deform[i], maxDeform,
                            activeSpring, permanentDelta);
                    while (torsionbars.restAngle[i] > Math.PI) torsionbars.restAngle[i] -= Math.PI * 2;
                    while (torsionbars.restAngle[i] < -Math.PI) torsionbars.restAngle[i] += Math.PI * 2;
                }
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

    public void solveInternalForces(float dt, float plasticRelaxation){
        float invDt = 1.0f / dt;

        for (int i = 0; i < nodes.count; i++) {
            nodes.forceX[i] = 0.0f;
            nodes.forceY[i] = 0.0f;
            nodes.forceZ[i] = 0.0f;

            nodes.prevPosX[i] = nodes.posX[i];
            nodes.prevPosY[i] = nodes.posY[i];
            nodes.prevPosZ[i] = nodes.posZ[i];
        }

        // ==========================================
        // ==========================================
        solveTirePressure();

        // ==========================================
        // ==========================================

        solveNormalBeams(plasticRelaxation, invDt);

        solveSupportBeams(plasticRelaxation, invDt);

        solveBoundedBeams(plasticRelaxation, invDt);

        // ========== 4. LBeams ==========
        solveLBeams(plasticRelaxation, invDt);

        solveAnisotropicBeams(plasticRelaxation, invDt);

        // ==========================================
        // ==========================================
        solveTorsionBars(plasticRelaxation, invDt);

        // ==========================================
        // ==========================================
        solveSlideNodes(dt, invDt);

        // ==========================================
        // ==========================================
        for (int i = 0; i < nodes.count; i++) {

            if (nodes.mass[i] < PhysicsWorld.KINDA_SMALL_NUMBER) continue;
            nodes.forceY[i] += PhysicsWorld.GRAVITY * nodes.mass[i];

            float invMass = 1.0f / nodes.mass[i];
            nodes.velX[i] += (nodes.forceX[i] * invMass) * dt;
            nodes.velY[i] += (nodes.forceY[i] * invMass) * dt;
            nodes.velZ[i] += (nodes.forceZ[i] * invMass) * dt;

            float speedSq = nodes.velX[i]*nodes.velX[i] + nodes.velY[i]*nodes.velY[i] + nodes.velZ[i]*nodes.velZ[i];

            final float K_V4 = 1.2e-7f;
            float v4 = speedSq * speedSq;
            float factor = 1.0f / (1.0f + K_V4 * v4 * dt);
            nodes.velX[i] *= factor;
            nodes.velY[i] *= factor;
            nodes.velZ[i] *= factor;

            if (Float.isNaN(nodes.velX[i]) || Float.isNaN(nodes.velY[i]) || Float.isNaN(nodes.velZ[i])) {
                nodes.velX[i] = 0.0f; nodes.velY[i] = 0.0f; nodes.velZ[i] = 0.0f;
            }

            nodes.posX[i] += nodes.velX[i] * dt;
            nodes.posY[i] += nodes.velY[i] * dt;
            nodes.posZ[i] += nodes.velZ[i] * dt;
        }
    }

    /**
     *
     *
     *
     *
     */
    public void generateCollisionCandidates(DynamicAxisSweep sap, SoftBodyCollisionManager manager, double dtPredict) {
        double eX = entityX, eY = entityY, eZ = entityZ;

        double BASE_MARGIN = 0.01;

        for (int i = 0; i < triangles.count; i++) {
            if (!triangles.collision[i] || triangles.broken[i]) continue;

            int nA = triangles.node1[i];
            int nB = triangles.node2[i];
            int nC = triangles.node3[i];

            double ax = eX + nodes.posX[nA], ay = eY + nodes.posY[nA], az = eZ + nodes.posZ[nA];
            double bx = eX + nodes.posX[nB], by = eY + nodes.posY[nB], bz = eZ + nodes.posZ[nB];
            double cx = eX + nodes.posX[nC], cy = eY + nodes.posY[nC], cz = eZ + nodes.posZ[nC];

            double minX = Math.min(ax, Math.min(bx, cx)) - BASE_MARGIN;
            double maxX = Math.max(ax, Math.max(bx, cx)) + BASE_MARGIN;
            double minY = Math.min(ay, Math.min(by, cy)) - BASE_MARGIN;
            double maxY = Math.max(ay, Math.max(by, cy)) + BASE_MARGIN;
            double minZ = Math.min(az, Math.min(bz, cz)) - BASE_MARGIN;
            double maxZ = Math.max(az, Math.max(bz, cz)) + BASE_MARGIN;

            double triVx = (nodes.velX[nA] + nodes.velX[nB] + nodes.velX[nC]) * 0.3333333333;
            double triVy = (nodes.velY[nA] + nodes.velY[nB] + nodes.velY[nC]) * 0.3333333333;
            double triVz = (nodes.velZ[nA] + nodes.velZ[nB] + nodes.velZ[nC]) * 0.3333333333;

            double dx = triVx * dtPredict;
            double dy = triVy * dtPredict;
            double dz = triVz * dtPredict;

            if (dx > 0) maxX += dx; else minX += dx;
            if (dy > 0) maxY += dy; else minY += dy;
            if (dz > 0) maxZ += dz; else minZ += dz;

            sweepResultBuffer.clear();
            sap.queryNodesInAABB(minX, minY, minZ, maxX, maxY, maxZ, sweepResultBuffer);

            for (int k = 0; k < sweepResultBuffer.count; k++) {
                SoftBodyVehicle hitVeh = sweepResultBuffer.vehicles[k];
                int hitNodeId = sweepResultBuffer.nodeIds[k];

                if (hitVeh == this && !nodes.selfCollision[hitNodeId]) continue;
                if (hitVeh == this && (hitNodeId == nA || hitNodeId == nB || hitNodeId == nC)) continue;

                if (hitVeh == this) {
                    int triPartId = triangles.partId[i];
                    if (triPartId >= 0 && triPartId < matrixPartStride) {
                        if (nodeInPartMatrix[hitNodeId * matrixPartStride + triPartId]) {
                            continue;
                        }
                    }
                }

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

                    double reboundCoef = PhysicsWorld.BLOCK_REBOUND;
                    double blockFriction = PhysicsWorld.BLOCK_FRICTION;

                    double gravityImpulse = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY) * dt;

                    double pushX = hitX ? Math.abs(nodes.posX[i] - oldLocalX) : 0.0;
                    double pushY = hitY ? Math.abs(nodes.posY[i] - oldLocalY) : 0.0;
                    double pushZ = hitZ ? Math.abs(nodes.posZ[i] - oldLocalZ) : 0.0;

                    double totalNormalPush = Math.sqrt(pushX*pushX + pushY*pushY + pushZ*pushZ);

                    double equivalentLoadN = (nodes.mass[i] * totalNormalPush) * (invDt * invDt);
                    double minGravityLoad = nodes.mass[i] * Math.abs(PhysicsWorld.GRAVITY);
                    if (equivalentLoadN < minGravityLoad) equivalentLoadN = minGravityLoad;

                    double mu_s = nodes.friction[i] * blockFriction;
                    double mu_k = nodes.slidingFriction[i] * blockFriction;

                    int wIdx = nodes.wheelId[i];
                    if (0 <= wIdx && wIdx < wheels.count) {
                        double staticBase  = wheels.frictionCoef[wIdx];
                        double slidingBase = wheels.slidingFrictionCoef[wIdx];
                        double noLoad      = wheels.noLoadCoef[wIdx];
                        double fullLoad    = wheels.fullLoadCoef[wIdx];
                        double slope       = wheels.loadSensitivitySlope[wIdx];
                        double treadCoef   = wheels.treadCoef[wIdx];

                        double loadFactor = noLoad - (slope * equivalentLoadN);
                        if (loadFactor < fullLoad) loadFactor = fullLoad;

                        double vx = nodes.velX[i], vy = nodes.velY[i], vz = nodes.velZ[i];
                        double tVelSq = (hitX ? 0 : vx*vx) + (hitY ? 0 : vy*vy) + (hitZ ? 0 : vz*vz);
                        double vtLen = Math.sqrt(tVelSq);

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
                        // J = m * |v| * (1 + e) + m * |g| * dt
                        double jn = nodes.mass[i] * Math.abs(nodes.velY[i]) * (1.0 + reboundCoef) + gravityImpulse;

                        nodes.velY[i] *= -reboundCoef;
                        nodes.posY[i] = oldLocalY;

                        double vx = nodes.velX[i], vz = nodes.velZ[i];
                        double vtLen = Math.sqrt(vx*vx + vz*vz);
                        double jtReq = vtLen * nodes.mass[i];

                        double velKeepRatio = 0.0;
                        if (jtReq > 1e-8) {
                            if (jtReq <= mu_s * jn) {
                                nodes.velX[i] = 0.0f; nodes.velZ[i] = 0.0f;
                            } else {
                                double frictionImpulse = mu_k * jn;
                                velKeepRatio = Math.max(0.0, 1.0 - (frictionImpulse / jtReq));
                                nodes.velX[i] *= velKeepRatio;
                                nodes.velZ[i] *= velKeepRatio;
                            }
                        }

                        double creepX = nodes.posX[i] - oldLocalX, creepZ = nodes.posZ[i] - oldLocalZ;
                        double creepLen = Math.sqrt(creepX*creepX + creepZ*creepZ);
                        double posForceReq = (creepLen * nodes.mass[i]) * (invDt * invDt);

                        if (posForceReq <= mu_s * (jn * invDt)) {
                            nodes.posX[i] = oldLocalX; nodes.posZ[i] = oldLocalZ;
                        } else {
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
