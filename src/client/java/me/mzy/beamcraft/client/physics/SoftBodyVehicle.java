package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.input.DriverInputFilter;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSystem;
import me.mzy.beamcraft.utility.Utility;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.World;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

public class SoftBodyVehicle {
    public static final float KINDA_SMALL_NUMBER = PhysicsWorld.KINDA_SMALL_NUMBER;
    public static final int MAX_AABB_SIZE = PhysicsWorld.MAX_AABB_SIZE;
    /** Numerical safety ceiling; Minecraft blocks and physics distances are treated as metres. */
    public static final float MAX_NODE_SPEED = 343.0f;
    final int brakeInputSignalId;
    final int parkingBrakeInputSignalId;
    public final PhysicsVehicleEntity parentEntity;
    public final float[] localOriginShift = new float[3];
    public int vehicleId = -1;
    public int globalNodeOffset = 0;

    public final NodeContainer nodes = new NodeContainer();
    public final ElectricBus electrics = new ElectricBus();
    public final BeamContainer normalBeams = new BeamContainer();
    public final CouplerContainer couplers = new CouplerContainer();
    public final HydroContainer hydros = new HydroContainer();
    /** Support beams do not honour dampCutoffHz, matching the documented BeamNG beam types. */
    public final BeamContainer supportBeams = new BeamContainer(false);
    public final BoundedBeamContainer boundedBeams = new BoundedBeamContainer();
    public final LBeamContainer lBeams = new LBeamContainer();
    public final AnisotropicBeamContainer anisotropicBeams = new AnisotropicBeamContainer();
    public final TriangleContainer triangles = new TriangleContainer();
    public final TorsionBarContainer torsionbars = new TorsionBarContainer();
    public final TorsionHydroContainer torsionHydros = new TorsionHydroContainer();
    public final SlideNodeContainer slidenodes = new SlideNodeContainer();
    public final WheelContainer wheels = new WheelContainer(this);
    public final PowertrainSystem powertrain = new PowertrainSystem(this);
    /** Standalone BeamNG adaptive damper actuator backend; no drive-mode/electrics dependency. */
    public final AdaptiveDamperActuators adaptiveDampers = new AdaptiveDamperActuators(this);
    public final DriverInputFilter driverInputs = new DriverInputFilter(electrics);
    private final VehicleInternalForceSolver internalForceSolver = new VehicleInternalForceSolver(this);
    public final FlexbodyContainer flexbodies = new FlexbodyContainer();
    public final VehicleCameraData cameras = new VehicleCameraData();
    public final PhysicsRenderTimeline renderTimeline = new PhysicsRenderTimeline();

    // Bounding box cache array for independent part culling
    private int maxTrackedPartId = -1;
    private double[] partMinX = new double[0], partMinY = new double[0], partMinZ = new double[0];
    private double[] partMaxX = new double[0], partMaxY = new double[0], partMaxZ = new double[0];
    private boolean[] partActive = new boolean[0];

    public boolean[] nodeInPartMatrix;
    public int matrixPartStride;

    /** Adaptive damper controllers parsed during assembly; registered by {@link #finalizePhysicsSetup()}. */
    private List<AdaptiveDamperSpec> adaptiveDamperSpecs = List.of();

    public java.util.Map<String, List<BeamPointer>> breakGroupMap = new java.util.HashMap<>();
    private final java.util.Set<String> triggeredBreakGroups = new java.util.HashSet<>();
    private final Set<String> triggeredDeformGroups = ConcurrentHashMap.newKeySet();

    final SweepResultBuffer sweepResultBuffer = new SweepResultBuffer();

    double entityX = 0.0;
    double entityY = 0.0;
    double entityZ = 0.0;

    public SoftBodyVehicle(PhysicsVehicleEntity parentEntity) {
        this.parentEntity = parentEntity;
        electrics.register(ElectricSignals.STEERING_INPUT);
        brakeInputSignalId = electrics.register(ElectricSignals.BRAKE_INPUT);
        parkingBrakeInputSignalId = electrics.register(ElectricSignals.PARKING_BRAKE_INPUT);
        this.flexbodies.vehicleNamespace = parentEntity != null ? parentEntity.getRootPartName() : "test";
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
    public void updateLocalOriginCache() {
        nodes.getMedianPosition(localOriginShift);
        nodes.moveNodes(-localOriginShift[0], -localOriginShift[1], -localOriginShift[2]);
    }

    /*
    Must call updateLocalOriginCache before
     */
    public void updateEntityLocation() {
        this.parentEntity.setVelocity(0, 0, 0);

        double newEntityX = entityX + localOriginShift[0];
        double newEntityY = entityY + localOriginShift[1];
        double newEntityZ = entityZ + localOriginShift[2];
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
        addBeamInternal(spec);
    }

    /** Adds a non-elastic, two-node coupler without changing the beam topology. */
    public boolean addCoupler(PhysicsSpecs.CouplerSpec spec) {
        Integer n1 = nodes.nameToIndex.get(spec.name1());
        Integer n2 = nodes.nameToIndex.get(spec.name2());
        if (n1 == null || n2 == null || n1.equals(n2)) return false;
        couplers.add(spec, n1, n2);
        return true;
    }

    public void addHydro(PhysicsSpecs.HydroSpec spec) {
        BeamPointer beam = addBeamInternal(spec.beam());
        if (beam != null) {
            hydros.addHydro(spec, beam.index, normalBeams, electrics);
        }
    }

    private BeamPointer addBeamInternal(PhysicsSpecs.BeamSpec spec) {
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
            return new BeamPointer(container, beamIdx);
        }
        return null;
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
        addTorsionBarInternal(spec);
    }

    public void addTorsionHydro(PhysicsSpecs.TorsionHydroSpec spec) {
        int torsionBarIndex = addTorsionBarInternal(spec.torsionBar());
        if (torsionBarIndex >= 0) {
            torsionHydros.addTorsionHydro(spec, torsionBarIndex, torsionbars, electrics);
        }
    }

    private int addTorsionBarInternal(PhysicsSpecs.TorsionBarSpec spec) {
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

            return torsionbars.addTorsionBar(spec, n1, n2, n3, n4, nodes);
        }
        return -1;
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

    /**
     * Hands the parsed adaptive damper controllers to the vehicle. They are
     * registered in {@link #finalizePhysicsSetup()}, once every named beam exists.
     */
    public void setAdaptiveDamperSpecs(List<AdaptiveDamperSpec> specs) {
        this.adaptiveDamperSpecs = specs == null ? List.of() : List.copyOf(specs);
    }

    public void finalizePhysicsSetup() {
        powertrain.finalizeSetup();
        flexbodies.compileGroupsCSR(nodes);
        triangles.buildBreakIndices();

        // Actuators address beams by their authored name, so the lookup must be
        // built only once every beam of every selected part exists.
        boundedBeams.rebuildNameIndex();
        adaptiveDampers.registerAll(adaptiveDamperSpecs);

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

        // Every channel is now bounded by the cutoff-aware stability ceiling, so the
        // neutral mode (when authored) can be applied without exceeding the budget.
        adaptiveDampers.applyDefaultModes();
    }

    private void limitConstraintStiffnessAndDamping(float invDt, float safetyFraction) {
        DirectionalStabilityLimiter limiter =
                new DirectionalStabilityLimiter(nodes.count, nodes.mass, invDt, safetyFraction);
        float dt = 1.0f / invDt;

        int[] normalIds = addAxialConstraints(limiter, normalBeams, normalBeams.spring, normalBeams.damp, invDt);
        int[] supportIds = addAxialConstraints(limiter, supportBeams, supportBeams.spring, supportBeams.damp, invDt);

        int[] boundedIds = new int[boundedBeams.count];
        for (int i = 0; i < boundedBeams.count; i++) {
            // A bounded beam transitions from its ordinary coefficient to its
            // limit coefficient; the two springs are not active in parallel.
            float stiffness = Math.max(Utility.positive(boundedBeams.spring[i]),
                    Utility.positive(boundedBeams.limitSpring[i]));
            float damping = Utility.maxPositive(
                    boundedBeams.damp[i], boundedBeams.limitDamp[i],
                    boundedBeams.limitDampRebound[i], boundedBeams.dampFast[i],
                    boundedBeams.dampRebound[i], boundedBeams.dampReboundFast[i]);
            boundedIds[i] = addAxialConstraint(limiter, boundedBeams, i, stiffness,
                    stabilityDamping(boundedBeams, i, damping, invDt, dt));
        }

        int[] lBeamIds = new int[lBeams.count];
        float[] lBeamDampingCeilings = new float[lBeams.count];
        for (int i = 0; i < lBeams.count; i++) {
            lBeamIds[i] = addLBeamConstraint(limiter, i, invDt, dt, lBeamDampingCeilings);
        }

        int[] anisotropicIds = new int[anisotropicBeams.count];
        for (int i = 0; i < anisotropicBeams.count; i++) {
            float stiffness = Math.max(Utility.positive(anisotropicBeams.spring[i]),
                    Utility.positive(anisotropicBeams.springExpansion[i]));
            float damping = Math.max(Utility.positive(anisotropicBeams.damp[i]),
                    Utility.positive(anisotropicBeams.dampExpansion[i]));
            anisotropicIds[i] = addAxialConstraint(limiter, anisotropicBeams, i, stiffness,
                    stabilityDamping(anisotropicBeams, i, damping, invDt, dt));
        }

        limiter.solve();

        allocateAxialBeams(normalBeams, normalIds, limiter, invDt, dt);
        allocateAxialBeams(supportBeams, supportIds, limiter, invDt, dt);
        for (int i = 0; i < boundedBeams.count; i++) {
            float dampingCeiling = axialDampingCeiling(boundedBeams, i, invDt, dt);
            float stiffness = Math.max(Utility.positive(boundedBeams.spring[i]),
                    Utility.positive(boundedBeams.limitSpring[i]));
            float damping = Utility.maxPositive(
                    boundedBeams.damp[i], boundedBeams.limitDamp[i],
                    boundedBeams.limitDampRebound[i], boundedBeams.dampFast[i],
                    boundedBeams.dampRebound[i], boundedBeams.dampReboundFast[i]);
            // The ceiling is the budget this constraint may reach, not the value it was
            // clamped to, so an actuator can raise a coefficient above the authored
            // magnitude (a "hard" damper mode) up to, but never past, this bound.
            boundedBeams.dampStabilityCeiling[i] =
                    limiter.maxDampingCeiling(boundedIds[i], dampingCeiling);
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    boundedIds[i], stiffness, damping, dampingCeiling);
            boundedBeams.spring[i] = Math.min(boundedBeams.spring[i], ceilings.maxStiffness());
            boundedBeams.limitSpring[i] = Math.min(
                    boundedBeams.limitSpring[i], ceilings.maxStiffness());
            boundedBeams.damp[i] = Math.min(boundedBeams.damp[i], ceilings.maxDamping());
            boundedBeams.limitDamp[i] = Math.min(boundedBeams.limitDamp[i], ceilings.maxDamping());
            boundedBeams.limitDampRebound[i] = Math.min(
                    boundedBeams.limitDampRebound[i], ceilings.maxDamping());
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
            float dampingCeiling = axialDampingCeiling(anisotropicBeams, i, invDt, dt);
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

    /**
     * Registers the damping the linear stability budget must see. A beam with an
     * authored {@code dampCutoffHz} only lets the attenuated high-frequency share
     * of its damping reach the fastest mode, so {@code damp * hfGain} is what can
     * actually destabilise the step; the safety budget itself is unchanged.
     */
    private float stabilityDamping(BeamContainer beams, int i, float damping, float invDt, float dt) {
        return Math.min(damping * beams.cutoffHighFrequencyGain(i, dt),
                dampingBudget(beams, i, invDt));
    }

    private int[] addAxialConstraints(DirectionalStabilityLimiter limiter, BeamContainer beams,
                                      float[] stiffness, float[] damping, float invDt) {
        float dt = 1.0f / invDt;
        int[] ids = new int[beams.count];
        for (int i = 0; i < beams.count; i++) {
            ids[i] = addAxialConstraint(limiter, beams, i, stiffness[i],
                    stabilityDamping(beams, i, damping[i], invDt, dt));
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

    private int addLBeamConstraint(DirectionalStabilityLimiter limiter, int i, float invDt, float dt,
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
        float budget = inverseGeneralizedMass > KINDA_SMALL_NUMBER
                ? (float) ((1.0 / inverseGeneralizedMass) * invDt * 0.95)
                : 0.0f;
        float hfGain = lBeams.cutoffHighFrequencyGain(i, dt);
        // The stored coefficient is the authored one; the ceiling it is allowed to
        // reach scales with 1 / hfGain because the filter attenuates it in practice.
        dampingCeilings[i] = budget / hfGain;

        return limiter.addThreeNode(
                n1, g1x, g1y, g1z,
                n2, g2x, g2y, g2z,
                n3, g3x, g3y, g3z,
                lBeams.spring[i], Math.min(lBeams.damp[i] * hfGain, budget));
    }

    private void allocateAxialBeams(BeamContainer beams, int[] ids,
                                    DirectionalStabilityLimiter limiter, float invDt, float dt) {
        for (int i = 0; i < beams.count; i++) {
            DirectionalStabilityLimiter.CoefficientCeilings ceilings = limiter.ceilings(
                    ids[i], beams.spring[i], beams.damp[i], axialDampingCeiling(beams, i, invDt, dt));
            beams.spring[i] = Math.min(beams.spring[i], ceilings.maxStiffness());
            beams.damp[i] = Math.min(beams.damp[i], ceilings.maxDamping());
        }
    }

    /**
     * Unfiltered damping budget the semi-implicit Euler step tolerates for this
     * beam's reduced mass; independent of {@code dampCutoffHz}.
     */
    private float dampingBudget(BeamContainer beams, int i, float invDt) {
        float m1 = nodes.mass[beams.node1[i]];
        float m2 = nodes.mass[beams.node2[i]];
        if (m1 <= KINDA_SMALL_NUMBER || m2 <= KINDA_SMALL_NUMBER) return 0.0f;
        return Utility.reducedMass(m1, m2) * invDt * 0.95f;
    }

    /**
     * Ceiling for the <em>authored</em> damping coefficient. With a cutoff the
     * high-frequency share reaching the fastest mode is only {@code hfGain} of it,
     * so authored suspension damping may be that much larger before the same
     * stability budget is exhausted.
     */
    private float axialDampingCeiling(BeamContainer beams, int i, float invDt, float dt) {
        return dampingBudget(beams, i, invDt) / beams.cutoffHighFrequencyGain(i, dt);
    }

    /**
     * Reset velocity and deformation state.
     */
    public void reset() {
        triggeredBreakGroups.clear();
        triggeredDeformGroups.clear();
        electrics.resetValues();
        nodes.reset();
        normalBeams.reset();
        couplers.reset();
        hydros.reset(normalBeams);
        supportBeams.reset();
        boundedBeams.reset();
        lBeams.reset();
        anisotropicBeams.reset();
        triangles.reset();
        torsionbars.reset();
        torsionHydros.reset(torsionbars);
        wheels.reset();
        powertrain.reset();
        // Matches the BeamNG controller's empty reset(): the selected damper mode
        // stays selected and is re-derived from the authored values.
        adaptiveDampers.reset();
        System.out.println("Vehicle reset.");
    }

    /**
     * Clear all physics container data and reset simulation world
     */
    public void clear() {
        nodes.clear();
        electrics.clear();
        normalBeams.clear();
        couplers.clear();
        hydros.clear();
        supportBeams.clear();
        boundedBeams.clear();
        lBeams.clear();
        anisotropicBeams.clear();
        triangles.clear();
        torsionbars.clear();
        torsionHydros.clear();
        slidenodes.clear();
        wheels.clear();
        powertrain.clear();
        adaptiveDampers.clear();
        adaptiveDamperSpecs = List.of();
        flexbodies.clear();
        cameras.clear();
        renderTimeline.clear();
        breakGroupMap.clear();
        triggeredBreakGroups.clear();
        triggeredDeformGroups.clear();
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

    public void triggerBreakGroup(String groupName) {
        if (!triggeredBreakGroups.add(groupName)) {
            return;
        }

        triangles.breakByGroup(groupName);
        couplers.breakByGroup(groupName);

        List<BeamPointer> linkedBeams = breakGroupMap.get(groupName);
        if (linkedBeams == null) return;

        for (BeamPointer ptr : linkedBeams) {
            breakBeamAt(ptr.container, ptr.index);
        }
    }

    /**
     * Latches a BeamNG deform group until the vehicle is reset. Rendering and
     * effects deliberately do not consume this state yet.
     */
    public void triggerDeformGroup(String groupName) {
        if (groupName != null && !groupName.isEmpty()) {
            triggeredDeformGroups.add(groupName);
        }
    }

    public boolean isDeformGroupTriggered(String groupName) {
        return groupName != null && triggeredDeformGroups.contains(groupName);
    }

    public Set<String> triggeredDeformGroups() {
        return Set.copyOf(triggeredDeformGroups);
    }

    /** Evaluates only beams that declared a finite deformation trigger ratio. */
    void updateDeformGroupTriggers() {
        updateDeformGroupTriggers(normalBeams);
        updateDeformGroupTriggers(supportBeams);
        updateDeformGroupTriggers(boundedBeams);
        updateDeformGroupTriggers(lBeams);
        updateDeformGroupTriggers(anisotropicBeams);
    }

    private void updateDeformGroupTriggers(BeamContainer container) {
        for (int trigger = 0; trigger < container.deformTriggerCount(); trigger++) {
            int beam = container.deformTriggerIndex(trigger);
            if (container.broken[beam] || container.deformGroupTriggered[beam]) continue;

            float restLength = container.effectiveRestLength(beam);
            if (!(restLength > KINDA_SMALL_NUMBER) || !Float.isFinite(restLength)) continue;

            int n1 = container.node1[beam];
            int n2 = container.node2[beam];
            double dx = nodes.posX[n2] - nodes.posX[n1];
            double dy = nodes.posY[n2] - nodes.posY[n1];
            double dz = nodes.posZ[n2] - nodes.posZ[n1];
            double currentLength = Math.sqrt(dx * dx + dy * dy + dz * dz);
            double strain = Math.abs(currentLength / restLength - 1.0);
            if (strain <= container.deformationTriggerRatio[beam]) continue;

            container.deformGroupTriggered[beam] = true;
            triggerDeformGroups(container.assignedDeformGroups[beam]);
        }
    }

    private void triggerDeformGroups(List<String> groups) {
        if (groups == null) return;
        for (String group : groups) {
            triggerDeformGroup(group);
        }
    }

    /**
     * Applies direct beam failures detected during the force-accumulation phase.
     * Keeping this as a separate commit phase prevents beam iteration order from
     * deciding whether another member of the same break group contributes force.
     */
    void commitPendingBeamBreaks() {
        commitPendingBeamBreaks(normalBeams);
        commitPendingBeamBreaks(supportBeams);
        commitPendingBeamBreaks(boundedBeams);
        commitPendingBeamBreaks(lBeams);
        commitPendingBeamBreaks(anisotropicBeams);
    }

    private void commitPendingBeamBreaks(BeamContainer container) {
        int pendingCount = container.pendingBreakCount();
        for (int i = 0; i < pendingCount; i++) {
            breakBeamAt(container, container.pendingBreakIndex(i));
        }
        container.clearPendingBreaks();
    }

    void breakBeamAt(BeamContainer container, int idx) {
        if (container.broken[idx]) return;
        container.broken[idx] = true;
        container.deformGroupTriggered[idx] = true;
        triggerDeformGroups(container.assignedDeformGroups[idx]);
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
     * Public internal-force entry points. The per-vehicle sub-step implementation now
     * lives in {@link VehicleInternalForceSolver}; these signatures and their call order
     * are unchanged.
     */
    public void solveInternalForces(float dt, float plasticRelaxation){
        solveInternalForces(dt, plasticRelaxation, electrics.snapshot());
    }

    public void solveInternalForces(float dt, float plasticRelaxation, ElectricSnapshot electricSnapshot){
        internalForceSolver.solve(dt, plasticRelaxation, electricSnapshot);
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

}
