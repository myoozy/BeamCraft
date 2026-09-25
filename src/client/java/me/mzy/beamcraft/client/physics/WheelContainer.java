package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class WheelContainer {
    public static final int INIT_WHEEL_CAP = 8;
    public static final int MAX_RAYS = 16; // 1D 数组的最大射线分配空间

    public int count = 0;
    public Map<String, Integer> nameToIndex = new HashMap<>();

    // 基础属性 SoA
    public String[] name = new String[INIT_WHEEL_CAP];
    public int[] node1 = new int[INIT_WHEEL_CAP];
    public int[] node2 = new int[INIT_WHEEL_CAP];
    public int[] wheelDir = new int[INIT_WHEEL_CAP];
    public int[] numRays = new int[INIT_WHEEL_CAP];

    // 物理参数
    public float[] hubRadius = new float[INIT_WHEEL_CAP];
    public float[] tireRadius = new float[INIT_WHEEL_CAP];
    public float[] tireWidth = new float[INIT_WHEEL_CAP];
    public float[] pressurePSI = new float[INIT_WHEEL_CAP];

    // Service-brake configuration and per-wheel pressure state.
    public float[] brakeTorque = new float[INIT_WHEEL_CAP];
    public float[] parkingTorque = new float[INIT_WHEEL_CAP];
    public float[] brakeSpring = new float[INIT_WHEEL_CAP];
    public float[] brakeInputSplit = new float[INIT_WHEEL_CAP];
    public float[] brakeSplitCoef = new float[INIT_WHEEL_CAP];
    public float[] brakePressureInDelay = new float[INIT_WHEEL_CAP];
    public float[] brakePressureOutDelay = new float[INIT_WHEEL_CAP];
    public float[] serviceBrakeTorque = new float[INIT_WHEEL_CAP];
    public float[] brakeAngle = new float[INIT_WHEEL_CAP];

    // 轮胎节点摩擦参数
    public float[] frictionCoef         = new float[INIT_WHEEL_CAP];
    public float[] slidingFrictionCoef  = new float[INIT_WHEEL_CAP];
    public float[] stribeckVelMult      = new float[INIT_WHEEL_CAP];
    public float[] stribeckExponent     = new float[INIT_WHEEL_CAP];
    public float[] treadCoef            = new float[INIT_WHEEL_CAP];
    public float[] noLoadCoef           = new float[INIT_WHEEL_CAP];
    public float[] loadSensitivitySlope = new float[INIT_WHEEL_CAP];
    public float[] fullLoadCoef         = new float[INIT_WHEEL_CAP];
    public float[] softnessCoef         = new float[INIT_WHEEL_CAP];

    // 一维展平数组，内存连续
    // 寻址方式： index = (wheelIndex * MAX_RAYS) + rayIndex
    public int[] hubInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] hubOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireInnerNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];
    public int[] tireOuterNodes = new int[INIT_WHEEL_CAP * MAX_RAYS];

    // 储存三角形的index (必须确保它们在数组中连续排列)
    public int[] tireTriangleIdxStart = new int[INIT_WHEEL_CAP];
    public int[] tireTriangleIdxEnd = new int[INIT_WHEEL_CAP];

    public float[] initialVolume = new float[INIT_WHEEL_CAP];

    public float[] prevVolume = new float[INIT_WHEEL_CAP];
    public float[] normalSign = new float[INIT_WHEEL_CAP];

    public boolean[] isDeflated = new boolean[INIT_WHEEL_CAP];

    // ================================================================
    // BeamNG pressure-wheel counter-torque nodes (node indices; -1 = not defined).
    // A wheel torque is applied to the hub ring by applyDriveTorque(); its equal-and-
    // opposite counter-torque is distributed over these nodes so the axle / suspension /
    // body receives the reaction without producing a net force. BeamNG semantics:
    //   * torqueCoupling + torqueArm + torqueArm2 receive the drivetrain counter-torque.
    //     No drivetrain reaction is generated unless both torqueCoupling and torqueArm are
    //     defined; an undefined torqueArm2 falls back to the inner axle node.
    //   * nodeArm (header column) + nodeCoupling receive the braking counter-torque. An
    //     undefined nodeCoupling falls back to the inner axle node; an undefined nodeArm
    //     means no explicit braking reaction (the load is carried structurally).
    // ================================================================
    public int[] torqueCouplingNode = newReactionNodeArray();
    public int[] torqueArmNode = newReactionNodeArray();
    public int[] torqueArm2Node = newReactionNodeArray();
    public int[] nodeCouplingNode = newReactionNodeArray();
    public int[] nodeArmNode = newReactionNodeArray();

    // Scratch buffer used to assemble reaction node sets without per-call allocation.
    private final int[] reactionScratch = new int[3];

    private final SoftBodyVehicle vehicle;
    private final PressureWheelBuilder pressureWheelBuilder;

    public WheelContainer(SoftBodyVehicle vehicle) {
        this.vehicle = vehicle;
        this.pressureWheelBuilder = new PressureWheelBuilder(this, vehicle);
    }

    /**
     * 生成轮毂 (Hub)
     */
    /** Delegates BeamNG-compatible pressure-wheel construction to its licensed builder. */
    public void generateHub(PhysicsSpecs.WheelHubSpec spec) {
        pressureWheelBuilder.generateHub(spec);
    }

    /** Delegates BeamNG-compatible pressure-wheel construction to its licensed builder. */
    public void generateTire(PhysicsSpecs.WheelTireSpec spec) {
        pressureWheelBuilder.generateTire(spec);
    }

    void ensureWheelCapacity() {
        if (count >= name.length) {
            int newSize = name.length * 2;
            // 扩容普通动态数组
            name = Utility.expand(name, newSize);
            node1 = Utility.expand(node1, newSize);
            node2 = Utility.expand(node2, newSize);
            wheelDir = Utility.expand(wheelDir, newSize);
            numRays = Utility.expand(numRays, newSize);
            hubRadius = Utility.expand(hubRadius, newSize);
            tireRadius = Utility.expand(tireRadius, newSize);
            tireWidth = Utility.expand(tireWidth, newSize);
            pressurePSI = Utility.expand(pressurePSI, newSize);
            brakeTorque = Utility.expand(brakeTorque, newSize);
            parkingTorque = Utility.expand(parkingTorque, newSize);
            brakeSpring = Utility.expand(brakeSpring, newSize);
            brakeInputSplit = Utility.expand(brakeInputSplit, newSize);
            brakeSplitCoef = Utility.expand(brakeSplitCoef, newSize);
            brakePressureInDelay = Utility.expand(brakePressureInDelay, newSize);
            brakePressureOutDelay = Utility.expand(brakePressureOutDelay, newSize);
            serviceBrakeTorque = Utility.expand(serviceBrakeTorque, newSize);
            brakeAngle = Utility.expand(brakeAngle, newSize);

            torqueCouplingNode = expandReactionArray(torqueCouplingNode, newSize);
            torqueArmNode = expandReactionArray(torqueArmNode, newSize);
            torqueArm2Node = expandReactionArray(torqueArm2Node, newSize);
            nodeCouplingNode = expandReactionArray(nodeCouplingNode, newSize);
            nodeArmNode = expandReactionArray(nodeArmNode, newSize);

            frictionCoef = Utility.expand(frictionCoef, newSize);
            slidingFrictionCoef = Utility.expand(slidingFrictionCoef, newSize);
            stribeckVelMult = Utility.expand(stribeckVelMult, newSize);
            stribeckExponent = Utility.expand(stribeckExponent, newSize);
            treadCoef = Utility.expand(treadCoef, newSize);
            noLoadCoef = Utility.expand(noLoadCoef, newSize);
            loadSensitivitySlope = Utility.expand(loadSensitivitySlope, newSize);
            fullLoadCoef = Utility.expand(fullLoadCoef, newSize);
            softnessCoef = Utility.expand(softnessCoef, newSize);

            tireTriangleIdxStart = Utility.expand(tireTriangleIdxStart, newSize);
            tireTriangleIdxEnd = Utility.expand(tireTriangleIdxEnd, newSize);

            initialVolume = Utility.expand(initialVolume, newSize);
            prevVolume = Utility.expand(prevVolume, newSize);
            normalSign = Utility.expand(normalSign, newSize);

            // 扩容展平数组（每个车轮 MAX_RAYS 个射线槽位）
            int newFlatSize = newSize * MAX_RAYS;
            hubInnerNodes = Utility.expand(hubInnerNodes, newFlatSize);
            hubOuterNodes = Utility.expand(hubOuterNodes, newFlatSize);
            tireInnerNodes = Utility.expand(tireInnerNodes, newFlatSize);
            tireOuterNodes = Utility.expand(tireOuterNodes, newFlatSize);

            isDeflated = Utility.expand(isDeflated, newFlatSize);

            System.out.println("⚠️ [WheelContainer] Resized to: " + newSize + " wheels, flat size: " + newFlatSize);
        }
    }

    private static int[] newReactionNodeArray() {
        int[] array = new int[INIT_WHEEL_CAP];
        Arrays.fill(array, -1);
        return array;
    }

    private static int[] expandReactionArray(int[] array, int newSize) {
        int oldSize = array.length;
        int[] expanded = Utility.expand(array, newSize);
        Arrays.fill(expanded, oldSize, newSize, -1);
        return expanded;
    }

    /**
     * Configures the BeamNG drivetrain counter-torque nodes for one wheel. Pass {@code -1}
     * for any node that is not defined; a drivetrain reaction only occurs at apply time when
     * both {@code torqueCoupling} and {@code torqueArm} are defined ({@code torqueArm2}
     * falls back to the inner axle node).
     */
    public void setReactionNodes(int wheelIdx, int torqueCoupling, int torqueArm, int torqueArm2) {
        if (wheelIdx < 0 || wheelIdx >= count) return;
        torqueCouplingNode[wheelIdx] = torqueCoupling;
        torqueArmNode[wheelIdx] = torqueArm;
        torqueArm2Node[wheelIdx] = torqueArm2;
    }

    /** Overrides the braking coupling node; pass {@code -1} to restore the inner-axle default. */
    public void setBrakeCouplingNode(int wheelIdx, int nodeCoupling) {
        if (wheelIdx < 0 || wheelIdx >= count) return;
        nodeCouplingNode[wheelIdx] = nodeCoupling;
    }

    public void deflateWheel(int idx) {
        if (idx >= 0 && idx < count) {
            if (!isDeflated[idx])isDeflated[idx] = true;
        }
    }

    /**
     * Returns the angular velocity of the soft-body wheel hub about its current
     * axle.  The sign is normalized with the JBeam wheelDir value, so wheels on
     * opposite sides of the vehicle report the same sign while rolling forward.
     */
    public float getAngularVelocity(int wheelIdx) {
        if (wheelIdx < 0 || wheelIdx >= count) return 0.0f;

        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return 0.0f;

        // Expose the same forward-positive convention as the powertrain. The old
        // node1-node2 axis made a forward-rolling wheel report a negative AV.
        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return 0.0f;
        double direction = wheelDir[wheelIdx] >= 0 ? 1.0 : -1.0;
        ax = ax * direction / axisLength;
        ay = ay * direction / axisLength;
        az = az * direction / axisLength;

        double totalMass = 0.0;
        double cx = 0.0, cy = 0.0, cz = 0.0;
        double cvx = 0.0, cvy = 0.0, cvz = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            double innerMass = Math.max(0.0, nodes.mass[inner]);
            double outerMass = Math.max(0.0, nodes.mass[outer]);
            totalMass += innerMass + outerMass;
            cx += nodes.posX[inner] * innerMass + nodes.posX[outer] * outerMass;
            cy += nodes.posY[inner] * innerMass + nodes.posY[outer] * outerMass;
            cz += nodes.posZ[inner] * innerMass + nodes.posZ[outer] * outerMass;
            cvx += nodes.velX[inner] * innerMass + nodes.velX[outer] * outerMass;
            cvy += nodes.velY[inner] * innerMass + nodes.velY[outer] * outerMass;
            cvz += nodes.velZ[inner] * innerMass + nodes.velZ[outer] * outerMass;
        }
        if (totalMass < 1e-9) return 0.0f;
        cx /= totalMass; cy /= totalMass; cz /= totalMass;
        cvx /= totalMass; cvy /= totalMass; cvz /= totalMass;

        double angularMomentum = 0.0;
        double inertia = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            angularMomentum += angularMomentum(nodes, inner, cx, cy, cz, cvx, cvy, cvz, ax, ay, az);
            angularMomentum += angularMomentum(nodes, outer, cx, cy, cz, cvx, cvy, cvz, ax, ay, az);
            inertia += polarInertia(nodes, inner, cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, outer, cx, cy, cz, ax, ay, az);
        }
        return inertia > 1e-9 ? (float) (angularMomentum / inertia) : 0.0f;
    }

    /** Returns the hub and tire nodes' instantaneous polar inertia. */
    public float getRotationalInertia(int wheelIdx) {
        if (wheelIdx < 0 || wheelIdx >= count) return 0.0f;
        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return 0.0f;

        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return 0.0f;
        ax /= axisLength; ay /= axisLength; az /= axisLength;

        double cx = (nodes.posX[node1[wheelIdx]] + nodes.posX[node2[wheelIdx]]) * 0.5;
        double cy = (nodes.posY[node1[wheelIdx]] + nodes.posY[node2[wheelIdx]]) * 0.5;
        double cz = (nodes.posZ[node1[wheelIdx]] + nodes.posZ[node2[wheelIdx]]) * 0.5;
        double inertia = 0.0;
        boolean hasTire = tireRadius[wheelIdx] > 0.0f;
        for (int ray = 0; ray < rays; ray++) {
            inertia += polarInertia(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
            if (hasTire) {
                inertia += polarInertia(nodes, tireInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
                inertia += polarInertia(nodes, tireOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
            }
        }
        return (float) inertia;
    }

    /**
     * Applies a pure axle torque to the hub ring without adding net force. This applies
     * the wheel torque <em>only</em> — it never touches the pressure-wheel counter-torque
     * nodes. Braking reuses this method, so the drivetrain reaction must be requested
     * explicitly through {@link #applyDriveReaction} / {@link #applyBrakeReaction}.
     */
    public void applyDriveTorque(int wheelIdx, float torque) {
        if (wheelIdx < 0 || wheelIdx >= count || Math.abs(torque) < 1e-8f) return;
        NodeContainer nodes = vehicle.nodes;
        int base = wheelIdx * MAX_RAYS;
        int rays = numRays[wheelIdx];
        if (rays <= 0) return;

        // Keep applied torque and reported AV on the same forward-positive axis.
        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return;
        double direction = wheelDir[wheelIdx] >= 0 ? 1.0 : -1.0;
        ax = ax * direction / axisLength;
        ay = ay * direction / axisLength;
        az = az * direction / axisLength;

        double totalMass = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            int inner = hubInnerNodes[base + ray];
            int outer = hubOuterNodes[base + ray];
            double innerMass = Math.max(0.0, nodes.mass[inner]);
            double outerMass = Math.max(0.0, nodes.mass[outer]);
            totalMass += innerMass + outerMass;
            cx += nodes.posX[inner] * innerMass + nodes.posX[outer] * outerMass;
            cy += nodes.posY[inner] * innerMass + nodes.posY[outer] * outerMass;
            cz += nodes.posZ[inner] * innerMass + nodes.posZ[outer] * outerMass;
        }
        if (totalMass < 1e-9) return;
        cx /= totalMass; cy /= totalMass; cz /= totalMass;

        double inertia = 0.0;
        for (int ray = 0; ray < rays; ray++) {
            inertia += polarInertia(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az);
            inertia += polarInertia(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az);
        }
        if (inertia < 1e-9) return;
        double angularAcceleration = torque / inertia;
        for (int ray = 0; ray < rays; ray++) {
            applyAngularForce(nodes, hubInnerNodes[base + ray], cx, cy, cz, ax, ay, az, angularAcceleration);
            applyAngularForce(nodes, hubOuterNodes[base + ray], cx, cy, cz, ax, ay, az, angularAcceleration);
        }
    }

    /**
     * Applies a wheel torque together with the BeamNG pressure-wheel drivetrain
     * counter-torque: the hub receives {@code torque} and an equal-and-opposite torque is
     * distributed over the wheel's torqueCoupling/torqueArm/torqueArm2 nodes. {@code torque}
     * is forward-positive. When the wheel defines no torque coupling nodes this is exactly
     * {@link #applyDriveTorque}.
     */
    public void applyDriveTorqueAndReaction(int wheelIdx, float torque) {
        applyDriveTorque(wheelIdx, torque);
        applyDriveReaction(wheelIdx, torque);
    }

    /**
     * Applies only the drivetrain counter-torque for a wheel torque that the caller has
     * already applied. The reaction is generated only when the wheel defines both
     * {@code torqueCoupling} and {@code torqueArm} (BeamNG semantics); an undefined
     * {@code torqueArm2} falls back to the inner axle node. No reaction implies no wheel
     * torque was applied (e.g. neutral), so nothing is generated here either.
     */
    public void applyDriveReaction(int wheelIdx, float torque) {
        if (wheelIdx < 0 || wheelIdx >= count) return;
        int torqueCoupling = torqueCouplingNode[wheelIdx];
        int torqueArm = torqueArmNode[wheelIdx];
        if (torqueCoupling < 0 || torqueArm < 0) return;
        int torqueArm2 = torqueArm2Node[wheelIdx];
        if (torqueArm2 < 0) torqueArm2 = node2[wheelIdx];
        applyTorqueReaction(wheelIdx, torque, torqueCoupling, torqueArm, torqueArm2);
    }

    /**
     * Applies the BeamNG braking counter-torque for a brake torque the caller already
     * applied to the wheel. The reaction is distributed over nodeArm (the header brake
     * lever node) and nodeCoupling (defaulting to the inner axle node). A wheel without a
     * nodeArm has no explicit braking reaction: the counter-torque is carried structurally
     * through the hub/suspension beams, exactly as before this feature existed.
     */
    public void applyBrakeReaction(int wheelIdx, float appliedWheelTorque) {
        if (wheelIdx < 0 || wheelIdx >= count) return;
        int nodeArm = nodeArmNode[wheelIdx];
        if (nodeArm < 0) return;
        int nodeCoupling = nodeCouplingNode[wheelIdx];
        if (nodeCoupling < 0) nodeCoupling = node2[wheelIdx];
        applyTorqueReaction(wheelIdx, appliedWheelTorque, nodeCoupling, nodeArm, -1);
    }

    /**
     * Distributes the equal-and-opposite counter-torque for {@code wheelTorque} over the
     * given reaction nodes as a zero-net-force pure torque about the wheel's forward-positive
     * axle. {@code nodeC} may be -1. Degenerate or invalid geometry simply produces no forces.
     */
    private void applyTorqueReaction(int wheelIdx, float wheelTorque, int nodeA, int nodeB, int nodeC) {
        if (wheelIdx < 0 || wheelIdx >= count || Math.abs(wheelTorque) < 1e-8f) return;
        NodeContainer nodes = vehicle.nodes;

        // The reaction uses the same forward-positive axle axis as applyDriveTorque() so the
        // counter-torque is exactly equal and opposite to the torque applied to the wheel.
        double ax = nodes.posX[node2[wheelIdx]] - nodes.posX[node1[wheelIdx]];
        double ay = nodes.posY[node2[wheelIdx]] - nodes.posY[node1[wheelIdx]];
        double az = nodes.posZ[node2[wheelIdx]] - nodes.posZ[node1[wheelIdx]];
        double axisLength = Math.sqrt(ax * ax + ay * ay + az * az);
        if (axisLength < 1e-9) return;
        double direction = wheelDir[wheelIdx] >= 0 ? 1.0 : -1.0;
        ax = ax * direction / axisLength;
        ay = ay * direction / axisLength;
        az = az * direction / axisLength;

        // Collect the valid, unique reaction nodes into the scratch buffer (which is
        // reused across wheels, so no per-call allocation). At least two are needed to
        // carry a torque.
        reactionScratch[0] = nodeA;
        reactionScratch[1] = nodeB;
        reactionScratch[2] = nodeC;
        int valid = 0;
        for (int i = 0; i < 3; i++) {
            int candidate = reactionScratch[i];
            if (candidate < 0 || candidate >= nodes.count) continue;
            if (nodes.mass[candidate] <= 0.0f) continue;
            boolean duplicate = false;
            for (int j = 0; j < valid; j++) {
                if (reactionScratch[j] == candidate) { duplicate = true; break; }
            }
            if (!duplicate) reactionScratch[valid++] = candidate;
        }
        if (valid < 2) return;

        // Counter-torque = -(wheel torque) about the forward-positive axle.
        TorqueReactionSolver.apply(nodes, reactionScratch, 0, valid,
                (float) (-wheelTorque * ax),
                (float) (-wheelTorque * ay),
                (float) (-wheelTorque * az));
    }

    /** Applies the JBeam service brake curve and pressure delays to every wheel. */
    public void applyServiceBrakes(float brakeInput, float dt) {
        applyBrakes(brakeInput, 0.0f, dt);
    }

    /** Applies service and parking-brake inputs; parking input is intentionally unbound for now. */
    public void applyBrakes(float brakeInput, float parkingBrakeInput, float dt) {
        if (dt <= 0.0f) return;
        float input = Math.clamp(brakeInput, 0.0f, 1.0f);
        float parkingInput = Math.clamp(parkingBrakeInput, 0.0f, 1.0f);
        for (int wheel = 0; wheel < count; wheel++) {
            float maximum = brakeTorque[wheel];
            float target = calculateServiceBrakeTorque(maximum, input,
                    brakeInputSplit[wheel], brakeSplitCoef[wheel]);
            float delay = target > serviceBrakeTorque[wheel]
                    ? brakePressureInDelay[wheel] : brakePressureOutDelay[wheel];
            float rate = delay > 1.0e-6f ? maximum / delay : Float.POSITIVE_INFINITY;
            serviceBrakeTorque[wheel] = moveTowards(serviceBrakeTorque[wheel], target, rate * dt);

            float capacity = Math.max(serviceBrakeTorque[wheel], parkingTorque[wheel] * parkingInput);
            float stiffness = Math.max(Math.max(brakeTorque[wheel], parkingTorque[wheel]), 1.0f)
                    * brakeSpring[wheel];
            if (capacity <= 1.0e-8f || stiffness <= 1.0e-8f) {
                brakeAngle[wheel] = 0.0f;
                continue;
            }

            float angularVelocity = getAngularVelocity(wheel);
            float angleLimit = capacity / stiffness;
            brakeAngle[wheel] = Math.clamp(
                    brakeAngle[wheel] + angularVelocity * dt,
                    -angleLimit,
                    angleLimit);

            // The torsion spring itself supplies the static holding torque. Do not cap it
            // to the impulse that would stop the pre-contact angular velocity: ground
            // contact is resolved later in the substep and may require the brake to keep
            // resisting after the wheel has reached zero speed.
            float wheelTorque = -brakeAngle[wheel] * stiffness;
            applyDriveTorque(wheel, wheelTorque);
            // The pressure-wheel braking counter-torque is the exact opposite of the wheel
            // torque, distributed over nodeArm/nodeCoupling (no-op without a nodeArm).
            applyBrakeReaction(wheel, wheelTorque);
        }
    }

    static float calculateServiceBrakeTorque(float maximum, float input, float split, float splitCoef) {
        float clampedInput = Math.clamp(input, 0.0f, 1.0f);
        float clampedSplit = Math.clamp(split, 0.0f, 1.0f);
        float clampedCoef = Math.clamp(splitCoef, 0.0f, 1.0f);
        return Math.max(0.0f, maximum) * (Math.min(clampedInput, clampedSplit)
                + Math.max(clampedInput - clampedSplit, 0.0f) * clampedCoef);
    }

    private static float moveTowards(float current, float target, float maximumDelta) {
        if (maximumDelta == Float.POSITIVE_INFINITY) return target;
        if (current < target) return Math.min(current + maximumDelta, target);
        return Math.max(current - maximumDelta, target);
    }

    private static double angularMomentum(NodeContainer nodes, int node, double cx, double cy, double cz,
                                          double cvx, double cvy, double cvz,
                                          double ax, double ay, double az) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double vx = nodes.velX[node] - cvx, vy = nodes.velY[node] - cvy, vz = nodes.velZ[node] - cvz;
        double mass = Math.max(0.0, nodes.mass[node]);
        return mass * (ax * (ry * vz - rz * vy)
                + ay * (rz * vx - rx * vz) + az * (rx * vy - ry * vx));
    }

    private static double polarInertia(NodeContainer nodes, int node, double cx, double cy, double cz,
                                       double ax, double ay, double az) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double axial = rx * ax + ry * ay + rz * az;
        return Math.max(0.0, nodes.mass[node])
                * Math.max(0.0, rx * rx + ry * ry + rz * rz - axial * axial);
    }

    private static void applyAngularForce(NodeContainer nodes, int node, double cx, double cy, double cz,
                                          double ax, double ay, double az, double angularAcceleration) {
        double rx = nodes.posX[node] - cx, ry = nodes.posY[node] - cy, rz = nodes.posZ[node] - cz;
        double scale = angularAcceleration * Math.max(0.0, nodes.mass[node]);
        nodes.forceX[node] += (float) ((ay * rz - az * ry) * scale);
        nodes.forceY[node] += (float) ((az * rx - ax * rz) * scale);
        nodes.forceZ[node] += (float) ((ax * ry - ay * rx) * scale);
    }

    public void reset() {
        for (int i = 0; i < count; i++) {
            isDeflated[i] = false;
            serviceBrakeTorque[i] = 0.0f;
            brakeAngle[i] = 0.0f;
        }
    }

    public void clear() {
        count = 0;
        nameToIndex.clear();
    }
}
