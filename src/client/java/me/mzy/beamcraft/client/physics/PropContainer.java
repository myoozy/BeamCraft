/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, You can obtain one at
 * https://beamng.com/bCDDL-1.1.txt
 */
package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.PhysicsSpecs.PropSpec;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.utility.Utility;

import java.util.Arrays;

/**
 * BeamNG rigid props: non-deforming meshes attached to a three-node frame and
 * optionally animated by an electric signal.
 *
 * <p>Each prop exposes three synthetic render nodes after the vehicle's real
 * physics nodes. The existing transform-feedback skinning shader can therefore
 * render a prop as a rigid local basis without adding a second mesh pipeline or
 * letting render-only nodes enter the physics solver.</p>
 */
public final class PropContainer {
    private static final int INITIAL_CAPACITY = 16;
    private static final float AXIS_EPSILON_SQUARED = 1.0e-10f;
    private static final float DEFAULT_STEERING_WHEEL_LOCK_DEGREES = 450.0f;

    public int count;
    public String[] function = new String[INITIAL_CAPACITY];
    public int[] meshIndex = new int[INITIAL_CAPACITY];
    public int[] refNode = new int[INITIAL_CAPACITY];
    public int[] xNode = new int[INITIAL_CAPACITY];
    public int[] yNode = new int[INITIAL_CAPACITY];

    public float[] baseRotationX = new float[INITIAL_CAPACITY];
    public float[] baseRotationY = new float[INITIAL_CAPACITY];
    public float[] baseRotationZ = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalX = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalY = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseRotationGlobal = new boolean[INITIAL_CAPACITY];
    public float[] rotationX = new float[INITIAL_CAPACITY];
    public float[] rotationY = new float[INITIAL_CAPACITY];
    public float[] rotationZ = new float[INITIAL_CAPACITY];
    public float[] translationX = new float[INITIAL_CAPACITY];
    public float[] translationY = new float[INITIAL_CAPACITY];
    public float[] translationZ = new float[INITIAL_CAPACITY];
    public float[] min = new float[INITIAL_CAPACITY];
    public float[] max = new float[INITIAL_CAPACITY];
    public float[] offset = new float[INITIAL_CAPACITY];
    public float[] multiplier = new float[INITIAL_CAPACITY];
    public boolean[] translationUseMeters = new boolean[INITIAL_CAPACITY];

    /** Initial prop origin in its rest-pose idRef/idX/idY frame, filled by the binder. */
    public float[] originLocalX = new float[INITIAL_CAPACITY];
    public float[] originLocalY = new float[INITIAL_CAPACITY];
    public float[] originLocalZ = new float[INITIAL_CAPACITY];
    public boolean[] originBound = new boolean[INITIAL_CAPACITY];
    /** Parsed origin override retained until the DAE pivot is available to the binder. */
    public float[] baseTranslationX = new float[INITIAL_CAPACITY];
    public float[] baseTranslationY = new float[INITIAL_CAPACITY];
    public float[] baseTranslationZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseTranslation = new boolean[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalX = new float[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalY = new float[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseTranslationGlobal = new boolean[INITIAL_CAPACITY];

    private final float[] referenceBasis = new float[9];
    private final float[] propOrientation = new float[9];
    private final float[] globalScratch = new float[9];

    public int register(PropSpec spec, int owningMeshIndex) {
        ensureCapacity();
        int index = count++;
        function[index] = spec.function();
        meshIndex[index] = owningMeshIndex;
        refNode[index] = spec.refNode();
        xNode[index] = spec.xNode();
        yNode[index] = spec.yNode();

        baseRotationX[index] = spec.baseRotation().x();
        baseRotationY[index] = spec.baseRotation().y();
        baseRotationZ[index] = spec.baseRotation().z();
        baseRotationGlobalX[index] = spec.baseRotationGlobal().x();
        baseRotationGlobalY[index] = spec.baseRotationGlobal().y();
        baseRotationGlobalZ[index] = spec.baseRotationGlobal().z();
        hasBaseRotationGlobal[index] = spec.hasBaseRotationGlobal();
        rotationX[index] = spec.rotation().x();
        rotationY[index] = spec.rotation().y();
        rotationZ[index] = spec.rotation().z();
        translationX[index] = spec.translation().x();
        translationY[index] = spec.translation().y();
        translationZ[index] = spec.translation().z();
        min[index] = spec.min();
        max[index] = spec.max();
        offset[index] = spec.offset();
        multiplier[index] = spec.multiplier();
        translationUseMeters[index] = spec.translationUseMeters();

        baseTranslationX[index] = spec.baseTranslation().x();
        baseTranslationY[index] = spec.baseTranslation().y();
        baseTranslationZ[index] = spec.baseTranslation().z();
        hasBaseTranslation[index] = spec.hasBaseTranslation();
        baseTranslationGlobalX[index] = spec.baseTranslationGlobal().x();
        baseTranslationGlobalY[index] = spec.baseTranslationGlobal().y();
        baseTranslationGlobalZ[index] = spec.baseTranslationGlobal().z();
        hasBaseTranslationGlobal[index] = spec.hasBaseTranslationGlobal();
        return index;
    }

    public int renderNodeCount(int physicsNodeCount) {
        return Math.addExact(physicsNodeCount, Math.multiplyExact(count, 3));
    }

    public int syntheticNodeBase(int physicsNodeCount, int propIndex) {
        return physicsNodeCount + propIndex * 3;
    }

    /**
     * Appends the render-only prop frames to interpolated physical node arrays.
     * All input and output coordinates are in BeamCraft/Minecraft local space.
     */
    public void appendRenderNodes(SoftBodyVehicle vehicle, float[] x, float[] y, float[] z,
                                  int physicsNodeCount) {
        float steeringDegrees = steeringWheelDegrees(vehicle, x, y, z);
        for (int prop = 0; prop < count; prop++) {
            int syntheticBase = syntheticNodeBase(physicsNodeCount, prop);
            float value = functionValue(vehicle, prop, steeringDegrees);
            boolean orientationReady = hasBaseRotationGlobal[prop]
                    ? buildGlobalPropOrientation(x, y, z, vehicle.cameras.refNodes(),
                    baseRotationGlobalX[prop], baseRotationGlobalY[prop], baseRotationGlobalZ[prop],
                    rotationX[prop] * value, rotationY[prop] * value, rotationZ[prop] * value,
                    propOrientation, globalScratch)
                    : buildPropOrientation(x, y, z, refNode[prop], xNode[prop], yNode[prop],
                    baseRotationX[prop], baseRotationY[prop], baseRotationZ[prop],
                    rotationX[prop] * value, rotationY[prop] * value, rotationZ[prop] * value,
                    propOrientation);
            if (!originBound[prop]
                    || !buildReferenceBasis(x, y, z, refNode[prop], xNode[prop], yNode[prop], referenceBasis)
                    || !orientationReady) {
                // Degenerate authored reference frames are kept finite and invisible at the ref node.
                float cx = x[refNode[prop]], cy = y[refNode[prop]], cz = z[refNode[prop]];
                setSyntheticFrame(x, y, z, syntheticBase, cx, cy, cz,
                        1, 0, 0, 0, 1, 0);
                continue;
            }

            float bx0 = referenceBasis[0], bx1 = referenceBasis[3], bx2 = referenceBasis[6];
            float by0 = referenceBasis[1], by1 = referenceBasis[4], by2 = referenceBasis[7];
            float bz0 = referenceBasis[2], bz1 = referenceBasis[5], bz2 = referenceBasis[8];
            float refX = x[refNode[prop]], refY = y[refNode[prop]], refZ = z[refNode[prop]];

            float centerX = refX + bx0 * originLocalX[prop] + by0 * originLocalY[prop] + bz0 * originLocalZ[prop];
            float centerY = refY + bx1 * originLocalX[prop] + by1 * originLocalY[prop] + bz1 * originLocalZ[prop];
            float centerZ = refZ + bx2 * originLocalX[prop] + by2 * originLocalY[prop] + bz2 * originLocalZ[prop];

            float ax0 = propOrientation[0], ax1 = propOrientation[3], ax2 = propOrientation[6];
            float ay0 = propOrientation[1], ay1 = propOrientation[4], ay2 = propOrientation[7];

            float xScale = translationUseMeters[prop] ? 1.0f : distance(x, y, z, refNode[prop], xNode[prop]);
            float yScale = translationUseMeters[prop] ? 1.0f : distance(x, y, z, refNode[prop], yNode[prop]);
            float tx = translationX[prop] * value * xScale;
            float ty = translationY[prop] * value * yScale;
            float tz = translationZ[prop] * value;
            float az0 = ax1 * ay2 - ax2 * ay1;
            float az1 = ax2 * ay0 - ax0 * ay2;
            float az2 = ax0 * ay1 - ax1 * ay0;
            centerX += bx0 * tx + by0 * ty + bz0 * tz;
            centerY += bx1 * tx + by1 * ty + bz1 * tz;
            centerZ += bx2 * tx + by2 * ty + bz2 * tz;

            setSyntheticFrame(x, y, z, syntheticBase, centerX, centerY, centerZ,
                    ax0, ax1, ax2, ay0, ay1, ay2);
        }
    }

    public static boolean buildReferenceBasis(float[] x, float[] y, float[] z,
                                              int ref, int xNode, int yNode, float[] out) {
        float xx = x[xNode] - x[ref], xy = y[xNode] - y[ref], xz = z[xNode] - z[ref];
        float xLengthSquared = xx * xx + xy * xy + xz * xz;
        if (xLengthSquared <= AXIS_EPSILON_SQUARED) return false;
        float inverseXLength = inverseSqrt(xLengthSquared);
        xx *= inverseXLength; xy *= inverseXLength; xz *= inverseXLength;

        float yx = x[yNode] - x[ref], yy = y[yNode] - y[ref], yz = z[yNode] - z[ref];
        float projection = yx * xx + yy * xy + yz * xz;
        yx -= projection * xx; yy -= projection * xy; yz -= projection * xz;
        float yLengthSquared = yx * yx + yy * yy + yz * yz;
        if (yLengthSquared <= AXIS_EPSILON_SQUARED) return false;
        float inverseYLength = inverseSqrt(yLengthSquared);
        yx *= inverseYLength; yy *= inverseYLength; yz *= inverseYLength;

        float zx = xy * yz - xz * yy;
        float zy = xz * yx - xx * yz;
        float zz = xx * yy - xy * yx;
        // Row-major matrix with the frame axes as columns.
        out[0] = xx; out[1] = yx; out[2] = zx;
        out[3] = xy; out[4] = yy; out[5] = zy;
        out[6] = xz; out[7] = yz; out[8] = zz;
        return true;
    }

    /**
     * Builds the prop object orientation used by BeamNG's native async update.
     * The unusual initial direction and auto-yaw are part of the prop contract;
     * replacing this with a conventional three-node basis gives vehicle-specific
     * 90/180 degree errors when DAE nodes use different authored transforms.
     */
    public static boolean buildPropOrientation(float[] px, float[] py, float[] pz,
                                               int ref, int xNode, int yNode,
                                               float baseX, float baseY, float baseZ,
                                               float animationX, float animationY, float animationZ,
                                               float[] out) {
        // Run the native algorithm in BeamNG's X/Y/Z-up coordinates. BeamCraft
        // stores the same points as (X, Z, -Y), so convert the two directions
        // back here and map the resulting basis to render coordinates at the end.
        float nx0 = px[xNode] - px[ref];
        float nx1 = -(pz[xNode] - pz[ref]);
        float nx2 = py[xNode] - py[ref];
        float ny0 = px[yNode] - px[ref];
        float ny1 = -(pz[yNode] - pz[ref]);
        float ny2 = py[yNode] - py[ref];
        float nxLength = length(nx0, nx1, nx2), nyLength = length(ny0, ny1, ny2);
        if (nxLength <= 1.0e-5f || nyLength <= 1.0e-5f) return false;
        nx0 /= nxLength; nx1 /= nxLength; nx2 /= nxLength;

        // C++ prop direction: nz = normalize(ny cross nx).
        float dir0 = ny1 * nx2 - ny2 * nx1;
        float dir1 = ny2 * nx0 - ny0 * nx2;
        float dir2 = ny0 * nx1 - ny1 * nx0;
        float dirLength = length(dir0, dir1, dir2);
        if (dirLength <= 1.0e-5f) return false;
        dir0 /= dirLength; dir1 /= dirLength; dir2 /= dirLength;

        // camUpwards = -normalize(nz cross nx), matching asyncUpdate.
        float cam0 = -(dir1 * nx2 - dir2 * nx1);
        float cam1 = -(dir2 * nx0 - dir0 * nx2);
        float cam2 = -(dir0 * nx1 - dir1 * nx0);
        float camLength = length(cam0, cam1, cam2);
        if (camLength <= 1.0e-5f) return false;
        cam0 /= camLength; cam1 /= camLength; cam2 /= camLength;

        // Exact LuaQuat:setFromDir(nz), including its deterministic near-up
        // perturbation and row-oriented axes-to-quaternion conversion.
        if (Math.abs(dir2) > 0.9999f) {
            float k = Math.abs(dir0) + 0.5f;
            k -= (float) Math.floor(k);
            float pp0 = -dir1, pp1 = dir0 - k * dir2, pp2 = k * dir1;
            float ppLength = length(pp0, pp1, pp2);
            if (ppLength > 1.0e-30f) {
                dir0 += pp0 / ppLength * 1.0e-5f;
                dir1 += pp1 / ppLength * 1.0e-5f;
                dir2 += pp2 / ppLength * 1.0e-5f;
                dirLength = length(dir0, dir1, dir2);
                dir0 /= dirLength; dir1 /= dirLength; dir2 /= dirLength;
            }
        }
        float row0x = dir1, row0y = -dir0, row0z = 0.0f; // dir cross global Z
        float row0Length = length(row0x, row0y, row0z);
        if (row0Length <= 1.0e-8f) return false;
        row0x /= row0Length; row0y /= row0Length;
        float row2x = row0y * dir2 - row0z * dir1;
        float row2y = row0z * dir0 - row0x * dir2;
        float row2z = row0x * dir1 - row0y * dir0;
        float row2Length = length(row2x, row2y, row2z);
        row2x /= row2Length; row2y /= row2Length; row2z /= row2Length;

        Utility.quaternionFromRotationRows(out,
                row0x, row0y, row0z,
                dir0, dir1, dir2,
                row2x, row2y, row2z);
        float initialQx = out[0], initialQy = out[1], initialQz = out[2], initialQw = out[3];
        float rotatedUpX = -2.0f * initialQw * initialQy + 2.0f * initialQz * initialQx;
        float rotatedUpY = 2.0f * initialQw * initialQx + 2.0f * initialQz * initialQy;
        float rotatedUpZ = 1.0f - 2.0f * initialQx * initialQx - 2.0f * initialQy * initialQy;
        float autoYaw = (float) Math.atan2(
                rotatedUpX * nx0 + rotatedUpY * nx1 + rotatedUpZ * nx2,
                rotatedUpX * cam0 + rotatedUpY * cam1 + rotatedUpZ * cam2);

        Utility.premultiplyEulerYzx(out, 0.0f, autoYaw + (float) Math.toRadians(baseY), 0.0f);
        Utility.premultiplyEulerYzx(out, 0.0f, 0.0f, (float) Math.toRadians(-baseZ));
        Utility.premultiplyEulerYzx(out, (float) Math.toRadians(-baseX), 0.0f, 0.0f);
        Utility.premultiplyEulerYzx(out, 0.0f, (float) Math.toRadians(-animationY), 0.0f);
        Utility.premultiplyEulerYzx(out, 0.0f, 0.0f, (float) Math.toRadians(-animationZ));
        Utility.premultiplyEulerYzx(out, (float) Math.toRadians(-animationX), 0.0f, 0.0f);

        float qx = out[0], qy = out[1], qz = out[2], qw = out[3];
        storeMappedQuaternionAxis(out, 0, qx, qy, qz, qw, 1.0f, 0.0f, 0.0f);
        storeMappedQuaternionAxis(out, 1, qx, qy, qz, qw, 0.0f, 1.0f, 0.0f);
        storeMappedQuaternionAxis(out, 2, qx, qy, qz, qw, 0.0f, 0.0f, 1.0f);
        return true;
    }

    /** Absolute vehicle-frame baseRotationGlobal, followed by the normal live rotation. */
    public static boolean buildGlobalPropOrientation(float[] px, float[] py, float[] pz,
                                                     VehicleCameraData.RefNodes refs,
                                                     float globalX, float globalY, float globalZ,
                                                     float animationX, float animationY, float animationZ,
                                                     float[] out, float[] scratch) {
        float lx = 1.0f, ly = 0.0f, lz = 0.0f;
        float bx = 0.0f, by = 0.0f, bz = -1.0f;
        float ux = 0.0f, uy = 1.0f, uz = 0.0f;
        if (refs != null
                && validNode(refs.ref(), px.length) && validNode(refs.back(), px.length)
                && validNode(refs.left(), px.length)) {
            int ref = refs.ref();
            lx = px[refs.left()] - px[ref];
            ly = py[refs.left()] - py[ref];
            lz = pz[refs.left()] - pz[ref];
            float leftLength = length(lx, ly, lz);
            if (leftLength <= 1.0e-5f) return false;
            lx /= leftLength; ly /= leftLength; lz /= leftLength;

            bx = px[refs.back()] - px[ref];
            by = py[refs.back()] - py[ref];
            bz = pz[refs.back()] - pz[ref];
            float projection = bx * lx + by * ly + bz * lz;
            bx -= projection * lx; by -= projection * ly; bz -= projection * lz;
            float backLength = length(bx, by, bz);
            if (backLength <= 1.0e-5f) return false;
            bx /= backLength; by /= backLength; bz /= backLength;
            ux = ly * bz - lz * by;
            uy = lz * bx - lx * bz;
            uz = lx * by - ly * bx;
            if (validNode(refs.up(), px.length)) {
                float authoredUpX = px[refs.up()] - px[ref];
                float authoredUpY = py[refs.up()] - py[ref];
                float authoredUpZ = pz[refs.up()] - pz[ref];
                if (ux * authoredUpX + uy * authoredUpY + uz * authoredUpZ < 0.0f) {
                    bx = -bx; by = -by; bz = -bz;
                    ux = -ux; uy = -uy; uz = -uz;
                }
            }
        }

        // BeamNG's setFromEuler plus conjugate vector convention is the global
        // YZX intrinsic rotation used by Prop::setBaseRotationGlobal.
        out[0] = out[1] = out[2] = 0.0f; out[3] = 1.0f;
        Utility.premultiplyEulerYzx(out,
                (float) Math.toRadians(globalX),
                (float) Math.toRadians(globalY),
                (float) Math.toRadians(globalZ));
        float eqx = out[0], eqy = out[1], eqz = out[2], eqw = out[3];
        // bodyMC maps Beam vehicle-axis components into MC space, so its right
        // operand remains the numeric Beam rotation matrix (no C*E*C^T here).
        storeQuaternionAxis(scratch, 0, eqx, eqy, eqz, eqw, 1.0f, 0.0f, 0.0f);
        storeQuaternionAxis(scratch, 1, eqx, eqy, eqz, eqw, 0.0f, 1.0f, 0.0f);
        storeQuaternionAxis(scratch, 2, eqx, eqy, eqz, eqw, 0.0f, 0.0f, 1.0f);

        // desired MC orientation = current vehicle body frame * authored global rotation
        for (int column = 0; column < 3; column++) {
            float ex = scratch[column], ey = scratch[3 + column], ez = scratch[6 + column];
            out[column] = lx * ex + bx * ey + ux * ez;
            out[3 + column] = ly * ex + by * ey + uy * ez;
            out[6 + column] = lz * ex + bz * ey + uz * ez;
        }

        // Convert the desired MC matrix to Beam coordinates, then to the
        // conjugate-convention quaternion needed for the live rotations.
        float m00 = out[0], m01 = out[1], m02 = out[2];
        float m10 = -out[6], m11 = -out[7], m12 = -out[8];
        float m20 = out[3], m21 = out[4], m22 = out[5];
        Utility.quaternionFromRotationRows(out,
                m00, m10, m20,
                m01, m11, m21,
                m02, m12, m22);
        Utility.premultiplyEulerYzx(out, 0.0f, (float) Math.toRadians(-animationY), 0.0f);
        Utility.premultiplyEulerYzx(out, 0.0f, 0.0f, (float) Math.toRadians(-animationZ));
        Utility.premultiplyEulerYzx(out, (float) Math.toRadians(-animationX), 0.0f, 0.0f);
        float qx = out[0], qy = out[1], qz = out[2], qw = out[3];
        storeMappedQuaternionAxis(out, 0, qx, qy, qz, qw, 1.0f, 0.0f, 0.0f);
        storeMappedQuaternionAxis(out, 1, qx, qy, qz, qw, 0.0f, 1.0f, 0.0f);
        storeMappedQuaternionAxis(out, 2, qx, qy, qz, qw, 0.0f, 0.0f, 1.0f);
        return true;
    }

    private static boolean validNode(int node, int length) {
        return node >= 0 && node < length;
    }

    private static void storeMappedQuaternionAxis(float[] out, int column,
                                                   float qx, float qy, float qz, float qw,
                                                   float vx, float vy, float vz) {
        Utility.rotateVectorByInverseQuaternion(
                out, column, 3 + column, 6 + column,
                qx, qy, qz, qw, vx, vy, vz);
        float beamX = out[column];
        float beamY = out[3 + column];
        float beamZ = out[6 + column];
        out[column] = beamX;
        out[3 + column] = beamZ;
        out[6 + column] = -beamY;
    }

    private static void storeQuaternionAxis(float[] out, int column,
                                            float qx, float qy, float qz, float qw,
                                            float vx, float vy, float vz) {
        Utility.rotateVectorByInverseQuaternion(
                out, column, 3 + column, 6 + column,
                qx, qy, qz, qw, vx, vy, vz);
    }

    private static float length(float x, float y, float z) {
        return (float) Math.sqrt(x * x + y * y + z * z);
    }

    private float functionValue(SoftBodyVehicle vehicle, int prop, float steeringDegrees) {
        String name = function[prop];
        double raw = switch (name == null ? "nop" : name) {
            case "nop" -> 0.0;
            case "steering" -> steeringDegrees;
            case "throttle" -> vehicle.electrics.get(ElectricSignals.THROTTLE_INPUT);
            case "brake" -> vehicle.electrics.get(ElectricSignals.BRAKE_INPUT);
            case "parkingbrake" -> vehicle.electrics.get(ElectricSignals.PARKING_BRAKE_INPUT);
            case "clutch" -> vehicle.electrics.get(ElectricSignals.CLUTCH_INPUT);
            default -> vehicle.electrics.get(name);
        };
        float transformed = (float) ((raw + offset[prop]) * multiplier[prop]);
        return Math.max(min[prop], Math.min(max[prop], transformed));
    }

    private static float steeringWheelDegrees(SoftBodyVehicle vehicle, float[] x, float[] y, float[] z) {
        HydroActuatorController linear = vehicle.hydros.controls;
        HydroActuatorController torsional = vehicle.torsionHydros.controls;
        for (int pass = 0; pass < 2; pass++) {
            boolean requireAuthoredLock = pass == 0;
            for (int i = 0; i < vehicle.hydros.count; i++) {
                if (!isSteeringControl(linear, i)
                        || (requireAuthoredLock && !hasSteeringLock(linear, i))) continue;
                int beam = vehicle.hydros.beamIndex[i];
                if (beam < 0 || beam >= vehicle.normalBeams.count || vehicle.normalBeams.broken[beam]) continue;
                float baseLength = vehicle.normalBeams.baseRestLength[beam];
                if (!(baseLength > 1.0e-8f)) continue;
                int a = vehicle.normalBeams.node1[beam], b = vehicle.normalBeams.node2[beam];
                float actualRatio = distance(x, y, z, a, b) / baseLength;
                return actuatorDegrees(linear, i, actualRatio);
            }

            for (int i = 0; i < vehicle.torsionHydros.count; i++) {
                if (!isSteeringControl(torsional, i)
                        || (requireAuthoredLock && !hasSteeringLock(torsional, i))) continue;
                int bar = vehicle.torsionHydros.torsionBarIndex[i];
                if (bar < 0 || bar >= vehicle.torsionbars.count || vehicle.torsionbars.broken[bar]) continue;
                float actualAngle = torsionAngle(x, y, z,
                        vehicle.torsionbars.node1[bar], vehicle.torsionbars.node2[bar],
                        vehicle.torsionbars.node3[bar], vehicle.torsionbars.node4[bar]);
                if (!Float.isFinite(actualAngle)) continue;
                float actualOffset = wrapRadians(actualAngle - vehicle.torsionbars.baseRestAngle[bar]);
                return actuatorDegrees(torsional, i, actualOffset);
            }
        }
        return 0.0f;
    }

    private static boolean isSteeringControl(HydroActuatorController controls, int index) {
        return ElectricSignals.STEERING_INPUT.equals(controls.inputSource[index]);
    }

    private static boolean hasSteeringLock(HydroActuatorController controls, int index) {
        return Float.isFinite(controls.steeringWheelLock[index]) && controls.steeringWheelLock[index] > 0.0f;
    }

    private static float actuatorDegrees(HydroActuatorController controls, int index, float actualState) {
        float center = controls.center[index];
        float normalized;
        if (actualState >= center) {
            normalized = safeDivide(actualState - center, controls.outLimit[index] - center);
        } else {
            normalized = -safeDivide(center - actualState, center - controls.inLimit[index]);
        }
        normalized = Math.max(-1.0f, Math.min(1.0f, normalized));
        float steeringInput = controls.inputFactor[index] < 0.0f ? -normalized : normalized;
        float lock = controls.steeringWheelLock[index];
        if (!Float.isFinite(lock) || lock <= 0.0f) lock = DEFAULT_STEERING_WHEEL_LOCK_DEGREES;
        // BeamNG's steering prop signal is opposite to positive driver input.
        return -steeringInput * lock;
    }

    public void clear() {
        count = 0;
    }

    private void ensureCapacity() {
        if (count < function.length) return;
        int size = function.length * 2;
        function = Arrays.copyOf(function, size);
        meshIndex = Utility.expand(meshIndex, size);
        refNode = Utility.expand(refNode, size);
        xNode = Utility.expand(xNode, size);
        yNode = Utility.expand(yNode, size);
        baseRotationX = Utility.expand(baseRotationX, size);
        baseRotationY = Utility.expand(baseRotationY, size);
        baseRotationZ = Utility.expand(baseRotationZ, size);
        baseRotationGlobalX = Utility.expand(baseRotationGlobalX, size);
        baseRotationGlobalY = Utility.expand(baseRotationGlobalY, size);
        baseRotationGlobalZ = Utility.expand(baseRotationGlobalZ, size);
        hasBaseRotationGlobal = Utility.expand(hasBaseRotationGlobal, size);
        rotationX = Utility.expand(rotationX, size);
        rotationY = Utility.expand(rotationY, size);
        rotationZ = Utility.expand(rotationZ, size);
        translationX = Utility.expand(translationX, size);
        translationY = Utility.expand(translationY, size);
        translationZ = Utility.expand(translationZ, size);
        min = Utility.expand(min, size);
        max = Utility.expand(max, size);
        offset = Utility.expand(offset, size);
        multiplier = Utility.expand(multiplier, size);
        translationUseMeters = Utility.expand(translationUseMeters, size);
        originLocalX = Utility.expand(originLocalX, size);
        originLocalY = Utility.expand(originLocalY, size);
        originLocalZ = Utility.expand(originLocalZ, size);
        originBound = Utility.expand(originBound, size);
        baseTranslationX = Utility.expand(baseTranslationX, size);
        baseTranslationY = Utility.expand(baseTranslationY, size);
        baseTranslationZ = Utility.expand(baseTranslationZ, size);
        hasBaseTranslation = Utility.expand(hasBaseTranslation, size);
        baseTranslationGlobalX = Utility.expand(baseTranslationGlobalX, size);
        baseTranslationGlobalY = Utility.expand(baseTranslationGlobalY, size);
        baseTranslationGlobalZ = Utility.expand(baseTranslationGlobalZ, size);
        hasBaseTranslationGlobal = Utility.expand(hasBaseTranslationGlobal, size);
    }

    private static float distance(float[] x, float[] y, float[] z, int a, int b) {
        float dx = x[b] - x[a], dy = y[b] - y[a], dz = z[b] - z[a];
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    private static float inverseSqrt(float value) {
        return 1.0f / (float) Math.sqrt(value);
    }

    private static float safeDivide(float numerator, float denominator) {
        return Math.abs(denominator) <= 1.0e-8f ? 0.0f : numerator / denominator;
    }

    private static float wrapRadians(float angle) {
        while (angle > Math.PI) angle -= (float) (Math.PI * 2.0);
        while (angle < -Math.PI) angle += (float) (Math.PI * 2.0);
        return angle;
    }

    private static float torsionAngle(float[] x, float[] y, float[] z, int n1, int n2, int n3, int n4) {
        float b1x = x[n2] - x[n1], b1y = y[n2] - y[n1], b1z = z[n2] - z[n1];
        float b2x = x[n3] - x[n2], b2y = y[n3] - y[n2], b2z = z[n3] - z[n2];
        float b3x = x[n4] - x[n3], b3y = y[n4] - y[n3], b3z = z[n4] - z[n3];
        float c1x = b1y * b2z - b1z * b2y, c1y = b1z * b2x - b1x * b2z, c1z = b1x * b2y - b1y * b2x;
        float c2x = b2y * b3z - b2z * b3y, c2y = b2z * b3x - b2x * b3z, c2z = b2x * b3y - b2y * b3x;
        float b2Length = (float) Math.sqrt(b2x * b2x + b2y * b2y + b2z * b2z);
        if (b2Length <= 1.0e-8f) return Float.NaN;
        float crossX = c1y * c2z - c1z * c2y;
        float crossY = c1z * c2x - c1x * c2z;
        float crossZ = c1x * c2y - c1y * c2x;
        float sin = (crossX * b2x + crossY * b2y + crossZ * b2z) / b2Length;
        float cos = c1x * c2x + c1y * c2y + c1z * c2z;
        return (float) Math.atan2(sin, cos);
    }

    private static void setSyntheticFrame(float[] x, float[] y, float[] z, int base,
                                          float cx, float cy, float cz,
                                          float xx, float xy, float xz,
                                          float yx, float yy, float yz) {
        x[base] = cx; y[base] = cy; z[base] = cz;
        x[base + 1] = cx + xx; y[base + 1] = cy + xy; z[base + 1] = cz + xz;
        x[base + 2] = cx + yx; y[base + 2] = cy + yy; z[base + 2] = cz + yz;
    }

}
