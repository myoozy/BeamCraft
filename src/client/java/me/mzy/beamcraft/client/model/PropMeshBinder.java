/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see
 * LICENSES/bCDDL-1.1.txt.
 *
 * Adapted from BeamNG.drive rigid-prop placement behavior. Java adaptation
 * and modifications contributed by M1AO and BeamCraft contributors.
 */
package me.mzy.beamcraft.client.model;

import me.mzy.beamcraft.client.physics.FlexbodyContainer;
import me.mzy.beamcraft.client.physics.JBeamAssembler;
import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.PropContainer;
import me.mzy.beamcraft.client.physics.PropRuntime;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;

/** Binds one rigid prop mesh to its three render-only frame nodes. */
final class PropMeshBinder {
    private PropMeshBinder() {
    }

    static int bind(FlexbodyContainer flex, SoftBodyVehicle vehicle, NodeContainer nodes,
                    int mesh, int prop, DaeMeshLoader.RawGeometry geom, int ptr) {
        PropContainer props = vehicle.props;
        float[] basis = new float[9];
        if (!PropRuntime.buildReferenceBasis(
                nodes.baseX, nodes.baseY, nodes.baseZ,
                props.refNode[prop], props.xNode[prop], props.yNode[prop], basis)) {
            flex.meshName[mesh] = "";
            return ptr;
        }

        JBeamAssembler.TransformContext slotContext = flex.slotContext[mesh];
        double[] daePivot = transformDaePoint(geom.originX, geom.originY, geom.originZ, slotContext);
        float pivotX = (float) daePivot[0];
        float pivotY = (float) daePivot[2];
        float pivotZ = (float) -daePivot[1];

        int ref = props.refNode[prop];
        float refX = nodes.baseX[ref], refY = nodes.baseY[ref], refZ = nodes.baseZ[ref];
        float bx0 = basis[0], bx1 = basis[3], bx2 = basis[6];
        float by0 = basis[1], by1 = basis[4], by2 = basis[7];
        float bz0 = basis[2], bz1 = basis[5], bz2 = basis[8];

        double[] objectAxisX = transformDaeDirection(geom.axisXX, geom.axisXY, geom.axisXZ, slotContext);
        double[] objectAxisY = transformDaeDirection(geom.axisYX, geom.axisYY, geom.axisYZ, slotContext);
        float[] objectBasis = {
                (float) objectAxisX[0], (float) objectAxisY[0], 0.0f,
                (float) objectAxisX[2], (float) objectAxisY[2], 0.0f,
                (float) -objectAxisX[1], (float) -objectAxisY[1], 0.0f
        };
        if (!orthonormalizeBasisColumns(objectBasis)) {
            flex.meshName[mesh] = "";
            return ptr;
        }
        float mx0 = objectBasis[0], mx1 = objectBasis[3], mx2 = objectBasis[6];
        float my0 = objectBasis[1], my1 = objectBasis[4], my2 = objectBasis[7];
        float mz0 = objectBasis[2], mz1 = objectBasis[5], mz2 = objectBasis[8];

        float[] orientedBasis = new float[9];
        boolean orientationReady = props.hasBaseRotationGlobal[prop]
                ? PropRuntime.buildGlobalPropOrientation(
                nodes.baseX, nodes.baseY, nodes.baseZ, vehicle.cameras.refNodes(),
                props.baseRotationGlobalX[prop], props.baseRotationGlobalY[prop], props.baseRotationGlobalZ[prop],
                0.0f, 0.0f, 0.0f, orientedBasis, new float[9])
                : PropRuntime.buildPropOrientation(
                nodes.baseX, nodes.baseY, nodes.baseZ,
                props.refNode[prop], props.xNode[prop], props.yNode[prop],
                props.baseRotationX[prop], props.baseRotationY[prop], props.baseRotationZ[prop],
                0.0f, 0.0f, 0.0f, orientedBasis);
        if (!orientationReady) {
            flex.meshName[mesh] = "";
            return ptr;
        }

        float targetOriginX = pivotX, targetOriginY = pivotY, targetOriginZ = pivotZ;
        if (props.hasBaseTranslationGlobal[prop]) {
            double[] translated = transformDaePoint(
                    props.baseTranslationGlobalX[prop],
                    props.baseTranslationGlobalY[prop],
                    props.baseTranslationGlobalZ[prop],
                    slotContext);
            targetOriginX = (float) translated[0];
            targetOriginY = (float) translated[2];
            targetOriginZ = (float) -translated[1];
        } else if (props.hasBaseTranslation[prop]) {
            targetOriginX = refX
                    + bx0 * props.baseTranslationX[prop]
                    + by0 * props.baseTranslationY[prop]
                    + bz0 * props.baseTranslationZ[prop];
            targetOriginY = refY
                    + bx1 * props.baseTranslationX[prop]
                    + by1 * props.baseTranslationY[prop]
                    + bz1 * props.baseTranslationZ[prop];
            targetOriginZ = refZ
                    + bx2 * props.baseTranslationX[prop]
                    + by2 * props.baseTranslationY[prop]
                    + bz2 * props.baseTranslationZ[prop];
        }

        float originDx = targetOriginX - refX;
        float originDy = targetOriginY - refY;
        float originDz = targetOriginZ - refZ;
        props.originLocalX[prop] = dot(originDx, originDy, originDz, bx0, bx1, bx2);
        props.originLocalY[prop] = dot(originDx, originDy, originDz, by0, by1, by2);
        props.originLocalZ[prop] = dot(originDx, originDy, originDz, bz0, bz1, bz2);
        props.originBound[prop] = true;

        int syntheticBase = props.syntheticNodeBase(nodes.count, prop);
        for (int vertex = 0; vertex < geom.vertexCount; vertex++, ptr++) {
            double[] transformed = transformDaePoint(
                    geom.positions[vertex * 3], geom.positions[vertex * 3 + 1], geom.positions[vertex * 3 + 2],
                    slotContext);
            float vx = (float) transformed[0];
            float vy = (float) transformed[2];
            float vz = (float) -transformed[1];
            float dx = vx - pivotX, dy = vy - pivotY, dz = vz - pivotZ;
            float localX = dot(dx, dy, dz, mx0, mx1, mx2);
            float localY = dot(dx, dy, dz, my0, my1, my2);
            float localZ = dot(dx, dy, dz, mz0, mz1, mz2);

            flex.vCenterNode[ptr] = syntheticBase;
            flex.vVxNode[ptr] = syntheticBase + 1;
            flex.vVyNode[ptr] = syntheticBase + 2;
            flex.vVzNode[ptr] = -1;
            flex.vWeightX[ptr] = localX;
            flex.vWeightY[ptr] = localY;
            flex.vWeightZ[ptr] = localZ;
            flex.vUseCrossZ[ptr] = true;
            flex.vRestCrossLength[ptr] = 1.0f;

            float normalX = 0.0f, normalY = 0.0f, normalZ = 1.0f;
            if (geom.normals != null && vertex * 3 + 2 < geom.normals.length) {
                double[] transformedNormal = transformDaeDirection(
                        geom.normals[vertex * 3], geom.normals[vertex * 3 + 1], geom.normals[vertex * 3 + 2],
                        slotContext);
                normalX = (float) transformedNormal[0];
                normalY = (float) transformedNormal[2];
                normalZ = (float) -transformedNormal[1];
            }
            flex.vNormWeightX[ptr] = dot(normalX, normalY, normalZ, mx0, mx1, mx2);
            flex.vNormWeightY[ptr] = dot(normalX, normalY, normalZ, my0, my1, my2);
            flex.vNormWeightZ[ptr] = dot(normalX, normalY, normalZ, mz0, mz1, mz2);

            flex.skinnedPosX[ptr] = targetOriginX
                    + orientedBasis[0] * localX + orientedBasis[1] * localY + orientedBasis[2] * localZ;
            flex.skinnedPosY[ptr] = targetOriginY
                    + orientedBasis[3] * localX + orientedBasis[4] * localY + orientedBasis[5] * localZ;
            flex.skinnedPosZ[ptr] = targetOriginZ
                    + orientedBasis[6] * localX + orientedBasis[7] * localY + orientedBasis[8] * localZ;

            if (geom.uvs != null && vertex * 2 + 1 < geom.uvs.length) {
                flex.uvU[ptr] = geom.uvs[vertex * 2];
                flex.uvV[ptr] = geom.uvs[vertex * 2 + 1];
            }
        }
        return ptr;
    }

    private static double[] transformDaePoint(double x, double y, double z,
                                              JBeamAssembler.TransformContext context) {
        return context == null ? new double[]{x, y, z} : context.transformNode(x, y, z);
    }

    private static double[] transformDaeDirection(double x, double y, double z,
                                                  JBeamAssembler.TransformContext context) {
        if (context == null) return new double[]{x, y, z};
        double[] direction = context.transformNode(x, y, z);
        double[] origin = context.transformNode(0.0, 0.0, 0.0);
        return new double[]{
                direction[0] - origin[0],
                direction[1] - origin[1],
                direction[2] - origin[2]
        };
    }

    private static float dot(float ax, float ay, float az, float bx, float by, float bz) {
        return ax * bx + ay * by + az * bz;
    }

    private static boolean orthonormalizeBasisColumns(float[] basis) {
        float xx = basis[0], xy = basis[3], xz = basis[6];
        float xLengthSquared = xx * xx + xy * xy + xz * xz;
        if (xLengthSquared <= 1.0e-10f) return false;
        float inverseXLength = 1.0f / (float) Math.sqrt(xLengthSquared);
        xx *= inverseXLength; xy *= inverseXLength; xz *= inverseXLength;

        float yx = basis[1], yy = basis[4], yz = basis[7];
        float projection = xx * yx + xy * yy + xz * yz;
        yx -= projection * xx; yy -= projection * xy; yz -= projection * xz;
        float yLengthSquared = yx * yx + yy * yy + yz * yz;
        if (yLengthSquared <= 1.0e-10f) return false;
        float inverseYLength = 1.0f / (float) Math.sqrt(yLengthSquared);
        yx *= inverseYLength; yy *= inverseYLength; yz *= inverseYLength;

        basis[0] = xx; basis[3] = xy; basis[6] = xz;
        basis[1] = yx; basis[4] = yy; basis[7] = yz;
        basis[2] = xy * yz - xz * yy;
        basis[5] = xz * yx - xx * yz;
        basis[8] = xx * yy - xy * yx;
        return true;
    }
}
