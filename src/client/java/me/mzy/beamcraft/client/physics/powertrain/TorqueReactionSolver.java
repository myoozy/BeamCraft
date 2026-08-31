package me.mzy.beamcraft.client.physics.powertrain;

import me.mzy.beamcraft.client.physics.NodeContainer;

/** Converts a requested body torque into zero-net-force node forces. */
final class TorqueReactionSolver {
    private TorqueReactionSolver() {
    }

    static void apply(NodeContainer nodes, int[] reactionNodes, int offset, int count,
                      float torqueX, float torqueY, float torqueZ) {
        if (count < 2) return;

        double totalMass = 0.0, cx = 0.0, cy = 0.0, cz = 0.0;
        for (int i = 0; i < count; i++) {
            int node = reactionNodes[offset + i];
            if (node < 0 || node >= nodes.count) continue;
            double mass = Math.max(0.0, nodes.mass[node]);
            totalMass += mass;
            cx += nodes.posX[node] * mass;
            cy += nodes.posY[node] * mass;
            cz += nodes.posZ[node] * mass;
        }
        if (totalMass < 1e-9) return;
        cx /= totalMass; cy /= totalMass; cz /= totalMass;

        double ixx = 0.0, iyy = 0.0, izz = 0.0;
        double ixy = 0.0, ixz = 0.0, iyz = 0.0;
        for (int i = 0; i < count; i++) {
            int node = reactionNodes[offset + i];
            if (node < 0 || node >= nodes.count) continue;
            double mass = Math.max(0.0, nodes.mass[node]);
            double x = nodes.posX[node] - cx;
            double y = nodes.posY[node] - cy;
            double z = nodes.posZ[node] - cz;
            ixx += mass * (y * y + z * z);
            iyy += mass * (x * x + z * z);
            izz += mass * (x * x + y * y);
            ixy -= mass * x * y;
            ixz -= mass * x * z;
            iyz -= mass * y * z;
        }

        double regularization = Math.max(1e-9, (ixx + iyy + izz) * 1e-8);
        ixx += regularization;
        iyy += regularization;
        izz += regularization;

        double c00 = iyy * izz - iyz * iyz;
        double c01 = ixz * iyz - ixy * izz;
        double c02 = ixy * iyz - ixz * iyy;
        double c11 = ixx * izz - ixz * ixz;
        double c12 = ixy * ixz - ixx * iyz;
        double c22 = ixx * iyy - ixy * ixy;
        double determinant = ixx * c00 + ixy * c01 + ixz * c02;
        if (Math.abs(determinant) < 1e-18) return;
        double invDet = 1.0 / determinant;
        double alphaX = (c00 * torqueX + c01 * torqueY + c02 * torqueZ) * invDet;
        double alphaY = (c01 * torqueX + c11 * torqueY + c12 * torqueZ) * invDet;
        double alphaZ = (c02 * torqueX + c12 * torqueY + c22 * torqueZ) * invDet;

        for (int i = 0; i < count; i++) {
            int node = reactionNodes[offset + i];
            if (node < 0 || node >= nodes.count) continue;
            double mass = Math.max(0.0, nodes.mass[node]);
            double x = nodes.posX[node] - cx;
            double y = nodes.posY[node] - cy;
            double z = nodes.posZ[node] - cz;
            nodes.forceX[node] += (float) (mass * (alphaY * z - alphaZ * y));
            nodes.forceY[node] += (float) (mass * (alphaZ * x - alphaX * z));
            nodes.forceZ[node] += (float) (mass * (alphaX * y - alphaY * x));
        }
    }
}
