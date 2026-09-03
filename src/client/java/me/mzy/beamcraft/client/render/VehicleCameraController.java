package me.mzy.beamcraft.client.render;

import me.mzy.beamcraft.client.physics.NodeContainer;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.VehicleCameraData;
import net.minecraft.client.render.Camera;
import net.minecraft.util.math.Vec3d;
import org.joml.Vector3f;

/** Resolves the best available BeamNG camera into Minecraft world space. */
public final class VehicleCameraController {
    private VehicleCameraController() {
    }

    public static Vec3d resolve(Camera camera, SoftBodyVehicle vehicle, boolean thirdPerson, float tickDelta) {
        VehicleCameraData.InternalCamera internal = vehicle.cameras.preferredInternal();
        if (!thirdPerson && internal != null) {
            return sampleWorldNode(vehicle, internal.nodeIndex(), tickDelta);
        }
        return resolveExterior(camera, vehicle, tickDelta);
    }

    /** Returns Minecraft yaw derived from BeamNG's deforming ref/back axis. */
    public static Float resolveVehicleYaw(SoftBodyVehicle vehicle, float tickDelta) {
        VehicleCameraData.RefNodes refs = vehicle.cameras.refNodes();
        if (refs == null) {
            return null;
        }
        float[] ref = new float[3];
        float[] back = new float[3];
        if (!sampleLocalNode(vehicle, refs.ref(), tickDelta, ref)
                || !sampleLocalNode(vehicle, refs.back(), tickDelta, back)) {
            return null;
        }
        double forwardX = ref[0] - back[0];
        double forwardZ = ref[2] - back[2];
        if (forwardX * forwardX + forwardZ * forwardZ <= 1.0e-8) {
            return null;
        }
        return (float) Math.toDegrees(Math.atan2(-forwardX, forwardZ));
    }

    private static Vec3d resolveExterior(Camera camera, SoftBodyVehicle vehicle, float tickDelta) {
        if (vehicle.nodes.count == 0) {
            return null;
        }

        Bounds bounds = bounds(vehicle, tickDelta);
        Vec3d center = bounds.center();
        VehicleCameraData.ExternalCamera metadata = vehicle.cameras.exteriorFallback();
        if (metadata != null) {
            VehicleCameraData.RefNodes refs = vehicle.cameras.refNodes();
            if (refs != null) {
                Vec3d ref = sampleWorldNode(vehicle, refs.ref(), tickDelta);
                Vec3d back = sampleWorldNode(vehicle, refs.back(), tickDelta);
                Vec3d left = sampleWorldNode(vehicle, refs.left(), tickDelta);
                Vec3d up = sampleWorldNode(vehicle, refs.up(), tickDelta);
                if (ref != null && back != null && left != null && up != null) {
                    Vec3d leftVector = left.subtract(ref);
                    Vec3d backVector = back.subtract(ref);
                    Vec3d upVector = up.subtract(ref);
                    if (leftVector.lengthSquared() > 1.0e-8
                            && backVector.lengthSquared() > 1.0e-8
                            && upVector.lengthSquared() > 1.0e-8) {
                        center = ref
                                .add(leftVector.normalize().multiply(metadata.offsetX()))
                                .add(upVector.normalize().multiply(metadata.offsetY()))
                                .add(backVector.normalize().multiply(-metadata.offsetZ()));
                    }
                }
            } else {
                center = center.add(metadata.offsetX(), metadata.offsetY(), metadata.offsetZ());
            }
        } else {
            center = center.add(0.0, Math.max(1.0, bounds.height() * 0.75), 0.0);
        }

        double distance = metadata != null
                ? Math.max(1.0, metadata.distance())
                : Math.max(3.0, bounds.horizontalLength() * 1.25);
        Vector3f forward = camera.getHorizontalPlane();
        return center.subtract(forward.x * distance, forward.y * distance, forward.z * distance);
    }

    private static Vec3d sampleWorldNode(SoftBodyVehicle vehicle, int node, float tickDelta) {
        float[] local = new float[3];
        if (!sampleLocalNode(vehicle, node, tickDelta, local)) {
            return null;
        }
        return new Vec3d(
                vehicle.parentEntity.getX() + local[0],
                vehicle.parentEntity.getY() + local[1],
                vehicle.parentEntity.getZ() + local[2]
        );
    }

    private static boolean sampleLocalNode(SoftBodyVehicle vehicle, int node, float tickDelta, float[] out) {
        if (node < 0 || node >= vehicle.nodes.count) {
            return false;
        }
        if (!vehicle.renderTimeline.sampleNode(System.nanoTime(), node, out)) {
            NodeContainer nodes = vehicle.nodes;
            out[0] = nodes.posX[node];
            out[1] = nodes.posY[node];
            out[2] = nodes.posZ[node];
        }
        return true;
    }

    private static Bounds bounds(SoftBodyVehicle vehicle, float tickDelta) {
        float[] point = new float[3];
        sampleLocalNode(vehicle, 0, tickDelta, point);
        double minBaseX = vehicle.nodes.baseX[0];
        double minBaseY = vehicle.nodes.baseY[0];
        double minBaseZ = vehicle.nodes.baseZ[0];
        double maxBaseX = minBaseX;
        double maxBaseY = minBaseY;
        double maxBaseZ = minBaseZ;
        double weightedX = 0.0;
        double weightedY = 0.0;
        double weightedZ = 0.0;
        double totalMass = 0.0;

        for (int node = 0; node < vehicle.nodes.count; node++) {
            sampleLocalNode(vehicle, node, tickDelta, point);
            minBaseX = Math.min(minBaseX, vehicle.nodes.baseX[node]);
            minBaseY = Math.min(minBaseY, vehicle.nodes.baseY[node]);
            minBaseZ = Math.min(minBaseZ, vehicle.nodes.baseZ[node]);
            maxBaseX = Math.max(maxBaseX, vehicle.nodes.baseX[node]);
            maxBaseY = Math.max(maxBaseY, vehicle.nodes.baseY[node]);
            maxBaseZ = Math.max(maxBaseZ, vehicle.nodes.baseZ[node]);
            double mass = Math.max(0.0, vehicle.nodes.mass[node]);
            weightedX += point[0] * mass;
            weightedY += point[1] * mass;
            weightedZ += point[2] * mass;
            totalMass += mass;
        }

        Vec3d entity = vehicle.parentEntity.getPos();
        Vec3d center = totalMass > 1.0e-8
                ? entity.add(weightedX / totalMass, weightedY / totalMass, weightedZ / totalMass)
                : entity;
        return new Bounds(
                center,
                maxBaseY - minBaseY,
                Math.max(maxBaseX - minBaseX, maxBaseZ - minBaseZ));
    }

    private record Bounds(Vec3d center, double height, double horizontalLength) {}
}
