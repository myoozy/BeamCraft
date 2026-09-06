package me.mzy.beamcraft.client.mixin;

import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.render.VehicleCameraController;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.client.network.ClientPlayerEntity;
import net.minecraft.client.render.Camera;
import net.minecraft.entity.Entity;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.BlockView;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(Camera.class)
public abstract class VehicleCameraMixin {
    @Unique
    private int beamcraft$trackedVehicleId = Integer.MIN_VALUE;
    @Unique
    private float beamcraft$lastVehicleYaw;
    @Unique
    private float beamcraft$vehicleYawOffset;

    @Shadow
    protected abstract void setPos(Vec3d pos);

    @Shadow
    protected abstract void setRotation(float yaw, float pitch);

    @Inject(method = "update", at = @At("TAIL"))
    private void beamcraft$placeVehicleCamera(
            BlockView area, Entity focusedEntity, boolean thirdPerson,
            boolean inverseView, float tickDelta, CallbackInfo ci) {
        if (!(focusedEntity instanceof ClientPlayerEntity player)
                || !(player.getVehicle() instanceof PhysicsVehicleEntity entity)) {
            beamcraft$finishTracking(focusedEntity);
            return;
        }
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) {
            return;
        }

        Float vehicleYaw = VehicleCameraController.resolveVehicleYaw(vehicle, tickDelta);
        if (vehicleYaw == null) {
            beamcraft$trackedVehicleId = Integer.MIN_VALUE;
            beamcraft$vehicleYawOffset = 0.0f;
        } else if (beamcraft$trackedVehicleId != entity.getId()) {
            beamcraft$trackedVehicleId = entity.getId();
            beamcraft$lastVehicleYaw = vehicleYaw;
            beamcraft$vehicleYawOffset = 0.0f;
        } else {
            float deltaYaw = MathHelper.wrapDegrees(vehicleYaw - beamcraft$lastVehicleYaw);
            if (Float.isFinite(deltaYaw)) {
                beamcraft$vehicleYawOffset = MathHelper.wrapDegrees(
                        beamcraft$vehicleYawOffset + deltaYaw);
            }
            beamcraft$lastVehicleYaw = vehicleYaw;
        }

        Camera camera = (Camera) (Object) this;
        if (vehicleYaw != null) {
            // Camera.update has already applied the player's interpolated manual
            // yaw. Add the accumulated vehicle rotation only to this render
            // camera; feeding it back into player.prevYaw/yaw every frame makes
            // vanilla interpolate the automatic motion a second time.
            this.setRotation(
                    camera.getYaw() + beamcraft$vehicleYawOffset,
                    camera.getPitch()
            );
        }

        // Exterior placement depends on Camera#getHorizontalPlane. Resolve it
        // only after applying this frame's automatic vehicle-yaw delta, just as
        // vanilla already does for manual mouse yaw before this tail injection.
        Vec3d position = VehicleCameraController.resolve(
                (Camera) (Object) this, vehicle, thirdPerson, tickDelta);
        if (position != null) {
            this.setPos(position);
        }
    }

    @Unique
    private void beamcraft$finishTracking(Entity focusedEntity) {
        if (beamcraft$trackedVehicleId != Integer.MIN_VALUE
                && focusedEntity instanceof ClientPlayerEntity player
                && Float.isFinite(beamcraft$vehicleYawOffset)) {
            // Preserve the visible world-facing direction when dismounting,
            // while keeping the per-frame riding path free of player-state
            // feedback.
            player.prevYaw += beamcraft$vehicleYawOffset;
            player.setYaw(player.getYaw() + beamcraft$vehicleYawOffset);
            Camera camera = (Camera) (Object) this;
            this.setRotation(
                    camera.getYaw() + beamcraft$vehicleYawOffset,
                    camera.getPitch()
            );
        }
        beamcraft$trackedVehicleId = Integer.MIN_VALUE;
        beamcraft$vehicleYawOffset = 0.0f;
    }
}
