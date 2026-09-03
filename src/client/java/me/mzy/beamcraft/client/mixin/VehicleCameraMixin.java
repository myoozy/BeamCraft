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
            beamcraft$trackedVehicleId = Integer.MIN_VALUE;
            return;
        }
        SoftBodyVehicle vehicle = ClientVehicleManager.getVehicle(entity.getId());
        if (vehicle == null) {
            return;
        }
        Vec3d position = VehicleCameraController.resolve(
                (Camera) (Object) this, vehicle, thirdPerson, tickDelta);
        if (position != null) {
            this.setPos(position);
        }

        Float vehicleYaw = VehicleCameraController.resolveVehicleYaw(vehicle, tickDelta);
        if (vehicleYaw == null) {
            beamcraft$trackedVehicleId = Integer.MIN_VALUE;
            return;
        }
        if (beamcraft$trackedVehicleId == entity.getId()) {
            float deltaYaw = MathHelper.wrapDegrees(vehicleYaw - beamcraft$lastVehicleYaw);
            if (Float.isFinite(deltaYaw)) {
                player.prevYaw += deltaYaw;
                player.setYaw(player.getYaw() + deltaYaw);
                Camera camera = (Camera) (Object) this;
                this.setRotation(camera.getYaw() + deltaYaw, camera.getPitch());
            }
        }
        beamcraft$trackedVehicleId = entity.getId();
        beamcraft$lastVehicleYaw = vehicleYaw;
    }
}
