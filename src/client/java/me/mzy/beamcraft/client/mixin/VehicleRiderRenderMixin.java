package me.mzy.beamcraft.client.mixin;

import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import net.minecraft.client.network.AbstractClientPlayerEntity;
import net.minecraft.client.render.VertexConsumerProvider;
import net.minecraft.client.render.entity.PlayerEntityRenderer;
import net.minecraft.client.util.math.MatrixStack;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(PlayerEntityRenderer.class)
public class VehicleRiderRenderMixin {
    @Inject(
            method = "render(Lnet/minecraft/client/network/AbstractClientPlayerEntity;FFLnet/minecraft/client/util/math/MatrixStack;Lnet/minecraft/client/render/VertexConsumerProvider;I)V",
            at = @At("HEAD"),
            cancellable = true
    )
    private void beamcraft$hideVehicleRider(
            AbstractClientPlayerEntity player, float yaw, float tickDelta,
            MatrixStack matrices, VertexConsumerProvider vertices, int light, CallbackInfo ci) {
        if (player.getVehicle() instanceof PhysicsVehicleEntity) {
            ci.cancel();
        }
    }
}
