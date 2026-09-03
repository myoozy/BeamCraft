package me.mzy.beamcraft.client.mixin;

import me.mzy.beamcraft.client.input.VehicleInputHandler;
import net.minecraft.client.MinecraftClient;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(MinecraftClient.class)
public class VehicleUseMixin {
    @Inject(method = "doItemUse", at = @At("HEAD"), cancellable = true)
    private void beamcraft$consumeVehicleEnterUse(CallbackInfo ci) {
        // doItemUse only runs when the vanilla Use key is freshly pressed (or held with a
        // free cooldown). When the press is a BeamCraft vehicle-enter we send the ride
        // request here, at the real press edge, and cancel the vanilla use action.
        if (VehicleInputHandler.tryVehicleEnterFromUse((MinecraftClient) (Object) this)) {
            ci.cancel();
        }
    }
}
