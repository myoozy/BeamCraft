package me.mzy.beamcraft.client.mixin;

import me.mzy.beamcraft.client.input.VehicleInputHandler;
import net.minecraft.client.input.KeyboardInput;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(KeyboardInput.class)
public class VehicleKeyboardInputMixin {
    @Inject(method = "tick", at = @At("TAIL"))
    private void beamcraft$suppressVanillaVehicleInput(boolean slowDown, float slowDownFactor, CallbackInfo ci) {
        VehicleInputHandler.suppressVanillaRidingInput((KeyboardInput) (Object) this);
    }
}
