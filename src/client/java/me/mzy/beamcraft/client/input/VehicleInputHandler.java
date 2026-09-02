package me.mzy.beamcraft.client.input;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.config.BeamCraftConfig;
import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.util.InputUtil;

/** Polls configured vehicle controls and owns their edge-triggered state. */
public final class VehicleInputHandler {
    private final InputUtil.Key throttle;
    private final InputUtil.Key clutch;
    private final InputUtil.Key starter;
    private final InputUtil.Key shiftUp;
    private final InputUtil.Key shiftDown;
    private final InputUtil.Key resetVehicle;

    private boolean shiftUpWasPressed;
    private boolean shiftDownWasPressed;
    private boolean resetWasPressed;

    public VehicleInputHandler(BeamCraftConfig.Input input) {
        BeamCraftConfig.Input defaults = new BeamCraftConfig.Input();
        BeamCraftConfig.Input configured = input == null ? defaults : input;
        throttle = resolve(configured.throttle, defaults.throttle, "throttle");
        clutch = resolve(configured.clutch, defaults.clutch, "clutch");
        starter = resolve(configured.starter, defaults.starter, "starter");
        shiftUp = resolve(configured.shiftUp, defaults.shiftUp, "shiftUp");
        shiftDown = resolve(configured.shiftDown, defaults.shiftDown, "shiftDown");
        resetVehicle = resolve(configured.resetVehicle, defaults.resetVehicle, "resetVehicle");
    }

    public void tick(MinecraftClient client, PhysicsWorld world) {
        long window = client.getWindow().getHandle();
        boolean resetPressed = pressed(window, resetVehicle);
        boolean shiftUpPressed = pressed(window, shiftUp);
        boolean shiftDownPressed = pressed(window, shiftDown);
        float throttleValue = pressed(window, throttle) ? 1.0f : 0.0f;
        float clutchValue = pressed(window, clutch) ? 1.0f : 0.0f;
        boolean starterPressed = pressed(window, starter);

        if (resetPressed && !resetWasPressed) {
            resetVehiclesAtPlayer(client, world);
        }

        for (SoftBodyVehicle vehicle : world.vehicles) {
            vehicle.powertrain.setControls(throttleValue, clutchValue, starterPressed);
            if (shiftUpPressed && !shiftUpWasPressed) {
                vehicle.powertrain.requestShiftUp();
            }
            if (shiftDownPressed && !shiftDownWasPressed) {
                vehicle.powertrain.requestShiftDown();
            }
        }

        resetWasPressed = resetPressed;
        shiftUpWasPressed = shiftUpPressed;
        shiftDownWasPressed = shiftDownPressed;
    }

    private static void resetVehiclesAtPlayer(MinecraftClient client, PhysicsWorld world) {
        if (client.player == null) {
            return;
        }
        for (SoftBodyVehicle vehicle : world.vehicles) {
            vehicle.reset();
            vehicle.parentEntity.setPosition(client.player.getX(), client.player.getY() + 1.0, client.player.getZ());
            vehicle.nodes.rotateNodes(client.player.getYaw(), 0, 0);
        }
    }

    private static boolean pressed(long window, InputUtil.Key key) {
        return InputUtil.isKeyPressed(window, key.getCode());
    }

    private static InputUtil.Key resolve(String configured, String fallback, String action) {
        String translationKey = configured == null || configured.isBlank() ? fallback : configured.trim();
        try {
            return InputUtil.fromTranslationKey(translationKey);
        } catch (IllegalArgumentException exception) {
            BeamCraft.LOGGER.warn("Invalid BeamCraft input key '{}' for {}; using '{}'",
                    translationKey, action, fallback);
            return InputUtil.fromTranslationKey(fallback);
        }
    }
}
