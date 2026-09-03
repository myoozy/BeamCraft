package me.mzy.beamcraft.client.input;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.config.BeamCraftConfig;
import me.mzy.beamcraft.client.physics.PhysicsWorld;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.util.InputUtil;

/** Polls configured vehicle controls and owns their edge-triggered state. */
public final class VehicleInputHandler {
    private final InputUtil.Key steerLeft;
    private final InputUtil.Key steerRight;
    private final InputUtil.Key throttle;
    private final InputUtil.Key brake;
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
        steerLeft = resolve(configured.steerLeft, defaults.steerLeft, "steerLeft");
        steerRight = resolve(configured.steerRight, defaults.steerRight, "steerRight");
        throttle = resolve(configured.throttle, defaults.throttle, "throttle");
        brake = resolve(configured.brake, defaults.brake, "brake");
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
        float steeringValue = steeringValue(pressed(window, steerLeft), pressed(window, steerRight));
        float throttleValue = pressed(window, throttle) ? 1.0f : 0.0f;
        float brakeValue = pressed(window, brake) ? 1.0f : 0.0f;
        float clutchValue = pressed(window, clutch) ? 1.0f : 0.0f;
        boolean starterPressed = pressed(window, starter);

        if (resetPressed && !resetWasPressed) {
            resetVehiclesAtPlayer(client, world);
        }

        for (SoftBodyVehicle vehicle : world.vehicles) {
            ElectricBus electrics = vehicle.electrics;
            electrics.set(ElectricSignals.STEERING_INPUT, steeringValue);
            electrics.set(ElectricSignals.THROTTLE_INPUT, throttleValue);
            electrics.set(ElectricSignals.BRAKE_INPUT, brakeValue);
            electrics.set(ElectricSignals.CLUTCH_INPUT, clutchValue);
            electrics.set(ElectricSignals.STARTER_INPUT, starterPressed ? 1.0 : 0.0);
            if (shiftUpPressed && !shiftUpWasPressed) {
                incrementEvent(electrics, ElectricSignals.SHIFT_UP_EVENT);
            }
            if (shiftDownPressed && !shiftDownWasPressed) {
                incrementEvent(electrics, ElectricSignals.SHIFT_DOWN_EVENT);
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

    static float steeringValue(boolean leftPressed, boolean rightPressed) {
        return (rightPressed ? 1.0f : 0.0f) - (leftPressed ? 1.0f : 0.0f);
    }

    private static void incrementEvent(ElectricBus electrics, String signal) {
        electrics.set(signal, electrics.get(signal) + 1.0);
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
