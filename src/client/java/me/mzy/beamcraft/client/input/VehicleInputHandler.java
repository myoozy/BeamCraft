package me.mzy.beamcraft.client.input;

import me.mzy.beamcraft.BeamCraft;
import me.mzy.beamcraft.client.config.BeamCraftConfig;
import me.mzy.beamcraft.client.ClientVehicleManager;
import me.mzy.beamcraft.client.physics.SoftBodyVehicle;
import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSignals;
import me.mzy.beamcraft.entity.PhysicsVehicleEntity;
import me.mzy.beamcraft.network.VehicleRidePayload;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.input.Input;
import net.minecraft.client.util.InputUtil;
import net.minecraft.entity.Entity;
import net.minecraft.util.hit.EntityHitResult;
import org.lwjgl.glfw.GLFW;

/** Polls configured vehicle controls and owns their edge-triggered state. */
public final class VehicleInputHandler {
    private static final String DEFAULT_EXIT_VEHICLE = "key.keyboard.left.shift";
    private static final String DEFAULT_STEER_LEFT = "key.keyboard.left";
    private static final String DEFAULT_STEER_RIGHT = "key.keyboard.right";
    private static final String DEFAULT_THROTTLE = "key.keyboard.up";
    private static final String DEFAULT_BRAKE = "key.keyboard.down";
    private static final String DEFAULT_CLUTCH = "key.keyboard.c";
    private static final String DEFAULT_STARTER = "key.keyboard.v";
    private static final String DEFAULT_SHIFT_UP = "key.keyboard.x";
    private static final String DEFAULT_SHIFT_DOWN = "key.keyboard.z";
    private static final String DEFAULT_RESET_VEHICLE = "key.keyboard.g";
    private static final double MAX_ENTER_DISTANCE_SQUARED = 36.0;

    private final InputUtil.Key exitVehicle;
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
    private boolean exitWasPressed;
    private SoftBodyVehicle controlledVehicle;

    public VehicleInputHandler(BeamCraftConfig.Input input) {
        BeamCraftConfig.Input configured = input == null ? new BeamCraftConfig.Input() : input;
        exitVehicle = resolve(configured.exitVehicle, DEFAULT_EXIT_VEHICLE, "exitVehicle");
        steerLeft = resolve(configured.steerLeft, DEFAULT_STEER_LEFT, "steerLeft");
        steerRight = resolve(configured.steerRight, DEFAULT_STEER_RIGHT, "steerRight");
        throttle = resolve(configured.throttle, DEFAULT_THROTTLE, "throttle");
        brake = resolve(configured.brake, DEFAULT_BRAKE, "brake");
        clutch = resolve(configured.clutch, DEFAULT_CLUTCH, "clutch");
        starter = resolve(configured.starter, DEFAULT_STARTER, "starter");
        shiftUp = resolve(configured.shiftUp, DEFAULT_SHIFT_UP, "shiftUp");
        shiftDown = resolve(configured.shiftDown, DEFAULT_SHIFT_DOWN, "shiftDown");
        resetVehicle = resolve(configured.resetVehicle, DEFAULT_RESET_VEHICLE, "resetVehicle");
    }

    public void tick(MinecraftClient client) {
        long window = client.getWindow().getHandle();
        boolean gameplayInput = client.currentScreen == null;
        boolean exitPressed = gameplayInput && pressed(window, exitVehicle);
        boolean resetPressed = gameplayInput && pressed(window, resetVehicle);
        boolean shiftUpPressed = gameplayInput && pressed(window, shiftUp);
        boolean shiftDownPressed = gameplayInput && pressed(window, shiftDown);

        SoftBodyVehicle nextControlled = findControlledVehicle(client);
        if (controlledVehicle != nextControlled) {
            releaseContinuousInputs(controlledVehicle);
            controlledVehicle = nextControlled;
        }

        if (controlledVehicle == null) {
            updateEdgeState(exitPressed, resetPressed, shiftUpPressed, shiftDownPressed);
            return;
        }

        if (exitPressed && !exitWasPressed) {
            requestExitControlledVehicle(client);
        }

        float steeringValue = gameplayInput
                ? steeringValue(pressed(window, steerLeft), pressed(window, steerRight))
                : 0.0f;
        float throttleValue = gameplayInput && pressed(window, throttle) ? 1.0f : 0.0f;
        float brakeValue = gameplayInput && pressed(window, brake) ? 1.0f : 0.0f;
        float clutchValue = gameplayInput && pressed(window, clutch) ? 1.0f : 0.0f;
        boolean starterPressed = gameplayInput && pressed(window, starter);

        if (resetPressed && !resetWasPressed) {
            resetControlledVehicleAtPlayer(client, controlledVehicle);
        }

        ElectricBus electrics = controlledVehicle.electrics;
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

        updateEdgeState(exitPressed, resetPressed, shiftUpPressed, shiftDownPressed);
    }

    private static SoftBodyVehicle findControlledVehicle(MinecraftClient client) {
        if (client.player == null) {
            return null;
        }
        Entity ridden = client.player.getVehicle();
        if (!(ridden instanceof PhysicsVehicleEntity)) {
            return null;
        }
        return ClientVehicleManager.getVehicle(ridden.getId());
    }

    private static void requestEnterTargetedVehicle(MinecraftClient client) {
        if (client.player == null || !(client.crosshairTarget instanceof EntityHitResult hit)) {
            return;
        }
        Entity target = hit.getEntity();
        if (!(target instanceof PhysicsVehicleEntity)
                || client.player.squaredDistanceTo(target) > MAX_ENTER_DISTANCE_SQUARED
                || !ClientPlayNetworking.canSend(VehicleRidePayload.ID)) {
            return;
        }
        ClientPlayNetworking.send(new VehicleRidePayload(target.getId(), true));
    }

    private static void requestExitControlledVehicle(MinecraftClient client) {
        if (client.player == null || !(client.player.getVehicle() instanceof PhysicsVehicleEntity vehicle)
                || !ClientPlayNetworking.canSend(VehicleRidePayload.ID)) {
            return;
        }
        ClientPlayNetworking.send(new VehicleRidePayload(vehicle.getId(), false));
    }

    private static void resetControlledVehicleAtPlayer(MinecraftClient client, SoftBodyVehicle vehicle) {
        if (client.player == null) {
            return;
        }
        vehicle.reset();
        vehicle.nodes.rotateNodes(client.player.getYaw(), 0, 0);
    }

    private static boolean pressed(long window, InputUtil.Key key) {
        if (key.getCategory() == InputUtil.Type.MOUSE) {
            return GLFW.glfwGetMouseButton(window, key.getCode()) == GLFW.GLFW_PRESS;
        }
        return InputUtil.isKeyPressed(window, key.getCode());
    }

    private static void releaseContinuousInputs(SoftBodyVehicle vehicle) {
        if (vehicle == null) {
            return;
        }
        ElectricBus electrics = vehicle.electrics;
        electrics.set(ElectricSignals.STEERING_INPUT, 0.0);
        electrics.set(ElectricSignals.THROTTLE_INPUT, 0.0);
        electrics.set(ElectricSignals.BRAKE_INPUT, 0.0);
        electrics.set(ElectricSignals.CLUTCH_INPUT, 0.0);
        electrics.set(ElectricSignals.STARTER_INPUT, 0.0);
    }

    private void updateEdgeState(boolean exitPressed, boolean resetPressed,
                                 boolean shiftUpPressed, boolean shiftDownPressed) {
        exitWasPressed = exitPressed;
        resetWasPressed = resetPressed;
        shiftUpWasPressed = shiftUpPressed;
        shiftDownWasPressed = shiftDownPressed;
    }

    /** Removes vanilla mount movement and sneak-dismount behavior for BeamCraft vehicles. */
    public static void suppressVanillaRidingInput(Input input) {
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.player == null || !(client.player.getVehicle() instanceof PhysicsVehicleEntity)) {
            return;
        }
        input.pressingForward = false;
        input.pressingBack = false;
        input.pressingLeft = false;
        input.pressingRight = false;
        input.movementForward = 0.0f;
        input.movementSideways = 0.0f;
        input.jumping = false;
        input.sneaking = false;
    }

    /**
     * Called at the exact point where Minecraft processes its Use action (right mouse by
     * default). Vehicle entry deliberately follows that vanilla action rather than adding
     * a second configurable BeamCraft binding.
     *
     * <p>Routing the request through the vanilla press itself (instead of a 20 Hz poll of the
     * raw mouse state) makes a fast momentary right-click reliable.
     */
    public static boolean tryVehicleEnterFromUse(MinecraftClient client) {
        if (client.player == null
                || client.player.hasVehicle()
                || client.currentScreen != null
                || !(client.crosshairTarget instanceof EntityHitResult hit)
                || !(hit.getEntity() instanceof PhysicsVehicleEntity)) {
            return false;
        }
        requestEnterTargetedVehicle(client);
        return true;
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
