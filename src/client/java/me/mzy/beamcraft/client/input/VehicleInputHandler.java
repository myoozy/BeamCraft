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

import java.util.ArrayList;
import java.util.List;

/** Polls configured vehicle controls and owns their edge-triggered state. */
public final class VehicleInputHandler {
    private static final double MAX_ENTER_DISTANCE_SQUARED = 36.0;

    private final List<InputUtil.Key> exitVehicle;
    private final List<AxisKey> steering;
    private final List<AxisKey> throttle;
    private final List<AxisKey> brake;
    private final List<AxisKey> clutch;
    private final List<InputUtil.Key> starter;
    private final List<InputUtil.Key> shiftUp;
    private final List<InputUtil.Key> shiftDown;
    private final List<InputUtil.Key> rangeBoxToggle;
    private final List<InputUtil.Key> resetVehicle;

    private final LinearRamp throttleRamp;
    private final LinearRamp brakeRamp;
    private final LinearRamp clutchRamp;

    private boolean shiftUpWasPressed;
    private boolean shiftDownWasPressed;
    private boolean rangeBoxToggleWasPressed;
    private boolean resetWasPressed;
    private boolean exitWasPressed;
    private SoftBodyVehicle controlledVehicle;

    public VehicleInputHandler(BeamCraftConfig.Input input) {
        BeamCraftConfig.Input defaults = BeamCraftConfig.Input.defaults();
        BeamCraftConfig.Input configured = input == null ? new BeamCraftConfig.Input() : input;
        exitVehicle = resolve(configured.exitVehicle, defaults.exitVehicle, "exitVehicle");
        steering = resolve(configured.steering, defaults.steering, "steering");
        throttle = resolve(configured.throttle, defaults.throttle, "throttle");
        brake = resolve(configured.brake, defaults.brake, "brake");
        clutch = resolve(configured.clutch, defaults.clutch, "clutch");
        starter = resolve(configured.starter, defaults.starter, "starter");
        shiftUp = resolve(configured.shiftUp, defaults.shiftUp, "shiftUp");
        shiftDown = resolve(configured.shiftDown, defaults.shiftDown, "shiftDown");
        rangeBoxToggle = resolve(configured.rangeBoxToggle, defaults.rangeBoxToggle, "rangeBoxToggle");
        resetVehicle = resolve(configured.resetVehicle, defaults.resetVehicle, "resetVehicle");

        throttleRamp = ramp(configured.throttle, defaults.throttle);
        brakeRamp = ramp(configured.brake, defaults.brake);
        clutchRamp = ramp(configured.clutch, defaults.clutch);
    }

    public void tick(MinecraftClient client, float deltaTime) {
        long window = client.getWindow().getHandle();
        boolean gameplayInput = client.currentScreen == null;
        boolean exitPressed = gameplayInput && pressed(window, exitVehicle);
        boolean resetPressed = gameplayInput && pressed(window, resetVehicle);
        boolean shiftUpPressed = gameplayInput && pressed(window, shiftUp);
        boolean shiftDownPressed = gameplayInput && pressed(window, shiftDown);
        boolean rangeBoxTogglePressed = gameplayInput && pressed(window, rangeBoxToggle);

        SoftBodyVehicle nextControlled = findControlledVehicle(client);
        if (controlledVehicle != nextControlled) {
            releaseContinuousInputs(controlledVehicle);
            resetContinuousState();
            controlledVehicle = nextControlled;
        }

        if (controlledVehicle == null) {
            updateEdgeState(exitPressed, resetPressed, shiftUpPressed, shiftDownPressed, rangeBoxTogglePressed);
            return;
        }

        if (exitPressed && !exitWasPressed) {
            requestExitControlledVehicle(client);
        }

        if (!gameplayInput) {
            resetContinuousState();
        }
        // Steering hydros already apply their JBeam rates at the physics substep frequency.
        // A second 20 Hz ramp here would turn their target into visible staircase motion.
        float steeringValue = gameplayInput
                ? axisValue(window, steering)
                : 0.0f;
        float throttleValue = gameplayInput ? throttleRamp.update(axisValue(window, throttle), deltaTime) : 0.0f;
        float brakeValue = gameplayInput ? brakeRamp.update(axisValue(window, brake), deltaTime) : 0.0f;
        float clutchValue = gameplayInput ? clutchRamp.update(axisValue(window, clutch), deltaTime) : 0.0f;
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
        if (rangeBoxTogglePressed && !rangeBoxToggleWasPressed) {
            incrementEvent(electrics, ElectricSignals.RANGE_BOX_TOGGLE_EVENT);
        }

        updateEdgeState(exitPressed, resetPressed, shiftUpPressed, shiftDownPressed, rangeBoxTogglePressed);
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

    private static boolean pressed(long window, List<InputUtil.Key> keys) {
        for (InputUtil.Key key : keys) {
            if (pressed(window, key)) {
                return true;
            }
        }
        return false;
    }

    private static boolean pressed(long window, InputUtil.Key key) {
        if (key.getCategory() == InputUtil.Type.MOUSE) {
            return GLFW.glfwGetMouseButton(window, key.getCode()) == GLFW.GLFW_PRESS;
        }
        return InputUtil.isKeyPressed(window, key.getCode());
    }

    private static float axisValue(long window, List<AxisKey> keys) {
        float positive = 0.0f;
        float negative = 0.0f;
        for (AxisKey key : keys) {
            if (!pressed(window, key.key())) {
                continue;
            }
            positive = Math.max(positive, key.value());
            negative = Math.min(negative, key.value());
        }
        return Math.max(-1.0f, Math.min(1.0f, positive + negative));
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

    private void resetContinuousState() {
        throttleRamp.reset();
        brakeRamp.reset();
        clutchRamp.reset();
    }

    private void updateEdgeState(boolean exitPressed, boolean resetPressed,
                                 boolean shiftUpPressed, boolean shiftDownPressed,
                                 boolean rangeBoxTogglePressed) {
        exitWasPressed = exitPressed;
        resetWasPressed = resetPressed;
        shiftUpWasPressed = shiftUpPressed;
        shiftDownWasPressed = shiftDownPressed;
        rangeBoxToggleWasPressed = rangeBoxTogglePressed;
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

    private static List<InputUtil.Key> resolve(BeamCraftConfig.KeyBinding configured,
                                                BeamCraftConfig.KeyBinding fallback,
                                                String action) {
        List<String> configuredKeys = configured == null ? null : configured.keys;
        List<String> translationKeys = configuredKeys == null || configuredKeys.isEmpty()
                ? fallback.keys : configuredKeys;
        List<InputUtil.Key> resolved = new ArrayList<>(translationKeys.size());
        for (String translationKey : translationKeys) {
            if (translationKey == null || translationKey.isBlank()) {
                continue;
            }
            try {
                resolved.add(InputUtil.fromTranslationKey(translationKey.trim()));
            } catch (IllegalArgumentException exception) {
                BeamCraft.LOGGER.warn("Invalid BeamCraft input key '{}' for {}; ignoring it",
                        translationKey, action);
            }
        }
        if (!resolved.isEmpty() || translationKeys == fallback.keys) {
            return List.copyOf(resolved);
        }
        BeamCraft.LOGGER.warn("No valid BeamCraft input keys configured for {}; using defaults", action);
        return resolve(fallback, fallback, action);
    }

    private static List<AxisKey> resolve(BeamCraftConfig.DirectionalBinding configured,
                                         BeamCraftConfig.DirectionalBinding fallback,
                                         String action) {
        List<BeamCraftConfig.AxisKey> configuredKeys = configured == null ? null : configured.keys;
        List<BeamCraftConfig.AxisKey> source = configuredKeys == null || configuredKeys.isEmpty()
                ? fallback.keys : configuredKeys;
        List<AxisKey> resolved = new ArrayList<>(source.size());
        for (BeamCraftConfig.AxisKey binding : source) {
            if (binding == null || binding.key == null || binding.key.isBlank()
                    || !Double.isFinite(binding.value) || binding.value == 0.0) {
                continue;
            }
            try {
                InputUtil.Key key = InputUtil.fromTranslationKey(binding.key.trim());
                resolved.add(new AxisKey(key, (float) Math.max(-1.0, Math.min(1.0, binding.value))));
            } catch (IllegalArgumentException exception) {
                BeamCraft.LOGGER.warn("Invalid BeamCraft input key '{}' for {}; ignoring it",
                        binding.key, action);
            }
        }
        if (!resolved.isEmpty() || source == fallback.keys) {
            return List.copyOf(resolved);
        }
        BeamCraft.LOGGER.warn("No valid BeamCraft input keys configured for {}; using defaults", action);
        return resolve(fallback, fallback, action);
    }

    private static LinearRamp ramp(BeamCraftConfig.AxisBinding configured,
                                   BeamCraftConfig.AxisBinding fallback) {
        double riseTime = configured != null && configured.riseTime != null
                ? configured.riseTime : fallback.riseTime;
        double fallTime = configured != null && configured.fallTime != null
                ? configured.fallTime : fallback.fallTime;
        return new LinearRamp(riseTime, fallTime);
    }

    private record AxisKey(InputUtil.Key key, float value) {
    }
}
