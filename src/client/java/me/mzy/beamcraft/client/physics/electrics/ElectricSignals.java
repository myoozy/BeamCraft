package me.mzy.beamcraft.client.physics.electrics;

/** Common signal names shared by input, controllers, and physical consumers. */
public final class ElectricSignals {
    public static final String STEERING_INPUT = "steering_input";
    public static final String THROTTLE_INPUT = "throttle_input";
    public static final String BRAKE_INPUT = "brake_input";
    public static final String PARKING_BRAKE_INPUT = "parking_brake_input";
    public static final String CLUTCH_INPUT = "clutch_input";
    public static final String STARTER_INPUT = "starter_input";
    public static final String SHIFT_UP_EVENT = "shift_up_event";
    public static final String SHIFT_DOWN_EVENT = "shift_down_event";

    private ElectricSignals() {
    }
}
