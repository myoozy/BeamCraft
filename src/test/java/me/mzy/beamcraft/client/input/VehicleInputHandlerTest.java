package me.mzy.beamcraft.client.input;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VehicleInputHandlerTest {
    @Test
    void steeringUsesTheHydroExpectedSign() {
        assertEquals(-1.0f, VehicleInputHandler.steeringValue(true, false));
        assertEquals(1.0f, VehicleInputHandler.steeringValue(false, true));
        assertEquals(0.0f, VehicleInputHandler.steeringValue(false, false));
        assertEquals(0.0f, VehicleInputHandler.steeringValue(true, true));
    }
}
