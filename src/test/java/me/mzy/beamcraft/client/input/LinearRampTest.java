package me.mzy.beamcraft.client.input;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class LinearRampTest {
    @Test
    void reachesFullValueLinearlyUsingRiseTime() {
        LinearRamp ramp = new LinearRamp(0.20, 0.40);

        assertEquals(0.25f, ramp.update(true, 0.05f), 0.0001f);
        assertEquals(0.50f, ramp.update(true, 0.05f), 0.0001f);
        assertEquals(0.75f, ramp.update(true, 0.05f), 0.0001f);
        assertEquals(1.00f, ramp.update(true, 0.05f), 0.0001f);
        assertEquals(1.00f, ramp.update(true, 0.05f), 0.0001f);
    }

    @Test
    void usesIndependentFallTimeAndClampsAtZero() {
        LinearRamp ramp = new LinearRamp(0.0, 0.20);
        assertEquals(1.0f, ramp.update(true, 0.05f), 0.0001f);

        assertEquals(0.75f, ramp.update(false, 0.05f), 0.0001f);
        assertEquals(0.50f, ramp.update(false, 0.05f), 0.0001f);
        assertEquals(0.25f, ramp.update(false, 0.05f), 0.0001f);
        assertEquals(0.00f, ramp.update(false, 0.05f), 0.0001f);
        assertEquals(0.00f, ramp.update(false, 0.05f), 0.0001f);
    }

    @Test
    void invalidTimesRespondImmediatelyAndResetClearsState() {
        LinearRamp ramp = new LinearRamp(Double.NaN, -1.0);

        assertEquals(1.0f, ramp.update(true, 0.05f), 0.0001f);
        ramp.reset();
        assertEquals(0.0f, ramp.value(), 0.0001f);
        assertEquals(0.0f, ramp.update(false, 0.05f), 0.0001f);
    }

    @Test
    void reversingDirectionFallsToZeroBeforeRisingOpposite() {
        LinearRamp ramp = new LinearRamp(0.20, 0.10);
        assertEquals(1.0f, ramp.update(1.0f, 0.20f), 0.0001f);

        assertEquals(0.5f, ramp.update(-1.0f, 0.05f), 0.0001f);
        assertEquals(0.0f, ramp.update(-1.0f, 0.05f), 0.0001f);
        assertEquals(-0.25f, ramp.update(-1.0f, 0.05f), 0.0001f);
    }
}
