package me.mzy.beamcraft.client.render;

import net.minecraft.util.math.MathHelper;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class VehicleDamageWobbleTest {

    @Test
    void matchesVanillaVehicleWobbleCurve() {
        float expected = MathHelper.sin(9.5f) * 9.5f * 19.5f / 10.0f;

        assertEquals(expected,
                PhysicsVehicleRenderer.damageWobbleAngleDegrees(10, 20.0f, 1, 0.5f),
                1.0e-6f);
        assertEquals(-expected,
                PhysicsVehicleRenderer.damageWobbleAngleDegrees(10, 20.0f, -1, 0.5f),
                1.0e-6f);
    }

    @Test
    void stopsWhenTicksOrStrengthExpire() {
        assertEquals(0.0f,
                PhysicsVehicleRenderer.damageWobbleAngleDegrees(0, 20.0f, 1, 0.0f));
        assertEquals(0.0f,
                PhysicsVehicleRenderer.damageWobbleAngleDegrees(10, 0.25f, 1, 0.5f));
    }
}
