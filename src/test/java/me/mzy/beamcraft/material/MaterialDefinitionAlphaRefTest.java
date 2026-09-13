package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Tests the {@code alphaRef} scale: BeamNG writes it as a byte (the stock grille
 * material uses 127, the ETK800 interior glass 235, most materials 0 for "no
 * cutout"), while the render threshold here is a 0-1 fraction. Reading the raw number
 * as a fraction made every value above 1 nonsense — 22 instead of 0.09 — which is
 * harmless only while nothing downstream consumes it.
 */
class MaterialDefinitionAlphaRefTest {

    private static MaterialDefinition def(String json) {
        JsonObject obj = JsonParser.parseString(json).getAsJsonObject();
        MaterialDefinition definition = MaterialDefinition.fromJson("x", obj, "test");
        assertEquals(true, definition != null);
        return definition;
    }

    @Test
    void byteScaleIsNormalisedToAFraction() {
        assertEquals(127f / 255f, def("{\"name\":\"grille\",\"alphaRef\":127}").alphaRef, 1e-6f);
        assertEquals(235f / 255f, def("{\"name\":\"glass\",\"alphaRef\":235}").alphaRef, 1e-6f);
    }

    @Test
    void absentOrZeroMeansNoCutout() {
        assertEquals(0f, def("{\"name\":\"paint\"}").alphaRef);
        assertEquals(0f, def("{\"name\":\"paint\",\"alphaRef\":0}").alphaRef);
    }

    @Test
    void aFractionalThresholdIsLeftAlone() {
        assertEquals(0.5f, def("{\"name\":\"window\",\"alphaRef\":0.5}").alphaRef, 1e-6f);
    }

    @Test
    void normalisingDoesNotChangeWhichPassTheMaterialTakes() {
        MaterialDefinition grille = def("""
                {
                  "name": "grille",
                  "alphaRef": 127,
                  "Stages": [ { "baseColorMap": "grille_b.color.png", "opacityMap": "grille_o.data.png" } ]
                }
                """);

        MaterialRenderPlan plan = MaterialRenderPlanner.plan(grille);

        assertEquals(MaterialRenderPlan.RenderMode.CUTOUT, plan.mode());
        assertEquals(127f / 255f, plan.alphaRef(), 1e-6f);
    }
}
