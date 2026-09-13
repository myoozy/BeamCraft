package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Coverage for BeamNG's {@code precompressionRange}.
 *
 * <p>It is a metric length delta applied on spawn, and authoring it <em>replaces</em>
 * the {@code beamPrecompression} multiplier outright - including an explicit
 * {@code 0} or a negative delta. Because those are all legal values, presence is
 * tracked explicitly rather than inferred from the number: an absent key keeps the
 * active scope, an empty value cancels the scope and restores the multiplier.
 */
class BeamPrecompressionRangeTest {

    private static final float EPS = 1.0e-4f;

    // --- override semantics ------------------------------------------------

    @Test
    void authoredRangeReplacesThePrecompressionMultiplier() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.9, "precompressionRange": 0.1},
                  ["a", "b"]
                ]
                """);

        assertEquals(1.1f, beams.targetRestLength[0], EPS,
                "1 m + 0.1 m, not 1 m * 0.9 + 0.1 m");
    }

    @Test
    void explicitZeroRangeOverridesTheMultiplier() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.9, "precompressionRange": 0.0},
                  ["a", "b"]
                ]
                """);

        assertEquals(1.0f, beams.targetRestLength[0], EPS,
                "an explicit 0 range means the spawn length itself");
    }

    @Test
    void negativeRangeShortensTheSpawnLength() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 2.0, "precompressionRange": -0.05},
                  ["a", "b"]
                ]
                """);

        assertEquals(0.95f, beams.targetRestLength[0], EPS,
                "a negative range is a valid compression delta");
    }

    @Test
    void absentRangeKeepsTheMultiplier() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.85},
                  ["a", "b"]
                ]
                """);

        assertEquals(0.85f, beams.targetRestLength[0], EPS);
    }

    @Test
    void authoredRangeWithoutAnAuthoredMultiplierIgnoresTheDefault() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"precompressionRange": 0.25},
                  ["a", "b"]
                ]
                """);

        assertEquals(1.25f, beams.targetRestLength[0], EPS,
                "the range wins even though beamPrecompression defaults to 1");
    }

    // --- scoped / inline presence ------------------------------------------

    @Test
    void inlineRangeOverridesTheScopedValue() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.5, "precompressionRange": 0.2},
                  ["a", "b", {"precompressionRange": 0.4}]
                ]
                """);

        assertEquals(1.4f, beams.targetRestLength[0], EPS);
    }

    @Test
    void emptyScopedRangeRestoresTheMultiplier() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.5, "precompressionRange": 0.2},
                  ["a", "b"],
                  {"precompressionRange": ""},
                  ["a", "b"]
                ]
                """);

        assertEquals(2, beams.count);
        assertEquals(1.2f, beams.targetRestLength[0], EPS);
        assertEquals(0.5f, beams.targetRestLength[1], EPS,
                "an empty range cancels the override and restores beamPrecompression");
    }

    @Test
    void emptyInlineRangeRestoresTheMultiplier() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.5, "precompressionRange": 0.2},
                  ["a", "b", {"precompressionRange": ""}]
                ]
                """);

        assertEquals(0.5f, beams.targetRestLength[0], EPS);
    }

    @Test
    void emptyRangeWithNoAuthoredMultiplierFallsBackToUnity() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"precompressionRange": 0.3},
                  ["a", "b"],
                  {"precompressionRange": ""},
                  ["a", "b"]
                ]
                """);

        assertEquals(1.0f, beams.targetRestLength[1], EPS);
    }

    // --- timed precompression ---------------------------------------------

    @Test
    void timedPrecompressionInterpolatesTowardTheRangeTarget() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.5, "precompressionRange": 0.2,
                   "beamPrecompressionTime": 1.0},
                  ["a", "b"]
                ]
                """);

        assertEquals(1.2f, beams.targetRestLength[0], EPS);
        assertEquals(1.0f, beams.restLength[0], EPS, "spawn starts at the node distance");
        assertEquals(1.0f, beams.precompTimeTotal[0], EPS);

        beams.updatePrecompression(0.5f);
        assertEquals(1.1f, beams.restLength[0], EPS, "halfway through the ramp");

        beams.updatePrecompression(0.5f);
        assertEquals(1.2f, beams.restLength[0], EPS, "the ramp settles on the range target");
    }

    @Test
    void resetRestartsTheTimedRangeRamp() {
        BeamContainer beams = parseNormalBeam("""
                [
                  ["id1:", "id2:"],
                  {"beamPrecompression": 0.5, "precompressionRange": 0.2,
                   "beamPrecompressionTime": 1.0},
                  ["a", "b"]
                ]
                """);

        beams.updatePrecompression(1.0f);
        assertEquals(1.2f, beams.restLength[0], EPS);

        beams.reset();
        assertEquals(1.0f, beams.restLength[0], EPS, "reset returns to the node distance");
        assertEquals(1.0f, beams.precompTimer[0], EPS, "and restarts the timer");
    }

    // --- L-beams -----------------------------------------------------------

    @Test
    void lBeamRangeMatchesAnEquivalentMultiplier() {
        // target = node12Dist + range == node12Dist * 1.5, so the target cosine must match.
        SoftBodyVehicle ranged = parseLBeam("""
                [
                  ["id1:", "id2:", "id3:"],
                  ["a", "b", {"beamType": "|LBEAM", "id3:": "c",
                              "precompressionRange": 0.5}]
                ]
                """);
        SoftBodyVehicle scaled = parseLBeam("""
                [
                  ["id1:", "id2:", "id3:"],
                  ["a", "b", {"beamType": "|LBEAM", "id3:": "c",
                              "beamPrecompression": 1.5}]
                ]
                """);

        assertEquals(1, ranged.lBeams.count, "ranged row must build an L-beam");
        assertEquals(1, scaled.lBeams.count, "scaled row must build an L-beam");
        assertEquals(scaled.lBeams.targetCosTheta[0], ranged.lBeams.targetCosTheta[0], EPS,
                "the L-beam target angle follows the same range override");
        assertEquals(1.5f, ranged.lBeams.restLength[0], EPS,
                "the L-beam's rest length carries the metric range override too");

        // A 1 m spawn, a 1 m third-node leg and a sqrt(2) m opposite leg.
        double expected = (1.0 + 2.0 - 1.5 * 1.5) / (2.0 * Math.sqrt(2.0));
        assertEquals((float) expected, ranged.lBeams.targetCosTheta[0], EPS,
                "the target angle is the cosine of the 1.5 m ranged distance");
        assertEquals((float) expected, ranged.lBeams.restCosTheta[0], EPS,
                "without a ramp the spawn angle is already the target");
        assertTrue(Math.abs(expected - 0.70710678) > 0.4,
                "the ranged target must be visibly different from the 1 m spawn angle");
    }

    // --- helpers -----------------------------------------------------------

    private static BeamContainer parseNormalBeam(String json) {
        SoftBodyVehicle vehicle = twoNodeVehicle();
        JBeamParser.parseBeams(rows(json), vehicle, partEntry());
        return vehicle.normalBeams;
    }

    private static SoftBodyVehicle parseLBeam(String json) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", 1.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("c", 0.0f, 1.0f, 0.0f));
        JBeamParser.parseBeams(rows(json), vehicle, partEntry());
        return vehicle;
    }

    private static SoftBodyVehicle twoNodeVehicle() {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", 1.0f, 0.0f, 0.0f));
        return vehicle;
    }

    private static JsonArray rows(String json) {
        return JsonParser.parseString(json).getAsJsonArray();
    }

    private static JBeamAssembler.PartEntry partEntry() {
        return new JBeamAssembler.PartEntry(
                new JsonObject(), 1, "test", new JBeamAssembler.TransformContext(), Map.of());
    }
}
