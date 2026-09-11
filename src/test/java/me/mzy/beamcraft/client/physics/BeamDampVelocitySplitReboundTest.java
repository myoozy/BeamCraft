package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Coverage for {@code beamDampVelocitySplitRebound}.
 *
 * <p>BeamNG only replaces the common {@code beamDampVelocitySplit} while the beam
 * lengthens, where it decides whether {@code beamDampReboundFast} applies instead
 * of {@code beamDampRebound}. Compression keeps using the common split, and an
 * unauthored rebound split falls straight back to it.
 *
 * <p>The solver cases use massless nodes, a zero spring and a beam sitting at its
 * rest length, so {@code nodes.forceX[0]} is exactly {@code relVel * activeDamp}
 * for the selected channel.
 */
class BeamDampVelocitySplitReboundTest {

    private static final float DT = 1.0f / PhysicsWorld.invPhysicsDT;
    private static final float EPS = 1.0e-2f;

    private static final float DAMP = 10.0f;
    private static final float DAMP_FAST = 100.0f;
    private static final float DAMP_REBOUND = 20.0f;
    private static final float DAMP_REBOUND_FAST = 200.0f;

    private static final float COMMON_SPLIT = 1.0f;
    private static final float REBOUND_SPLIT = 5.0f;

    // --- parsing -----------------------------------------------------------

    @Test
    void unauthoredReboundSplitFallsBackToTheCommonSplit() {
        BoundedBeamContainer beams = parse("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamDampVelocitySplit": 2.0},
                  ["a", "b"]
                ]
                """);

        assertEquals(2.0f, beams.dampVelocitySplit[0], EPS);
        assertEquals(2.0f, beams.dampVelocitySplitRebound[0], EPS,
                "an unauthored rebound split must reuse beamDampVelocitySplit");
    }

    @Test
    void reboundSplitCanBeAuthoredWithoutACommonSplit() {
        BoundedBeamContainer beams = parse("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamDampVelocitySplitRebound": 4.0},
                  ["a", "b"]
                ]
                """);

        assertEquals(Float.MAX_VALUE, beams.dampVelocitySplit[0], EPS,
                "the common split stays unbounded");
        assertEquals(4.0f, beams.dampVelocitySplitRebound[0], EPS);
    }

    @Test
    void emptyScopedReboundSplitRestoresTheCommonSplit() {
        BoundedBeamContainer beams = parse("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamDampVelocitySplit": 2.0,
                   "beamDampVelocitySplitRebound": 7.0},
                  ["a", "b"],
                  {"beamDampVelocitySplitRebound": ""},
                  ["a", "b"]
                ]
                """);

        assertEquals(2, beams.count);
        assertEquals(7.0f, beams.dampVelocitySplitRebound[0], EPS);
        assertEquals(2.0f, beams.dampVelocitySplitRebound[1], EPS,
                "an empty value cancels the scope and restores the fallback");
        assertEquals(2.0f, beams.dampVelocitySplit[1], EPS,
                "the common scoped split is untouched by the empty rebound value");
    }

    @Test
    void inlineReboundSplitOverridesTheScopedValue() {
        BoundedBeamContainer beams = parse("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamDampVelocitySplit": 2.0,
                   "beamDampVelocitySplitRebound": 7.0},
                  ["a", "b", {"beamDampVelocitySplitRebound": 9.0}]
                ]
                """);

        assertEquals(9.0f, beams.dampVelocitySplitRebound[0], EPS);
        assertEquals(2.0f, beams.dampVelocitySplit[0], EPS);
    }

    @Test
    void inlineEmptyReboundSplitRestoresTheFallback() {
        BoundedBeamContainer beams = parse("""
                [
                  ["id1:", "id2:"],
                  {"beamType": "|BOUNDED", "beamDampVelocitySplit": 2.0,
                   "beamDampVelocitySplitRebound": 7.0},
                  ["a", "b", {"beamDampVelocitySplitRebound": ""}]
                ]
                """);

        assertEquals(2.0f, beams.dampVelocitySplitRebound[0], EPS);
    }

    // --- solver ------------------------------------------------------------

    @Test
    void withoutAReboundSplitTheCommonSplitDecidesReboundToo() {
        assertEquals(3.0f * DAMP_REBOUND_FAST, axialForce(3.0f, -1.0f),
                EPS, "3 m/s exceeds the common 1 m/s split, so beamDampReboundFast applies");
    }

    @Test
    void slowReboundStillUsesTheReboundChannel() {
        assertEquals(0.5f * DAMP_REBOUND, axialForce(0.5f, REBOUND_SPLIT), EPS);
        assertEquals(-0.5f * DAMP, axialForce(-0.5f, REBOUND_SPLIT), EPS,
                "slow compression keeps the plain beamDamp");
    }

    @Test
    void reboundSplitOnlyAffectsTheLengtheningDirection() {
        assertEquals(3.0f * DAMP_REBOUND, axialForce(3.0f, REBOUND_SPLIT), EPS);
        assertEquals(3.0f * DAMP_REBOUND_FAST, axialForce(3.0f, -1.0f), EPS);
        assertEquals(-3.0f * DAMP_FAST, axialForce(-3.0f, REBOUND_SPLIT), EPS,
                "the compression side is identical with and without the override");
        assertEquals(-3.0f * DAMP_FAST, axialForce(-3.0f, -1.0f), EPS);
    }

    /**
     * Builds a massless two-node bounded beam at rest length, applies an axial
     * velocity to node 2 and returns the resulting force on node 1.
     */
    private static float axialForce(float relVel, float reboundSplit) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", 1.0f, 0.0f, 0.0f));
        vehicle.addBeam(TestBeamBuilder.bounded()
                .damp(DAMP)
                .dampSplits(COMMON_SPLIT, reboundSplit)
                .dampChannels(DAMP_FAST, DAMP_REBOUND, DAMP_REBOUND_FAST)
                .build());

        vehicle.nodes.velX[1] = relVel;
        vehicle.solveInternalForces(DT, 1.0f);
        return vehicle.nodes.forceX[0];
    }

    private static BoundedBeamContainer parse(String json) {
        SoftBodyVehicle vehicle = new SoftBodyVehicle(null);
        vehicle.addNode(TestBeamBuilder.masslessNode("a", 0.0f, 0.0f, 0.0f));
        vehicle.addNode(TestBeamBuilder.masslessNode("b", 1.0f, 0.0f, 0.0f));
        JBeamParser.parseBeams(rows(json), vehicle, partEntry());
        return vehicle.boundedBeams;
    }

    private static JsonArray rows(String json) {
        return JsonParser.parseString(json).getAsJsonArray();
    }

    private static JBeamAssembler.PartEntry partEntry() {
        return new JBeamAssembler.PartEntry(
                new JsonObject(), 1, "test", new JBeamAssembler.TransformContext(), Map.of());
    }
}
