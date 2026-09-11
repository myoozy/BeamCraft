package me.mzy.beamcraft.client.physics;

import java.util.List;

/**
 * Test-only {@link PhysicsSpecs.BeamSpec} builder holding the official BeamNG
 * defaults, so a test only names the properties it actually exercises.
 *
 * <p>The base spec is a strain-free, unbounded normal beam with zero spring and
 * damping: solving it leaves massless nodes untouched and makes
 * {@code nodes.forceX[0]} exactly the damping force for the configured velocity.
 * Negative sentinels mirror the parser's "unspecified, substitute the fallback"
 * convention.
 */
final class TestBeamBuilder {
    int type = BeamContainer.BEAM_NORMAL;
    String name1 = "a";
    String name2 = "b";
    String name3;

    float spring = 0.0f;
    float damp = 0.0f;
    float dampCutoffHz = -1.0f;
    float deform = Float.MAX_VALUE;
    float strength = Float.MAX_VALUE;

    float precomp = 1.0f;
    float precompRange = 0.0f;
    boolean precompRangeDefined = false;
    float precompTime = 0.0f;

    float shortBound = 1.0f;
    float longBound = 1.0f;
    float shortBoundRange = -1.0f;
    float longBoundRange = -1.0f;
    float boundZone = 1.0f;
    float limitSpring = 0.0f;
    float limitDamp = 0.0f;
    float limitDampRebound = -1.0f;

    float dampVelSplit = -1.0f;
    float dampVelSplitRebound = -1.0f;
    float dampFast = -1.0f;
    float dampRebound = -1.0f;
    float dampReboundFast = -1.0f;

    float springExpansion = 0.0f;
    float dampExpansion = 0.0f;
    float transitionZone = 0.0f;
    float deformLimitStress = Float.MAX_VALUE;

    static TestBeamBuilder normal() {
        return new TestBeamBuilder();
    }

    static TestBeamBuilder bounded() {
        TestBeamBuilder builder = new TestBeamBuilder();
        builder.type = BeamContainer.BEAM_BOUNDED;
        return builder;
    }

    static TestBeamBuilder lBeam(String name3) {
        TestBeamBuilder builder = new TestBeamBuilder();
        builder.type = BeamContainer.BEAM_LBEAM;
        builder.name3 = name3;
        return builder;
    }

    TestBeamBuilder type(int value) {
        type = value;
        return this;
    }

    TestBeamBuilder spring(float value) {
        spring = value;
        return this;
    }

    TestBeamBuilder damp(float value) {
        damp = value;
        return this;
    }

    TestBeamBuilder dampCutoffHz(float value) {
        dampCutoffHz = value;
        return this;
    }

    TestBeamBuilder deform(float value) {
        deform = value;
        return this;
    }

    TestBeamBuilder strength(float value) {
        strength = value;
        return this;
    }

    TestBeamBuilder precomp(float value) {
        precomp = value;
        return this;
    }

    TestBeamBuilder precompRange(float value) {
        precompRange = value;
        precompRangeDefined = true;
        return this;
    }

    TestBeamBuilder precompTime(float value) {
        precompTime = value;
        return this;
    }

    TestBeamBuilder bounds(float shortBound, float longBound, float boundZone) {
        this.shortBound = shortBound;
        this.longBound = longBound;
        this.boundZone = boundZone;
        return this;
    }

    TestBeamBuilder dampSplits(float velocitySplit, float velocitySplitRebound) {
        this.dampVelSplit = velocitySplit;
        this.dampVelSplitRebound = velocitySplitRebound;
        return this;
    }

    TestBeamBuilder dampChannels(float fast, float rebound, float reboundFast) {
        this.dampFast = fast;
        this.dampRebound = rebound;
        this.dampReboundFast = reboundFast;
        return this;
    }

    TestBeamBuilder expansion(float springExpansion, float dampExpansion, float transitionZone) {
        this.springExpansion = springExpansion;
        this.dampExpansion = dampExpansion;
        this.transitionZone = transitionZone;
        return this;
    }

    PhysicsSpecs.BeamSpec build() {
        return new PhysicsSpecs.BeamSpec(
                type, name1, name2, name3,
                List.of(), Float.POSITIVE_INFINITY,
                List.of(), 0, false,
                spring, damp, dampCutoffHz, deform, strength,
                precomp, precompRange, precompRangeDefined, precompTime,
                shortBound, longBound, shortBoundRange, longBoundRange, boundZone,
                limitSpring, limitDamp, limitDampRebound,
                dampVelSplit, dampVelSplitRebound, dampFast, dampRebound, dampReboundFast,
                springExpansion, dampExpansion, transitionZone, deformLimitStress);
    }

    /** Massless node, so a solve never integrates it nor applies gravity. */
    static PhysicsSpecs.NodeSpec masslessNode(String name, float x, float y, float z) {
        return new PhysicsSpecs.NodeSpec(name, x, y, z, 0.0f, 1.0f, 1.0f, 0, false, false, List.of());
    }

    static PhysicsSpecs.NodeSpec massNode(String name, float x, float y, float z, float mass) {
        return new PhysicsSpecs.NodeSpec(name, x, y, z, mass, 1.0f, 1.0f, 0, false, false, List.of());
    }
}
