/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see LICENSES/bCDDL-1.1.txt.
 * Adapted from BeamNG.drive differential.lua. Java adaptation and modifications
 * contributed by M1AO and BeamCraft contributors. See SOURCE_PROVENANCE.md.
 */
package me.mzy.beamcraft.client.physics.powertrain;

import java.util.Locale;

/** Passive and commanded coupling torque between a differential's two output domains. */
public final class DifferentialSolver {
    private static final float MIN_INERTIA = 1.0e-7f;

    private DifferentialSolver() {
    }

    public static float solve(float dt, float outputAV1, float outputAV2,
                              float outputInertia1, float outputInertia2,
                              float inputTorque, DifferentialContainer state, int differential) {
        if (dt <= 0.0f) {
            clear(state, differential);
            return 0.0f;
        }
        float slip = outputAV1 - outputAV2;
        state.inputTorque[differential] = inputTorque;
        return switch (state.activeMode[differential]) {
            case DifferentialContainer.MODE_LSD -> solveLsd(
                    dt, slip, outputInertia1, outputInertia2, inputTorque, state, differential);
            case DifferentialContainer.MODE_VISCOUS -> solveViscous(
                    dt, slip, outputInertia1, outputInertia2, state, differential);
            case DifferentialContainer.MODE_LOCKED -> solveSpringLock(
                    dt, slip, outputInertia1, outputInertia2,
                    state.lockCapacity[differential], true, state, differential);
            case DifferentialContainer.MODE_ACTIVE_LOCK -> solveSpringLock(
                    dt, slip, outputInertia1, outputInertia2,
                    state.activeLockCapacity[differential] * state.activeLockCoef[differential],
                    true, state, differential);
            default -> solveOpen(dt, slip, outputInertia1, outputInertia2,
                    inputTorque, state, differential);
        };
    }

    private static float solveOpen(float dt, float slip, float inertia1, float inertia2,
                                   float inputTorque, DifferentialContainer state, int differential) {
        state.diffAngle[differential] = 0.0f;
        state.viscousTorque[differential] = 0.0f;
        float torque = 0.034f * Math.abs(inputTorque) * Math.clamp(slip, -0.1f, 0.1f);
        torque = clampToNoSlipImpulse(dt, slip, inertia1, inertia2, torque);
        state.lockTorque[differential] = torque;
        return torque;
    }

    private static float solveLsd(float dt, float slip, float inertia1, float inertia2,
                                  float inputTorque, DifferentialContainer state, int differential) {
        float sensing = inputTorque >= 0.0f
                ? state.lsdLockCoef[differential] : state.lsdRevLockCoef[differential];
        float requestedCapacity = state.lsdPreload[differential]
                + Math.max(0.0f, sensing) * Math.abs(inputTorque)
                + state.activeLockCapacity[differential] * state.activeLockCoef[differential];
        float configuredCapacity = state.lockCapacity[differential];
        float capacity = configuredCapacity > 0.0f
                ? Math.min(configuredCapacity, requestedCapacity) : requestedCapacity;
        state.viscousTorque[differential] = 0.0f;
        return solveSpringLock(dt, slip, inertia1, inertia2, capacity,
                false, state, differential);
    }

    private static float solveViscous(float dt, float slip, float inertia1, float inertia2,
                                      DifferentialContainer state, int differential) {
        state.diffAngle[differential] = 0.0f;
        float magnitude = state.viscousCoef[differential]
                * (float) Math.pow(Math.abs(slip), state.viscousExponent[differential]);
        float target = Math.clamp(Math.copySign(magnitude, slip),
                -state.viscousCapacity[differential], state.viscousCapacity[differential]);
        float alpha = 1.0f - (float) Math.exp(-dt * state.viscousSmoothing[differential]);
        float torque = Math.fma(target - state.viscousTorque[differential],
                Math.clamp(alpha, 0.0f, 1.0f), state.viscousTorque[differential]);
        torque = clampToNoSlipImpulse(dt, slip, inertia1, inertia2, torque);
        state.viscousTorque[differential] = torque;
        state.lockTorque[differential] = torque;
        return torque;
    }

    private static float solveSpringLock(float dt, float slip, float inertia1, float inertia2,
                                         float capacity, boolean damped,
                                         DifferentialContainer state, int differential) {
        capacity = Math.max(0.0f, capacity);
        float spring = Math.max(0.0f, state.lockSpring[differential]);
        if (capacity <= 0.0f || spring <= 0.0f) {
            state.diffAngle[differential] = 0.0f;
            state.lockTorque[differential] = 0.0f;
            return 0.0f;
        }

        float reducedInertia = reducedInertia(inertia1, inertia2);
        float maxAngle = (float) Math.sqrt(capacity / spring);
        float candidateAngle = Math.clamp(state.diffAngle[differential] + slip * dt,
                -maxAngle, maxAngle);
        float springTorque = Math.copySign(candidateAngle * candidateAngle * spring, candidateAngle);
        float damping = damped
                ? 2.0f * Math.max(0.0f, state.lockDampingRatio[differential])
                * (float) Math.sqrt(spring * reducedInertia)
                : 0.0f;
        float torque = Math.clamp(springTorque + damping * slip, -capacity, capacity);
        torque = clampToNoSlipImpulse(dt, slip, inertia1, inertia2, torque);

        // Advance stored wind-up from the post-impulse slip. This keeps the physical
        // spring state while retaining KinetiForge's useful no-overshoot impulse bound.
        float postSlip = slip - dt * torque / reducedInertia;
        state.diffAngle[differential] = Math.clamp(
                state.diffAngle[differential] + postSlip * dt, -maxAngle, maxAngle);
        state.viscousTorque[differential] = 0.0f;
        state.lockTorque[differential] = torque;
        return torque;
    }

    private static float clampToNoSlipImpulse(float dt, float slip, float inertia1,
                                               float inertia2, float torque) {
        float limit = Math.abs(slip) * reducedInertia(inertia1, inertia2) / dt;
        return Math.clamp(torque, -limit, limit);
    }

    private static float reducedInertia(float inertia1, float inertia2) {
        float first = Math.max(inertia1, MIN_INERTIA);
        float second = Math.max(inertia2, MIN_INERTIA);
        return first * second / (first + second);
    }

    public static byte mode(String mode) {
        return switch (mode == null ? "" : mode.toLowerCase(Locale.ROOT)) {
            case "lsd" -> DifferentialContainer.MODE_LSD;
            case "viscous" -> DifferentialContainer.MODE_VISCOUS;
            case "locked", "dually" -> DifferentialContainer.MODE_LOCKED;
            case "activelock" -> DifferentialContainer.MODE_ACTIVE_LOCK;
            case "open", "torquevectoring" -> DifferentialContainer.MODE_OPEN;
            default -> -1;
        };
    }

    public static int modeBit(String mode) {
        byte resolved = mode(mode);
        return resolved < 0 ? 0 : 1 << resolved;
    }

    public static void clear(DifferentialContainer state, int differential) {
        state.inputTorque[differential] = 0.0f;
        state.lockTorque[differential] = 0.0f;
        state.diffAngle[differential] = 0.0f;
        state.viscousTorque[differential] = 0.0f;
    }
}
