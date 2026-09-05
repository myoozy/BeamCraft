package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.utility.Utility;

/** Controls the reference-length ratio of BeamNG-style linear hydros. */
public final class HydroContainer {
    private static final int INITIAL_CAPACITY = 16;
    private static final float MIN_RATIO = 1.0e-8f;

    public int count;
    public int[] beamIndex = new int[INITIAL_CAPACITY];
    public final HydroActuatorController controls = new HydroActuatorController();

    public int addHydro(PhysicsSpecs.HydroSpec spec, int linkedBeamIndex,
                        BeamContainer beams, ElectricBus electrics) {
        ensureCapacity();
        int index = count++;
        beamIndex[index] = linkedBeamIndex;
        int controlIndex = controls.add(spec, 1.0f, electrics);
        if (controlIndex != index) {
            throw new IllegalStateException("Hydro actuator indices are out of sync");
        }
        applyRatio(beams, linkedBeamIndex, controls.state[index]);
        return index;
    }

    /** Advances all actuator states by one physics substep. */
    public void update(float dt, BeamContainer beams, ElectricSnapshot electrics) {
        for (int i = 0; i < count; i++) {
            int linkedBeam = beamIndex[i];
            if (linkedBeam < 0 || linkedBeam >= beams.count || beams.broken[linkedBeam]) {
                continue;
            }
            applyRatio(beams, linkedBeam, controls.update(i, dt, electrics));
        }
    }

    public void reset(BeamContainer beams) {
        controls.reset();
        for (int i = 0; i < count; i++) {
            int linkedBeam = beamIndex[i];
            if (linkedBeam >= 0 && linkedBeam < beams.count) {
                applyRatio(beams, linkedBeam, controls.state[i]);
            }
        }
    }

    public void clear() {
        count = 0;
        controls.clear();
    }

    private void ensureCapacity() {
        if (count >= beamIndex.length) {
            beamIndex = Utility.expand(beamIndex, beamIndex.length * 2);
        }
    }

    private static void applyRatio(BeamContainer beams, int beamIndex, float ratio) {
        beams.actuationRatio[beamIndex] = Math.max(MIN_RATIO, ratio);
    }
}
