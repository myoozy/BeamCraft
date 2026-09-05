package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.electrics.ElectricBus;
import me.mzy.beamcraft.client.physics.electrics.ElectricSnapshot;
import me.mzy.beamcraft.utility.Utility;

/** Controls the target-angle offset of torsion bars used as actuators. */
public final class TorsionHydroContainer {
    private static final int INITIAL_CAPACITY = 16;

    public int count;
    public int[] torsionBarIndex = new int[INITIAL_CAPACITY];
    public final HydroActuatorController controls = new HydroActuatorController();

    public int addTorsionHydro(PhysicsSpecs.TorsionHydroSpec spec, int linkedTorsionBarIndex,
                               TorsionBarContainer torsionBars, ElectricBus electrics) {
        ensureCapacity();
        int index = count++;
        torsionBarIndex[index] = linkedTorsionBarIndex;
        int controlIndex = controls.add(spec, 0.0f, electrics);
        if (controlIndex != index) {
            throw new IllegalStateException("Torsion hydro actuator indices are out of sync");
        }
        applyAngle(torsionBars, linkedTorsionBarIndex, controls.state[index]);
        return index;
    }

    public void update(float dt, TorsionBarContainer torsionBars, ElectricSnapshot electrics) {
        for (int i = 0; i < count; i++) {
            int linkedTorsionBar = torsionBarIndex[i];
            if (linkedTorsionBar < 0 || linkedTorsionBar >= torsionBars.count
                    || torsionBars.broken[linkedTorsionBar]) {
                continue;
            }
            applyAngle(torsionBars, linkedTorsionBar, controls.update(i, dt, electrics));
        }
    }

    public void reset(TorsionBarContainer torsionBars) {
        controls.reset();
        for (int i = 0; i < count; i++) {
            int linkedTorsionBar = torsionBarIndex[i];
            if (linkedTorsionBar >= 0 && linkedTorsionBar < torsionBars.count) {
                applyAngle(torsionBars, linkedTorsionBar, controls.state[i]);
            }
        }
    }

    public void clear() {
        count = 0;
        controls.clear();
    }

    private void ensureCapacity() {
        if (count >= torsionBarIndex.length) {
            torsionBarIndex = Utility.expand(torsionBarIndex, torsionBarIndex.length * 2);
        }
    }

    private static void applyAngle(TorsionBarContainer torsionBars, int index, float angle) {
        torsionBars.actuationAngle[index] = angle;
    }
}
