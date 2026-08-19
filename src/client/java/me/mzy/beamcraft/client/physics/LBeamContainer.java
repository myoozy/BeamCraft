package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class LBeamContainer extends BeamContainer {
    public int[] node3;
    public float[] restCosTheta;
    public float[] baseCosTheta;
    public float[] targetCosTheta;

    public LBeamContainer() {
        super();
        node3 = new int[INIT_BEAM_CAP];
        restCosTheta = new float[INIT_BEAM_CAP];
        baseCosTheta = new float[INIT_BEAM_CAP];
        targetCosTheta = new float[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        node3 = Utility.expand(node3, newSize);
        restCosTheta = Utility.expand(restCosTheta, newSize);
        baseCosTheta = Utility.expand(baseCosTheta, newSize);
        targetCosTheta = Utility.expand(targetCosTheta, newSize);
    }

    public int addBeam(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, int node3Idx,
                       float node12Dist, float node13Dist, float node23Dist) {
        int idx = addBeamInternal(spec, node1Idx, node2Idx, node12Dist);
        node3[idx] = node3Idx;

        double numerator = node13Dist * node13Dist + node23Dist * node23Dist - node12Dist * node12Dist;
        double denominator = 2.0 * node13Dist * node23Dist;
        double baseCos = numerator / denominator;
        baseCos = Math.clamp(baseCos, -1.0, 1.0);

        double targetNode12Dist = (node12Dist * spec.precomp()) + spec.precompRange();
        numerator = node13Dist * node13Dist + node23Dist * node23Dist - targetNode12Dist * targetNode12Dist;
        denominator = 2.0 * node13Dist * node23Dist;
        double targetCos = numerator / denominator;
        targetCos = Math.clamp(targetCos, -1.0, 1.0);

        if (Double.isNaN(targetCos) || Double.isNaN(baseCos)) broken[idx] = true;

        targetCosTheta[idx] = (float) targetCos;

        if (spec.precompTime() > 0.0f) {
            restCosTheta[idx] = (float) baseCos;
            precompTimer[idx] = spec.precompTime();
            precompTimeTotal[idx] = spec.precompTime();
        } else {
            restCosTheta[idx] = (float) targetCos;
            precompTimer[idx] = 0.0f;
            precompTimeTotal[idx] = 0.0f;
        }

        baseCosTheta[idx] = restCosTheta[idx];

        return idx;
    }

    public void reset() {
        super.reset();
        for (int i = 0; i < count; i++) {
            restCosTheta[i] = baseCosTheta[i];
        }
    }

    public void updatePrecompression(float mcDt) {
        for (int i = 0; i < count; i++) {
            if (precompTimer[i] > 0) {
                precompTimer[i] -= mcDt;
                if (precompTimer[i] <= 0) {
                    precompTimer[i] = 0;
                    restCosTheta[i] = targetCosTheta[i];
                } else {
                    float progress = 1.0f - (precompTimer[i] / precompTimeTotal[i]);
                    restCosTheta[i] = baseCosTheta[i] + (targetCosTheta[i] - baseCosTheta[i]) * progress;
                }
            }
        }
    }
}
