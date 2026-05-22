package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class LBeamContainer extends BeamContainer {
    public int[] node3;
    public double[] restCosTheta;
    public double[] baseCosTheta;
    public double[] targetCosTheta;

    public LBeamContainer() {
        super();
        node3 = new int[INIT_BEAM_CAP];
        restCosTheta = new double[INIT_BEAM_CAP];
        baseCosTheta = new double[INIT_BEAM_CAP];
        targetCosTheta = new double[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        node3 = Utility.expand(node3, newSize);
        restCosTheta = Utility.expand(restCosTheta, newSize);
        baseCosTheta = Utility.expand(baseCosTheta, newSize);
        targetCosTheta = Utility.expand(targetCosTheta, newSize);
    }

    public int addBeam(java.util.List<String> breakGroups,
                        int node1Idx, int node2Idx, int node3Idx,
                        double node12Dist, double node13Dist, double node23Dist,
                        double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength,
                        double precomp, double precompRange, double precompTime) {
        int idx = addBeamInternal(breakGroups, node1Idx, node2Idx, node12Dist, beamSpring, beamDamp,
                beamDeform, beamStrength, precomp, precompRange, precompTime);
        node3[idx] = node3Idx;

        double numerator = node13Dist * node13Dist + node23Dist * node23Dist - node12Dist * node12Dist;
        double denominator = 2.0 * node13Dist * node23Dist;
        double baseCos = numerator / denominator;
        baseCos = Math.clamp(baseCos, -1.0, 1.0);

        double targetNode12Dist = (node12Dist * precomp) + precompRange;
        numerator = node13Dist * node13Dist + node23Dist * node23Dist - targetNode12Dist * targetNode12Dist;
        denominator = 2.0 * node13Dist * node23Dist;
        double targetCos = numerator / denominator;
        targetCos = Math.clamp(targetCos, -1.0, 1.0);

        if (Double.isNaN(targetCos) || Double.isNaN(baseCos)) broken[idx] = true;

        targetCosTheta[idx] = targetCos;

        if (precompTime > 0.0) {
            restCosTheta[idx] = baseCos;
            precompTimer[idx] = precompTime;
            precompTimeTotal[idx] = precompTime;
        } else {
            restCosTheta[idx] = targetCos;
            precompTimer[idx] = 0.0;
            precompTimeTotal[idx] = 0.0;
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

    public void updatePrecompression(double mcDt) {
        for (int i = 0; i < count; i++) {
            if (precompTimer[i] > 0) {
                precompTimer[i] -= mcDt;
                if (precompTimer[i] <= 0) {
                    precompTimer[i] = 0;
                    restCosTheta[i] = targetCosTheta[i];
                } else {
                    double progress = 1.0 - (precompTimer[i] / precompTimeTotal[i]);
                    restCosTheta[i] = baseCosTheta[i] + (targetCosTheta[i] - baseCosTheta[i]) * progress;
                }
            }
        }
    }
}
