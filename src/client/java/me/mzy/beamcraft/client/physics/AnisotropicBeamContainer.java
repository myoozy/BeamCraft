package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class AnisotropicBeamContainer extends BeamContainer {
    public double[] springExpansion;
    public double[] dampExpansion;
    public double[] transitionZone;

    public AnisotropicBeamContainer() {
        springExpansion = new double[INIT_BEAM_CAP];
        dampExpansion = new double[INIT_BEAM_CAP];
        transitionZone = new double[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        springExpansion = Utility.expand(springExpansion, newSize);
        dampExpansion = Utility.expand(dampExpansion, newSize);
        transitionZone = Utility.expand(transitionZone, newSize);
    }

    public void addBeam(int node1Idx, int node2Idx, double nodeDist,
                        double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength,
                        double precomp, double precompRange, double precompTime,
                        double beamSpringExpansion, double beamDampExpansion, double beamTransitionZone) {
        int idx = addBeamInternal(node1Idx, node2Idx, nodeDist, beamSpring, beamDamp,
                beamDeform, beamStrength, precomp, precompRange, precompTime);
        springExpansion[idx] = beamSpringExpansion;
        dampExpansion[idx] = beamDampExpansion;
        transitionZone[idx] = beamTransitionZone;
    }
}
