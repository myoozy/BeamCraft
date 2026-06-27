package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.utility.Utility;

public class AnisotropicBeamContainer extends BeamContainer {
    public float[] springExpansion;
    public float[] dampExpansion;
    public float[] transitionZone;

    public AnisotropicBeamContainer() {
        springExpansion = new float[INIT_BEAM_CAP];
        dampExpansion = new float[INIT_BEAM_CAP];
        transitionZone = new float[INIT_BEAM_CAP];
    }

    @Override
    protected void resize(int newSize) {
        super.resize(newSize);
        springExpansion = Utility.expand(springExpansion, newSize);
        dampExpansion = Utility.expand(dampExpansion, newSize);
        transitionZone = Utility.expand(transitionZone, newSize);
    }

    public int addBeam(java.util.List<String> breakGroups, int breakGroupType,
                        int node1Idx, int node2Idx, double nodeDist,
                        double beamSpring, double beamDamp,
                        double beamDeform, double beamStrength,
                        double precomp, double precompRange, double precompTime,
                        double beamSpringExpansion, double beamDampExpansion, double beamTransitionZone) {
        int idx = addBeamInternal(breakGroups, breakGroupType, node1Idx, node2Idx, nodeDist, beamSpring, beamDamp,
                beamDeform, beamStrength, precomp, precompRange, precompTime);
        springExpansion[idx] = (float) beamSpringExpansion;
        dampExpansion[idx] = (float) beamDampExpansion;
        transitionZone[idx] = (float) beamTransitionZone;
        return idx;
    }
}
