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

    public int addBeam(PhysicsSpecs.BeamSpec spec, int node1Idx, int node2Idx, float nodeDist) {
        int idx = addBeamInternal(spec, node1Idx, node2Idx, nodeDist);
        springExpansion[idx] = spec.springExpansion();
        dampExpansion[idx] = spec.dampExpansion();
        transitionZone[idx] = spec.transitionZone();
        return idx;
    }
}
