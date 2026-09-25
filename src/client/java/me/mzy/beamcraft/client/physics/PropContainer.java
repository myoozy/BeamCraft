package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.PhysicsSpecs.PropSpec;
import me.mzy.beamcraft.utility.Utility;

import java.util.Arrays;

/**
 * BeamNG rigid props: non-deforming meshes attached to a three-node frame and
 * optionally animated by an electric signal.
 *
 * <p>Each prop exposes three synthetic render nodes after the vehicle's real
 * physics nodes. The existing transform-feedback skinning shader can therefore
 * render a prop as a rigid local basis without adding a second mesh pipeline or
 * letting render-only nodes enter the physics solver.</p>
 */
public final class PropContainer {
    private static final int INITIAL_CAPACITY = 16;

    public int count;
    public String[] function = new String[INITIAL_CAPACITY];
    public int[] meshIndex = new int[INITIAL_CAPACITY];
    public int[] refNode = new int[INITIAL_CAPACITY];
    public int[] xNode = new int[INITIAL_CAPACITY];
    public int[] yNode = new int[INITIAL_CAPACITY];

    public float[] baseRotationX = new float[INITIAL_CAPACITY];
    public float[] baseRotationY = new float[INITIAL_CAPACITY];
    public float[] baseRotationZ = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalX = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalY = new float[INITIAL_CAPACITY];
    public float[] baseRotationGlobalZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseRotationGlobal = new boolean[INITIAL_CAPACITY];
    public float[] rotationX = new float[INITIAL_CAPACITY];
    public float[] rotationY = new float[INITIAL_CAPACITY];
    public float[] rotationZ = new float[INITIAL_CAPACITY];
    public float[] translationX = new float[INITIAL_CAPACITY];
    public float[] translationY = new float[INITIAL_CAPACITY];
    public float[] translationZ = new float[INITIAL_CAPACITY];
    public float[] min = new float[INITIAL_CAPACITY];
    public float[] max = new float[INITIAL_CAPACITY];
    public float[] offset = new float[INITIAL_CAPACITY];
    public float[] multiplier = new float[INITIAL_CAPACITY];
    public boolean[] translationUseMeters = new boolean[INITIAL_CAPACITY];

    /** Initial prop origin in its rest-pose idRef/idX/idY frame, filled by the binder. */
    public float[] originLocalX = new float[INITIAL_CAPACITY];
    public float[] originLocalY = new float[INITIAL_CAPACITY];
    public float[] originLocalZ = new float[INITIAL_CAPACITY];
    public boolean[] originBound = new boolean[INITIAL_CAPACITY];
    /** Parsed origin override retained until the DAE pivot is available to the binder. */
    public float[] baseTranslationX = new float[INITIAL_CAPACITY];
    public float[] baseTranslationY = new float[INITIAL_CAPACITY];
    public float[] baseTranslationZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseTranslation = new boolean[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalX = new float[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalY = new float[INITIAL_CAPACITY];
    public float[] baseTranslationGlobalZ = new float[INITIAL_CAPACITY];
    public boolean[] hasBaseTranslationGlobal = new boolean[INITIAL_CAPACITY];


    public int register(PropSpec spec, int owningMeshIndex) {
        ensureCapacity();
        int index = count++;
        function[index] = spec.function();
        meshIndex[index] = owningMeshIndex;
        refNode[index] = spec.refNode();
        xNode[index] = spec.xNode();
        yNode[index] = spec.yNode();

        baseRotationX[index] = spec.baseRotation().x();
        baseRotationY[index] = spec.baseRotation().y();
        baseRotationZ[index] = spec.baseRotation().z();
        baseRotationGlobalX[index] = spec.baseRotationGlobal().x();
        baseRotationGlobalY[index] = spec.baseRotationGlobal().y();
        baseRotationGlobalZ[index] = spec.baseRotationGlobal().z();
        hasBaseRotationGlobal[index] = spec.hasBaseRotationGlobal();
        rotationX[index] = spec.rotation().x();
        rotationY[index] = spec.rotation().y();
        rotationZ[index] = spec.rotation().z();
        translationX[index] = spec.translation().x();
        translationY[index] = spec.translation().y();
        translationZ[index] = spec.translation().z();
        min[index] = spec.min();
        max[index] = spec.max();
        offset[index] = spec.offset();
        multiplier[index] = spec.multiplier();
        translationUseMeters[index] = spec.translationUseMeters();

        baseTranslationX[index] = spec.baseTranslation().x();
        baseTranslationY[index] = spec.baseTranslation().y();
        baseTranslationZ[index] = spec.baseTranslation().z();
        hasBaseTranslation[index] = spec.hasBaseTranslation();
        baseTranslationGlobalX[index] = spec.baseTranslationGlobal().x();
        baseTranslationGlobalY[index] = spec.baseTranslationGlobal().y();
        baseTranslationGlobalZ[index] = spec.baseTranslationGlobal().z();
        hasBaseTranslationGlobal[index] = spec.hasBaseTranslationGlobal();
        return index;
    }

    public int renderNodeCount(int physicsNodeCount) {
        return Math.addExact(physicsNodeCount, Math.multiplyExact(count, 3));
    }

    public int syntheticNodeBase(int physicsNodeCount, int propIndex) {
        return physicsNodeCount + propIndex * 3;
    }

    public void clear() {
        count = 0;
    }

    private void ensureCapacity() {
        if (count < function.length) return;
        int size = function.length * 2;
        function = Arrays.copyOf(function, size);
        meshIndex = Utility.expand(meshIndex, size);
        refNode = Utility.expand(refNode, size);
        xNode = Utility.expand(xNode, size);
        yNode = Utility.expand(yNode, size);
        baseRotationX = Utility.expand(baseRotationX, size);
        baseRotationY = Utility.expand(baseRotationY, size);
        baseRotationZ = Utility.expand(baseRotationZ, size);
        baseRotationGlobalX = Utility.expand(baseRotationGlobalX, size);
        baseRotationGlobalY = Utility.expand(baseRotationGlobalY, size);
        baseRotationGlobalZ = Utility.expand(baseRotationGlobalZ, size);
        hasBaseRotationGlobal = Utility.expand(hasBaseRotationGlobal, size);
        rotationX = Utility.expand(rotationX, size);
        rotationY = Utility.expand(rotationY, size);
        rotationZ = Utility.expand(rotationZ, size);
        translationX = Utility.expand(translationX, size);
        translationY = Utility.expand(translationY, size);
        translationZ = Utility.expand(translationZ, size);
        min = Utility.expand(min, size);
        max = Utility.expand(max, size);
        offset = Utility.expand(offset, size);
        multiplier = Utility.expand(multiplier, size);
        translationUseMeters = Utility.expand(translationUseMeters, size);
        originLocalX = Utility.expand(originLocalX, size);
        originLocalY = Utility.expand(originLocalY, size);
        originLocalZ = Utility.expand(originLocalZ, size);
        originBound = Utility.expand(originBound, size);
        baseTranslationX = Utility.expand(baseTranslationX, size);
        baseTranslationY = Utility.expand(baseTranslationY, size);
        baseTranslationZ = Utility.expand(baseTranslationZ, size);
        hasBaseTranslation = Utility.expand(hasBaseTranslation, size);
        baseTranslationGlobalX = Utility.expand(baseTranslationGlobalX, size);
        baseTranslationGlobalY = Utility.expand(baseTranslationGlobalY, size);
        baseTranslationGlobalZ = Utility.expand(baseTranslationGlobalZ, size);
        hasBaseTranslationGlobal = Utility.expand(hasBaseTranslationGlobal, size);
    }

}
