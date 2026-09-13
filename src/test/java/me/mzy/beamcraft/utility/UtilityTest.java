package me.mzy.beamcraft.utility;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class UtilityTest {
    @Test
    void capPairToSumPreservesTheSmallerValue() {
        Utility.FloatPair capped = Utility.capPairToSum(20.0f, 100.0f, 70.0f);

        assertEquals(20.0f, capped.first());
        assertEquals(50.0f, capped.second());
    }

    @Test
    void capPairToSumSharesBudgetWhenBothValuesAreLarge() {
        Utility.FloatPair capped = Utility.capPairToSum(80.0f, 100.0f, 70.0f);

        assertEquals(35.0f, capped.first());
        assertEquals(35.0f, capped.second());
    }

    @Test
    void dominantEigenpairFindsLargestAxis() {
        Utility.SymmetricEigenpair3 eigenpair = Utility.dominantEigenpairSym3x3(
                1.0, 0.0, 0.0,
                5.0, 0.0, 2.0);

        assertEquals(5.0, eigenpair.value(), 1.0e-12);
        assertEquals(0.0, eigenpair.x(), 1.0e-12);
        assertEquals(1.0, Math.abs(eigenpair.y()), 1.0e-12);
        assertEquals(0.0, eigenpair.z(), 1.0e-12);
    }

    @Test
    void reducedMassRejectsNonPositiveMasses() {
        assertEquals(2.0f, Utility.reducedMass(3.0f, 6.0f));
        assertEquals(0.0f, Utility.reducedMass(0.0f, 6.0f));
    }
}
