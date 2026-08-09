package me.mzy.beamcraft.client.physics;

import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Focused tests for {@link JBeamParser#evaluateBeamNGExpression} covering the
 * narrowly-scoped support for top-level
 * {@code case(cond, trueVal, falseVal)} with {@code == nil} conditions, while
 * preserving the existing plain-variable and arithmetic behaviour.
 */
class JBeamExpressionTest {

    // Conditional front wheel nodeOffset.x expression.
    private static final String CONDITIONAL_FRONT_OFFSET =
            "$=case($trackwidth_F == nil, $trackoffset_F+0.25, $trackwidth_F)";
    // Equivalent rear wheel expression using rear axle variables.
    private static final String CONDITIONAL_REAR_OFFSET =
            "$=case($trackwidth_R == nil, $trackoffset_R+0.25, $trackwidth_R)";
    // Simple arithmetic offset expression without a conditional.
    private static final String SIMPLE_FRONT_OFFSET =
            "$=$trackoffset_F+0.235";

    private static Map<String, Double> vars(Object... kv) {
        Map<String, Double> map = new HashMap<>();
        for (int i = 0; i < kv.length; i += 2) {
            map.put((String) kv[i], (Double) kv[i + 1]);
        }
        return map;
    }

    @Test
    void undefinedFrontTrackwidthFallsBackToTrackoffset() {
        // 前轮：trackwidth_F 未定义 → case 视为 nil → 取 trackoffset_F + 0.25 = 0 + 0.25
        Map<String, Double> variables = vars("trackoffset_F", 0.0);
        assertEquals(0.25, JBeamParser.evaluateBeamNGExpression(CONDITIONAL_FRONT_OFFSET, variables), 1e-6);
    }

    @Test
    void rearOffsetSupportsNonZeroTrackoffset() {
        // 后轮：trackwidth_R 未定义，trackoffset_R 非零 → 0.75 + 0.25
        Map<String, Double> variables = vars("trackoffset_R", 0.75);
        assertEquals(1.0, JBeamParser.evaluateBeamNGExpression(CONDITIONAL_REAR_OFFSET, variables), 1e-6);
    }

    @Test
    void definedTrackwidthTakesPrecedenceOverFallback() {
        // trackwidth_F 已定义 → case 直接采用该值，忽略 trackoffset_F 回退分支
        Map<String, Double> variables = vars("trackwidth_F", 1.6, "trackoffset_F", 0.0);
        assertEquals(1.6, JBeamParser.evaluateBeamNGExpression(CONDITIONAL_FRONT_OFFSET, variables), 1e-6);
    }

    @Test
    void simpleOffsetExpressionRemainsCorrect() {
        // The existing arithmetic path must still evaluate the offset.
        Map<String, Double> variables = vars("trackoffset_F", 0.0);
        assertEquals(0.235, JBeamParser.evaluateBeamNGExpression(SIMPLE_FRONT_OFFSET, variables), 1e-6);
    }

    @Test
    void plainVariableReferenceUnchanged() {
        Map<String, Double> variables = vars("trackoffset_F", 0.5);
        assertEquals(0.5, JBeamParser.evaluateBeamNGExpression("$trackoffset_F", variables), 1e-6);
        // 未定义变量按 0 的既有兼容行为
        assertEquals(0.0, JBeamParser.evaluateBeamNGExpression("$trackwidth_F", variables), 1e-6);
    }

    @Test
    void existingArithmeticExpressionUnchanged() {
        // 既有四则运算不回归：$tirepressure_F * 550 + 10
        Map<String, Double> variables = vars("tirepressure_F", 10.0);
        assertEquals(5510.0, JBeamParser.evaluateBeamNGExpression(
                "$=$tirepressure_F * 550 + 10", variables), 1e-6);
    }
}
