package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.physics.JBeamAssembler.PartEntry;
import me.mzy.beamcraft.client.physics.JBeamAssembler.TransformContext;
import me.mzy.beamcraft.client.physics.JBeamParser.NodeRowState;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

/**
 * Integration test for the {@link JBeamParser#parseNodes} row pipeline
 * ({@link JBeamParser#buildNodeSpec}): confirms that node coordinates written as
 * {@code $=...} expressions are evaluated into numbers so the node row is NOT
 * skipped, while genuinely invalid coordinates still skip the row.
 */
class JBeamParserNodesIntegrationTest {

    private static JsonArray row(String json) {
        return JsonParser.parseString(json).getAsJsonArray();
    }

    private static PartEntry entry(Map<String, Double> vars) {
        return new PartEntry(null, 1, "testpart", new TransformContext(), vars);
    }

    private static NodeRowState state() {
        return new NodeRowState();
    }

    @Test
    void expressionCoordinatesProduceNodeInsteadOfSkipping() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("caster_F", 2.0);
        PartEntry part = entry(vars);

        // ["fh1r", -0.57, "$=-1.147-$caster_F", 0.217]
        PhysicsSpecs.NodeSpec spec = JBeamParser.buildNodeSpec(
                row("[\"fh1r\", -0.57, \"$=-1.147-$caster_F\", 0.217]"),
                state(), part, new CouplerRegistry());

        assertNotNull(spec, "expression coordinate row must not be skipped");
        // raw = (-0.57, -1.147-2, 0.217); identity transform; engine flip → x, y=z, z=-y
        assertEquals(-0.57f, spec.x(), 1e-6f);
        assertEquals(0.217f, spec.y(), 1e-6f);
        assertEquals(3.147f, spec.z(), 1e-6f);
        assertEquals("fh1r", spec.name());
    }

    @Test
    void mirroredCoordinateExpressionsResolve() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("ackf", 0.1);
        PartEntry part = entry(vars);

        PhysicsSpecs.NodeSpec right = JBeamParser.buildNodeSpec(
                row("[\"fh3r\", \"$=$ackf-0.745\", -1.660, 0.258]"),
                state(), part, new CouplerRegistry());
        assertNotNull(right, "mirrored right node must not be skipped");
        // rawX = 0.1 - 0.745 = -0.645; flip: x=-0.645, y=0.258, z=1.660
        assertEquals(-0.645f, right.x(), 1e-6f);
        assertEquals(1.660f, right.z(), 1e-6f);

        PhysicsSpecs.NodeSpec left = JBeamParser.buildNodeSpec(
                row("[\"fh3l\", \"$=-$ackf+0.742\", -1.660, 0.258]"),
                state(), part, new CouplerRegistry());
        assertNotNull(left, "mirrored left node must not be skipped");
        // rawX = -0.1 + 0.742 = 0.642; flip: x=0.642, z=1.660
        assertEquals(0.642f, left.x(), 1e-6f);
    }

    @Test
    void plainNumericCoordinatesStillParse() {
        PartEntry part = entry(new HashMap<>());
        PhysicsSpecs.NodeSpec spec = JBeamParser.buildNodeSpec(
                row("[\"n1\", 0.5, 1.0, 0.3]"),
                state(), part, new CouplerRegistry());
        assertNotNull(spec);
        // flip: x=0.5, y=0.3, z=-1.0
        assertEquals(0.5f, spec.x(), 1e-6f);
        assertEquals(0.3f, spec.y(), 1e-6f);
        assertEquals(-1.0f, spec.z(), 1e-6f);
    }

    @Test
    void pureVariableCoordinatesResolveFromVariables() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("vehicle_width", 1.8);
        PartEntry part = entry(vars);
        PhysicsSpecs.NodeSpec spec = JBeamParser.buildNodeSpec(
                row("[\"edge\", \"$vehicle_width/2\", 0.4, 1.1]"),
                state(), part, new CouplerRegistry());
        assertNotNull(spec, "pure-$var coordinate with defined variable must not be skipped");
        assertEquals(0.9f, spec.x(), 1e-6f);
    }

    @Test
    void genuinelyInvalidCoordinatesStillSkipTheRow() {
        PartEntry part = entry(new HashMap<>());
        // coordinate that is neither a number nor a resolvable expression
        assertNull(JBeamParser.buildNodeSpec(
                row("[\"bad\", \"notanumber\", 0.0, 0.0]"),
                state(), part, new CouplerRegistry()));
        // expression that evaluates to nil → skip
        assertNull(JBeamParser.buildNodeSpec(
                row("[\"badnil\", \"$=nil\", 0.0, 0.0]"),
                state(), part, new CouplerRegistry()));
    }

    @Test
    void expressionGroupEntriesAreEvaluated() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("prefix", 1.0);
        vars.put("suffix", 2.0);
        PartEntry part = entry(vars);

        // group 数组里含 "$= $prefix..$suffix" 这类字符串拼接表达式（数值变量 → "12"）
        JsonArray nodes = JsonParser.parseString(
                "[[\"posX\",\"posY\",\"posZ\"], " +
                "[\"g1\", 0, 0, 0, {\"group\": [\"$= $prefix..$suffix\", \"static_grp\"]}]]").getAsJsonArray();

        NodeRowState st = state();
        PhysicsSpecs.NodeSpec spec = JBeamParser.buildNodeSpec(nodes.get(1).getAsJsonArray(), st, part, new CouplerRegistry());
        assertNotNull(spec);
        assertNotNull(spec.groups());
        assertEquals(java.util.List.of("12", "static_grp"), spec.groups());
    }

    @Test
    void modifierObjectUpdatesRowDefaults() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("baseWeight", 3.0);
        PartEntry part = entry(vars);

        JsonArray nodes = JsonParser.parseString(
                "[[\"posX\",\"posY\",\"posZ\"], " +
                "{\"nodeWeight\": \"$= $baseWeight + 0.5\"}, " +
                "[\"nw\", 0, 0, 0]]").getAsJsonArray();

        NodeRowState st = new NodeRowState();
        // first apply the modifier exactly as parseNodes does
        st.weight = JBeamParser.getFloatSafe(nodes.get(1).getAsJsonObject(), "nodeWeight", st.weight, vars);

        PhysicsSpecs.NodeSpec spec = JBeamParser.buildNodeSpec(nodes.get(2).getAsJsonArray(), st, part, new CouplerRegistry());
        assertNotNull(spec);
        assertEquals(3.5f, spec.mass(), 1e-6f);
    }
}
