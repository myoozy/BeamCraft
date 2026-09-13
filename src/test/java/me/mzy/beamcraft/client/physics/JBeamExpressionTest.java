package me.mzy.beamcraft.client.physics;

import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalOutcome;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalStatus;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Comprehensive generic tests for JBeam {@code $=} expression evaluation.
 *
 * <p>Covers the documented core grammar (precedence, right-associative {@code ^},
 * concatenation, Lua truthiness, operand-returning {@code and}/{@code or},
 * nil-tertiary idiom, dotted resolver, multi-branch {@code case()}, builtins,
 * parentheses and unary minus) plus the historically-preserved regressions
 * (front/rear wheel offsets, plain variable references, arithmetic).
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

    private static EvalOutcome eval(String expr) {
        return JBeamExpressionEvaluator.evaluate(expr, BeamExpressionContext.empty());
    }

    private static EvalOutcome eval(String expr, Map<String, Double> vars) {
        return JBeamExpressionEvaluator.evaluate(expr, vars);
    }

    private static void assertFloat(double expected, EvalOutcome out, String expr) {
        assertTrue(out.ok(), () -> "expected success for [" + expr + "] but got " + out.status() + ": " + out.reason());
        assertEquals(expected, ((Number) out.value()).doubleValue(), 1e-9, () -> "value mismatch for [" + expr + "]");
    }

    private static void assertStr(String expected, EvalOutcome out, String expr) {
        assertTrue(out.ok(), () -> "expected success for [" + expr + "] but got " + out.status() + ": " + out.reason());
        assertEquals(expected, out.value(), () -> "string mismatch for [" + expr + "]");
    }

    // ------------------------------------------------------------------
    // Historical regressions (must keep passing)
    // ------------------------------------------------------------------

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

    // ------------------------------------------------------------------
    // Core grammar: operators, precedence, associativity
    // ------------------------------------------------------------------

    @Test
    void operatorPrecedence() {
        assertFloat(7, eval("$=1+2*3"), "1+2*3");
        assertFloat(9, eval("$=(1+2)*3"), "(1+2)*3");
        assertFloat(3, eval("$=10-4-3"), "10-4-3");       // left assoc
        assertFloat(20, eval("$=10+2*5"), "10+2*5");
        assertFloat(14, eval("$=2+3*4"), "2+3*4");
        assertFloat(1, eval("$=7%3"), "7%3");             // modulo
        assertFloat(1024, eval("$=2^10"), "2^10");
    }

    @Test
    void powerIsRightAssociative() {
        assertFloat(512, eval("$=2^3^2"), "2^3^2");        // 2^(3^2), not (2^3)^2=64
        assertFloat(-4, eval("$=-2^2"), "-2^2");           // -(2^2)
        assertFloat(0.25, eval("$=2^-2"), "2^-2");         // 2^(-2)
    }

    @Test
    void concatenation() {
        assertStr("ab", eval("$='a'..'b'"), "'a'..'b'");
        assertStr("x1", eval("$='x'..1"), "'x'..1");       // number → "1"
        assertStr("2.51", eval("$=2.5..1"), "2.5..1");
        assertStr("2.53", eval("$=2.5..3"), "2.5..3");
        assertStr("pre_suf", eval("$='pre'..'_'..'suf'"), "'pre'..'_'..'suf'");
        // Lua-style: integral doubles drop the decimal point
        assertStr("ab1", eval("$='ab'..(1.0)"), "'ab'..1.0");
    }

    @Test
    void luaTruthinessAndOperandReturningAndOr() {
        // and/or return operands, not booleans
        assertStr("b", eval("$='a' and 'b'"), "'a' and 'b'");
        assertEquals(5.0, ((Number) eval("$=5 or 7").value()).doubleValue(), "5 or 7");
        // nil and 5 → nil；nil or 7 → 7（nil 为假，or 取右侧操作数）
        assertEquals(7.0, ((Number) eval("$=nil and 5 or 7").value()).doubleValue(), "nil and 5 or 7");
        assertStr("x", eval("$=false or 'x'"), "false or 'x'");
        // 0 and "" are truthy in Lua
        assertEquals(5.0, ((Number) eval("$=0 and 5 or 7").value()).doubleValue(), "0 and 5 or 7");
        assertStr("", eval("$='' or 'y'"), "'' or 'y'");
        // nil is falsy
        assertEquals(7.0, ((Number) eval("$=nil or 7").value()).doubleValue(), "nil or 7");
    }

    @Test
    void nilTernaryIdiom() {
        // $x == nil and fallback or $x  (undefined variable inside expression → nil)
        assertFloat(3, eval("$=$x == nil and 3 or $x", vars()), "$x==nil and 3 or $x");
        assertFloat(5, eval("$=$x == nil and 3 or $x", vars("x", 5.0)), "$x==nil and 3 or $x (x=5)");
    }

    @Test
    void stringAndBooleanComparisons() {
        assertTrue((Boolean) eval("$='a' < 'b'").value(), "'a' < 'b'");
        assertTrue((Boolean) eval("$=3 >= 3").value(), "3 >= 3");
        assertTrue((Boolean) eval("$=1 ~= 2").value(), "1 ~= 2");
        assertTrue((Boolean) eval("$=not nil").value(), "not nil");
        assertTrue(!(Boolean) eval("$=not 0").value(), "not 0 (0 is truthy)");
        assertTrue((Boolean) eval("$='block_a' == 'block_a'").value(), "string equality");
    }

    @Test
    void stringLengthOperator() {
        assertFloat(5, eval("$=#'hello'"), "#('hello')");
    }

    @Test
    void parenthesesAndUnaryMinus() {
        assertFloat(5, eval("$=-(-5)"), "-(-5)");
        assertFloat(-5, eval("$=-(2+3)"), "-(2+3)");
        assertFloat(0.5, eval("$=2^-1"), "2^-1");
        assertFloat(-1.147, eval("$=-1.147"), "-1.147");
        assertFloat(0.742, eval("$=-(-0.742)"), "-(-0.742)");
    }

    // ------------------------------------------------------------------
    // Variables and dotted resolver
    // ------------------------------------------------------------------

    @Test
    void undefinedVariablesAreNilInsideExpressions() {
        // 表达式内未定义变量按 nil，而非 0
        assertTrue((Boolean) eval("$=$missing == nil").value(), "undefined $var == nil inside $=");
        assertTrue(!(Boolean) eval("$=$missing ~= nil").value(), "undefined $var ~= nil inside $=");
        // 已定义变量可参与算术
        assertFloat(6, eval("$=$a*2", vars("a", 3.0)), "defined var arithmetic");
    }

    @Test
    void dottedResolverAgainstTypedContext() {
        Map<String, Object> root = new HashMap<>();
        Map<String, Object> wheelsR = new HashMap<>();
        wheelsR.put("duallyR", 0.0);
        wheelsR.put("singleR", 1.0);
        Map<String, Object> wheelsF = new HashMap<>();
        wheelsF.put("duallyF", 0.0);
        wheelsF.put("singleF", 1.0);
        root.put("components", Map.of("wheelsR", wheelsR, "wheelsF", wheelsF, "exhaustTip", Map.of("singleTip", true)));
        BeamExpressionContext ctx = BeamExpressionContext.of(vars(), root);

        EvalOutcome dually = JBeamExpressionEvaluator.evaluate(
                "$=$components.wheelsR.duallyR == 0 or $components.wheelsR.singleR == 1", ctx);
        assertTrue(dually.ok(), "dotted resolve: " + dually.reason());
        assertTrue((Boolean) dually.value(), "$components.wheelsR.duallyR == 0 or ...");

        EvalOutcome mesh = JBeamExpressionEvaluator.evaluate(
                "$=$components.exhaustTip.singleTip == true and 'ardente_single' or 'ardente'", ctx);
        assertStr("ardente_single", mesh, "exhaustTip ternary");
    }

    @Test
    void dottedComponentsAbsentResolveToNil() {
        // BeamCraft 当前不加载 components 数据：$components.* 必须按 nil 处理（== nil 成立，布尔条件为 false）
        assertTrue(!(Boolean) eval("$=$components.wheelsR.duallyR == 0 or $components.wheelsR.singleR == 1").value(),
                "absent $components resolves to nil → both conditions false → whole expression false");
        assertTrue((Boolean) eval("$=$components.transmission == nil").value(), "$components.transmission == nil");
        assertTrue(!(Boolean) eval("$=$components.transmission ~= nil").value(), "$components.transmission ~= nil");
        // dotted 三元回退取默认分支
        assertStr("noYpipe", eval("$=$components.exhaustYpipe.YpipeMesh_longnose_single == nil and 'noYpipe' or $components.exhaustYpipe.YpipeMesh_longnose_single"),
                "nil-dotted ternary picks fallback");
    }

    @Test
    void bareDottedPureVariableCompatReturnsZero() {
        // 纯 "$components.foo"（无 $=）在数值上下文回退 0.0f（既有兼容行为）
        assertEquals(0.0, JBeamParser.evaluateBeamNGExpression("$components.wheelsR.duallyR", vars()), 1e-6);
    }

    // ------------------------------------------------------------------
    // Documented case() behavior: boolean ternary / numeric index selection
    // ------------------------------------------------------------------

    @Test
    void caseBooleanSelectorIsTernary() {
        assertFloat(10, eval("$=case(true, 10, 20)"), "case(true,10,20)");
        assertFloat(20, eval("$=case(false, 10, 20)"), "case(false,10,20)");
        assertStr("yes", eval("$=case(true, 'yes', 'no')"), "case(true,'yes','no')");
        assertStr("no", eval("$=case(false, 'yes', 'no')"), "case(false,'yes','no')");
    }

    @Test
    void caseNumericSelectorIndexesParameters() {
        // Official BeamNG documentation example: case(1, 'foo', 'bar', 'baz') → 'foo'
        assertStr("foo", eval("$=case(1, 'foo', 'bar', 'baz')"), "doc example case(1,'foo','bar','baz')");
        assertStr("bar", eval("$=case(2, 'foo', 'bar', 'baz')"), "case(2,...)");
        assertStr("baz", eval("$=case(3, 'foo', 'bar', 'baz')"), "case(3,...)");
        // numeric selector is floored before indexing
        assertStr("foo", eval("$=case(1.9, 'foo', 'bar', 'baz')"), "case(1.9,...) floors to 1");
        assertStr("bar", eval("$=case(2.1, 'foo', 'bar', 'baz')"), "case(2.1,...) floors to 2");
    }

    @Test
    void caseNumericSelectorOutOfRangeFallsBackToLast() {
        // arg[n] is nil for n <= 0 (Lua is 1-based) or n > #params → last parameter
        assertStr("baz", eval("$=case(0, 'foo', 'bar', 'baz')"), "index 0 → last");
        assertStr("baz", eval("$=case(5, 'foo', 'bar', 'baz')"), "index > #params → last");
        assertStr("baz", eval("$=case(-1, 'foo', 'bar', 'baz')"), "negative index → last");
        assertStr("baz", eval("$=case(100, 'foo', 'bar', 'baz')"), "huge index → last");
    }

    @Test
    void caseInvalidSelectorTypeFallsBackToLast() {
        // nil / string selectors are neither boolean nor number → default to last argument
        assertFloat(68, eval("$=case($components.noSections, 36, 44, 52, 60, 68)"), "nil selector → last");
        assertStr("baz", eval("$=case('x', 'foo', 'bar', 'baz')"), "string selector → last");
        assertStr("baz", eval("$=case($undefined_var, 'foo', 'bar', 'baz')"), "undefined var selector → last");
    }

    @Test
    void caseFalsySelectedBranchFallsBackToLast() {
        // `arg[index] or arg[last]`: a selected branch that is nil or literal false falls through
        assertStr("x", eval("$=case(true, false, 'x')"), "selected literal false → last");
        assertStr("x", eval("$=case(1, false, 'x')"), "selected false at index 1 → last");
        assertStr("", eval("$=case(true, '', 'x')"), "empty string is truthy in Lua → returned");
    }

    @Test
    void caseEvaluatesAllBranchesEagerly() {
        // Lua evaluates every case() argument before the call, so an undefined-variable branch
        // errors exactly as in the real engine. Real vehicles define the branch variables, so
        // the corpus-style ternary works when trackoffset_F is present.
        EvalOutcome out = eval("$=case(true, $missing+1, 0)", vars());
        assertEquals(EvalStatus.EVAL_ERROR, out.status(), "eager case errors on nil arithmetic in any branch");
        Map<String, Double> vars = vars("trackwidth_F", 1.6, "trackoffset_F", 0.0);
        assertEquals(1.6, JBeamParser.evaluateBeamNGExpression(CONDITIONAL_FRONT_OFFSET, vars), 1e-6);
    }

    @Test
    void realCorpusStyleCaseShapes() {
        // 真实语料：case 全部为 2-3 参布尔三元形式。numeric-index 形态按真实源码取 arg[floor(n)]。
        assertFloat(2, eval("$=case(round(2.7), 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 5)"),
                "round(2.7)=3 → arg[3]=2");
        assertFloat(0.25, eval("$=case($trackwidth_F == nil, $trackoffset_F+0.25, $trackwidth_F)", vars("trackoffset_F", 0.0)),
                "covet-style ternary");
        // 分支变量未定义 → eager case 报 EVAL_ERROR → 数值兼容路径返回 null（真实引擎同样报错）
        assertEquals(null, JBeamParser.evaluateBeamNGExpression(
                "$=case($rideheight_F == nil, ($springheight_F + 0.04) * 0.6, '')", vars("rideheight_F", 0.1)));
    }

    // ------------------------------------------------------------------
    // Functions
    // ------------------------------------------------------------------

    @Test
    void mathBuiltins() {
        assertFloat(5, eval("$=max(2, 5)"), "max(2,5)");
        assertFloat(2, eval("$=min(2, 5)"), "min(2,5)");
        assertFloat(4, eval("$=sqrt(16)"), "sqrt(16)");
        assertFloat(3, eval("$=round(2.7)"), "round(2.7)");
        assertFloat(3, eval("$=clamp(5, 1, 3)"), "clamp(5,1,3)");
        assertFloat(2, eval("$=abs(-2)"), "abs(-2)");
        assertFloat(3, eval("$=ceil(2.1)"), "ceil(2.1)");
        assertFloat(2, eval("$=floor(2.9)"), "floor(2.9)");
        assertFloat(1, eval("$=sin(pi/2)"), "sin(pi/2)");
        assertFloat(12, eval("$=max(3, 12, 5)"), "max(3,12,5)");
    }

    // ------------------------------------------------------------------
    // Documented JBeam expression builtins
    // ------------------------------------------------------------------

    /** Table-driven coverage of every scalar builtin (first return value where Lua returns multiple). */
    @Test
    void scalarBuiltinValueTable() {
        record Case(String expr, double expected) {}
        List<Case> cases = List.of(
                new Case("$=square(-3)", 9),
                new Case("$=square(0.5)", 0.25),
                new Case("$=sign(-0.001)", -1),
                new Case("$=sign(0)", 0),
                new Case("$=sign(0.001)", 1),
                new Case("$=acos(1)", 0),
                new Case("$=acos(0)", Math.PI / 2),
                new Case("$=asin(1)", Math.PI / 2),
                new Case("$=atan(0)", 0),
                new Case("$=atan(1)", Math.PI / 4),
                new Case("$=atan2(1, 0)", Math.PI / 2),
                new Case("$=atan2(0, 1)", 0),
                new Case("$=atan2(1, 1)", Math.PI / 4),
                new Case("$=cosh(0)", 1),
                new Case("$=cosh(1)", Math.cosh(1)),
                new Case("$=sinh(0)", 0),
                new Case("$=sinh(1)", Math.sinh(1)),
                new Case("$=tanh(0)", 0),
                new Case("$=tanh(1)", Math.tanh(1)),
                new Case("$=deg(pi)", 180),
                new Case("$=deg(pi/2)", 90),
                new Case("$=rad(180)", Math.PI),
                new Case("$=log10(100)", 2),
                new Case("$=log10(1)", 0),
                new Case("$=log(100, 10)", 2),
                new Case("$=log(8, 2)", 3),
                new Case("$=pow(2, 10)", 1024),
                new Case("$=pow(0.5, 2)", 0.25),
                new Case("$=fmod(5.5, 2)", 1.5),
                new Case("$=fmod(-7, 3)", -1),
                new Case("$=mod(7, 4)", 3),
                new Case("$=mod(-7, 4)", 1),
                new Case("$=ldexp(1, 10)", 1024),
                new Case("$=ldexp(3, 2)", 12),
                new Case("$=modf(3.7)", 3),
                new Case("$=modf(-2.7)", -2),
                new Case("$=frexp(8)", 0.5),
                new Case("$=frexp(0.25)", 0.5),
                new Case("$=frexp(1.5)", 0.75),
                new Case("$=round(-1.5)", -1),
                new Case("$=round(-2.6)", -3),
                new Case("$=clamp(5, 0, 1)", 1)
        );
        for (Case c : cases) {
            assertFloat(c.expected(), eval(c.expr()), c.expr());
        }
    }

    @Test
    void smoothstepFamilyBoundaries() {
        // JBeam compatibility: smoothstep clamps the input, smootherstep clamps the output,
        // smootheststep clamps the input; all three equal 0.5 at x=0.5
        assertFloat(0, eval("$=smoothstep(-1)"), "smoothstep(-1) → 0");
        assertFloat(1, eval("$=smoothstep(2)"), "smoothstep(2) → 1");
        assertFloat(0.5, eval("$=smoothstep(0.5)"), "smoothstep(0.5) = 0.5");
        assertFloat(0, eval("$=smootherstep(0)"), "smootherstep(0)");
        assertFloat(1, eval("$=smootherstep(1)"), "smootherstep(1)");
        assertFloat(0.5, eval("$=smootherstep(0.5)"), "smootherstep(0.5) = 0.5");
        assertFloat(0, eval("$=smootheststep(-1)"), "smootheststep(-1) → 0");
        assertFloat(1, eval("$=smootheststep(2)"), "smootheststep(2) → 1");
        assertFloat(0.5, eval("$=smootheststep(0.5)"), "smootheststep(0.5) = 0.5");
    }

    @Test
    void smoothminMatchesMathlib() {
        // h = clamp(0.5 + (b-a)/k, 0, 1); result = h*a + (1-h)*(b - h*k*0.5)
        assertFloat(1, eval("$=smoothmin(1, 2)"), "smoothmin(1,2) k=0.1 → h clamps to 1 → a");
        assertFloat(1, eval("$=smoothmin(2, 1)"), "smoothmin(2,1) → h clamps to 0 → b");
        assertFloat(0.2, eval("$=smoothmin(1, 2, 10)"), "smoothmin(1,2,10) blended h=0.6 → 0.2");
    }

    @Test
    void hugeConstantIsPositiveInfinity() {
        Object v = eval("$=huge").value();
        assertTrue(v instanceof Double && Double.isInfinite((Double) v), "$=huge → +inf");
        assertTrue(Double.isInfinite((Double) eval("$=huge/2").value()), "huge/2 → +inf");
        assertTrue((Boolean) eval("$=huge > 1e300").value(), "huge > 1e300");
    }

    @Test
    void printReturnsArgumentAndLabels() {
        assertFloat(42, eval("$=print(42)"), "print(42) returns 42");
        assertFloat(2.5, eval("$=print(2.5, 'x')"), "print(2.5, 'x') returns 2.5");
        assertStr("ok", eval("$=print('ok')"), "print('ok') returns 'ok'");
        EvalOutcome nil = eval("$=print($missing)");
        assertTrue(nil.ok() && nil.value() == null, "print(nil) returns nil");
    }

    @Test
    void vec3AndQuatAreUnsupportedNotSilent() {
        EvalOutcome v = eval("$=vec3(1, 2, 3)");
        assertEquals(EvalStatus.UNSUPPORTED, v.status(), "vec3 returns a table — outside the scalar value model");
        EvalOutcome q = eval("$=quat(1, 0, 0, 0)");
        assertEquals(EvalStatus.UNSUPPORTED, q.status(), "quat returns a table — outside the scalar value model");
        assertTrue(v.reason() != null && !v.reason().isEmpty(), "unsupported must carry a reason");
    }

    @Test
    void concatErrorsLikeLuaWithoutTables() {
        EvalOutcome out = eval("$=concat('a', 'b')");
        assertEquals(EvalStatus.EVAL_ERROR, out.status(), "concat on a non-table first arg is a Lua error");
        assertTrue(out.reason() != null && out.reason().contains("table expected"), out.reason());
    }

    @Test
    void wrongArityFunctionsErrorLikeLua() {
        // Lua 5.1: atan is single-argument (two-arg lives in atan2); fmod/pow need two
        assertEquals(EvalStatus.EVAL_ERROR, eval("$=atan(1, 2)").status(), "atan(y, x) does not exist in Lua 5.1");
        assertEquals(EvalStatus.EVAL_ERROR, eval("$=fmod(5)").status(), "fmod(x) is missing its second argument");
        assertEquals(EvalStatus.EVAL_ERROR, eval("$=pow(2)").status(), "pow(x) is missing its second argument");
    }

    @Test
    void randomBuiltinsStayInRange() {
        Double r0 = (Double) eval("$=random()").value();
        assertTrue(r0 >= 0 && r0 < 1, "random() in [0,1)");
        Double r1 = (Double) eval("$=random(50,100)").value();
        assertTrue(r1 >= 50 && r1 <= 100, "random(50,100) in [50,100]");
        Double r2 = (Double) eval("$=random(-10,10)").value();
        assertTrue(r2 >= -10 && r2 <= 10, "random(-10,10) in [-10,10]");
        Double rot = (Double) eval("$=round(random())*180").value();
        assertTrue(rot == 0 || rot == 180, "round(random())*180 ∈ {0,180}");
    }

    @Test
    void randomOneArgIsInclusiveOneToX() {
        // Lua 5.1: math.random(m) → integer in [1, m]
        Double r = (Double) eval("$=random(6)").value();
        assertTrue(r >= 1 && r <= 6, "random(6) in [1,6], got " + r);
        Double one = (Double) eval("$=random(1)").value();
        assertEquals(1.0, one, "random(1) is always 1");
        EvalOutcome bad = eval("$=random(0)");
        assertEquals(EvalStatus.EVAL_ERROR, bad.status(), "random(0) is an empty interval in Lua");
        EvalOutcome bad2 = eval("$=random(10, 5)");
        assertEquals(EvalStatus.EVAL_ERROR, bad2.status(), "random(10,5) is an empty interval in Lua");
    }

    @Test
    void unknownFunctionIsUnsupportedNotSilent() {
        EvalOutcome out = eval("$=include('vehicles/common/torque.csv')");
        assertNotEquals(EvalStatus.OK, out.status(), "include() must be reported unsupported");
        assertEquals(EvalStatus.UNSUPPORTED, out.status(), "include() is engine-specific");
        assertTrue(out.reason() != null && !out.reason().isEmpty(), "unsupported must carry a reason");

        EvalOutcome unknown = eval("$=frobnicate(1, 2)");
        assertEquals(EvalStatus.UNSUPPORTED, unknown.status(), "unknown function is unsupported");
    }

    @Test
    void malformedExpressionsNeverThrow() {
        // evaluator 对所有语法/求值失败返回 outcome，绝不抛出
        assertNotEquals(EvalStatus.OK, eval("$=(1+").status());
        assertNotEquals(EvalStatus.OK, eval("$=1 +* 2").status());
        assertNotEquals(EvalStatus.OK, eval("$='unterminated").status());
        assertNotEquals(EvalStatus.OK, eval("$=@#$%").status());
        assertNotEquals(EvalStatus.OK, eval("$=nil + 1").status());
    }

    @Test
    void unsupportedNumericFieldFallsBackToDefault() {
        // $=include(...) 在数值字段中：compat 返回 null → 调用方回退默认值
        assertEquals(null, JBeamParser.evaluateBeamNGExpression("$=include('vehicles/common/x.csv')", Map.of()));
        assertEquals(5.0f, JBeamParser.getFloatSafe(jsonObj("spring", "$=include('x.csv')"), "spring", 5.0f, Map.of()), 1e-6f);
    }

    @Test
    void failingExpressionsEmitDeduplicatedDiagnostics() {
        ExpressionDiagnostics.reset();
        int before = ExpressionDiagnostics.loggedCount();
        String expr = "$=include('vehicles/common/torque.csv')";
        for (int i = 0; i < 3; i++) {
            JBeamParser.evaluateBeamNGExpression(expr, Map.of());
        }
        // 相同 (表达式, 原因) 只记录一次
        assertEquals(before + 1, ExpressionDiagnostics.loggedCount(), "duplicate failures must be deduplicated");
        ExpressionDiagnostics.reset();
    }

    @Test
    void diagnosticsAreThrottledToCap() {
        ExpressionDiagnostics.reset();
        for (int i = 0; i < 200; i++) {
            JBeamParser.evaluateBeamNGExpression("$=unknown_fn_" + i + "(1)", Map.of());
        }
        assertTrue(ExpressionDiagnostics.loggedCount() <= 40, "distinct failures must be capped");
        ExpressionDiagnostics.reset();
    }

    private static com.google.gson.JsonObject jsonObj(String key, String value) {
        com.google.gson.JsonObject o = new com.google.gson.JsonObject();
        o.addProperty(key, value);
        return o;
    }

    @Test
    void concatNumberStringFormat() {
        // Lua-style: integral numbers → no decimal point
        EvalOutcome half = eval("$='x'..(2.5)");
        assertTrue(half.ok(), half.reason());
        assertEquals("x2.5", half.value());
        EvalOutcome integral = eval("$='x'..(2.0)");
        assertEquals("x2", integral.value());
    }
}
