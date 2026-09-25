/*
 * This Source Code Form is subject to the terms of the bCDDL, v. 1.1.
 * If a copy of the bCDDL was not distributed with this file, see
 * LICENSES/bCDDL-1.1.txt.
 *
 * Adapted from BeamNG.drive lua/common/jbeam/expressionParser.lua and
 * lua/common/mathlib.lua. Java adaptation and modifications contributed by
 * M1AO and BeamCraft contributors.
 */
package me.mzy.beamcraft.client.physics;

import java.util.Random;

import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalException;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalStatus;

/** BeamNG-compatible function environment used by the independent expression parser. */
final class JBeamExpressionBuiltins {
    interface Arguments {
        int size();

        Object evaluate(int index) throws EvalException;
    }

    private static final Random RAND = new Random(0x5EED_2026L);

    private JBeamExpressionBuiltins() {
    }

    static synchronized void setRandomSeed(long seed) {
        RAND.setSeed(seed);
    }

    static Object invoke(String name, Arguments args) throws EvalException {
        switch (name) {
            case "case":
                return evalCase(args);
            case "random":
                return random(args);
            case "randomseed":
                return randomSeed(args);
            case "print":
                return print(args);
            case "include":
                throw error(EvalStatus.UNSUPPORTED,
                        "include() reads external CSV files at game load and is not supported");
            case "vec3":
            case "quat":
                throw error(EvalStatus.UNSUPPORTED,
                        name + "() constructs a vector/quaternion table, which is not representable "
                                + "in BeamCraft's scalar value model (Double/String/Boolean/nil)");
            case "concat":
                return concat(args);
            default:
                break;
        }

        double a;
        double b;
        switch (name) {
            case "round": a = number(single(name, args)); return Math.floor(a + 0.5);
            case "abs": a = number(single(name, args)); return Math.abs(a);
            case "ceil": a = number(single(name, args)); return Math.ceil(a);
            case "floor": a = number(single(name, args)); return Math.floor(a);
            case "sqrt": a = number(single(name, args)); return Math.sqrt(a);
            case "sin": a = number(single(name, args)); return Math.sin(a);
            case "cos": a = number(single(name, args)); return Math.cos(a);
            case "tan": a = number(single(name, args)); return Math.tan(a);
            case "exp": a = number(single(name, args)); return Math.exp(a);
            case "log":
                a = number(single(name, args));
                if (args.size() >= 2) {
                    b = number(argument(name, args, 1));
                    return Math.log(a) / Math.log(b);
                }
                return Math.log(a);
            case "square": a = number(single(name, args)); return a * a;
            case "smoothstep": a = number(single(name, args)); return smoothstep(a);
            case "smootherstep": a = number(single(name, args)); return smootherstep(a);
            case "smootheststep": a = number(single(name, args)); return smootheststep(a);
            case "sign": a = number(single(name, args)); return Math.signum(a);
            case "acos": a = number(single(name, args)); return Math.acos(a);
            case "asin": a = number(single(name, args)); return Math.asin(a);
            case "atan":
                if (args.size() != 1) throw error(EvalStatus.EVAL_ERROR, "wrong number of arguments to 'atan'");
                return Math.atan(number(single(name, args)));
            case "cosh": a = number(single(name, args)); return Math.cosh(a);
            case "sinh": a = number(single(name, args)); return Math.sinh(a);
            case "tanh": a = number(single(name, args)); return Math.tanh(a);
            case "deg": a = number(single(name, args)); return Math.toDegrees(a);
            case "rad": a = number(single(name, args)); return Math.toRadians(a);
            case "log10": a = number(single(name, args)); return Math.log10(a);
            case "frexp": a = number(single(name, args)); return frexpMantissa(a);
            case "modf": a = number(single(name, args)); return modfIntegerPart(a);
            case "atan2":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                return Math.atan2(a, b);
            case "fmod":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                return a % b;
            case "mod":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                return a - Math.floor(a / b) * b;
            case "ldexp":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                return Math.scalb(a, (int) b);
            case "pow":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                return Math.pow(a, b);
            case "smoothmin":
                return smoothmin(args);
            case "min":
            case "max":
                return minMax(name, args);
            case "clamp":
                a = number(single(name, args));
                b = number(argument(name, args, 1));
                double max = number(argument(name, args, 2));
                return Math.max(b, Math.min(a, max));
            default:
                throw error(EvalStatus.UNSUPPORTED, "unknown function '" + name + "'");
        }
    }

    private static Object single(String name, Arguments args) throws EvalException {
        return argument(name, args, 0);
    }

    private static Object argument(String name, Arguments args, int index) throws EvalException {
        if (index >= args.size()) {
            throw error(EvalStatus.EVAL_ERROR, name + "() requires " + (index + 1) + " argument(s)");
        }
        return args.evaluate(index);
    }

    private static Object minMax(String name, Arguments args) throws EvalException {
        if (args.size() == 0) throw error(EvalStatus.EVAL_ERROR, name + "() requires at least one argument");
        double result = number(args.evaluate(0));
        for (int i = 1; i < args.size(); i++) {
            double value = number(args.evaluate(i));
            result = name.equals("max") ? Math.max(result, value) : Math.min(result, value);
        }
        return result;
    }

    private static Object random(Arguments args) throws EvalException {
        if (args.size() == 0) {
            synchronized (RAND) {
                return RAND.nextDouble();
            }
        }
        if (args.size() > 2) throw error(EvalStatus.EVAL_ERROR, "wrong number of arguments to 'random'");
        double lower = number(args.evaluate(0));
        if (args.size() == 1) {
            if (lower < 1.0) {
                throw error(EvalStatus.EVAL_ERROR,
                        "random(" + lower + ") interval is empty (upper bound must be >= 1)");
            }
            synchronized (RAND) {
                return Math.floor(RAND.nextDouble() * lower) + 1.0;
            }
        }
        double upper = number(args.evaluate(1));
        if (lower > upper) {
            throw error(EvalStatus.EVAL_ERROR,
                    "random(" + lower + ", " + upper + ") interval is empty");
        }
        synchronized (RAND) {
            return Math.floor(RAND.nextDouble() * (upper - lower + 1.0)) + lower;
        }
    }

    private static Object randomSeed(Arguments args) throws EvalException {
        if (args.size() == 1) {
            Object value = args.evaluate(0);
            if (value instanceof Number number) {
                synchronized (RAND) {
                    RAND.setSeed((long) number.doubleValue());
                }
            }
        }
        return null;
    }

    private static Object print(Arguments args) throws EvalException {
        Object value = args.size() == 0 ? null : args.evaluate(0);
        String label = args.size() > 1 ? string(args.evaluate(1)) : null;
        String line = label != null ? label + " = " + luaTostring(value) : luaTostring(value);
        System.out.println("[BeamExpression] " + line);
        return value;
    }

    private static Object concat(Arguments args) throws EvalException {
        if (args.size() == 0) {
            throw error(EvalStatus.EVAL_ERROR,
                    "bad argument #1 to 'concat' (table expected, got no value)");
        }
        Object first = args.evaluate(0);
        throw error(EvalStatus.EVAL_ERROR,
                "bad argument #1 to 'concat' (table expected, got " + typeName(first) + ")");
    }

    private static Object evalCase(Arguments args) throws EvalException {
        if (args.size() == 0) return null;
        Object selector = args.evaluate(0);
        int count = args.size() - 1;
        Object[] choices = new Object[count];
        for (int i = 0; i < count; i++) choices[i] = args.evaluate(i + 1);

        int index;
        if (selector instanceof Boolean value) {
            index = value ? 1 : 2;
        } else if (selector instanceof Number number) {
            index = (int) Math.floor(number.doubleValue());
        } else {
            index = 0;
        }

        Object selected = index >= 1 && index <= count ? choices[index - 1] : null;
        if (selected != null && !Boolean.FALSE.equals(selected)) return selected;
        return count >= 1 ? choices[count - 1] : null;
    }

    private static double smoothstep(double value) {
        value = Math.max(0, Math.min(1, value));
        return value * value * (3 - 2 * value);
    }

    private static double smootherstep(double value) {
        double result = value * value * value * (value * (value * 6 - 15) + 10);
        return Math.max(0, Math.min(1, result));
    }

    private static double smootheststep(double value) {
        value = Math.max(0, Math.min(1, value));
        double squared = value * value;
        return squared * squared * (35 - value * (value * (value * 20 - 70) + 84));
    }

    private static Object smoothmin(Arguments args) throws EvalException {
        double a = number(single("smoothmin", args));
        double b = number(argument("smoothmin", args, 1));
        double k = 0.1;
        if (args.size() > 2) {
            Object value = args.evaluate(2);
            if (value != null) k = number(value);
        }
        double h = Math.max(0, Math.min(1, 0.5 + (b - a) / k));
        return h * a + (1 - h) * (b - h * k * 0.5);
    }

    private static double frexpMantissa(double value) {
        if (value == 0.0 || Double.isInfinite(value) || Double.isNaN(value)) return value;
        double mantissa = Math.scalb(value, -Math.getExponent(value));
        if (mantissa >= 1.0) mantissa = Math.scalb(mantissa, -1);
        return mantissa;
    }

    private static double modfIntegerPart(double value) {
        return value > 0 ? Math.floor(value) : Math.ceil(value);
    }

    private static double number(Object value) throws EvalException {
        return JBeamExpressionEvaluator.asNumber(value);
    }

    private static String string(Object value) throws EvalException {
        return JBeamExpressionEvaluator.asString(value);
    }

    private static String typeName(Object value) {
        return JBeamExpressionEvaluator.typeName(value);
    }

    private static String luaTostring(Object value) {
        if (value == null) return "nil";
        if (value instanceof String string) return string;
        if (value instanceof Number number) {
            return JBeamExpressionEvaluator.numToString(number.doubleValue());
        }
        if (value instanceof Boolean bool) return bool ? "true" : "false";
        return value.toString();
    }

    private static EvalException error(EvalStatus status, String message) {
        return JBeamExpressionEvaluator.err(status, message);
    }
}
