package me.mzy.beamcraft.client.physics;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Tokenizer + precedence-parser evaluator for BeamNG JBeam {@code $=} expressions.
 *
 * <p>Implements the documented core grammar with Lua semantics (BeamNG expressions are a
 * sandboxed Lua subset): numbers / strings / booleans / nil, {@code + - * / % ^}, unary
 * {@code - not #}, comparisons {@code == ~= < > <= >=}, {@code and or}, string
 * concatenation {@code ..}, parentheses, variables ({@code $name}, dotted component access
 * like {@code $components.foo.bar}), function calls, and the {@code case()} selector form.
 * {@code and}/{@code or} return their operands and are lazy (Lua semantics). The documented
 * JBeam {@code case()} function evaluates all of its arguments eagerly (Lua evaluates function
 * arguments before the call) and dispatches on the first argument —
 * a {@code boolean} behaves as a ternary, a {@code number} indexes into the remaining
 * arguments (1 = second parameter), and any other / out-of-range / nil-selected index falls
 * back to the last argument. It is <em>not</em> a key-value switch and is not lazy.
 *
 * <p>Exposed builtins mirror the documented JBeam expression context: the Lua {@code math}
 * library ({@code abs ceil floor sqrt sin cos
 * tan exp log log10 acos asin atan atan2 cosh sinh tanh deg rad fmod frexp ldexp modf pow huge
 * max min random randomseed pi ...}) plus {@code round clamp square smoothstep smootherstep
 * smootheststep smoothmin sign case concat print vec3 quat include}. {@code log} accepts an
 * optional base ({@code log(x, base) = ln(x)/ln(base)}); {@code random(x)} returns an integer
 * in {@code [1, x]}, {@code random(a, b)} in {@code [a, b]} exactly like Lua 5.1.
 *
 * <p>Functions that depend on game-runtime state or external files (currently
 * {@code include()}, which reads external CSVs) are reported as {@link EvalStatus#UNSUPPORTED}
 * rather than silently mis-evaluated. {@code vec3()}/{@code quat()} construct table values that
 * the scalar value model ({@code Double}/{@code String}/{@code Boolean}/{@code null}) cannot
 * represent, so they are reported {@link EvalStatus#UNSUPPORTED}; {@code concat} is
 * {@code table.concat} and therefore errors like Lua when handed a non-table first argument.
 * {@code frexp()}/{@code modf()} return Lua's <em>first</em> return value (mantissa / integer
 * part), which is all a single-value expression context can observe. {@code print()} prints
 * {@code [label = ] value} to stdout and returns its first argument unchanged. All failures —
 * including syntax errors and unexpected internal errors — are captured into an
 * {@link EvalOutcome}; {@link #evaluate} never throws.
 */
public final class JBeamExpressionEvaluator {

    private JBeamExpressionEvaluator() {}

    /** Classification of an evaluation result. */
    public enum EvalStatus {
        /** Evaluated successfully (value may be nil for a bare undefined variable). */
        OK,
        /** Lexer/parser rejected the expression. */
        SYNTAX_ERROR,
        /** Well-formed expression that failed at runtime (e.g. arithmetic on nil). */
        EVAL_ERROR,
        /** Well-formed expression calling a function this evaluator deliberately does not support. */
        UNSUPPORTED,
        /** Unexpected internal failure — treated as a bug. */
        INTERNAL
    }

    /** Outcome of an evaluation: the value (null = nil) plus status and a human-readable reason. */
    public record EvalOutcome(Object value, EvalStatus status, String reason) {
        public boolean ok() {
            return status == EvalStatus.OK;
        }
    }

    // ------------------------------------------------------------------
    // Value model helpers (values are Double / String / Boolean / null=nil)
    // ------------------------------------------------------------------

    /** Lua truthiness: only nil and false are falsy. */
    static boolean truthy(Object v) {
        return v != null && !Boolean.FALSE.equals(v);
    }

    static String typeName(Object v) {
        if (v == null) return "nil";
        if (v instanceof Double || v instanceof Integer || v instanceof Float || v instanceof Long) return "number";
        if (v instanceof String) return "string";
        if (v instanceof Boolean) return "boolean";
        return v.getClass().getSimpleName();
    }

    static double asNumber(Object v) throws EvalException {
        if (v instanceof Number n) return n.doubleValue();
        if (v instanceof String s) {
            try {
                return Double.parseDouble(s.trim());
            } catch (NumberFormatException e) {
                throw err(EvalStatus.EVAL_ERROR, "cannot convert string to number: '" + s + "'");
            }
        }
        throw err(EvalStatus.EVAL_ERROR, "expected number, got " + typeName(v));
    }

    /** Lua-style tostring for a number: integral values drop the decimal point. */
    static String numToString(double d) {
        if (!Double.isInfinite(d) && !Double.isNaN(d) && d == Math.rint(d) && Math.abs(d) < 1e15) {
            return Long.toString((long) d);
        }
        return Double.toString(d);
    }

    static String asString(Object v) throws EvalException {
        if (v == null) throw err(EvalStatus.EVAL_ERROR, "attempt to concatenate a nil value");
        if (v instanceof String s) return s;
        if (v instanceof Number n) return numToString(n.doubleValue());
        if (v instanceof Boolean b) return b ? "true" : "false";
        throw err(EvalStatus.EVAL_ERROR, "cannot concatenate " + typeName(v));
    }

    static boolean luaEquals(Object a, Object b) {
        if (a == null && b == null) return true;
        if (a == null || b == null) return false;
        if (a instanceof Number na && b instanceof Number nb) return na.doubleValue() == nb.doubleValue();
        return a.equals(b);
    }

    static boolean luaCompare(Object a, Object b, String op) throws EvalException {
        if (a instanceof Number && b instanceof Number) {
            double x = ((Number) a).doubleValue();
            double y = ((Number) b).doubleValue();
            return switch (op) {
                case "<" -> x < y;
                case ">" -> x > y;
                case "<=" -> x <= y;
                case ">=" -> x >= y;
                default -> throw err(EvalStatus.EVAL_ERROR, "bad comparison operator " + op);
            };
        }
        if (a instanceof String sa && b instanceof String sb) {
            int c = sa.compareTo(sb);
            return switch (op) {
                case "<" -> c < 0;
                case ">" -> c > 0;
                case "<=" -> c <= 0;
                case ">=" -> c >= 0;
                default -> throw err(EvalStatus.EVAL_ERROR, "bad comparison operator " + op);
            };
        }
        throw err(EvalStatus.EVAL_ERROR, "attempt to compare " + typeName(a) + " with " + typeName(b));
    }

    // ------------------------------------------------------------------
    // Tokenizer
    // ------------------------------------------------------------------

    enum Tok {
        NUMBER, STRING, IDENT, DOLLAR_VAR,
        KW_AND, KW_OR, KW_NOT, KW_NIL, KW_TRUE, KW_FALSE,
        PLUS, MINUS, STAR, SLASH, PERCENT, CARET, CONCAT,
        EQ, NEQ, LT, GT, LE, GE, HASH,
        LPAREN, RPAREN, COMMA, EOF
    }

    record Token(Tok type, String text, double num, String str, int pos) {
        static Token simple(Tok t, String text, int pos) {
            return new Token(t, text, 0, null, pos);
        }
    }

    private static final class Lexer {
        private final String src;
        private int pos = 0;

        Lexer(String src) {
            this.src = src == null ? "" : src;
        }

        boolean atEnd() {
            return pos >= src.length();
        }

        char peek() {
            return atEnd() ? '\0' : src.charAt(pos);
        }

        char peekAt(int off) {
            int p = pos + off;
            return (p < 0 || p >= src.length()) ? '\0' : src.charAt(p);
        }

        char next() {
            char c = peek();
            if (!atEnd()) pos++;
            return c;
        }

        boolean isIdentStart(char c) {
            return Character.isLetter(c) || c == '_' || Character.isDigit(c);
        }

        boolean isIdentPart(char c) {
            return Character.isLetterOrDigit(c) || c == '_';
        }

        Token nextToken() throws EvalException {
            while (!atEnd() && Character.isWhitespace(peek())) next();

            if (atEnd()) return Token.simple(Tok.EOF, "", pos);

            int start = pos;
            char c = peek();

            if (c == '\'' || c == '"') return lexString(c);

            if (Character.isDigit(c) || (c == '.' && Character.isDigit(peekAt(1)))) {
                return lexNumber();
            }

            if (c == '$') {
                next(); // '$'
                if (!isIdentStart(peek())) {
                    throw err(EvalStatus.SYNTAX_ERROR, "bad variable reference after '$' at position " + pos);
                }
                StringBuilder sb = new StringBuilder();
                while (isIdentPart(peek())) sb.append(next());
                // dotted access, but never swallow the '..' concatenation operator
                while (peek() == '.' && peekAt(1) != '.' && isIdentStart(peekAt(1))) {
                    sb.append('.');
                    next(); // '.'
                    while (isIdentPart(peek())) sb.append(next());
                }
                String name = sb.toString();
                return new Token(Tok.DOLLAR_VAR, name, 0, null, start);
            }

            if (Character.isLetter(c) || c == '_') {
                StringBuilder sb = new StringBuilder();
                while (isIdentPart(peek())) sb.append(next());
                String w = sb.toString();
                return switch (w) {
                    case "and" -> Token.simple(Tok.KW_AND, w, start);
                    case "or" -> Token.simple(Tok.KW_OR, w, start);
                    case "not" -> Token.simple(Tok.KW_NOT, w, start);
                    case "nil" -> Token.simple(Tok.KW_NIL, w, start);
                    case "true" -> Token.simple(Tok.KW_TRUE, w, start);
                    case "false" -> Token.simple(Tok.KW_FALSE, w, start);
                    default -> new Token(Tok.IDENT, w, 0, null, start);
                };
            }

            next();
            return switch (c) {
                case '+' -> Token.simple(Tok.PLUS, "+", start);
                case '-' -> Token.simple(Tok.MINUS, "-", start);
                case '*' -> Token.simple(Tok.STAR, "*", start);
                case '/' -> Token.simple(Tok.SLASH, "/", start);
                case '%' -> Token.simple(Tok.PERCENT, "%", start);
                case '^' -> Token.simple(Tok.CARET, "^", start);
                case '(' -> Token.simple(Tok.LPAREN, "(", start);
                case ')' -> Token.simple(Tok.RPAREN, ")", start);
                case ',' -> Token.simple(Tok.COMMA, ",", start);
                case '#' -> Token.simple(Tok.HASH, "#", start);
                case '.' -> {
                    if (peek() == '.') { next(); yield Token.simple(Tok.CONCAT, "..", start); }
                    throw err(EvalStatus.SYNTAX_ERROR, "unexpected '.' at position " + start);
                }
                case '=' -> {
                    if (peek() == '=') { next(); yield Token.simple(Tok.EQ, "==", start); }
                    throw err(EvalStatus.SYNTAX_ERROR, "single '=' is not a valid expression operator at position " + start);
                }
                case '~' -> {
                    if (peek() == '=') { next(); yield Token.simple(Tok.NEQ, "~=", start); }
                    throw err(EvalStatus.SYNTAX_ERROR, "unexpected '~' at position " + start);
                }
                case '<' -> {
                    if (peek() == '=') { next(); yield Token.simple(Tok.LE, "<=", start); }
                    yield Token.simple(Tok.LT, "<", start);
                }
                case '>' -> {
                    if (peek() == '=') { next(); yield Token.simple(Tok.GE, ">=", start); }
                    yield Token.simple(Tok.GT, ">", start);
                }
                default -> throw err(EvalStatus.SYNTAX_ERROR, "unexpected character '" + c + "' at position " + start);
            };
        }

        private Token lexString(char quote) throws EvalException {
            int start = pos;
            next(); // opening quote
            StringBuilder sb = new StringBuilder();
            while (true) {
                if (atEnd()) throw err(EvalStatus.SYNTAX_ERROR, "unterminated string at position " + start);
                char c = next();
                if (c == quote) break;
                if (c == '\\') {
                    if (atEnd()) throw err(EvalStatus.SYNTAX_ERROR, "unterminated escape at position " + start);
                    char e = next();
                    switch (e) {
                        case 'n' -> sb.append('\n');
                        case 't' -> sb.append('\t');
                        case 'r' -> sb.append('\r');
                        case '\\' -> sb.append('\\');
                        case '\'' -> sb.append('\'');
                        case '"' -> sb.append('"');
                        default -> sb.append(e);
                    }
                } else {
                    sb.append(c);
                }
            }
            return new Token(Tok.STRING, null, 0, sb.toString(), start);
        }

        private Token lexNumber() throws EvalException {
            int start = pos;
            StringBuilder sb = new StringBuilder();
            while (Character.isDigit(peek())) sb.append(next());
            if (peek() == '.' && peekAt(1) != '.') {
                sb.append(next());
                while (Character.isDigit(peek())) sb.append(next());
            }
            char c = peek();
            if (c == 'e' || c == 'E') {
                int save = pos;
                StringBuilder exp = new StringBuilder().append(next());
                if (peek() == '+' || peek() == '-') exp.append(next());
                if (Character.isDigit(peek())) {
                    while (Character.isDigit(peek())) exp.append(next());
                    sb.append(exp);
                } else {
                    pos = save; // not an exponent after all
                }
            }
            try {
                return new Token(Tok.NUMBER, sb.toString(), Double.parseDouble(sb.toString()), null, start);
            } catch (NumberFormatException e) {
                throw err(EvalStatus.SYNTAX_ERROR, "malformed number '" + sb + "' at position " + start);
            }
        }
    }

    // ------------------------------------------------------------------
    // AST
    // ------------------------------------------------------------------

    private sealed interface Node permits Lit, Var, Unary, Binary, Call {}

    private record Lit(Object value) implements Node {}

    private record Var(String path) implements Node {}

    private record Unary(Tok op, Node operand) implements Node {}

    private record Binary(Tok op, Node left, Node right) implements Node {}

    private record Call(String name, List<Node> args) implements Node {}

    // ------------------------------------------------------------------
    // Parser (Pratt / precedence climbing)
    // ------------------------------------------------------------------

    private static final int PREC_OR = 1;
    private static final int PREC_AND = 2;
    private static final int PREC_CMP = 3;
    private static final int PREC_CONCAT = 4; // right-assoc
    private static final int PREC_ADD = 5;
    private static final int PREC_MUL = 6;
    private static final int PREC_UNARY = 7;
    private static final int PREC_POW = 8; // right-assoc

    private static final class Parser {
        private final Lexer lexer;
        private Token cur;

        Parser(Lexer lexer) throws EvalException {
            this.lexer = lexer;
            advance();
        }

        void advance() throws EvalException {
            cur = lexer.nextToken();
        }

        Node parseExpression() throws EvalException {
            return parseExpr(PREC_OR);
        }

        Node parseExpr(int minPrec) throws EvalException {
            Node left = parseUnary();
            while (true) {
                Tok t = cur.type();
                int prec = binaryPrec(t);
                if (prec < 0 || prec < minPrec) break;
                boolean rightAssoc = (t == Tok.CONCAT || t == Tok.CARET);
                advance();
                Node right = parseExpr(rightAssoc ? prec : prec + 1);
                left = new Binary(t, left, right);
            }
            return left;
        }

        private static int binaryPrec(Tok t) {
            return switch (t) {
                case KW_OR -> PREC_OR;
                case KW_AND -> PREC_AND;
                case EQ, NEQ, LT, GT, LE, GE -> PREC_CMP;
                case CONCAT -> PREC_CONCAT;
                case PLUS, MINUS -> PREC_ADD;
                case STAR, SLASH, PERCENT -> PREC_MUL;
                case CARET -> PREC_POW;
                default -> -1;
            };
        }

        Node parseUnary() throws EvalException {
            Tok t = cur.type();
            if (t == Tok.MINUS || t == Tok.KW_NOT || t == Tok.HASH) {
                advance();
                return new Unary(t, parseExpr(PREC_UNARY));
            }
            return parsePrimary();
        }

        Node parsePrimary() throws EvalException {
            Token t = cur;
            switch (t.type()) {
                case NUMBER -> { advance(); return new Lit(t.num()); }
                case STRING -> { advance(); return new Lit(t.str()); }
                case KW_TRUE -> { advance(); return new Lit(Boolean.TRUE); }
                case KW_FALSE -> { advance(); return new Lit(Boolean.FALSE); }
                case KW_NIL -> { advance(); return new Lit(null); }
                case DOLLAR_VAR -> { advance(); return new Var(t.text()); }
                case IDENT -> {
                    advance();
                    // math table globals exposed as bare identifiers by the expression context
                    if (t.text().equals("pi")) return new Lit(Math.PI);
                    if (t.text().equals("huge")) return new Lit(Double.POSITIVE_INFINITY);
                    if (cur.type() == Tok.LPAREN) {
                        return new Call(t.text(), parseArgs());
                    }
                    return new Var(t.text());
                }
                case LPAREN -> {
                    advance();
                    Node inner = parseExpr(PREC_OR);
                    expect(Tok.RPAREN, "expected ')'");
                    return inner;
                }
                default -> throw err(EvalStatus.SYNTAX_ERROR,
                        "unexpected token '" + display(t) + "' at position " + t.pos());
            }
        }

        List<Node> parseArgs() throws EvalException {
            expect(Tok.LPAREN, "expected '('");
            List<Node> args = new ArrayList<>();
            if (cur.type() == Tok.RPAREN) {
                advance();
                return args;
            }
            while (true) {
                args.add(parseExpr(PREC_OR));
                if (cur.type() == Tok.RPAREN) { advance(); break; }
                expect(Tok.COMMA, "expected ',' or ')' in argument list");
            }
            return args;
        }

        void expectEof() throws EvalException {
            if (cur.type() != Tok.EOF) {
                throw err(EvalStatus.SYNTAX_ERROR,
                        "unexpected trailing content '" + display(cur) + "' at position " + cur.pos());
            }
        }

        private void expect(Tok t, String msg) throws EvalException {
            if (cur.type() != t) throw err(EvalStatus.SYNTAX_ERROR, msg + " (found '" + display(cur) + "')");
            advance();
        }

        private static String display(Token t) {
            if (t.text() != null) return t.text();
            if (t.str() != null) return "'" + t.str() + "'";
            return t.type().name();
        }
    }

    // ------------------------------------------------------------------
    // Evaluator
    // ------------------------------------------------------------------

    /** Seed the shared random stream used by {@code random()}/{@code randomseed()}. */
    public static void setRandomSeed(long seed) {
        JBeamExpressionBuiltins.setRandomSeed(seed);
    }

    static Object eval(Node n, BeamExpressionContext ctx) throws EvalException {
        return switch (n) {
            case Lit(Object v) -> v;
            case Var(String path) -> ctx.get(path);
            case Unary(Tok op, Node operand) -> evalUnary(op, operand, ctx);
            case Binary(Tok op, Node left, Node right) -> evalBinary(op, left, right, ctx);
            case Call(String name, List<Node> args) -> evalCall(name, args, ctx);
        };
    }

    private static Object evalUnary(Tok op, Node operand, BeamExpressionContext ctx) throws EvalException {
        Object v = eval(operand, ctx);
        return switch (op) {
            case MINUS -> -asNumber(v);
            case KW_NOT -> !truthy(v);
            case HASH -> {
                if (v instanceof String s) {
                    yield (double) s.length();
                }
                throw err(EvalStatus.EVAL_ERROR, "length operator (#) is only supported on strings, got " + typeName(v));
            }
            default -> throw err(EvalStatus.SYNTAX_ERROR, "bad unary operator");
        };
    }

    private static Object evalBinary(Tok op, Node left, Node right, BeamExpressionContext ctx) throws EvalException {
        switch (op) {
            case KW_AND -> {
                Object l = eval(left, ctx);
                return truthy(l) ? eval(right, ctx) : l;
            }
            case KW_OR -> {
                Object l = eval(left, ctx);
                return truthy(l) ? l : eval(right, ctx);
            }
            default -> { }
        }

        Object a = eval(left, ctx);
        Object b = eval(right, ctx);
        return switch (op) {
            case CONCAT -> asString(a) + asString(b);
            case PLUS -> asNumber(a) + asNumber(b);
            case MINUS -> asNumber(a) - asNumber(b);
            case STAR -> asNumber(a) * asNumber(b);
            case SLASH -> asNumber(a) / asNumber(b);
            case PERCENT -> {
                double x = asNumber(a);
                double y = asNumber(b);
                yield x - Math.floor(x / y) * y; // Lua % semantics
            }
            case CARET -> Math.pow(asNumber(a), asNumber(b));
            case EQ -> luaEquals(a, b);
            case NEQ -> !luaEquals(a, b);
            case LT -> luaCompare(a, b, "<");
            case GT -> luaCompare(a, b, ">");
            case LE -> luaCompare(a, b, "<=");
            case GE -> luaCompare(a, b, ">=");
            default -> throw err(EvalStatus.SYNTAX_ERROR, "bad binary operator");
        };
    }

    private static Object evalCall(String name, List<Node> args, BeamExpressionContext ctx) throws EvalException {
        return JBeamExpressionBuiltins.invoke(name, new JBeamExpressionBuiltins.Arguments() {
            @Override
            public int size() {
                return args.size();
            }

            @Override
            public Object evaluate(int index) throws EvalException {
                return eval(args.get(index), ctx);
            }
        });
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /**
     * Evaluate an expression. Accepts an optional {@code $=} prefix. Undefined variables
     * resolve to {@code nil} (Lua semantics). Never throws — every failure, including
     * unexpected internal errors, is folded into an {@link EvalOutcome}.
     */
    public static EvalOutcome evaluate(String expression, BeamExpressionContext ctx) {
        String src = expression == null ? "" : expression.trim();
        if (src.startsWith("$=")) {
            src = src.substring(2).trim();
        }
        try {
            Parser parser = new Parser(new Lexer(src));
            Node ast = parser.parseExpression();
            parser.expectEof();
            Object value = eval(ast, ctx);
            return new EvalOutcome(value, EvalStatus.OK, null);
        } catch (EvalException e) {
            return new EvalOutcome(null, e.status, e.getMessage());
        } catch (Throwable t) {
            return new EvalOutcome(null, EvalStatus.INTERNAL,
                    "internal evaluator error: " + t.getClass().getSimpleName() + ": " + t.getMessage());
        }
    }

    /** Evaluate with a flat variable map (jbeam {@code variables} defaults). */
    public static EvalOutcome evaluate(String expression, Map<String, Double> vars) {
        return evaluate(expression, BeamExpressionContext.ofVariables(vars));
    }

    /** Build a context from a flat variable map. */
    public static BeamExpressionContext contextOf(Map<String, Double> vars) {
        return BeamExpressionContext.ofVariables(vars);
    }

    static EvalException err(EvalStatus status, String message) {
        return new EvalException(status, message);
    }

    /** Typed, expected evaluation failure (syntax, arithmetic, unsupported). */
    static final class EvalException extends RuntimeException {
        final EvalStatus status;

        EvalException(EvalStatus status, String message) {
            super(message);
            this.status = status;
        }
    }
}
