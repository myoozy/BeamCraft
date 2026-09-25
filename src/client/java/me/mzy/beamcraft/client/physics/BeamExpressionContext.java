package me.mzy.beamcraft.client.physics;

import java.util.Map;

/**
 * Typed resolver used by {@link JBeamExpressionEvaluator} to look up variables
 * during expression evaluation.
 *
 * <p>Values returned are the evaluator's value model: {@code Double} for numbers,
 * {@code String}, {@code Boolean}, {@code null} for {@code nil}. Dotted names
 * (e.g. {@code components.foo.bar} for {@code $components.foo.bar}) are resolved
 * against the context's root object when available; the legacy flat
 * {@code Map<String,Double>} path only knows plain (non-dotted) variable names
 * and yields {@code nil} for anything unknown.
 */
public interface BeamExpressionContext {

    /**
     * Resolve a variable by its name without the leading {@code '$'}.
     * The name may contain dots for component-style access
     * ({@code "components.wheelsR.duallyR"}).
     *
     * @return the value, or {@code null} for {@code nil} (undefined variable)
     */
    Object get(String name);

    /** Wrap a flat variable map (jbeam {@code variables} defaults). Dotted names resolve to nil. */
    static BeamExpressionContext ofVariables(Map<String, Double> vars) {
        return new FlatContext(vars);
    }

    /**
     * Wrap flat numeric variables plus a root object for dotted access. Dotted paths are
     * resolved by walking nested {@code Map<String,Object>} values; if the walk fails,
     * the full dotted name is looked up in the flat map; otherwise nil.
     */
    static BeamExpressionContext of(Map<String, Double> vars, Map<String, Object> root) {
        return new RootedContext(vars, root);
    }

    /** Empty context: every variable is nil. */
    static BeamExpressionContext empty() {
        return FlatContext.EMPTY;
    }

    final class FlatContext implements BeamExpressionContext {
        static final BeamExpressionContext EMPTY = new FlatContext(Map.of());
        private final Map<String, Double> vars;

        FlatContext(Map<String, Double> vars) {
            this.vars = vars == null ? Map.of() : vars;
        }

        @Override
        public Object get(String name) {
            if (name == null || name.indexOf('.') >= 0) return null; // flat map has no dotted keys
            Double v = vars.get(name);
            return v;
        }
    }

    final class RootedContext implements BeamExpressionContext {
        private final Map<String, Double> vars;
        private final Map<String, Object> root;

        RootedContext(Map<String, Double> vars, Map<String, Object> root) {
            this.vars = vars == null ? Map.of() : vars;
            this.root = root == null ? Map.of() : root;
        }

        @Override
        @SuppressWarnings("unchecked")
        public Object get(String name) {
            if (name == null) return null;
            if (name.indexOf('.') >= 0) {
                String[] parts = name.split("\\.");
                Object cur = root.get(parts[0]);
                for (int i = 1; i < parts.length && cur != null; i++) {
                    if (!(cur instanceof Map)) return null;
                    cur = ((Map<String, Object>) cur).get(parts[i]);
                }
                if (cur != null) return cur;
                // fall back to a flat key that happens to contain dots
                return vars.get(name);
            }
            Object direct = root.get(name);
            if (direct != null) return direct;
            return vars.get(name);
        }
    }
}
