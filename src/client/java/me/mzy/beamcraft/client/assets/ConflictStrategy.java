package me.mzy.beamcraft.client.assets;

import java.util.Locale;

/**
 * How {@link AssetScanner} picks a winner when the same asset path (one
 * {@code vehicles/<namespace>/...} logical entry) exists in several sources.
 * All three strategies are simple to implement, so the config exposes all of
 * them; {@code LATER_ROOT} is the default because it lets a user's own root
 * (listed after the bundled default) override the default assets.
 */
public enum ConflictStrategy {

    /** Newest file modification time wins; zip entries with an unknown (-1) timestamp are treated as oldest. */
    NEWER,

    /** The root listed later in the config's {@code assetRoots} wins. */
    LATER_ROOT,

    /** The root listed earlier in the config's {@code assetRoots} wins. */
    EARLIER_ROOT;

    /** Parses a config string; null/blank/unknown falls back to {@link #LATER_ROOT}. */
    public static ConflictStrategy parse(String s) {
        if (s == null || s.isBlank()) {
            return LATER_ROOT;
        }
        switch (s.trim().toLowerCase(Locale.ROOT)) {
            case "newer":
                return NEWER;
            case "earlier-root":
                return EARLIER_ROOT;
            case "later-root":
            default:
                return LATER_ROOT;
        }
    }
}
