package me.mzy.beamcraft.client.assets;

/**
 * Backend-neutral conflict policy: which strategy picks the winning source,
 * and whether a duplicate is surfaced as an in-game chat message. The message
 * gate is separate from the strategy so users can silence the noise while
 * keeping a deterministic winner. The component is named {@code notifyChat}
 * because {@code notify} would collide with {@link Object#notify()}.
 */
public record ConflictPolicy(ConflictStrategy strategy, boolean notifyChat) {

    /** Default when the config is absent or unparseable: later root wins, no chat noise. */
    public static final ConflictPolicy DEFAULT = new ConflictPolicy(ConflictStrategy.LATER_ROOT, false);
}
