package me.mzy.beamcraft.client.assets;

import net.minecraft.client.MinecraftClient;
import net.minecraft.text.Text;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

/**
 * Reports asset-path conflicts found by {@link AssetScanner}. Always writes a
 * log warning; when the policy enables {@code notify}, additionally posts a
 * one-time (per logical path, process-wide) in-game chat message so a recurring
 * duplicate is surfaced once instead of on every vehicle spawn.
 *
 * <p>The chat sink is replaceable for tests; the default posts via
 * {@link MinecraftClient#execute} so it is safe from any thread.
 */
public final class ConflictReporter {

    public static final ConflictReporter INSTANCE = new ConflictReporter();

    // Local logger (same name as BeamCraft.LOGGER) so reporting never forces the
    // BeamCraft class to initialise in a headless unit-test JVM.
    private static final Logger LOGGER = LoggerFactory.getLogger("beamcraft");

    private volatile Consumer<String> chatSink;
    private final Set<String> reportedChat = ConcurrentHashMap.newKeySet();

    private ConflictReporter() {
    }

    /** Injects a chat sink (tests); {@code null} restores the Minecraft default. */
    public void setChatSink(Consumer<String> sink) {
        this.chatSink = sink;
    }

    public void report(String logicalKey, String winnerAddress, List<String> loserAddresses, boolean notify) {
        StringBuilder message = new StringBuilder();
        message.append("Asset conflict for '").append(logicalKey).append("': using ").append(winnerAddress);
        if (loserAddresses != null && !loserAddresses.isEmpty()) {
            message.append("; also found ").append(String.join(", ", loserAddresses));
        }
        String text = message.toString();
        LOGGER.warn("BeamCraft: {}", text);
        if (notify && reportedChat.add(logicalKey)) {
            postChat("[BeamCraft] " + text);
        }
    }

    private void postChat(String message) {
        Consumer<String> sink = chatSink;
        if (sink != null) {
            sink.accept(message);
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        client.execute(() -> {
            if (client.player != null) {
                client.player.sendMessage(Text.literal(message), false);
            }
        });
    }
}
