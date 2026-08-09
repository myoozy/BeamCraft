package me.mzy.beamcraft.client.physics;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalOutcome;
import me.mzy.beamcraft.client.physics.JBeamExpressionEvaluator.EvalStatus;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.zip.ZipEntry;
import java.util.zip.ZipFile;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Opt-in, read-only smoke test over an installed JBeam vehicle corpus. Every
 * {@code $=...} string found in the
 * original vehicle JBeam files is fed to {@link JBeamExpressionEvaluator}; the hard
 * requirement is that evaluation NEVER crashes (no expression may throw), and the
 * result distribution (OK / syntax / eval-error / unsupported / internal) is printed
 * as a report. Unexpected internal errors fail the test because they are evaluator bugs.
 *
 * <p>Enable with the {@code BEAMCRAFT_JBEAM_CORPUS} environment variable, or
 * the {@code beamcraft.jbeam.corpus} system property when the test JVM is configured
 * to receive it. The test skips when neither value identifies a directory.
 */
class JBeamExpressionCorpusSmokeTest {

    private static final String CORPUS_PROPERTY = "beamcraft.jbeam.corpus";
    private static final String CORPUS_ENV = "BEAMCRAFT_JBEAM_CORPUS";

    private static String readIS(InputStream is) throws Exception {
        ByteArrayOutputStream b = new ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        int n;
        while ((n = is.read(buf)) > 0) b.write(buf, 0, n);
        return b.toString("UTF-8");
    }

    /** Extract double-quoted JSON string values that start with {@code $=}. */
    static List<String> extractExpressions(String content) {
        List<String> out = new ArrayList<>();
        int i = 0;
        while (i < content.length()) {
            char c = content.charAt(i);
            if (c == '"') {
                StringBuilder sb = new StringBuilder();
                int end = i + 1;
                while (end < content.length()) {
                    char cc = content.charAt(end);
                    if (cc == '\\') { end += 2; continue; }
                    if (cc == '"') break;
                    sb.append(cc);
                    end++;
                }
                String val = sb.toString().trim();
                if (val.startsWith("$=")) out.add(val);
                i = Math.max(end + 1, i + 1);
            } else {
                i++;
            }
        }
        return out;
    }

    /** Merge all jbeam {@code variables} defaults of a zip into {@code vars} (numeric + string). */
    private static void collectVars(ZipFile zf, Map<String, Object> vars) {
        Enumeration<? extends ZipEntry> en = zf.entries();
        while (en.hasMoreElements()) {
            ZipEntry e = en.nextElement();
            String name = e.getName();
            if (e.isDirectory() || name.contains("__MACOSX") || !name.endsWith(".jbeam")) continue;
            String content;
            try {
                content = readIS(zf.getInputStream(e));
            } catch (Exception ex) {
                continue;
            }
            String clean;
            try {
                clean = JBeamLoader.cleanJBeamSafe(content);
            } catch (Throwable t) {
                continue;
            }
            try {
                JsonObject fileJson = JsonParser.parseString(clean).getAsJsonObject();
                for (String partName : fileJson.keySet()) {
                    JsonObject part = fileJson.getAsJsonObject(partName);
                    if (part == null || !part.has("variables")) continue;
                    JsonArray varsArr = part.getAsJsonArray("variables");
                    boolean header = true;
                    for (JsonElement el : varsArr) {
                        if (el.isJsonArray()) {
                            JsonArray row = el.getAsJsonArray();
                            if (header) { header = false; continue; }
                            if (row.size() >= 5) {
                                String varName = row.get(0).getAsString();
                                if (varName.startsWith("$")) varName = varName.substring(1);
                                JsonElement def = row.get(4);
                                if (def instanceof JsonPrimitive p) {
                                    if (p.isNumber()) vars.put(varName, p.getAsDouble());
                                    else if (p.isString()) vars.put(varName, p.getAsString());
                                }
                            }
                        }
                    }
                }
            } catch (Throwable t) {
                // unparseable file — skip
            }
        }
    }

    private static String unsupportedBucket(String reason) {
        if (reason == null) return "?";
        String r = reason.trim();
        if (r.startsWith("include")) return "include()";
        int idx = r.indexOf("unknown function '");
        if (idx >= 0) {
            int end = r.indexOf('\'', idx + "unknown function '".length());
            if (end > 0) return r.substring(idx + "unknown function '".length(), end);
        }
        return r.length() > 60 ? r.substring(0, 60) : r;
    }

    @Test
    void allCorpusExpressionsNeverCrashAndAreClassified() throws Exception {
        String configuredRoot = System.getProperty(CORPUS_PROPERTY);
        if (configuredRoot == null || configuredRoot.isBlank()) {
            configuredRoot = System.getenv(CORPUS_ENV);
        }
        Assumptions.assumeTrue(configuredRoot != null && !configuredRoot.isBlank(),
                "JBeam corpus path not configured; set " + CORPUS_ENV);
        Path corpusRoot = Path.of(configuredRoot);
        Assumptions.assumeTrue(Files.isDirectory(corpusRoot),
                "Configured JBeam corpus path is not a directory: " + corpusRoot);

        // Variables from common.zip are shared with every vehicle.
        Map<String, Object> commonVars = new HashMap<>();
        Path commonZip = corpusRoot.resolve("common.zip");
        if (Files.exists(commonZip)) {
            try (ZipFile zf = new ZipFile(commonZip.toFile())) {
                collectVars(zf, commonVars);
            }
        }

        long total = 0, ok = 0, syntaxErr = 0, evalErr = 0, unsupported = 0, internal = 0;
        Map<String, Integer> unsupportedReasons = new TreeMap<>();
        Map<String, Integer> evalErrorReasons = new TreeMap<>();
        Map<String, Integer> syntaxReasons = new TreeMap<>();
        List<String> internalSamples = new ArrayList<>();
        int zipCount = 0;

        try (DirectoryStream<Path> ds = Files.newDirectoryStream(corpusRoot, "*.zip")) {
            for (Path zip : ds) {
                String zname = zip.getFileName().toString();
                if (zname.equals("common.zip")) continue;
                zipCount++;
                Map<String, Object> vars = new HashMap<>(commonVars);
                try (ZipFile zf = new ZipFile(zip.toFile())) {
                    collectVars(zf, vars);
                    BeamExpressionContext ctx = BeamExpressionContext.of(Map.of(), vars);
                    Enumeration<? extends ZipEntry> en = zf.entries();
                    while (en.hasMoreElements()) {
                        ZipEntry e = en.nextElement();
                        String name = e.getName();
                        if (e.isDirectory() || name.contains("__MACOSX") || !name.endsWith(".jbeam")) continue;
                        String content = readIS(zf.getInputStream(e));
                        for (String expr : extractExpressions(content)) {
                            total++;
                            EvalOutcome out = JBeamExpressionEvaluator.evaluate(expr, ctx);
                            switch (out.status()) {
                                case OK -> ok++;
                                case SYNTAX_ERROR -> {
                                    syntaxErr++;
                                    String key = out.reason() == null ? "?" : out.reason();
                                    syntaxReasons.merge(key, 1, Integer::sum);
                                }
                                case EVAL_ERROR -> {
                                    evalErr++;
                                    String key = out.reason() == null ? "?" : out.reason();
                                    evalErrorReasons.merge(key, 1, Integer::sum);
                                }
                                case UNSUPPORTED -> {
                                    unsupported++;
                                    unsupportedReasons.merge(unsupportedBucket(out.reason()), 1, Integer::sum);
                                }
                                case INTERNAL -> {
                                    internal++;
                                    if (internalSamples.size() < 10) internalSamples.add(expr + " → " + out.reason());
                                }
                            }
                        }
                    }
                }
            }
        }

        System.out.println("========== Corpus expression smoke test ==========");
        System.out.println("vehicles(zips)=" + zipCount);
        System.out.println("total expressions=" + total);
        System.out.println("  OK=" + ok + "  SYNTAX_ERROR=" + syntaxErr
                + "  EVAL_ERROR=" + evalErr + "  UNSUPPORTED=" + unsupported + "  INTERNAL=" + internal);
        System.out.println("  supported(evaluated OK)=" + pct(ok, total)
                + "  unsupported/syntax=" + pct(syntaxErr + unsupported, total)
                + "  data-dependent(eval error)=" + pct(evalErr, total));
        if (!unsupportedReasons.isEmpty()) {
            System.out.println("unsupported function breakdown:");
            unsupportedReasons.entrySet().stream().sorted(Map.Entry.<String, Integer>comparingByValue().reversed())
                    .forEach(e -> System.out.println("  " + e.getKey() + "\t" + e.getValue()));
        }
        if (!evalErrorReasons.isEmpty()) {
            System.out.println("top eval-error reasons (data-dependent, e.g. undefined string vars):");
            evalErrorReasons.entrySet().stream().sorted(Map.Entry.<String, Integer>comparingByValue().reversed())
                    .limit(12).forEach(e -> System.out.println("  " + e.getValue() + "\t" + e.getKey()));
        }
        if (!syntaxReasons.isEmpty()) {
            System.out.println("syntax-error reasons:");
            syntaxReasons.entrySet().stream().sorted(Map.Entry.<String, Integer>comparingByValue().reversed())
                    .limit(10).forEach(e -> System.out.println("  " + e.getValue() + "\t" + e.getKey()));
        }
        if (internal > 0) {
            System.out.println("INTERNAL errors (evaluator bugs):");
            internalSamples.forEach(s -> System.out.println("  " + s));
        }
        System.out.println("===================================================");

        assertEquals(total, ok + syntaxErr + evalErr + unsupported + internal,
                "classification must account for every expression");
        assertEquals(0, internal, "evaluator must never crash on any corpus expression");
    }

    private static String pct(long part, long total) {
        if (total == 0) return "0%";
        return String.format("%.2f%%", 100.0 * part / total);
    }
}
