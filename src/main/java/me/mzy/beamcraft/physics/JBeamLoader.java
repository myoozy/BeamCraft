package me.mzy.beamcraft.physics;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.BeamCraft;

import java.io.InputStream;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public class JBeamLoader {
    private static String readEntry(ZipInputStream zis) throws Exception {
        java.io.ByteArrayOutputStream baos = new java.io.ByteArrayOutputStream();
        byte[] buffer = new byte[2048];
        int len;
        while ((len = zis.read(buffer)) > 0) {
            baos.write(buffer, 0, len);
        }
        return baos.toString(java.nio.charset.StandardCharsets.UTF_8.name());
    }

    public static String cleanJBeamSafe(String input) {
        StringBuilder out = new StringBuilder();
        int len = input.length();
        int i = 0;

        // state machine
        boolean lastWasValueEnder = false; // Is the previous character the end of a value (e.g., the end of a number, a quotation mark, }, or ])?
        boolean lastWasComma = false;      // Was the last character entered a comma?

        while (i < len) {
            char c = input.charAt(i);

            // 1. skip all spaces
            if (Character.isWhitespace(c)) {
                i++; continue;
            }

            // 2. skip comments
            if (c == '/' && i + 1 < len) {
                char nc = input.charAt(i + 1);
                if (nc == '/') {
                    i += 2; // skip "//"
                    while (i < len && input.charAt(i) != '\n' && input.charAt(i) != '\r') i++;
                    continue;
                } else if (nc == '*') {
                    i += 2; // skip "/*"
                    while (i + 1 < len && !(input.charAt(i) == '*' && input.charAt(i + 1) == '/')) i++;
                    i += 2; // skip "*/"
                    continue;
                }
            }

            // 3. Automatically insert missing commas
            // If a quotation mark, minus sign, number, letter, {, or [ is encountered, it indicates that a new value is about to begin
            boolean currIsStarter = (c == '"' || c == '{' || c == '[' || c == '-' || c == '.' || Character.isLetterOrDigit(c));

            // If the previous value just ended and a new value starts here, there must be a missing comma in between!
            if (lastWasValueEnder && currIsStarter) {
                out.append(",\n");
                lastWasComma = true;
            }

            // 4. Process the string (without altering its contents)
            if (c == '"') {
                out.append(c);
                i++;
                while (i < len) {
                    char sc = input.charAt(i);
                    out.append(sc);
                    if (sc == '\\' && i + 1 < len) { // process \"
                        i++; out.append(input.charAt(i));
                    } else if (sc == '"') {
                        i++; break; // string end
                    }
                    i++;
                }
                lastWasValueEnder = true;
                lastWasComma = false;
                continue;
            }

            // 5. Processing structural symbols
            if (c == '{' || c == '[') {
                out.append(c);
                lastWasValueEnder = false; lastWasComma = false;
                i++; continue;
            }

            if (c == '}' || c == ']') {
                // Remove trailing commas (e.g., [1, 2, ] → [1, 2])
                if (lastWasComma) {
                    for (int j = out.length() - 1; j >= 0; j--) {
                        if (out.charAt(j) == ',') {
                            out.deleteCharAt(j);
                            break;
                        }
                    }
                }
                out.append(c);
                lastWasValueEnder = true; lastWasComma = false;
                i++; continue;
            }

            if (c == ':') {
                out.append(c);
                lastWasValueEnder = false; lastWasComma = false;
                i++; continue;
            }

            if (c == ',') {
                if (!lastWasComma && lastWasValueEnder) { // Prevent the occurrence of consecutive commas (, ,)
                    out.append(c);
                    lastWasComma = true;
                }
                lastWasValueEnder = false;
                i++; continue;
            }

            // 6. Handling unquoted words (numbers, true/false, or even unquoted keys)
            if (Character.isLetterOrDigit(c) || c == '-' || c == '.' || c == '+') {
                int start = i;
                while (i < len) {
                    char lc = input.charAt(i);
                    // Read through to the end, as long as the character is allowed in a word
                    if (Character.isLetterOrDigit(lc) || lc == '-' || lc == '.' || lc == '+' || lc == '_') {
                        i++;
                    } else {
                        break;
                    }
                }
                String word = input.substring(start, i);

                // Determine whether this string is a valid number or Boolean value
                boolean isNumberOrBool = word.equals("true") || word.equals("false") || word.equals("null");
                if (!isNumberOrBool) {
                    try {
                        Double.parseDouble(word); // Anything that can be parsed as a double is a number (including values like .5, which are fully supported)
                        isNumberOrBool = true;
                    } catch (NumberFormatException e) {
                        isNumberOrBool = false;
                    }
                }

                // If it's a standard number, just enter it as is;
                // if it's a letter without quotation marks (such as “Key”), force it into quotes!
                if (isNumberOrBool) {
                    out.append(word);
                } else {
                    out.append('"').append(word).append('"');
                }

                lastWasValueEnder = true; lastWasComma = false;
                continue;
            }

            // Skip any other unknown characters to prevent dirty data from causing interference
            i++;
        }

        String outString = out.toString();

        // Fix numbers like .5 that don't have leading zeros
        outString = outString.replaceAll("(?<=[\\s,\\[\\{:])\\.([0-9]+)", "0.$1");
        outString = outString.replaceAll("(?<=[\\s,\\[\\{:])-\\.([0-9]+)", "-0.$1");

        return outString;
    }

    public static void loadJBeamByPC(String[] zipFiles, String pcFile, Map<String, JsonObject> partRegistry, Map<String, String> userConfig ) {
        String pcContent = "";
        int loadedFiles = 0;

        // loop through all zips
        for (String zipPath : zipFiles) {
            System.out.println("🔍 Scanning zip: " + zipPath);
            try (InputStream fis = BeamCraft.class.getResourceAsStream(zipPath)) {

                if (fis == null) {
                    System.err.println("⚠️ Zip does not exist: " + zipPath + ". Please make sure it is placed in the root directory of `resources`.");
                    continue;
                }

                try (ZipInputStream zis = new ZipInputStream(fis)) {
                    ZipEntry entry;
                    while ((entry = zis.getNextEntry()) != null) {
                        String name = entry.getName();

                        if ((name.endsWith(".jbeam") || name.endsWith(".pc")) && !name.contains("__MACOSX")) {
                            String content = readEntry(zis);

                            // lock pc file
                            if (name.endsWith(".pc")) {
                                if (name.endsWith("/" + pcFile) || name.equals(pcFile)) {
                                    pcContent = cleanJBeamSafe(content);
                                    System.out.println("📄 Config .pc file locked: " + name + " (from " + zipPath + ")");
                                }
                                continue;
                            }

                            // Normal JBeam part cleaning and loading
                            String cleanJson = cleanJBeamSafe(content);

                            try {
                                com.google.gson.stream.JsonReader reader = new com.google.gson.stream.JsonReader(new java.io.StringReader(cleanJson));
                                reader.setLenient(true); // Enable tolerance mode
                                JsonObject fileJson = JsonParser.parseReader(reader).getAsJsonObject();

                                for (String partName : fileJson.keySet()) {
                                    partRegistry.put(partName, fileJson.getAsJsonObject(partName));
                                }
                                loadedFiles++;
                            } catch (Exception e) {
                                // Ignore errors to keep the console clean
                            }
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        } // End ZIP traversal

        System.out.println("📦 Joint loading of the part library complete. A total of " +
                loadedFiles + " drawing files were read, and " +
                partRegistry.size() + " parts were extracted!");

        // --- Step 2: Parse the .pc configuration ---
        if (!pcContent.isEmpty()) {
            try {
                com.google.gson.stream.JsonReader pcReader = new com.google.gson.stream.JsonReader(new java.io.StringReader(pcContent));
                pcReader.setLenient(true);
                JsonObject pcJson = JsonParser.parseReader(pcReader).getAsJsonObject();

                JsonObject parts = pcJson.has("parts") ? pcJson.getAsJsonObject("parts") : pcJson;
                for (String slot : parts.keySet()) {
                    userConfig.put(slot, parts.get(slot).getAsString());
                }
            } catch (Exception e) {
                System.err.println("🚨 Failed to parse the .pc configuration");
            }
        }
    }
}
