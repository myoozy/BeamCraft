package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.stream.JsonReader;

import java.io.StringReader;

/**
 * Minimal helper for BeamNG's relaxed JSON dialect: C-style comments and
 * missing/trailing commas occur in real files (both *.jbeam and *.materials.json).
 *
 * <p>The cleaning pass is the exact algorithm historically living in
 * {@code JBeamLoader.cleanJBeamSafe}; it was extracted here so JBeam parsing and
 * material parsing share one implementation. {@link JBeamLoader} now delegates to
 * {@link #clean(String)} and therefore behaves identically.
 */
public final class RelaxedJson {

    private RelaxedJson() {
    }

    /**
     * Turns relaxed BeamNG JSON into strict JSON that Gson can parse.
     * Pure function; does not touch the content of string literals.
     */
    public static String clean(String input) {
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

    /**
     * Cleans relaxed JSON then parses it into a Gson {@link JsonObject}.
     * Throws a Gson exception when the content is not valid after cleaning.
     */
    public static JsonObject parse(String content) {
        String clean = clean(content);
        JsonReader reader = new JsonReader(new StringReader(clean));
        reader.setLenient(true);
        return JsonParser.parseReader(reader).getAsJsonObject();
    }
}
