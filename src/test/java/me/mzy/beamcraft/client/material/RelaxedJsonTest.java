package me.mzy.beamcraft.client.material;

import com.google.gson.JsonObject;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Tests the shared relaxed-JSON preprocessor. Every {@code .jbeam} and
 * {@code .materials.json} in the game goes through {@link RelaxedJson#clean}, so a
 * change here reaches all of them — which is why the leading-dot fixup it performs
 * is pinned at both the text and the parsed-value level.
 */
class RelaxedJsonTest {

    @Test
    void parsesCommentsAndMissingCommas() {
        JsonObject json = RelaxedJson.parse("""
                {
                  "part": {
                    // a line comment with a stray brace } inside
                    "spring": 12000
                    /* and a block comment with a quote " inside */
                    "damp": 800,
                  }
                }
                """);

        JsonObject part = json.getAsJsonObject("part");
        assertEquals(12000, part.get("spring").getAsInt());
        assertEquals(800, part.get("damp").getAsInt());
    }

    @Test
    void quotesUnquotedKeys() {
        assertEquals(7, RelaxedJson.parse("{ key: 7 }").get("key").getAsInt());
    }

    @Test
    void injectsTheZeroForLeadingDotNumbers() {
        assertEquals("{\"a\":0.5}", RelaxedJson.clean("{\"a\": .5}"));

        JsonObject json = RelaxedJson.parse("{\"a\": .5, \"b\": -.5, \"c\": +.5, \"d\": 1.5, \"e\": -.25}");

        assertEquals(0.5, json.get("a").getAsDouble());
        assertEquals(-0.5, json.get("b").getAsDouble());
        assertEquals(0.5, json.get("c").getAsDouble());
        assertEquals(1.5, json.get("d").getAsDouble(), "a mid-number dot must not gain a zero");
        assertEquals(-0.25, json.get("e").getAsDouble());
    }

    @Test
    void leavesStringLiteralsVerbatim() {
        // The leading-dot fixup used to run as a whole-string regex after cleaning, so
        // it rewrote text inside quoted values as well: "a .5 b" came out as
        // "a 0.5 b". Only unquoted values may be rewritten.
        JsonObject json = RelaxedJson.parse("""
                {"s": "a .5 b", "comment": "// not a comment", "block": "/* neither */"}
                """);

        assertEquals("a .5 b", json.get("s").getAsString());
        assertEquals("// not a comment", json.get("comment").getAsString());
        assertEquals("/* neither */", json.get("block").getAsString());
    }

    @Test
    void stripsTrailingCommas() {
        JsonObject json = RelaxedJson.parse("{\"list\": [1, 2, 3, ]}");

        assertEquals(3, json.getAsJsonArray("list").size());
    }
}
