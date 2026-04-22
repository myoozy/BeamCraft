package me.mzy.beamcraft;

import me.mzy.beamcraft.physics.PhysicsWorld;
import me.mzy.beamcraft.physics.JBeamParser;
import me.mzy.beamcraft.physics.JBeamAssembler;

import net.fabricmc.api.ModInitializer;
import java.io.InputStream;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.util.HashMap;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class BeamCraft implements ModInitializer {
	public static final String MOD_ID = "beamcraft";

	// This logger is used to write text to the console and the log file.
	// It is considered best practice to use your mod id as the logger's name.
	// That way, it's clear which mod wrote info, warnings, and errors.
	public static final Logger LOGGER = LoggerFactory.getLogger(MOD_ID);

	public static final PhysicsWorld PHYSICS_WORLD = new PhysicsWorld();
	// Parts Registry: The key is the part name (e.g., “pickup_frame”), and the value is the corresponding JSON object.
	public static final Map<String, JsonObject> PART_REGISTRY = new HashMap<>();
	// Player configuration: “Key” is the slot name, and “Value” is the name of the selected part.
	public static final Map<String, String> USER_CONFIG = new HashMap<>();

	@Override
	public void onInitialize() {
		// This code runs as soon as Minecraft is in a mod-load-ready state.
		// However, some things (like resources) may still be uninitialized.
		// Proceed with mild caution.

		LOGGER.info("Hello Fabric world!");

		// temporary, zips to load
		String[] zipFiles = {"/Debug/common.zip", "/Debug/pickup.zip"};
		String pcFile = "no_wheel_hub.pc";

		loadJBeamByPC(zipFiles, pcFile);

		JBeamAssembler assembler = new JBeamAssembler();
		assembler.assembleVehicle("pickup", USER_CONFIG, PART_REGISTRY, PHYSICS_WORLD);

		System.out.println("✅ Vehicle assembled, nodes: " + PHYSICS_WORLD.nodes.count +
				" | beams: " + PHYSICS_WORLD.beams.count +
				" | triangles: " + PHYSICS_WORLD.triangles.count +
				" | torsionbars: " + PHYSICS_WORLD.torsionbars.count +
				" | slidenodes: " +  PHYSICS_WORLD.slidenodes.count);
	}

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

	public static void loadJBeamByPC(String[] zipFiles, String pcFile) {
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
									PART_REGISTRY.put(partName, fileJson.getAsJsonObject(partName));
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
				PART_REGISTRY.size() + " parts were extracted!");

		// --- Step 2: Parse the .pc configuration ---
		if (!pcContent.isEmpty()) {
			try {
				com.google.gson.stream.JsonReader pcReader = new com.google.gson.stream.JsonReader(new java.io.StringReader(pcContent));
				pcReader.setLenient(true);
				JsonObject pcJson = JsonParser.parseReader(pcReader).getAsJsonObject();

				JsonObject parts = pcJson.has("parts") ? pcJson.getAsJsonObject("parts") : pcJson;
				for (String slot : parts.keySet()) {
					USER_CONFIG.put(slot, parts.get(slot).getAsString());
				}
			} catch (Exception e) {
				System.err.println("🚨 Failed to parse the .pc configuration");
			}
		}
	}

	public static void loadSinglePartDebug(String partName, Map<String, JsonObject> registry) {
		JsonObject part = registry.get(partName);
		if (part == null) {
			System.err.println("🚨 The specified part cannot be found in the parts library: " + partName);
			return;
		}

		int debugPartId = 999;
		if (part.has("nodes")) JBeamParser.parseNodes(part.getAsJsonArray("nodes"), PHYSICS_WORLD, debugPartId);
		if (part.has("beams")) JBeamParser.parseBeams(part.getAsJsonArray("beams"), PHYSICS_WORLD, debugPartId);
		if (part.has("hydros")) JBeamParser.parseBeams(part.getAsJsonArray("hydros"), PHYSICS_WORLD, debugPartId);
		if (part.has("triangles")) JBeamParser.parseTriangles(part.getAsJsonArray("triangles"), PHYSICS_WORLD, debugPartId);
		if (part.has("torsionbars")) JBeamParser.parseTorsionbars(part.getAsJsonArray("torsionbars"), PHYSICS_WORLD);
		if (part.has("rails")) JBeamParser.parseRails(part.getAsJsonObject("rails"));
		if (part.has("slidenodes")) JBeamParser.parseSlidenodes(part.getAsJsonArray("slidenodes"), PHYSICS_WORLD);
	}

	public static void loadSingleJBeamFileDebug(String targetFileName) {
		System.out.println("====== 🧪 启动单文件极限调试模式: " + targetFileName + " ======");

		String[] zipFiles = {"/Debug/pickup.zip", "/Debug/common.zip"};
		boolean found = false;

		for (String zipPath : zipFiles) {
			try (InputStream fis = BeamCraft.class.getResourceAsStream(zipPath)) {
				if (fis == null) continue;

				try (ZipInputStream zis = new ZipInputStream(fis)) {
					ZipEntry entry;
					while ((entry = zis.getNextEntry()) != null) {
						String name = entry.getName();

						// 只要文件名匹配（忽略路径前缀，方便你只输入 "pickup_frame.jbeam"）
						if (name.endsWith(targetFileName)) {
							System.out.println("📄 找到目标文件: " + name + " (在 " + zipPath + " 中)");
							found = true;

							String content = readEntry(zis);

							// 跑一遍我们最强大的数据清洗流水线
							String cleanJson = cleanJBeamSafe(content);

							try {
								com.google.gson.stream.JsonReader reader = new com.google.gson.stream.JsonReader(new java.io.StringReader(cleanJson));
								reader.setLenient(true);
								JsonObject fileJson = JsonParser.parseReader(reader).getAsJsonObject();

								int debugPartId = 1000;
								// 遍历该文件中的所有零件对象，强制解析！
								for (String partName : fileJson.keySet()) {
									System.out.println("   -> 强制装载零件模块: " + partName);
									JsonObject part = fileJson.getAsJsonObject(partName);
									debugPartId++;

									if (part.has("nodes")) JBeamParser.parseNodes(part.getAsJsonArray("nodes"), PHYSICS_WORLD, debugPartId);
									if (part.has("beams")) JBeamParser.parseBeams(part.getAsJsonArray("beams"), PHYSICS_WORLD, debugPartId);
									if (part.has("triangles")) JBeamParser.parseTriangles(part.getAsJsonArray("triangles"), PHYSICS_WORLD, debugPartId);
									if (part.has("torsionbars")) JBeamParser.parseTorsionbars(part.getAsJsonArray("torsionbars"), PHYSICS_WORLD);
									if (part.has("rails")) JBeamParser.parseRails(part.getAsJsonObject("rails"));
									if (part.has("slidenodes")) JBeamParser.parseSlidenodes(part.getAsJsonArray("slidenodes"), PHYSICS_WORLD);
								}

								System.out.println("====== 🧪 文件加载完毕！节点数: " + PHYSICS_WORLD.nodes.count + " | 梁数: " + PHYSICS_WORLD.beams.count + " ======");
							} catch (Exception e) {
								System.err.println("🚨 GSON 解析该文件失败: " + e.getMessage());
							}
							return; // 找到并加载完毕，直接退出
						}
					}
				}
			} catch (Exception e) {
				System.err.println("🚨 ZIP 读取出错: " + e.getMessage());
			}
		}

		if (!found) {
			System.err.println("🚨 找不到文件: " + targetFileName + "，请检查文件名拼写！");
		}
	}
}