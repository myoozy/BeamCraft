package me.mzy.beamcraft.client.physics.powertrain;

import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import me.mzy.beamcraft.client.material.RelaxedJson;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.CombustionEngineSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DeviceSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DevicePatchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.DifferentialSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.FrictionClutchSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.GearboxSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ShaftSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorquePoint;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.TorsionReactorSpec;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.UnsupportedConfig;
import me.mzy.beamcraft.client.physics.powertrain.PowertrainSpecs.ValueModifier;
import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 构建期 powertrain 解析层的单元测试。覆盖：
 * 无 powertrain → 空、header+独立 modifier 合并、命名配置对象合并与优先级、
 * {@code $=...} 表达式变量、第一正挡速比选择、未知类型保留为 {@link UnsupportedConfig}，
 * 以及 shaft/differential/engine/clutch 的字段解析。
 */
class JBeamPowertrainParserTest {

    private static JsonObject part(String json) {
        return JsonParser.parseString(json).getAsJsonObject();
    }

    private static List<DeviceSpec> parse(String json) {
        return JBeamPowertrainParser.parsePart(part(json), Map.of());
    }

    private static List<DeviceSpec> parse(String json, Map<String, Double> vars) {
        return JBeamPowertrainParser.parsePart(part(json), vars);
    }

    // ---------------------------------------------------------------- 基本行为

    @Test
    void noPowertrainReturnsEmpty() {
        assertEquals(List.of(), JBeamPowertrainParser.parsePart(part("{\"nodes\":[]}"), Map.of()));
        assertEquals(List.of(), JBeamPowertrainParser.parsePart(null, Map.of()));
        assertEquals(List.of(), JBeamPowertrainParser.parsePart(part("{\"powertrain\":[]}"), Map.of()));
    }

    @Test
    void finalDrivePartWithoutPowertrainProducesNamedDevicePatch() {
        List<DeviceSpec> specs = parse("""
                {
                  "slotType": "sunburst2_finaldrive_F",
                  "differential_F": {"gearRatio": 3.90}
                }
                """);

        DevicePatchSpec patch = assertInstanceOf(DevicePatchSpec.class, specs.get(0));
        assertEquals("differential_F", patch.name());
        assertEquals(1, patch.valueModifiers().size());
        assertEquals("gearRatio", patch.valueModifiers().get(0).targetKey());
        assertEquals('=', patch.valueModifiers().get(0).operation());
        assertEquals(3.90, patch.valueModifiers().get(0).value(), 1e-6);
    }

    @Test
    void headerSkippedAndStandaloneModifierAppliedToFollowingRows() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    {"friction": 2.5},
                    {"inertia": 0.5},
                    ["combustionEngine","mainEngine","dummy",0]
                  ]
                }
                """);
        assertEquals(1, specs.size());
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertEquals("combustionEngine", engine.type());
        assertEquals("mainEngine", engine.name());
        assertEquals("dummy", engine.inputName());
        assertEquals(0, engine.inputIndex());
        assertEquals(2.5, engine.friction(), 1e-6);
        assertEquals(0.5, engine.inertia(), 1e-6);
    }

    @Test
    void noHeaderTreatsFirstRowAsDevice() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["combustionEngine","mainEngine","dummy",0]
                  ]
                }
                """);
        assertEquals(1, specs.size());
        assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
    }

    // ---------------------------------------------------------------- 命名配置合并与优先级

    @Test
    void namedConfigMergesAndOverridesRowInline() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["differential","differential_R","driveshaft",1,{"diffType":"open","friction":1.0,"gearRatio":1}]
                  ],
                  "differential_R": {
                    "friction": 3.92,
                    "dynamicFriction": 0.00184,
                    "torqueLossCoef": 0.03
                  }
                }
                """);
        DifferentialSpec diff = assertInstanceOf(DifferentialSpec.class, specs.get(0));
        // 行内独有 → 保留
        assertEquals("open", diff.diffType());
        assertEquals(1.0, diff.gearRatio(), 1e-6);
        // 命名对象覆盖行内（BeamNG tableMergeRecursive 语义）
        assertEquals(3.92, diff.friction(), 1e-6);
        assertEquals(0.00184, diff.dynamicFriction(), 1e-6);
        assertEquals(0.03, diff.torqueLossCoef(), 1e-6);
        // diffTorqueSplit 未给出 → 默认
        assertEquals(0.5, diff.diffTorqueSplit(), 1e-6);
    }

    @Test
    void namedConfigProvidesEverythingWhenRowHasNoInline() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["manualGearbox","gearbox","clutch",1]
                  ],
                  "gearbox": {
                    "gearRatios": [-2.95, 0, 2.85, 1.68, 1.0],
                    "friction": 1.38,
                    "dynamicFriction": 0.0014,
                    "torqueLossCoef": 0.0155
                  }
                }
                """);
        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(0));
        assertEquals(List.of(-2.95, 0.0, 2.85, 1.68, 1.0), gb.gearRatios());
        assertEquals(1.38, gb.friction(), 1e-6);
        assertEquals(0.0014, gb.dynamicFriction(), 1e-6);
        assertEquals(0.0155, gb.torqueLossCoef(), 1e-6);
    }

    @Test
    void modifierBlackboardBeatsDefaultsButLosesToNamedConfig() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    {"friction": 9.9},
                    ["combustionEngine","mainEngine","dummy",0]
                  ],
                  "mainEngine": {"friction": 11.5}
                }
                """);
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        // 命名对象（11.5）> 黑板（9.9）> 默认（0）
        assertEquals(11.5, engine.friction(), 1e-6);
    }

    // ---------------------------------------------------------------- 表达式变量

    @Test
    void expressionVariablesResolveInConfigAndGearRatios() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("baseMaxRPM", 4000.0);
        vars.put("ratio1", 3.45);

        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["combustionEngine","mainEngine","dummy",0],
                    ["manualGearbox","gearbox","mainEngine",1]
                  ],
                  "mainEngine": {"maxRPM": "$= $baseMaxRPM + 1000"},
                  "gearbox": {"gearRatios": [-2.95, 0, "$= $ratio1"]}
                }
                """, vars);

        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertEquals(5000.0, engine.maxRPM(), 1e-6);

        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(1));
        // 表达式求值经 evaluateBeamNGExpression 走 float，逐元素按 float 精度比较
        assertEquals(3, gb.gearRatios().size());
        assertEquals(-2.95, gb.gearRatios().get(0), 1e-6);
        assertEquals(0.0, gb.gearRatios().get(1), 1e-6);
        assertEquals(3.45, gb.gearRatios().get(2), 1e-6);
    }

    @Test
    void bareVariableReferencesResolveFromVariables() {
        Map<String, Double> vars = new HashMap<>();
        vars.put("difftorquesplit_R", 0.4);

        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["differential","differential_R","driveshaft",1,{"diffTorqueSplit":"$difftorquesplit_R"}]
                  ]
                }
                """, vars);

        DifferentialSpec diff = assertInstanceOf(DifferentialSpec.class, specs.get(0));
        assertEquals(0.4, diff.diffTorqueSplit(), 1e-6);
    }

    // ---------------------------------------------------------------- 第一正挡速比

    @Test
    void firstPositiveGearRatioIsSelectedAsFixedFirstGear() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["manualGearbox","gearbox","clutch",1]
                  ],
                  "gearbox": {
                    "gearRatios": [-2.95, 0, 2.85, 1.68, 1.0],
                    "fixedFirstGear": true
                  }
                }
                """);
        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(0));
        assertEquals(2.85, gb.firstPositiveGearRatio(), 1e-6);
        assertTrue(gb.fixedFirstGear());
    }

    @Test
    void firstPositiveGearRatioWithoutExplicitNeutral() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["manualGearbox","gearbox","clutch",1]
                  ],
                  "gearbox": {"gearRatios": [-3.0, 2.5, 1.6]}
                }
                """);
        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(0));
        assertEquals(2.5, gb.firstPositiveGearRatio(), 1e-6);
    }

    // ---------------------------------------------------------------- 未知类型保留

    @Test
    void unknownTypePreservedAsUnsupportedConfig() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["fluxCapacitor","fc","mainEngine",1,{"foo":1.5,"bar":"x"}],
                    ["combustionEngine","mainEngine","dummy",0]
                  ]
                }
                """);
        assertEquals(2, specs.size());

        UnsupportedConfig uc = assertInstanceOf(UnsupportedConfig.class, specs.get(0));
        assertEquals("fluxCapacitor", uc.type());
        assertEquals("fc", uc.name());
        assertEquals("mainEngine", uc.inputName());
        assertEquals(1, uc.inputIndex());
        assertNotNull(uc.reason());
        assertEquals("1.5", uc.config().get("foo"));
        assertEquals("x", uc.config().get("bar"));

        // 后续行仍然正常解析，未被连带丢弃
        assertInstanceOf(CombustionEngineSpec.class, specs.get(1));
    }

    @Test
    void cvtGearboxKeptAsUnsupportedConfig() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["cvtGearbox","gearbox","mainEngine",1]
                  ]
                }
                """);
        assertInstanceOf(UnsupportedConfig.class, specs.get(0));
    }

    // ---------------------------------------------------------------- 各设备字段

    @Test
    void shaftParsesGearRatioConnectedWheelAndOutputPortOverride() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["shaft","driveshaft","torsionReactorR",1,
                      {"outputPortOverride":[2], "friction":0.26, "dynamicFriction":0.00051,
                       "connectedWheel":"fl_wheel", "gearRatio":1.2}],
                    ["torsionReactor","torsionReactorR","gearbox",1]
                  ],
                  "torsionReactorR": {"torqueReactionNodes:":["e1l","e2l","e4r"]}
                }
                """);
        ShaftSpec shaft = assertInstanceOf(ShaftSpec.class, specs.get(0));
        assertEquals(1.2, shaft.gearRatio(), 1e-6);
        assertEquals("fl_wheel", shaft.connectedWheel());
        assertEquals(0.26, shaft.friction(), 1e-6);
        assertEquals(0.00051, shaft.dynamicFriction(), 1e-6);
        assertEquals(List.of(2), shaft.outputPortOverride());

        TorsionReactorSpec reactor = assertInstanceOf(TorsionReactorSpec.class, specs.get(1));
        assertEquals(List.of("e1l", "e2l", "e4r"), reactor.torqueReactionNodes());
        assertEquals(1.0, reactor.gearRatio(), 1e-6);
    }

    @Test
    void openDifferentialMapsToDifferentialSpec() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["openDifferential","diff_R","driveshaft",1,{"gearRatio":3.9,"diffTorqueSplit":0.6}]
                  ]
                }
                """);
        DifferentialSpec diff = assertInstanceOf(DifferentialSpec.class, specs.get(0));
        assertEquals("openDifferential", diff.type());
        assertEquals(3.9, diff.gearRatio(), 1e-6);
        assertEquals(0.6, diff.diffTorqueSplit(), 1e-6);
    }

    @Test
    void combustionEngineParsesTorqueCurveAndReactionNodes() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["combustionEngine","mainEngine","dummy",0]
                  ],
                  "mainEngine": {
                    "torque": [["rpm","torque"],[0,0],[500,62],[1000,105]],
                    "friction": 11.5,
                    "torqueReactionNodes:": ["e1l","e2l","e4r"]
                  }
                }
                """);
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertEquals(List.of(new TorquePoint(0, 0), new TorquePoint(500, 62), new TorquePoint(1000, 105)),
                engine.torqueCurve());
        assertEquals(List.of("e1l", "e2l", "e4r"), engine.torqueReactionNodes());
        // engineBrakeTorque 未显式给出 → 按 BeamNG 派生为 friction * 2
        assertEquals(23.0, engine.engineBrakeTorque(), 1e-6);
    }

    @Test
    void frictionClutchUsesBeamNGDefaults() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["frictionClutch","clutch","mainEngine",1]
                  ],
                  "clutch": {"clutchFreePlay": 0.35}
                }
                """);
        FrictionClutchSpec clutch = assertInstanceOf(FrictionClutchSpec.class, specs.get(0));
        assertEquals(0.35, clutch.clutchFreePlay(), 1e-6);
        assertEquals(0.15, clutch.lockDampRatio(), 1e-6);   // BeamNG 默认
        assertEquals(1.0, clutch.lockSpringCoef(), 1e-6);   // BeamNG 默认
        assertEquals(1.0, clutch.clutchStiffness(), 1e-6);  // BeamNG 默认
        assertEquals(0.0, clutch.lockTorque(), 1e-6);       // 未给出 → 0（运行时由父引擎派生）
    }

    // ---------------------------------------------------------------- 真实 Sunburst 风格数据

    @Test
    void sunburstLikeRawJsonProducesNonEmptyTorqueCurve() {
        // Mirrors the real vehicles/sunburst2/sunburst2_engine_2_0.jbeam part: relaxed
        // dialect (missing comma after the torque array) is cleaned exactly like
        // JBeamLoader.cleanJBeamSafe does, then the inner part object is parsed.
        String raw = """
                {
                "sunburst2_engine_2_0": {
                    "information":{"authors":"BeamNG","name":"2.0L F4 Engine"},
                    "powertrain": [
                        ["type", "name", "inputName", "inputIndex"],
                        ["combustionEngine", "mainEngine", "dummy", 0],
                    ],
                    "mainEngine":{
                        "torque":[
                        ["rpm", "torque"],
                        [0,      0],
                        [500,    62],
                        [1000,   105],
                        [1500,   142],
                        [2000,   176],
                        [2500,   198],
                        [3000,   210],
                        [3500,   216],
                        [4000,   221],
                        [4500,   222],
                        [5000,   221],
                        [5500,   218],
                        [6000,   210],
                        [6500,   199],
                        [7000,   186],
                        [7500,   168],
                        [8000,   149],
                        [8500,   135],
                        [9000,   122],
                        [9500,   108],
                        [10000,   90]
                    ]
                        "idleRPM":850,
                        "maxRPM":7500,
                        "inertia":0.072,
                        "friction":11.5,
                        "dynamicFriction":0.024,
                        "engineBrakeTorque":38
                    }
                }
                }
                """;
        JsonObject part = RelaxedJson.parse(raw).getAsJsonObject("sunburst2_engine_2_0");
        List<DeviceSpec> specs = JBeamPowertrainParser.parsePart(part, Map.of());
        assertEquals(1, specs.size());
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertFalse(engine.torqueCurve().isEmpty());
        assertEquals(21, engine.torqueCurve().size());
        assertEquals(new TorquePoint(4500, 222), engine.torqueCurve().get(9));
        assertEquals(850.0, engine.idleRPM(), 1e-6);
        assertEquals(0.072, engine.inertia(), 1e-6);
        assertEquals(11.5, engine.friction(), 1e-6);
        assertEquals(38.0, engine.engineBrakeTorque(), 1e-6);
    }

    // ---------------------------------------------------------------- 值修改器

    @Test
    void dollarValueModifiersCapturedNotResolved() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["combustionEngine","mainEngine","dummy",0]
                  ],
                  "mainEngine": {"$*friction": 1.2, "$+maxRPM": 1500}
                }
                """);
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertEquals(2, engine.valueModifiers().size());
        ValueModifier frictionMod = engine.valueModifiers().stream()
                .filter(vm -> vm.targetKey().equals("friction"))
                .findFirst().orElseThrow();
        assertEquals('*', frictionMod.operation());
        assertEquals(1.2, frictionMod.value(), 1e-6);
        ValueModifier maxRpmMod = engine.valueModifiers().stream()
                .filter(vm -> vm.targetKey().equals("maxRPM"))
                .findFirst().orElseThrow();
        assertEquals('+', maxRpmMod.operation());
        assertEquals(1500.0, maxRpmMod.value(), 1e-6);
        // 修改器不解析：字段仍是内置默认
        assertEquals(6000.0, engine.maxRPM(), 1e-6);
        assertEquals(0.0, engine.friction(), 1e-6);
    }

    // ---------------------------------------------------------------- defensive copy

    @Test
    void returnedListsAreDefensiveCopies() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["manualGearbox","gearbox","clutch",1]
                  ],
                  "gearbox": {"gearRatios": [-2.95, 0, 2.85]}
                }
                """);
        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(0));
        assertFalse(gb.gearRatios().isEmpty());
        try {
            gb.gearRatios().clear();
        } catch (UnsupportedOperationException expected) {
            // unmodifiable copy — 预期行为
        }
        // 原始配置不受影响
        assertEquals(3, gb.gearRatios().size());
    }

    // ---------------------------------------------------------------- 起动机 / 限速器 / 换挡时间

    @Test
    void sunburstLikeStarterLimiterAndShiftFieldsParse() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["combustionEngine","mainEngine","dummy",0],
                    ["sequentialGearbox","gearbox","mainEngine",1]
                  ],
                  "mainEngine": {
                    "starterTorque": 55,
                    "starterMaxRPM": 500,
                    "crankingRPM": 120,
                    "revLimiterRPM": 6200,
                    "revLimiterType": "soft",
                    "revLimiterCutTime": 0.1,
                    "revLimiterMaxRPMDrop": 200
                  },
                  "gearbox": {"gearChangeTime": 0.3, "gearRatios": [-3.0, 0, 3.2, 2.0]}
                }
                """);
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        assertEquals(55.0, engine.starterTorque(), 1e-6);
        assertEquals(500.0, engine.starterMaxRPM(), 1e-6);
        assertEquals(120.0, engine.crankingRPM(), 1e-6);
        assertEquals(6200.0, engine.revLimiterRPM(), 1e-6);
        assertEquals("soft", engine.revLimiterType());
        assertEquals(0.1, engine.revLimiterCutTime(), 1e-6);
        assertEquals(200.0, engine.revLimiterMaxRPMDrop(), 1e-6);

        GearboxSpec gb = assertInstanceOf(GearboxSpec.class, specs.get(1));
        assertEquals(0.3, gb.shiftTime(), 1e-6);
    }

    @Test
    void revLimiterFallsBackToMaxRpmAndShiftTimeDefaultsByType() {
        List<DeviceSpec> specs = parse("""
                {
                  "powertrain": [
                    ["type","name","inputName","inputIndex"],
                    ["combustionEngine","mainEngine","dummy",0,{"maxRPM": 7000}],
                    ["automaticGearbox","gearbox","mainEngine",1],
                    ["manualGearbox","manual","mainEngine",2]
                  ]
                }
                """);
        CombustionEngineSpec engine = assertInstanceOf(CombustionEngineSpec.class, specs.get(0));
        // revLimiterRPM 未给出 → 回退到 maxRPM
        assertEquals(7000.0, engine.revLimiterRPM(), 1e-6);
        GearboxSpec auto = assertInstanceOf(GearboxSpec.class, specs.get(1));
        assertEquals(0.4, auto.shiftTime(), 1e-6);  // 自动挡默认
        GearboxSpec manual = assertInstanceOf(GearboxSpec.class, specs.get(2));
        assertEquals(0.25, manual.shiftTime(), 1e-6); // 手动/序列挡默认
    }
}
