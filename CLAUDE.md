# CLAUDE.md

**Read `ARCHITECTURE.md` before working here.** The repository keeps one
architecture map for everyone — Claude, other coding agents and human maintainers
read the same file, so it is not named after a tool.

It covers the source-set split, the client vehicle lifecycle, the physics step
(asynchronous, and split so the physics thread never touches the Minecraft
world), the JBeam pipeline, the powertrain and materials subsystems, and
asset/config discovery.

The rules for coding agents — provenance and licensing — are in `AGENTS.md`.
