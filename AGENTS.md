# Repository instructions for coding agents

**Read `ARCHITECTURE.md` before changing code.** It is the repository's
architecture map, written for human maintainers and agents alike: the source-set
split, the client vehicle lifecycle, the physics step (asynchronous, and split so
the physics thread never touches the Minecraft world), the JBeam pipeline, the
powertrain and materials subsystems, and asset/config discovery. A comment-,
doc- or single-default edit does not need it.

The rules below are about provenance and licensing.

## Provenance and licensing

`SOURCE_PROVENANCE.md` inventories the files that adapt upstream source. When it
has to be consulted depends on the task:

- **New behavior from your own design, BeamNG's public documentation, or
  mathematics** — the inventory does not apply. The basic rule still does: do not
  copy in source you cannot license.
- **Editing a file that already carries a bCDDL header** — that license still
  governs the file, including the part you add. Keep the header, and record any
  newly consulted upstream path and contributor in `SOURCE_PROVENANCE.md`.
- **Working from BeamNG Lua or another upstream implementation** — check the
  license header of every file you consult, retain it, and add the adaptation to
  `SOURCE_PROVENANCE.md` together with the change.
- **Unrelated work (rendering, Minecraft integration, tooling)** — the basic rule
  is enough.

Regardless of which applies:

- Prefer BeamNG's public documentation for compatibility work.
- Do not claim clean-room implementation when BeamNG source was consulted, and do
  not make correct compatibility behavior intentionally worse to avoid it.
- Loading user-supplied compatible data does not authorize bundling or
  redistributing BeamNG assets.
- AI-generated or AI-assisted code follows the provenance and license of the
  material used to produce it; model output is not a license reset.
