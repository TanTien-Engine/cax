# cadcvt reader golden tests

This harness pins the behaviour of `cadcvt::FreeCadReader` / `cadcvt::SwReader`
(and the `cadapp::Replayer` step) against a set of fixtures, so any
change to a reader, the IR, or the replay path shows up as a
reviewable diff instead of a silent regression.

## SolidWorks fixtures (`.SLDPRT` / `.SLDASM`)

SolidWorks parts live under `fixtures/sw/`. Unlike FreeCAD (parsed
offline), `SwReader` drives the *installed* SolidWorks over COM, so
these fixtures can only be parsed on a machine with SolidWorks. They
are therefore **gated behind `--sw`** and skipped otherwise (CI has no
SolidWorks, so CI stays green):

```
cadcvt_golden --sw            # run SW fixtures too (needs SolidWorks)
cadcvt_golden --sw --update   # (re)mint SW goldens; first eval launches SLDWORKS.exe (slow)
cadcvt_golden                 # CI default: SW fixtures print [skip], exit 0
```

SW fixtures are **IR-only** (no `.geo.golden`): the geo layer would replay
the imported sketch back through the constraint solver, and the coincidents
the reader synthesizes for connected geometry (needed so RebuildHistory can
re-edit the sketch) make that solve mildly redundant -- DogLeg then
occasionally diverges, so the geo fingerprint is not run-to-run stable. The
`.ir.golden` fully pins the reader's behaviour, which is what these fixtures
test; spot-check geometry by opening the part in the editor (`test/cadcvt/sw.ves`).

The committed `sw/*.ir.golden` are minted on an SW-equipped machine and
reviewed like any other golden. Authoring a SW fixture is easiest done
interactively in SolidWorks (save the part into `fixtures/sw/`) or by
copying one of the bundled samples (`…\SOLIDWORKS\samples\tutorial\`);
scripted creation via the COM API is brittle.

## Layout

```
test/cadcvt_c/
  CMakeLists.txt          # builds the `cadcvt_golden` executable
  golden/
    golden_main.cpp       # discovery + diff harness
    IrFingerprint.{h,cpp} # DocumentIR -> stable text snapshot
    GeoFingerprint.{h,cpp}# replayed OCCT shape -> stable text snapshot
    fixtures/
      <name>.FCStd        # a FreeCAD document (or <name>.xml, see below)
      <name>.ir.golden    # committed IR snapshot
      <name>.geo.golden   # committed geometry snapshot
      README.md           # this file
```

## Two snapshot layers, on purpose

Each fixture produces two goldens:

- **`.ir.golden`** -- the parsed `DocumentIR`: feature count, types,
  modeling order, every sketch geometry and constraint. This layer
  is deterministic from parsing alone and should only change when
  the reader's behaviour changes.
- **`.geo.golden`** -- the replayed OCCT shape: solid/face/edge/vertex
  counts, bounding box, area, volume (all coarsely rounded). This
  layer can legitimately shift on an OpenCASCADE upgrade, so keeping
  it separate means an OCCT bump rewrites only `*.geo.golden` and
  never masks an IR regression.

If you only touched the parser and don't have OCCT handy, run with
`--ir-only` to skip the geometry layer.

## `.FCStd` vs raw `Document.xml`: pick by what the feature needs

Both formats are accepted; the harness's `IsFixture` walks .fcstd
and .xml alike, and `FreeCadReader::ReadFile` branches by suffix.

Which one to commit depends on the features in the fixture:

- **Bare `.xml` (preferred for parser-only coverage)** -- a single
  reviewable text file. Goldens diff cleanly in PRs. Use this for
  sketches, Pad / Pocket / Revolution / Groove, Primitives, and any
  feature whose IR is fully determined by the XML properties.
  Extract with:

  ```
  unzip -p MyPart.FCStd Document.xml > fixtures/my_part.xml
  ```

- **`.FCStd` (required for face / edge ref features)** -- Fillet,
  Chamfer, Shell, and any future feature that holds `LinkSub` refs
  like `"PolarPattern.Face5"` need the authored OCCT BRep entries
  inside the zip. The reader lifts the referent face's
  centroid / normal / area out of those entries to seed
  `TopoRefIR.point / normal / measure`, and `TopoRefResolver` then
  matches geometrically against cax's replayed body. A bare `.xml`
  fixture for such features parses fine but every ref stays a
  zero-geometry stub, so Fillet / Chamfer / Shell silently skip
  -- the IR golden looks right but the geo golden is wrong.

  `.FCStd` files are marked `binary` in `.gitattributes` so git
  skips CRLF and diff attempts.

Rule of thumb: if the IR has any `*_ref_<i>_name` ext_string, commit
`.FCStd`. Otherwise commit `.xml`.

When in doubt, commit `.FCStd`. The extra repo weight is small (a
typical fixture is tens of KB) and the geometric anchor still works
even if you don't end up using a ref-bearing feature.

## Workflow

Build, then from the build directory:

```
ctest -R cadcvt_golden            # run the comparison (CI does this)
./test/cadcvt_c/cadcvt_golden     # same, with full per-case output
```

After an intentional change, regenerate and review:

```
./test/cadcvt_c/cadcvt_golden --update
git diff test/cadcvt_c/golden/fixtures   # READ THIS before committing
```

The `--update` step is the only way a golden changes. Never edit a
`.golden` by hand.

## Adding a case

1. Build the part in FreeCAD. Aim each fixture at one thing:
   `pad_blind.xml`, `pocket_through.xml`, `fillet_edges.xml`,
   `revolve_360.xml`, ... A focused fixture makes a failing diff
   point straight at the cause.
2. Extract `Document.xml` (see above) into `fixtures/`.
3. Run `--update` to mint the goldens.
4. Open the goldens and sanity-check them -- the IR snapshot is
   designed to be read. Confirm the feature count, the sketch
   contents, and that nothing you support landed as `payload=opaque`.
5. Commit the fixture and both goldens together.

## Coverage to grow toward

Track these as fixtures so each reader change has a witness. An
unsupported feature is not a gap in the test -- it shows up as
`payload=opaque freecad_type=...` in the IR golden, so coverage
growth is literally visible as `opaque` lines turning into typed
ones across commits.

- one fixture per supported feature (pad, pocket, fillet, chamfer,
  shell, each primitive)
- sketch variety: line/arc/circle/ellipse/spline, construction
  geometry, each constraint type
- one fixture per FreeCAD schema version you care about (0.20 /
  0.21 / 0.22 differ, especially around the Body container)
- the not-yet-supported features (revolve, mirror, pattern, loft)
  as `opaque` baselines, so the day they become typed the diff is
  the proof
