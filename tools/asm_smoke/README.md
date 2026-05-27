# asm_smoke

Standalone Reader smoke test for FreeCAD Assembly4 files. Dumps the
DocumentIR shape produced by `cadcvt::FreeCadReader`: feature count,
per-feature type tags, App::Link payload (linked file, sub-tip,
placement), and AsmConstraint payload (LCS pair, offset, parent
chaining).

Useful while iterating on the Assembly4 reader paths without spinning
up the full Replayer/OCCT stack.

## Build (macOS / Homebrew OCCT, no CMake target)

```bash
REPO=/path/to/cax
clang++ -std=c++17 -O0 -g \
  -I${REPO} -I${REPO}/thirdparty -I${REPO}/thirdparty/miniz \
  -I${REPO}/thirdparty/pugixml/src -I${REPO}/thirdparty/sm \
  -I${REPO}/thirdparty/graph/include -I${REPO}/thirdparty/guard/include \
  -I${REPO}/thirdparty/geoshape/include -I${REPO}/thirdparty/objcomp/include \
  -I${REPO}/thirdparty/vessel/src/include -I${REPO}/thirdparty/wrapper/include \
  -I${REPO}/build_mac/cadcvt_generated -I/opt/homebrew/include/opencascade \
  ${REPO}/tools/asm_smoke/main.cpp \
  -L${REPO}/build_mac -lcax \
  -L/opt/homebrew/lib \
  -lTKernel -lTKMath -lTKG2d -lTKG3d -lTKGeomBase -lTKGeomAlgo \
  -lTKBRep -lTKTopAlgo -lTKShHealing -lTKPrim -lTKMesh -lTKOffset \
  -lTKDESTEP -lTKDE -lTKXSBase -lTKBO -lTKBool -lTKFillet -lTKHLR \
  -o /tmp/asm_smoke
```

## Run

```bash
/tmp/asm_smoke test/cadcvt_c/golden/fixtures/examples/freecad_assembly_test_corpus/Assembly4_examples/asm4-test.FCStd
```

Exit code:
- 0 : reader ok
- 1 : usage error
- 2 : reader failed (message in stderr)
