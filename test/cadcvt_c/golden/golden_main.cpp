#include "IrFingerprint.h"
#include "GeoFingerprint.h"

#include "cadcvt_c/reader/FreeCadReader.h"
#include "cadcvt_c/reader/SwReader.h"
#include "cadcvt_c/reader/ZwReader.h"
#include "cadapp_c/emitter/Replayer.h"
#include "brepkit_c/TopoShape.h"

#include <Standard_Failure.hxx>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// ============================================================
// test/cadcvt_c/golden/golden_main.cpp
//
// FreeCAD reader golden harness.
//
// For each fixture <name>.FCStd (or <name>.xml) under the fixtures
// directory it produces two snapshots:
//   <name>.ir.golden    -- DocumentIR fingerprint (IrFingerprint)
//   <name>.geo.golden   -- replayed OCCT shape fingerprint (GeoFingerprint)
//
// Modes:
//   (default)   compare each fixture against its goldens, print a
//               unified-ish diff for mismatches, exit non-zero on
//               any failure.
//   --update    (re)write every golden from the current output.
//               Review the diff in version control before commit.
//   --ir-only   skip the replay / geometry layer (useful when OCCT
//               is unavailable or you only touched the parser).
//
// Usage:
//   cadcvt_golden [--update] [--ir-only] [fixtures_dir]
//
// fixtures_dir defaults to the compile-time CADCVT_GOLDEN_FIXTURES
// define (set by CMake to test/cadcvt_c/golden/fixtures), so the
// binary can run with no arguments from any working directory.
// ============================================================

namespace fs = std::filesystem;

namespace
{

#ifndef CADCVT_GOLDEN_FIXTURES
#define CADCVT_GOLDEN_FIXTURES "fixtures"
#endif

struct Options
{
    bool        update   = false;
    bool        ir_only  = false;
    bool        with_sw  = false;   // process .SLDPRT/.SLDASM fixtures (needs SolidWorks)
    std::string fixtures = CADCVT_GOLDEN_FIXTURES;
};

Options ParseArgs(int argc, char** argv)
{
    Options o;
    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--update") {
            o.update = true;
        }
        else if (a == "--ir-only") {
            o.ir_only = true;
        }
        else if (a == "--sw") {
            o.with_sw = true;
        }
        else {
            o.fixtures = a;
        }
    }
    return o;
}

bool ReadTextFile(const fs::path& p, std::string& out)
{
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        return false;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

bool WriteTextFile(const fs::path& p, const std::string& text)
{
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f) {
        return false;
    }
    f << text;
    return (bool)f;
}

// Split into lines for a readable per-line diff. Keeps it simple:
// no LCS, just walk both side by side and flag the first divergence
// plus a small window. That is enough to locate a regression fast.
void PrintDiff(const std::string& expected, const std::string& actual)
{
    std::vector<std::string> e;
    std::vector<std::string> a;
    {
        std::istringstream es(expected);
        std::string line;
        while (std::getline(es, line)) {
            e.push_back(line);
        }
        std::istringstream as(actual);
        while (std::getline(as, line)) {
            a.push_back(line);
        }
    }

    size_t n = std::max(e.size(), a.size());
    int shown = 0;
    for (size_t i = 0; i < n && shown < 40; ++i)
    {
        const std::string* el = (i < e.size()) ? &e[i] : nullptr;
        const std::string* al = (i < a.size()) ? &a[i] : nullptr;
        bool same = el && al && (*el == *al);
        if (same) {
            continue;
        }
        if (el) {
            std::printf("    - %s\n", el->c_str());
        }
        if (al) {
            std::printf("    + %s\n", al->c_str());
        }
        ++shown;
    }
    if (shown >= 40) {
        std::printf("    ... (diff truncated)\n");
    }
}

// Lower-cased file extension (no dot), for fixture-kind dispatch.
std::string LowerExt(const fs::path& p)
{
    std::string ext = p.extension().string();
    if (!ext.empty() && ext[0] == '.') ext.erase(0, 1);
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return ext;
}

// SolidWorks fixtures (.SLDPRT / .SLDASM) are read through SwReader, which
// drives the installed SolidWorks via COM -- so they only run on a machine
// with SolidWorks (gated by --sw; skipped in CI).
bool IsSwFixture(const fs::path& p)
{
    std::string e = LowerExt(p);
    return e == "sldprt" || e == "sldasm";
}

// ZW3D fixtures are the neutral .cax.json intermediate the zw_export
// plugin emits. They are read through ZwReader (no ZW3D SDK / no running
// ZW3D needed -- the reader is pure JSON), so they run anywhere, in CI.
bool IsZwFixture(const fs::path& p)
{
    return LowerExt(p) == "json";
}

// Run the reader on one fixture, dispatching by extension. Returns false
// (and fills err) on a hard reader failure; a successful parse fills doc.
bool RunReader(const fs::path& fixture, cadapp::DocumentIR& doc, std::string& err)
{
    if (IsSwFixture(fixture)) {
        cadcvt::SwReader reader;
        return reader.ReadFile(fixture.string(), doc, &err);
    }
    if (IsZwFixture(fixture)) {
        cadcvt::ZwReader reader;
        return reader.ReadFile(fixture.string(), doc, &err);
    }
    cadcvt::FreeCadReader reader;
    return reader.ReadFile(fixture.string(), doc, &err);
}

// Replay the IR into an OCCT shape. Returns the geometry fingerprint.
std::string BuildGeoSnapshot(cadapp::DocumentIR& doc)
{
    try
    {
        cadapp::Replayer replayer;
        cadapp::ReplayOptions opt;
        cadapp::ReplayResult res;
        replayer.Replay(doc, opt, res);

        if (!res.ok) {
            // A replay failure is itself a stable, diffable fact. Record
            // the message so a regression that breaks replay shows up in
            // the geo golden rather than silently passing.
            return std::string("geo REPLAY_FAILED msg=") + res.err_msg + "\n";
        }
        return cadcvt_golden::FingerprintShape(res.shape);
    }
    catch (Standard_Failure& e)
    {
        // OCCT signals hard failures (e.g. StdFail_NotDone from a fillet
        // that cannot complete) by throwing rather than returning a
        // status. Pin it as a stable, diffable line like any other
        // replay failure -- one fixture must never abort the suite.
        const char* msg = e.GetMessageString();
        return std::string("geo REPLAY_THREW exception=") +
               (msg ? msg : "Standard_Failure") + "\n";
    }
    catch (const std::exception& e)
    {
        return std::string("geo REPLAY_THREW exception=") + e.what() + "\n";
    }
}

struct CaseResult
{
    std::string name;
    bool        passed = true;
    bool        reader_failed = false;
};

// One fixture: produce snapshot(s), then either compare or update.
CaseResult ProcessFixture(const fs::path& fixture, const Options& o)
{
    CaseResult cr;
    cr.name = fixture.stem().string();

    cadapp::DocumentIR doc;
    std::string err;
    if (!RunReader(fixture, doc, err))
    {
        // We still emit a golden so "reader rejects this file" is a
        // pinned behaviour rather than a silent crash next time.
        std::string snapshot = std::string("reader FAILED msg=") + err + "\n";
        fs::path ir_golden = fixture.parent_path() / (cr.name + ".ir.golden");

        if (o.update)
        {
            WriteTextFile(ir_golden, snapshot);
            std::printf("[update] %s (reader failed, recorded)\n", cr.name.c_str());
            cr.reader_failed = true;
            return cr;
        }

        std::string expected;
        if (!ReadTextFile(ir_golden, expected) || expected != snapshot)
        {
            std::printf("[FAIL]   %s  reader error not matching golden\n", cr.name.c_str());
            PrintDiff(expected, snapshot);
            cr.passed = false;
        }
        else {
            std::printf("[ok]     %s  (reader failure pinned)\n", cr.name.c_str());
        }
        cr.reader_failed = true;
        return cr;
    }

    // ---- IR layer ----
    std::string ir_snapshot = cadcvt_golden::FingerprintDocument(doc);
    fs::path ir_golden = fixture.parent_path() / (cr.name + ".ir.golden");

    auto check_one = [&](const fs::path& golden, const std::string& snapshot, const char* layer)
    {
        if (o.update)
        {
            WriteTextFile(golden, snapshot);
            std::printf("[update] %s [%s]\n", cr.name.c_str(), layer);
            return;
        }
        std::string expected;
        if (!ReadTextFile(golden, expected))
        {
            std::printf("[FAIL]   %s [%s]  missing golden (run --update)\n", cr.name.c_str(), layer);
            cr.passed = false;
            return;
        }
        if (expected != snapshot)
        {
            std::printf("[FAIL]   %s [%s]\n", cr.name.c_str(), layer);
            PrintDiff(expected, snapshot);
            cr.passed = false;
            return;
        }
        std::printf("[ok]     %s [%s]\n", cr.name.c_str(), layer);
    };

    check_one(ir_golden, ir_snapshot, "ir");

    // ---- Geometry layer ----
    // SolidWorks fixtures are IR-only: the geo layer replays the imported
    // sketch through the constraint solver, and the (necessary, for the
    // RebuildHistory path) synthesized coincidents make the solve mildly
    // redundant -- DogLeg then occasionally diverges to a degenerate result,
    // so the geo fingerprint is not run-to-run stable. The IR layer fully
    // pins the reader's behaviour, which is what these fixtures test.
    if (!o.ir_only && !IsSwFixture(fixture))
    {
        std::string geo_snapshot = BuildGeoSnapshot(doc);
        fs::path geo_golden = fixture.parent_path() / (cr.name + ".geo.golden");
        check_one(geo_golden, geo_snapshot, "geo");
    }

    return cr;
}

bool IsFixture(const fs::path& p)
{
    if (!p.has_extension()) {
        return false;
    }
    std::string ext = p.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return (char)std::tolower(c); });
    return ext == ".fcstd" || ext == ".xml"
        || ext == ".sldprt" || ext == ".sldasm"
        || ext == ".json";   // ZW3D neutral intermediate (.cax.json)
}

} // anonymous namespace

int main(int argc, char** argv)
{
    Options o = ParseArgs(argc, argv);

    fs::path dir(o.fixtures);
    if (!fs::exists(dir) || !fs::is_directory(dir))
    {
        std::fprintf(stderr, "fixtures dir not found: %s\n", dir.string().c_str());
        return 2;
    }

    // Recursive walk so fixtures organised into subfolders (e.g.
    // `fixtures/pads/`, `fixtures/dressup/`) are picked up. Each
    // golden lands in its fixture's own parent dir, so two fixtures
    // named `blind.xml` in different subdirs do not clobber each
    // other. Sorted by full path for deterministic run order.
    std::vector<fs::path> fixtures;
    for (const auto& entry : fs::recursive_directory_iterator(dir))
    {
        if (entry.is_regular_file() && IsFixture(entry.path())) {
            fixtures.push_back(entry.path());
        }
    }
    std::sort(fixtures.begin(), fixtures.end());

    if (fixtures.empty())
    {
        std::fprintf(stderr, "no .FCStd / .xml fixtures in %s\n", dir.string().c_str());
        return 2;
    }

    int failed  = 0;
    int skipped = 0;
    for (const auto& f : fixtures)
    {
        // SolidWorks fixtures need the installed SolidWorks (COM) to parse,
        // which CI does not have. Skip them unless --sw is passed on a
        // machine with SolidWorks; their committed goldens are minted there.
        if (IsSwFixture(f) && !o.with_sw)
        {
            std::printf("[skip]   %s  (SolidWorks fixture; pass --sw on a machine with SolidWorks)\n",
                        f.stem().string().c_str());
            ++skipped;
            continue;
        }

        // Safety net: the harness contract is "run every fixture, then
        // print the summary". An exception escaping any layer (reader,
        // fingerprint, OCCT) must degrade to a per-fixture failure, not
        // terminate the whole run.
        try
        {
            CaseResult cr = ProcessFixture(f, o);
            if (!cr.passed) {
                ++failed;
            }
        }
        catch (Standard_Failure& e)
        {
            std::printf("[FAIL]   %s  uncaught OCCT exception: %s\n",
                        f.stem().string().c_str(), e.GetMessageString());
            ++failed;
        }
        catch (const std::exception& e)
        {
            std::printf("[FAIL]   %s  uncaught exception: %s\n",
                        f.stem().string().c_str(), e.what());
            ++failed;
        }
    }

    std::printf("\n%zu fixtures, %d failed, %d skipped%s\n",
                fixtures.size(), failed, skipped, o.update ? " (update mode)" : "");

    return failed == 0 ? 0 : 1;
}
