// ============================================================
// plugins/zw_export/ZwCaxPlugin.cpp
//
// ZW3D plugin entry + command registration for the cax exporter.
// Target: ZW3D 2026 (v17, the Vx C API: VxApi.h + ZW3D.lib). The same
// shape works back to ~ZW3D 2020 and forward; the Vx API is what the
// official samples (ZW3D/ZW3D-OpenBuilds, 12.DllRegister) use.
//
// This file is the loadable plugin glue. It registers a command that
// calls ExportActivePartToCax (defined in ZwCaxExport.cpp) to write the
// neutral .cax.json that cadcvt::ZwReader consumes.
//
// The load/unload entry-point names follow ZW3D's convention, verified
// against the SDK's ZW3DApiCMakeTemplate (api/DevelopmentTemplate):
//
//   - For a plugin DLL named "CaxExport.dll", ZW3D calls an exported
//     "CaxExportInit" on load and "CaxExportExit" on unload (DLL base
//     name + "Init" / "Exit"). Rename the DLL and these two functions
//     together. They are exported via extern "C" __declspec(dllexport)
//     so the names are NOT C++-mangled -- exactly as the template's
//     ZW3DTemplateInit / ZW3DTemplateExit are declared.
//
// The Vx API calls below were checked against this SDK's headers:
//   cvxCmdFunc(const vxName, void*, double)  [zwapi_cmd.h]
//   cvxCmdFuncUnload(const vxName)           [zwapi_cmd.h]
//   cvxFileInqActive(char*, int)             [zwapi_file.h]
//   cvxMsgDisp(const char*)                  [zwapi_message.h]
//   VX_CODE_GENERAL (== 0.0)                 [zwapi_cmd_data.h]
//
// NOTE on naming: the binding namespace called "zwapi" in
// ZwCaxExport.cpp is MY layer, unrelated to ZW3D's zwapi_*.h headers.
// Don't confuse the two.
//
// A typed "~CaxExportRun" command needs no .zrc / form / .tcmd resource
// (those are only for ribbon/UI commands), so this plugin is just the
// two source files -- no zrc.exe step.
// ============================================================

#include "zwapi_cmd.h"       // cvxCmdFunc / cvxCmdFuncUnload / cvxCmdBuffer / VX_CODE_GENERAL
#include "zwapi_file.h"      // cvxFileInqActive / cvxFileOpen / cvxFileClose2 / cvxFileSessionClear
#include "zwapi_message.h"   // cvxMsgDisp / cvxUserActionStatus* / cvxCrashPrompt*

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

// Unicode-safe filesystem access for the batch queue/log. ZW3D's C API
// speaks UTF-8 paths, but the CRT's narrow fopen uses the ANSI code page,
// so Chinese part names (the HW cases) need the UTF-16 detour. Mirrors
// OpenWriteBinary in ZwCaxExport.cpp.
#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

// Defined in ZwCaxExport.cpp. Walks the active part's history and
// writes the .cax.json intermediate. Returns false + err on failure.
bool ExportActivePartToCax(const std::string& out_path, std::string& err);

namespace
{

// Show a message in the ZW3D message area.
void Report(const std::string& msg)
{
    cvxMsgDisp(msg.c_str());
}

// Derive the output path for the active part: "<dir>/<name>.cax.json",
// landing the snapshot right next to the source part. cvxFileInqActive
// returns the bare active file name; cvxFileDirectoryByLongPath returns
// the directory it was saved in (empty if the part was never saved, in
// which case we fall back to the bare name -> ZW3D's working directory).
// The ".cax.json" suffix is what cadcvt::ZwReader::ReadFile consumes.
std::string ActiveOutputPath()
{
    char name[512];
    name[0] = '\0';
    cvxFileInqActive(name, static_cast<int>(sizeof(name)));
    if (name[0] == '\0')
    {
        return std::string();   // no active file
    }

    char dir[512];
    dir[0] = '\0';
    cvxFileDirectoryByLongPath(dir, static_cast<int>(sizeof(dir)));

    std::string out;
    if (dir[0] != '\0')
    {
        out = dir;
        const char last = out.back();
        if (last != '/' && last != '\\')
        {
            out += '\\';
        }
    }
    out += name;
    out += ".cax.json";
    return out;
}

// The registered command callback. Vx command functions registered
// with the general echo code take no arguments and return 0 on success.
int CaxExportCmd(void)
{
    std::string out_path = ActiveOutputPath();
    if (out_path.empty())
    {
        Report("CaxExport: no active part to export.");
        return 1;
    }

    std::string err;
    if (!ExportActivePartToCax(out_path, err))
    {
        Report("CaxExport failed: " + err);
        return 1;
    }

    Report("CaxExport: wrote " + out_path);
    return 0;
}

// ============================================================
// Unattended batch export.
//
// Driven by the CAX_BATCH_QUEUE environment variable: when it names a
// readable list file (one absolute .Z3/.Z3PRT/.Z3ASM path per line,
// UTF-8, '#' comments allowed), CaxExportInit buffers ~CaxExportBatchRun
// so the whole queue is processed right after ZW3D finishes starting up
// -- no clicks needed. Results stream to "<queue>.log" (append, flushed
// per line) so an outside driver can watch progress, detect a crash
// mid-file (a BEGIN with no OK/FAIL after it), blacklist that file and
// relaunch. Files whose .cax.json already exists are skipped, which is
// what turns a relaunch into a resume instead of a redo.
// ============================================================

#ifdef _WIN32
// UTF-8 -> UTF-16, falling back to the ANSI code page when the bytes
// are not valid UTF-8 (an environment block or queue file in GBK).
std::wstring ToWide(const std::string& s)
{
    UINT cp  = CP_UTF8;
    int  len = MultiByteToWideChar(cp, MB_ERR_INVALID_CHARS, s.c_str(), -1, nullptr, 0);
    if (len == 0)
    {
        cp  = CP_ACP;
        len = MultiByteToWideChar(cp, 0, s.c_str(), -1, nullptr, 0);
    }
    if (len == 0)
    {
        return std::wstring();
    }
    std::wstring w(static_cast<size_t>(len), L'\0');
    MultiByteToWideChar(cp, (cp == CP_UTF8) ? MB_ERR_INVALID_CHARS : 0,
                        s.c_str(), -1, &w[0], len);
    w.resize(wcslen(w.c_str()));
    return w;
}

bool FileExistsU8(const std::string& path)
{
    const std::wstring w = ToWide(path);
    if (w.empty())
    {
        return false;
    }
    const DWORD attr = GetFileAttributesW(w.c_str());
    return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

FILE* OpenU8(const std::string& path, const wchar_t* mode)
{
    const std::wstring w = ToWide(path);
    return w.empty() ? nullptr : _wfopen(w.c_str(), mode);
}

// UTF-8 -> ANSI code page (GBK on a zh-CN system). cvxFileOpen, unlike
// the API's UTF-8 *outputs*, decodes its input path in the ANSI code
// page: a UTF-8 Chinese path fails with ZW_API_CMMD_EXEC_ERROR (-152)
// while the same file opens fine from an ASCII path (verified against
// the HW cases). Returns empty when the path has no ACP representation.
std::string ToAcp(const std::string& utf8)
{
    const std::wstring w = ToWide(utf8);
    if (w.empty())
    {
        return std::string();
    }
    BOOL lost = FALSE;
    const int len = WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1,
                                        nullptr, 0, nullptr, &lost);
    if (len <= 0 || lost)
    {
        return std::string();
    }
    std::string s(static_cast<size_t>(len), '\0');
    WideCharToMultiByte(CP_ACP, 0, w.c_str(), -1, &s[0], len, nullptr, nullptr);
    s.resize(strlen(s.c_str()));
    return s;
}
#endif // _WIN32

// One queue entry per non-empty, non-comment line; tolerates CRLF and a
// UTF-8 BOM (PowerShell loves to prepend one).
std::vector<std::string> ReadQueue(const std::string& queue_path)
{
    std::vector<std::string> lines;
    FILE* f = OpenU8(queue_path, L"rb");
    if (!f)
    {
        return lines;
    }
    std::string data;
    char buf[4096];
    size_t n = 0;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0)
    {
        data.append(buf, n);
    }
    fclose(f);

    if (data.size() >= 3 &&
        static_cast<unsigned char>(data[0]) == 0xEF &&
        static_cast<unsigned char>(data[1]) == 0xBB &&
        static_cast<unsigned char>(data[2]) == 0xBF)
    {
        data.erase(0, 3);
    }

    std::string cur;
    for (char c : data)
    {
        if (c == '\n')
        {
            if (!cur.empty() && cur.back() == '\r')
            {
                cur.pop_back();
            }
            if (!cur.empty() && cur[0] != '#')
            {
                lines.push_back(cur);
            }
            cur.clear();
        }
        else
        {
            cur += c;
        }
    }
    if (!cur.empty() && cur.back() == '\r')
    {
        cur.pop_back();
    }
    if (!cur.empty() && cur[0] != '#')
    {
        lines.push_back(cur);
    }
    return lines;
}

// The batch command. Crash-safe logging: every line is fflush'd before
// the next file is touched, so the driver always knows which file ZW3D
// died on.
int CaxExportBatchCmd(void)
{
#ifdef _WIN32
    // Both CaxExport.dll and CaxBatch.dll autostart this command when
    // CAX_BATCH_QUEUE is set (each buffers its own ~CaxExportBatchRun),
    // and ZW3D runs whatever got buffered. Observed live: the duplicate
    // logged a second BEGIN for the first file. One named mutex per
    // session makes every invocation after the first a no-op; it is
    // deliberately never released -- the unattended session is one-shot,
    // and a relaunch gets a fresh process anyway.
    HANDLE mtx = CreateMutexW(nullptr, TRUE, L"Local\\CaxExportBatchRunning");
    if (mtx != nullptr && GetLastError() == ERROR_ALREADY_EXISTS)
    {
        CloseHandle(mtx);
        Report("CaxExportBatch: duplicate invocation skipped.");
        return 0;
    }
#endif

    const char* qenv = std::getenv("CAX_BATCH_QUEUE");
    if (!qenv || !*qenv)
    {
        Report("CaxExportBatch: CAX_BATCH_QUEUE not set.");
        return 1;
    }
    const std::string queue_path(qenv);

    const std::vector<std::string> files = ReadQueue(queue_path);
    FILE* log = OpenU8(queue_path + ".log", L"ab");
    if (!log)
    {
        Report("CaxExportBatch: cannot open log next to queue.");
        return 1;
    }
    auto logline = [log](const std::string& s)
    {
        fwrite(s.data(), 1, s.size(), log);
        fwrite("\n", 1, 1, log);
        fflush(log);
    };

    if (files.empty())
    {
        logline("DONE\tempty queue");
        fclose(log);
        return 1;
    }

    // Whole-batch prompt suppression: 3 = answer "Yes (All)" wherever a
    // dialog would block (version migration, regen, missing refs), and
    // no crash-report dialog holding the process hostage. Restored below.
    int prev_action = 0;
    cvxUserActionStatusGet(&prev_action);
    cvxUserActionStatusSet(3);
    cvxCrashPromptDisable();

    int ok = 0, fail = 0, skip = 0;
    for (const std::string& path : files)
    {
        const std::string out = path + ".cax.json";
        if (FileExistsU8(out))
        {
            logline("SKIP\t" + path);
            ++skip;
            continue;
        }

        logline("BEGIN\t" + path);

        // UTF-8 first (matches the API's own outputs), then the ANSI
        // code page -- cvxFileOpen on this build only accepts the latter
        // for non-ASCII paths. Everything downstream (export json/step
        // paths) stays UTF-8; only the open call needs the ACP copy.
        int rc = static_cast<int>(cvxFileOpen(path.c_str()));
        if (rc != 0)
        {
            const std::string acp = ToAcp(path);
            if (!acp.empty() && acp != path)
            {
                rc = static_cast<int>(cvxFileOpen(acp.c_str()));
            }
        }
        if (rc != 0)
        {
            logline("FAIL\t" + path + "\tcvxFileOpen rc=" + std::to_string(rc));
            ++fail;
            continue;
        }

        std::string err;
        if (ExportActivePartToCax(out, err))
        {
            logline("OK\t" + path);
            ++ok;
        }
        else
        {
            logline("FAIL\t" + path + "\t" + err);
            ++fail;
        }

        // Discard (option 2) and drop the file from the session cache so
        // 70+ parts don't accumulate; "" / NULL mean "the active file".
        cvxFileClose2("", 2);
        cvxFileSessionClear(nullptr);
    }

    cvxUserActionStatusSet(prev_action);
    cvxCrashPromptEnable();

    logline("DONE\tok=" + std::to_string(ok) +
            " fail=" + std::to_string(fail) +
            " skip=" + std::to_string(skip));
    fclose(log);

    Report("CaxExportBatch: done (ok=" + std::to_string(ok) +
           ", fail=" + std::to_string(fail) +
           ", skip=" + std::to_string(skip) + ").");
    return fail == 0 ? 0 : 1;
}

} // namespace

// ---- load entry: <DllName>Init, exported undecorated (CaxExport.dll) ----
//
// Called by ZW3D when the DLL is loaded (via Application -> Applications
// and Plugin Manager -> Load Dll, or auto-loaded from the apilibs
// folder). Registers the command(s) this plugin exposes. The user then
// runs the command by name (typed at the input as ~CaxExportRun, or
// bound to a button / hotkey). Name = DLL base name + "Init", per the
// SDK template's ZW3DTemplateInit.
//
// CAX_PLUGIN_INIT/EXIT exist so the same two sources can also build as a
// differently-named DLL (e.g. CaxBatch.dll with CaxBatchInit) when the
// canonical CaxExport.dll is file-locked by a ZW3D instance that must
// stay alive. ZW3D resolves the entry points from the DLL base name, so
// a renamed DLL needs renamed exports; the command names inside do not
// change (cvxCmdFunc keeps the first registration and ignores a dup).
#ifndef CAX_PLUGIN_INIT
#define CAX_PLUGIN_INIT CaxExportInit
#define CAX_PLUGIN_EXIT CaxExportExit
#endif
extern "C" __declspec(dllexport) int CAX_PLUGIN_INIT(void)
{
    // cvxCmdFunc(name, function, code): bind a command name to a
    // callback. VX_CODE_GENERAL (0.0) is the plain-command licensing
    // code; the callback is passed as a void*.
    cvxCmdFunc("CaxExportRun", reinterpret_cast<void*>(CaxExportCmd), VX_CODE_GENERAL);
    cvxCmdFunc("CaxExportBatchRun", reinterpret_cast<void*>(CaxExportBatchCmd), VX_CODE_GENERAL);

    // Unattended mode: when the launching process set CAX_BATCH_QUEUE,
    // queue the batch to run once startup settles. cvxCmdBuffer defers
    // execution to the command loop, so the export only starts after
    // the session is fully up; an interactive ZW3D never sets the env
    // var, so this is inert outside the batch driver.
    const char* queue = std::getenv("CAX_BATCH_QUEUE");
    if (queue && *queue)
    {
        cvxCmdBuffer("~CaxExportBatchRun", 0);
    }
    return 0;
}

// ---- unload entry: <DllName>Exit ----
extern "C" __declspec(dllexport) int CAX_PLUGIN_EXIT(void)
{
    cvxCmdFuncUnload("CaxExportRun");
    cvxCmdFuncUnload("CaxExportBatchRun");
    return 0;
}
