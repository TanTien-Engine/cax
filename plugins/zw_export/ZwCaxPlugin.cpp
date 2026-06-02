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

#include "zwapi_cmd.h"       // cvxCmdFunc / cvxCmdFuncUnload / VX_CODE_GENERAL
#include "zwapi_file.h"      // cvxFileInqActive
#include "zwapi_message.h"   // cvxMsgDisp

#include <cstring>
#include <string>

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

} // namespace

// ---- load entry: <DllName>Init, exported undecorated (CaxExport.dll) ----
//
// Called by ZW3D when the DLL is loaded (via Application -> Applications
// and Plugin Manager -> Load Dll, or auto-loaded from the apilibs
// folder). Registers the command(s) this plugin exposes. The user then
// runs the command by name (typed at the input as ~CaxExportRun, or
// bound to a button / hotkey). Name = DLL base name + "Init", per the
// SDK template's ZW3DTemplateInit.
extern "C" __declspec(dllexport) int CaxExportInit(void)
{
    // cvxCmdFunc(name, function, code): bind a command name to a
    // callback. VX_CODE_GENERAL (0.0) is the plain-command licensing
    // code; the callback is passed as a void*.
    cvxCmdFunc("CaxExportRun", reinterpret_cast<void*>(CaxExportCmd), VX_CODE_GENERAL);
    return 0;
}

// ---- unload entry: <DllName>Exit ----
extern "C" __declspec(dllexport) int CaxExportExit(void)
{
    cvxCmdFuncUnload("CaxExportRun");
    return 0;
}
