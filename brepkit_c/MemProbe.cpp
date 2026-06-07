#include "MemProbe.h"

#include <cstdio>
#include <cstdlib>

// windows.h is confined to this single translation unit so its min/max (and
// other) macros never leak into the OCCT-heavy headers used elsewhere.
#if defined(_WIN32)
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
  #include <psapi.h>
  #pragma comment(lib, "psapi.lib")
#endif

namespace brepkit
{

namespace
{

// Read CAX_MEM_PROBE once. Default ON; a value starting with 0/f/F/n/N -> OFF.
bool ProbeEnabled()
{
    static const bool enabled = [] {
        const char* v = std::getenv("CAX_MEM_PROBE");
        if (!v || !*v) return true;  // default on
        const char c = v[0];
        return !(c == '0' || c == 'f' || c == 'F' || c == 'n' || c == 'N');
    }();
    return enabled;
}

} // namespace

void MemProbe(const char* label)
{
    if (!ProbeEnabled()) return;

#if defined(_WIN32)
    PROCESS_MEMORY_COUNTERS pmc;
    if (::GetProcessMemoryInfo(::GetCurrentProcess(), &pmc, sizeof(pmc)))
    {
        const double cur  = static_cast<double>(pmc.WorkingSetSize)     / (1024.0 * 1024.0);
        const double peak = static_cast<double>(pmc.PeakWorkingSetSize) / (1024.0 * 1024.0);
        std::printf("[CAX_MEM] %-32s cur=%8.1f MB   peak=%8.1f MB\n",
                    label ? label : "(null)", cur, peak);
        std::fflush(stdout);
    }
#else
    (void)label;
#endif
}

} // namespace brepkit
