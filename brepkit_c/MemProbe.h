#pragma once

// ----------------------------------------------------------------------------
// MemProbe -- lightweight working-set memory probe.
//
// Logs the process's CURRENT resident size (WorkingSetSize) and its lifetime
// HIGH-WATER mark (PeakWorkingSetSize), tagged with a label. Because the peak
// is cumulative and never decreases, the *delta in peak* between two probe
// points tells you which phase caused a transient spike -- even though the
// memory has already been freed by the time the next probe runs.
//
// Used to localize the open -> replay -> mesh memory spike: place probes at
// phase boundaries (after parse, after replay, around BRepMesh) and watch
// where `peak` jumps.
//
// Enabled by default; set env CAX_MEM_PROBE=0 (or false/no) to silence.
// No-op on non-Windows platforms.
// ----------------------------------------------------------------------------

namespace brepkit
{

// Log "[CAX_MEM] <label>  cur=... MB  peak=... MB" to stdout (flushed).
void MemProbe(const char* label);

} // namespace brepkit
