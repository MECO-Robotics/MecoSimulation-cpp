# WPILib 2026 adapter build

This folder builds the optional Drive bridge as a static C++ library against the pinned WPILib 2026 toolchain and the matching MECO Base public headers. The workflow checks both Debug and Release on Linux and Windows. The bridge only becomes a supported vendordep artifact after clean Base plus Simulation consumer links pass on both targets.
