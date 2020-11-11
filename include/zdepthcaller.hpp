#include "zdepth.hpp"

#ifdef __cplusplus
extern "C"
{
#endif 
extern __declspec(dllexport) zdepth::DepthCompressor* CreateDepthCompressor();
extern __declspec(dllexport) void DisposeDepthCompressor(zdepth::DepthCompressor* a_pObject);
extern __declspec(dllexport) int CSDecompress(zdepth::DepthCompressor* a_pObject,
											const uint8_t* compressed,
											int comp_len,
											uint16_t* depth_out
											);

extern __declspec(dllexport) int GetFrameCount(zdepth::DepthCompressor* a_pCompressor);
#ifdef __cplusplus
}

#endif