// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"

#include "libdivide.h"

#include <zstd.h> // Zstd
#include <string.h> // memcpy

//extern "C"
//{

namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// Size of a block for predictor selection purposes
static const int kBlockSize = 8;

// Zstd compression level
static const int kZstdLevel = 1;

const char* DepthResultString(DepthResult result)
{
    switch (result)
    {
    case DepthResult::Success: return "Success";
    case DepthResult::FileTruncated: return "FileTruncated";
    case DepthResult::WrongFormat: return "WrongFormat";
    case DepthResult::Corrupted: return "Corrupted";
    case DepthResult::MissingFrame: return "MissingFrame";
    default: break;
    }
    return "Unknown";
}


//------------------------------------------------------------------------------
// Tools

bool IsDepthFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (file_bytes < kDepthHeaderBytes) {
        return false;
    }
    if (file_data[0] != kDepthFormatMagic) {
        return false;
    }
    return true;
}

bool IsKeyFrame(const uint8_t* file_data, unsigned file_bytes)
{
    if (!IsDepthFrame(file_data, file_bytes)) {
        return false;
    }
    return (file_data[1] & 1) != 0;
}


//------------------------------------------------------------------------------
// Depth Quantization

uint16_t AzureKinectDequantizeDepth(uint16_t quantized)
{
    if (quantized == 0) {
        return 0;
    }
    if (quantized < 550) {
        return quantized + 200;
    }
    if (quantized < 925) {
        return 750 + (quantized - 550) * 2;
    }
    if (quantized < 1300) {
        return 1500 + (quantized - 925) * 4;
    }
    if (quantized < 1675) {
        return 3000 + (quantized - 1300) * 8;
    }
    if (quantized < 2040) {
        return 6000 + (quantized - 1675) * 16;
    }
    return 0; // Invalid value
}

void DequantizeDepthImage(std::vector<uint16_t>& depth_inout)
{
    const int n = static_cast<int>( depth_inout.size() );
    uint16_t* depth = depth_inout.data();

    for (int i = 0; i < n; ++i) {
        depth[i] = AzureKinectDequantizeDepth(depth[i]);
    }
}


//------------------------------------------------------------------------------
// Depth Rescaling

void UndoRescaleImage_11Bits(
    uint16_t min_value,
    uint16_t max_value,
    std::vector<uint16_t>& quantized)
{
    uint16_t* data = quantized.data();
    const int size = static_cast<int>( quantized.size() );

    const unsigned smallest = min_value;
    const unsigned range = max_value - smallest + 1;
    if (range >= 2048) {
        return;
    }
    if (range <= 1) {
        for (int i = 0; i < size; ++i) {
            unsigned x = data[i];
            if (x == 0) {
                continue;
            }
            data[i] = static_cast<uint16_t>( x - 1 + smallest );
        }
        return;
    }

    // Rescale the data
    for (int i = 0; i < size; ++i) {
        unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        --x;
        const unsigned y = (x * range + 1023) / 2047;
        data[i] = static_cast<uint16_t>(y + smallest);
    }
}


//------------------------------------------------------------------------------
// Zstd
bool ZstdDecompress(
    const uint8_t* compressed_data,
    int compressed_bytes,
    int uncompressed_bytes,
    std::vector<uint8_t>& uncompressed)
{
    uncompressed.resize(uncompressed_bytes);
    const size_t size = ZSTD_decompress(
        uncompressed.data(),
        uncompressed.size(),
        compressed_data,
        compressed_bytes);
    if (ZSTD_isError(size)) {
        return false;
    }
    if (size != static_cast<size_t>( uncompressed_bytes )) {
        return false;
    }
    return true;
}




//------------------------------------------------------------------------------
// DepthCompressor
DepthResult DepthCompressor::Decompress(
    const std::vector<uint8_t>& compressed,
    int& width,
    int& height,
    std::vector<uint16_t>& depth_out)
{
    if (compressed.size() < kDepthHeaderBytes)
	{
        return DepthResult::FileTruncated;
    }
    const uint8_t* src = compressed.data();

    const DepthHeader* header = reinterpret_cast<const DepthHeader*>( src );
    if (header->Magic != kDepthFormatMagic) 
	{
        return DepthResult::WrongFormat;
    }
    const bool keyframe = (header->Flags & DepthFlags_Keyframe) != 0;
    VideoType video_codec_type = VideoType::H264;
    if ((header->Flags & DepthFlags_HEVC) != 0) 
	{
        video_codec_type = VideoType::H265;
    }
    const unsigned frame_number = header->FrameNumber;

    // We can only start decoding on a keyframe because these contain SPS/PPS.
    

    if (!keyframe && FrameCount == 0) { 
        return DepthResult::MissingFrame;
        
    }
    ++FrameCount;

#if 0 // This is okay I guess since we are using intra-frame compression.
    if (!keyframe && frame_number != CompressedFrameNumber + 1) {
		return DepthResult::MissingPFrame;
        
    }
#endif

    width = header->Width;
    height = header->Height;
    if (width < 1 || width > 4096 || height < 1 || height > 4096) {
		return DepthResult::Corrupted;
    }

    // Read header
    unsigned total_bytes = kDepthHeaderBytes + header->HighCompressedBytes + header->LowCompressedBytes;
    if (header->HighUncompressedBytes < 2) {
		return DepthResult::Corrupted;
	}
    if (compressed.size() != total_bytes) {
		return DepthResult::FileTruncated;
    }

    src += kDepthHeaderBytes;

    // Compress high bits
    bool success = ZstdDecompress(
        src,
        header->HighCompressedBytes,
        header->HighUncompressedBytes,
        High);
    if (!success) {
		return DepthResult::Corrupted;
    }

    src += header->HighCompressedBytes;

    success = Codec.Decode(
        width,
        height,
        video_codec_type,
        src,
        header->LowCompressedBytes,
        Low);
    if (!success) {
		return DepthResult::Corrupted;
    }

    src += header->LowCompressedBytes;

    Unfilter(width, height, depth_out);
    UndoRescaleImage_11Bits(header->MinimumDepth, header->MaximumDepth, depth_out);
    DequantizeDepthImage(depth_out);

	return DepthResult::Success;
}


//------------------------------------------------------------------------------
// DepthCompressor : Unfiltering

void DepthCompressor::Unfilter(
    int width,
    int height,
    std::vector<uint16_t>& depth_out)
{
    const int n = width * height;
    depth_out.resize(n);
    uint16_t* depth = depth_out.data();
    const uint8_t* low_data = Low.data();
    const uint8_t* high_data = High.data();

    for (int i = 0; i < n; i += 2) {
        const uint8_t high = high_data[i / 2];
        uint8_t low_0 = low_data[i];
        uint8_t low_1 = low_data[i + 1];
        unsigned high_0 = high & 15;
        unsigned high_1 = high >> 4;

        if (high_0 == 0) {
            depth[i] = 0;
        } else {
            high_0--;
            if (high_0 & 1) {
                low_0 = 255 - low_0;
            }
            uint16_t x = static_cast<uint16_t>(low_0 | (high_0 << 8));

            // This value is expected to always be at least 1
            if (x == 0) {
                x = 1;
            }

            depth[i] = x;
        }

        if (high_1 == 0) {
            depth[i + 1] = 0;
        } else {
            high_1--;
            if (high_1 & 1) {
                low_1 = 255 - low_1;
            }
            uint16_t y = static_cast<uint16_t>(low_1 | (high_1 << 8));

            // This value is expected to always be at least 1
            if (y == 0) {
                y = 1;
            }

            depth[i + 1] = y;
        }
    }
}


} // namespace zdepth

//} //extern "C"