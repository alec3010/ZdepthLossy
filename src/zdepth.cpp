// Copyright 2019 (c) Christopher A. Taylor.  All rights reserved.

#include "zdepth.hpp"

#include "libdivide.h"

#include <zstd.h> // Zstd
#include <string.h> // memcpy

namespace zdepth {


//------------------------------------------------------------------------------
// Constants

// Size of a block for predictor selection purposes
static const int kBlockSize = 8;

// Zstd compression level
static const int kZstdLevel = 1;


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

uint16_t AzureKinectQuantizeDepth(uint16_t depth)
{
    if (depth <= 200) {
        return 0; // Too close
    }
    if (depth < 750) {
        return depth - 200;
    }
    if (depth < 1500) {
        return 550 + (depth - 750) / 2;
    }
    if (depth < 3000) {
        return 925 + (depth - 1500) / 4;
    }
    if (depth < 6000) {
        return 1300 + (depth - 3000) / 8;
    }
    if (depth < 11840) {
        return 1675 + (depth - 6000) / 16;
    }
    return 0; // Too far
}


void QuantizeDepthImage(
    int n,
    const uint16_t* depth,
    std::vector<uint16_t>& quantized)
{
    quantized.resize(n);
    uint16_t* dest = quantized.data();

    for (int i = 0; i < n; ++i) {
        dest[i] = AzureKinectQuantizeDepth(depth[i]);
    }
}




//------------------------------------------------------------------------------
// Depth Rescaling

void RescaleImage_11Bits(
    std::vector<uint16_t>& quantized,
    uint16_t& min_value,
    uint16_t& max_value)
{
    uint16_t* data = quantized.data();
    const int size = static_cast<int>( quantized.size() );

    // Find extrema
    int i;
    for (i = 0; i < size; ++i) {
        if (data[i] != 0) {
            break;
        }
    }
    if (i >= size) {
        min_value = max_value = 0;
        return;
    }
    unsigned smallest = data[i];
    unsigned largest = smallest;
    for (; i < size; ++i) {
        const unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        if (smallest > x) {
            smallest = x;
        }
        if (largest < x) {
            largest = x;
        }
    }

    min_value = static_cast<uint16_t>( smallest );
    max_value = static_cast<uint16_t>( largest );

    // Handle edge cases
    const unsigned range = largest - smallest + 1;
    if (range >= 2048) {
        return;
    }
    if (range <= 1) {
        if (smallest != 0) {
            for (i = 0; i < size; ++i) {
                unsigned x = data[i];
                if (x == 0) {
                    continue;
                }
                data[i] = 1;
            }
        }
        return;
    }
    const unsigned rounder = range / 2;

    using branchfree_t = libdivide::branchfree_divider<unsigned>;
    branchfree_t fast_d = range;

    // Rescale the data
    for (i = 0; i < size; ++i) {
        unsigned x = data[i];
        if (x == 0) {
            continue;
        }
        x -= smallest;
        unsigned y = (x * 2047 + rounder) / fast_d;
        data[i] = static_cast<uint16_t>(y + 1);
    }
}

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

void ZstdCompress(
    const std::vector<uint8_t>& uncompressed,
    std::vector<uint8_t>& compressed)
{
    compressed.resize(ZSTD_compressBound(uncompressed.size()));
    const size_t size = ZSTD_compress(
        compressed.data(),
        compressed.size(),
        uncompressed.data(),
        uncompressed.size(),
        kZstdLevel);
    if (ZSTD_isError(size)) {
        compressed.clear();
        return;
    }
    compressed.resize(size);
}

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

void DepthCompressor::Compress(
    const VideoParameters& params,
    const uint16_t* unquantized_depth,
    std::vector<uint8_t>& compressed,
    bool keyframe)
{
    // Enforce keyframe if we have not compressed anything yet
    if (FrameCount == 0) {
        keyframe = true;
    }

    DepthHeader header;
    header.Magic = kDepthFormatMagic;
    header.Flags = 0;
    if (keyframe) {
        header.Flags |= DepthFlags_Keyframe;
    }
    if (params.Type == VideoType::H265) {
        header.Flags |= DepthFlags_HEVC;
    }
    header.Width = static_cast<uint16_t>( params.Width );
    header.Height = static_cast<uint16_t>( params.Height );
    const int n = params.Width * params.Height;
    
    
    header.FrameNumber = static_cast<uint16_t>( FrameCount );
    ++FrameCount;
    

    QuantizeDepthImage(n, unquantized_depth, QuantizedDepth);
    RescaleImage_11Bits(QuantizedDepth, header.MinimumDepth, header.MaximumDepth);
    Filter(QuantizedDepth);

    Codec.EncodeBegin(
        params,
        keyframe,
        Low,
        LowOut);

    // Interleave Zstd compression with video encoder work.
    // Only saves about 400 microseconds from a 5000 microsecond encode.
    ZstdCompress(High, HighOut);
    header.HighUncompressedBytes = static_cast<uint32_t>( High.size() );
    header.HighCompressedBytes = static_cast<uint32_t>( HighOut.size() );

    Codec.EncodeFinish(LowOut);
    header.LowCompressedBytes = static_cast<uint32_t>( LowOut.size() );

    // Calculate output size
    size_t total_size = kDepthHeaderBytes + HighOut.size() + LowOut.size();
    compressed.resize(total_size);
    uint8_t* copy_dest = compressed.data();

    // Write header
    memcpy(copy_dest, &header, kDepthHeaderBytes);
    copy_dest += kDepthHeaderBytes;

    // Concatenate the compressed data
    memcpy(copy_dest, HighOut.data(), HighOut.size());
    copy_dest += HighOut.size();
    memcpy(copy_dest, LowOut.data(), LowOut.size());
}


//------------------------------------------------------------------------------
// DepthCompressor : Filtering

void DepthCompressor::Filter(
    const std::vector<uint16_t>& depth_in)
{
    const int n = static_cast<int>( depth_in.size() );
    const uint16_t* depth = depth_in.data();

    High.clear();
    Low.clear();
    High.resize(n / 2); // One byte for every two depth values
    Low.resize(n + n / 2); // Leave room for unused chroma channel

    // Split data into high/low parts
    for (int i = 0; i < n; i += 2) {
        const uint16_t depth_0 = depth[i];
        const uint16_t depth_1 = depth[i + 1];

        unsigned high_0 = 0, high_1 = 0;
        uint8_t low_0 = static_cast<uint8_t>( depth_0 );
        uint8_t low_1 = static_cast<uint8_t>( depth_1 );

        if (depth_0 != 0) {
            // Read high bits
            high_0 = depth_0 >> 8;

            // Fold to avoid sharp transitions from 255..0
            if (high_0 & 1) {
                low_0 = 255 - low_0;
            }

            // Preserve zeroes by offseting the values by 1
            ++high_0;
        }

        if (depth_1 != 0) {
            // Read high bits
            high_1 = depth_1 >> 8;

            // Fold to avoid sharp transitions from 255..0
            if (high_1 & 1) {
                low_1 = 255 - low_1;
            }

            // Preserve zeroes by offseting the values by 1
            ++high_1;
        }

        High[i / 2] = static_cast<uint8_t>( high_0 | (high_1 << 4) );
        Low[i] = low_0;
        Low[i + 1] = low_1;
    }
}


} // namespace zdepth
