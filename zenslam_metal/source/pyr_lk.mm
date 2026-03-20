// pyr_lk.mm — Metal-accelerated Lucas-Kanade pyramid optical flow
// Compiled as Objective-C++ (OBJCXX) so we can use Metal's Obj-C API alongside C++.

#include "zenslam_metal/pyr_lk.h"

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <spdlog/spdlog.h>

// ---------------------------------------------------------------------------
// MSL source for the LK iteration kernel
// ---------------------------------------------------------------------------
static NSString* const kLKKernelSource = @R"MSL(
#include <metal_stdlib>
using namespace metal;

/// Single LK iteration: refines (dx, dy) for one point on one pyramid level.
/// Each thread handles one input point.
kernel void lk_optical_flow(
    texture2d<float, access::sample>  img0      [[ texture(0) ]],
    texture2d<float, access::sample>  img1      [[ texture(1) ]],
    device const float2*              pts0      [[ buffer(0)  ]],
    device float2*                    pts1      [[ buffer(1)  ]],
    device uchar*                     status    [[ buffer(2)  ]],
    constant int&                     win_half  [[ buffer(3)  ]],
    constant int&                     max_iters [[ buffer(4)  ]],
    uint                              gid       [[ thread_position_in_grid ]])
{
    constexpr sampler s(coord::pixel, address::clamp_to_edge, filter::nearest);

    if (status[gid] == 0u)
        return;

    const float2 p0 = pts0[gid];
    float2       p1 = pts1[gid];

    // Iterative LK with bounded Newton steps
    for (int iter = 0; iter < max_iters; ++iter)
    {
        float Ixx = 0.f, Ixy = 0.f, Iyy = 0.f;
        float bx  = 0.f, by  = 0.f;

        for (int dy = -win_half; dy <= win_half; ++dy)
        {
            for (int dx = -win_half; dx <= win_half; ++dx)
            {
                const float2 q0 = p0 + float2(dx, dy);

                // Spatial gradient via central differences in img0
                const float ix = (img0.sample(s, q0 + float2(1,0)).r
                                - img0.sample(s, q0 - float2(1,0)).r) * 0.5f;
                const float iy = (img0.sample(s, q0 + float2(0,1)).r
                                - img0.sample(s, q0 - float2(0,1)).r) * 0.5f;

                // Temporal difference
                const float It = img1.sample(s, p1 + float2(dx, dy)).r
                               - img0.sample(s, q0).r;

                Ixx += ix * ix;
                Ixy += ix * iy;
                Iyy += iy * iy;
                bx  -= ix * It;
                by  -= iy * It;
            }
        }

        // Solve 2×2 system: [Ixx Ixy; Ixy Iyy] * [vx; vy] = [bx; by]
        const float det = Ixx * Iyy - Ixy * Ixy;
        if (fabs(det) < 1e-6f)
        {
            status[gid] = 0u;
            return;
        }

        const float inv = 1.f / det;
        const float vx  = (Iyy * bx - Ixy * by) * inv;
        const float vy  = (Ixx * by - Ixy * bx) * inv;

        p1 += float2(vx, vy);

        if (fabs(vx) + fabs(vy) < 0.03f)
            break;
    }

    pts1[gid] = p1;

    // Mark lost if tracked point wandered outside the image
    const float w = img1.get_width();
    const float h = img1.get_height();
    if (p1.x < 0.f || p1.y < 0.f || p1.x >= w || p1.y >= h)
        status[gid] = 0u;
}
)MSL";

// ---------------------------------------------------------------------------
// Singleton GPU context — initialised once
// ---------------------------------------------------------------------------
namespace
{
    struct metal_context
    {
        id<MTLDevice>              device        = nil;
        id<MTLCommandQueue>        queue         = nil;
        id<MTLComputePipelineState> pipeline     = nil;

        metal_context()
        {
            device = MTLCreateSystemDefaultDevice();
            if (!device)
                throw std::runtime_error("zenslam_metal: no Metal device");

            queue = [device newCommandQueue];

            NSError* err = nil;
            id<MTLLibrary> lib = [device newLibraryWithSource:kLKKernelSource
                                                      options:nil
                                                        error:&err];
            if (!lib)
            {
                const char* msg = err ? [[err localizedDescription] UTF8String] : "unknown";
                throw std::runtime_error(std::string("zenslam_metal: shader compile error: ") + msg);
            }

            id<MTLFunction> fn = [lib newFunctionWithName:@"lk_optical_flow"];
            if (!fn)
                throw std::runtime_error("zenslam_metal: kernel 'lk_optical_flow' not found");

            pipeline = [device newComputePipelineStateWithFunction:fn error:&err];
            if (!pipeline)
            {
                const char* msg = err ? [[err localizedDescription] UTF8String] : "unknown";
                throw std::runtime_error(std::string("zenslam_metal: pipeline error: ") + msg);
            }

            SPDLOG_INFO("zenslam_metal: initialised on GPU '{}'",
                        [device.name UTF8String]);
        }
    };

    metal_context& get_context()
    {
        static metal_context ctx;   // initialised on first call, thread-safe in C++11+
        return ctx;
    }

    // Make an R8Unorm MTLTexture from a grayscale CV_8UC1 Mat
    id<MTLTexture> mat_to_texture(const cv::Mat& img, id<MTLDevice> device)
    {
        assert(img.type() == CV_8UC1);
        assert(img.isContinuous());

        MTLTextureDescriptor* desc =
            [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                               width:static_cast<NSUInteger>(img.cols)
                                                              height:static_cast<NSUInteger>(img.rows)
                                                           mipmapped:NO];
        desc.usage       = MTLTextureUsageShaderRead;
        desc.storageMode = MTLStorageModeShared;

        id<MTLTexture> tex = [device newTextureWithDescriptor:desc];

        [tex replaceRegion:MTLRegionMake2D(0, 0,
                                           static_cast<NSUInteger>(img.cols),
                                           static_cast<NSUInteger>(img.rows))
               mipmapLevel:0
                 withBytes:img.data
               bytesPerRow:static_cast<NSUInteger>(img.cols)];
        return tex;
    }

} // anonymous namespace

namespace
{
    class metal_pyr_lk_backend
    {
    public:
        void calc(
            const std::vector<cv::Mat>&     pyramid_0,
            const std::vector<cv::Mat>&     pyramid_1,
            const std::vector<cv::Point2f>& points_0,
            std::vector<cv::Point2f>&       points_1,
            std::vector<uchar>&             status,
            std::vector<float>&             errors,
            const cv::Size                  window_size,
            const int                       max_level,
            const int                       flags)
        {
            std::lock_guard lock { _mutex };

            const auto n = static_cast<NSUInteger>(points_0.size());
            if (n == 0) return;

            auto& ctx = get_context();

            const bool use_initial = (flags & cv::OPTFLOW_USE_INITIAL_FLOW) != 0;
            if (!use_initial || points_1.size() != n)
            {
                points_1 = points_0;
            }
            status.assign(n, 1u);
            errors.assign(n, 0.f);

            const int max_valid_level = std::min(
                static_cast<int>(std::min(pyramid_0.size(), pyramid_1.size())) - 1,
                max_level);
            if (max_valid_level < 0) return;

            ensure_buffers(ctx.device, n);
            ensure_texture_cache(static_cast<size_t>(max_valid_level) + 1);

            const int win_half = window_size.width / 2;
            const int max_iters = std::clamp(window_size.width / 2, 10, 16);
            *reinterpret_cast<int*>(_buf_win.contents) = win_half;
            *reinterpret_cast<int*>(_buf_iters.contents) = max_iters;
            memcpy(_buf_status.contents, status.data(), n * sizeof(uchar));

            const float initial_scale = 1.f / static_cast<float>(1 << max_valid_level);
            _pts1_scaled.resize(n);
            _pts0_level.resize(n);
            for (NSUInteger i = 0; i < n; ++i)
            {
                _pts1_scaled[i] = points_1[i] * initial_scale;
            }

            id<MTLCommandBuffer> cmd = [ctx.queue commandBuffer];
            const NSUInteger threads_per_group = std::min(n, ctx.pipeline.maxTotalThreadsPerThreadgroup);

            for (int lvl = max_valid_level; lvl >= 0; --lvl)
            {
                const cv::Mat& img0 = pyramid_0[static_cast<size_t>(lvl)];
                const cv::Mat& img1 = pyramid_1[static_cast<size_t>(lvl)];

                const float lvl_scale = static_cast<float>(1 << lvl);
                for (NSUInteger i = 0; i < n; ++i)
                {
                    _pts0_level[i] = points_0[i] / lvl_scale;
                }

                memcpy(_buf_pts0.contents, _pts0_level.data(), n * sizeof(cv::Point2f));
                memcpy(_buf_pts1.contents, _pts1_scaled.data(), n * sizeof(cv::Point2f));

                auto& textures = _textures[static_cast<size_t>(lvl)];
                ensure_texture(textures.tex0, img0.cols, img0.rows, ctx.device);
                ensure_texture(textures.tex1, img1.cols, img1.rows, ctx.device);

                upload_to_texture(textures.tex0, img0);
                upload_to_texture(textures.tex1, img1);

                id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
                [enc setComputePipelineState:ctx.pipeline];
                [enc setTexture:textures.tex0 atIndex:0];
                [enc setTexture:textures.tex1 atIndex:1];
                [enc setBuffer:_buf_pts0   offset:0 atIndex:0];
                [enc setBuffer:_buf_pts1   offset:0 atIndex:1];
                [enc setBuffer:_buf_status offset:0 atIndex:2];
                [enc setBuffer:_buf_win    offset:0 atIndex:3];
                [enc setBuffer:_buf_iters  offset:0 atIndex:4];
                [enc dispatchThreads:MTLSizeMake(n, 1, 1)
                    threadsPerThreadgroup:MTLSizeMake(threads_per_group, 1, 1)];
                [enc endEncoding];

                [cmd commit];
                [cmd waitUntilCompleted];

                memcpy(_pts1_scaled.data(), _buf_pts1.contents, n * sizeof(cv::Point2f));
                if (lvl > 0)
                {
                    for (auto& pt : _pts1_scaled)
                    {
                        pt *= 2.f;
                    }
                }

                if (lvl > 0)
                {
                    cmd = [ctx.queue commandBuffer];
                }
            }

            points_1.resize(n);
            memcpy(points_1.data(), _buf_pts1.contents, n * sizeof(cv::Point2f));
            memcpy(status.data(), _buf_status.contents, n * sizeof(uchar));
        }

    private:
        struct texture_pair
        {
            id<MTLTexture> tex0 = nil;
            id<MTLTexture> tex1 = nil;
        };

        static void ensure_texture(id<MTLTexture>& tex, const int width, const int height, id<MTLDevice> device)
        {
            if (tex && tex.width == static_cast<NSUInteger>(width) && tex.height == static_cast<NSUInteger>(height))
            {
                return;
            }

            MTLTextureDescriptor* desc =
                [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                                   width:static_cast<NSUInteger>(width)
                                                                  height:static_cast<NSUInteger>(height)
                                                               mipmapped:NO];
            desc.usage       = MTLTextureUsageShaderRead;
            desc.storageMode = MTLStorageModeShared;
            tex = [device newTextureWithDescriptor:desc];
        }

        static void upload_to_texture(id<MTLTexture> tex, const cv::Mat& img)
        {
            assert(img.type() == CV_8UC1);
            const cv::Mat contiguous = img.isContinuous() ? img : img.clone();
            [tex replaceRegion:MTLRegionMake2D(0, 0,
                                               static_cast<NSUInteger>(contiguous.cols),
                                               static_cast<NSUInteger>(contiguous.rows))
                   mipmapLevel:0
                     withBytes:contiguous.data
                   bytesPerRow:static_cast<NSUInteger>(contiguous.cols)];
        }

        void ensure_buffers(id<MTLDevice> device, const NSUInteger n)
        {
            const auto points_bytes = n * sizeof(cv::Point2f);
            if (_capacity >= n && _buf_pts0 && _buf_pts1 && _buf_status)
            {
                return;
            }

            _buf_pts0   = [device newBufferWithLength:points_bytes options:MTLResourceStorageModeShared];
            _buf_pts1   = [device newBufferWithLength:points_bytes options:MTLResourceStorageModeShared];
            _buf_status = [device newBufferWithLength:n * sizeof(uchar) options:MTLResourceStorageModeShared];
            if (!_buf_win)
            {
                _buf_win = [device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
            }
            if (!_buf_iters)
            {
                _buf_iters = [device newBufferWithLength:sizeof(int) options:MTLResourceStorageModeShared];
            }
            _capacity = n;
        }

        void ensure_texture_cache(const size_t levels)
        {
            if (_textures.size() < levels)
            {
                _textures.resize(levels);
            }
        }

        std::mutex _mutex;
        NSUInteger _capacity = 0;

        id<MTLBuffer> _buf_pts0 = nil;
        id<MTLBuffer> _buf_pts1 = nil;
        id<MTLBuffer> _buf_status = nil;
        id<MTLBuffer> _buf_win = nil;
        id<MTLBuffer> _buf_iters = nil;

        std::vector<cv::Point2f> _pts0_level;
        std::vector<cv::Point2f> _pts1_scaled;
        std::vector<texture_pair> _textures;
    };

    auto get_backend() -> metal_pyr_lk_backend&
    {
        static metal_pyr_lk_backend backend;
        return backend;
    }
} // namespace

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace zenslam::metal
{
    void calc_optical_flow_pyr_lk(
        const std::vector<cv::Mat>& pyramid_0,
        const std::vector<cv::Mat>& pyramid_1,
        const std::vector<cv::Point2f>& points_0,
        std::vector<cv::Point2f>&       points_1,
        std::vector<uchar>&             status,
        std::vector<float>&             errors,
        cv::Size                        window_size,
        int                             max_level,
        int                             flags)
    {
        get_backend().calc(
            pyramid_0,
            pyramid_1,
            points_0,
            points_1,
            status,
            errors,
            window_size,
            max_level,
            flags
        );
    }

} // namespace zenslam::metal
