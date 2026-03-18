#include "zenslam_metal/pyr_lk.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

namespace
{
    constexpr const char* kPyrLkMetalSource = R"(
#include <metal_stdlib>
using namespace metal;

struct LKParams {
    uint point_count;
    float half_window;
    float epsilon;
    float level_scale;
    uint flags;
    uint max_iterations;
};

kernel void pyr_lk_level(
    texture2d<float, access::sample> prev_tex [[texture(0)]],
    texture2d<float, access::sample> next_tex [[texture(1)]],
    device const float2* prev_points [[buffer(0)]],
    device float2* next_points [[buffer(1)]],
    device uchar* status [[buffer(2)]],
    device float* errors [[buffer(3)]],
    constant LKParams& params [[buffer(4)]],
    uint gid [[thread_position_in_grid]])
{
    if (gid >= params.point_count) {
        return;
    }

    if (status[gid] == 0) {
        return;
    }

    constexpr sampler s(coord::pixel, address::clamp_to_edge, filter::linear);

    const float2 texture_size = float2(prev_tex.get_width(), prev_tex.get_height());
    const float margin = params.half_window + 1.0;

    float2 p0 = prev_points[gid] * params.level_scale;
    float2 p1 = ((params.flags & 1u) != 0u ? next_points[gid] : prev_points[gid]) * params.level_scale;

    if (p0.x < margin || p0.y < margin || p0.x >= texture_size.x - margin || p0.y >= texture_size.y - margin) {
        status[gid] = 0;
        errors[gid] = 1e6f;
        return;
    }

    for (uint iter = 0; iter < params.max_iterations; ++iter) {
        if (p1.x < margin || p1.y < margin || p1.x >= texture_size.x - margin || p1.y >= texture_size.y - margin) {
            status[gid] = 0;
            errors[gid] = 1e6f;
            return;
        }

        float Gxx = 0.0f;
        float Gxy = 0.0f;
        float Gyy = 0.0f;
        float bx = 0.0f;
        float by = 0.0f;

        const int half_w = int(params.half_window);
        for (int dy = -half_w; dy <= half_w; ++dy) {
            for (int dx = -half_w; dx <= half_w; ++dx) {
                const float2 off = float2(float(dx), float(dy));
                const float I0 = prev_tex.sample(s, p0 + off).r;
                const float I1 = next_tex.sample(s, p1 + off).r;

                const float Ix = 0.5f * (next_tex.sample(s, p1 + off + float2(1.0f, 0.0f)).r - next_tex.sample(s, p1 + off - float2(1.0f, 0.0f)).r);
                const float Iy = 0.5f * (next_tex.sample(s, p1 + off + float2(0.0f, 1.0f)).r - next_tex.sample(s, p1 + off - float2(0.0f, 1.0f)).r);
                const float It = I1 - I0;

                Gxx += Ix * Ix;
                Gxy += Ix * Iy;
                Gyy += Iy * Iy;
                bx += Ix * It;
                by += Iy * It;
            }
        }

        const float det = Gxx * Gyy - Gxy * Gxy;
        if (fabs(det) < 1e-8f) {
            status[gid] = 0;
            errors[gid] = 1e6f;
            return;
        }

        const float inv_det = 1.0f / det;
        const float2 delta = -float2(
            (Gyy * bx - Gxy * by) * inv_det,
            (-Gxy * bx + Gxx * by) * inv_det);

        p1 += delta;
        errors[gid] = sqrt(bx * bx + by * by);

        if (length(delta) < params.epsilon) {
            break;
        }
    }

    next_points[gid] = p1 / params.level_scale;
}
)";

    struct lk_params_host
    {
        uint32_t point_count;
        float half_window;
        float epsilon;
        float level_scale;
        uint32_t flags;
        uint32_t max_iterations;
    };

    class metal_pyr_lk_backend
    {
    public:
        metal_pyr_lk_backend()
        {
            @autoreleasepool
            {
                _device = MTLCreateSystemDefaultDevice();
                if (_device == nil) return;

                NSError* error = nil;
                NSString* source = [NSString stringWithUTF8String:kPyrLkMetalSource];
                id<MTLLibrary> library = [_device newLibraryWithSource:source options:nil error:&error];
                if (library == nil || error != nil) return;

                id<MTLFunction> function = [library newFunctionWithName:@"pyr_lk_level"];
                if (function == nil) return;

                _pipeline = [_device newComputePipelineStateWithFunction:function error:&error];
                if (_pipeline == nil || error != nil) return;

                _queue = [_device newCommandQueue];
                _available = (_queue != nil);
            }
        }

        auto is_available() const -> bool { return _available; }

        auto make_texture(const cv::Mat& image) const -> id<MTLTexture>
        {
            cv::Mat gray;
            if (image.type() != CV_8UC1)
            {
                if (image.channels() == 3)
                {
                    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
                }
                else
                {
                    image.convertTo(gray, CV_8U);
                }
            }
            else
            {
                gray = image;
            }

            MTLTextureDescriptor* desc = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm
                                                                                            width:gray.cols
                                                                                           height:gray.rows
                                                                                        mipmapped:NO];
            desc.usage = MTLTextureUsageShaderRead;
            id<MTLTexture> texture = [_device newTextureWithDescriptor:desc];
            if (texture == nil) return nil;

            MTLRegion region = MTLRegionMake2D(0, 0, gray.cols, gray.rows);
            [texture replaceRegion:region mipmapLevel:0 withBytes:gray.data bytesPerRow:gray.step];
            return texture;
        }

        void track(
            const std::vector<cv::Mat>& prev_pyramid,
            const std::vector<cv::Mat>& next_pyramid,
            const std::vector<cv::Point2f>& prev_points,
            std::vector<cv::Point2f>& next_points,
            std::vector<uchar>& status,
            std::vector<float>& err,
            const cv::Size win_size,
            const int max_level,
            const cv::TermCriteria criteria,
            const int flags)
        {
            if (!_available || prev_points.empty() || prev_pyramid.empty() || next_pyramid.empty())
            {
                return;
            }

            const auto level_cap = std::min(
                max_level,
                static_cast<int>(std::min(prev_pyramid.size(), next_pyramid.size())) - 1);
            const auto levels = std::max(0, level_cap);

            if (!(flags & cv::OPTFLOW_USE_INITIAL_FLOW) || next_points.size() != prev_points.size())
            {
                next_points = prev_points;
            }

            status.assign(prev_points.size(), static_cast<uchar>(1));
            err.assign(prev_points.size(), 0.0f);

            @autoreleasepool
            {
                const auto points_bytes = static_cast<NSUInteger>(prev_points.size() * sizeof(cv::Point2f));
                id<MTLBuffer> prev_points_buffer = [_device newBufferWithBytes:prev_points.data() length:points_bytes options:MTLResourceStorageModeShared];
                id<MTLBuffer> next_points_buffer = [_device newBufferWithBytes:next_points.data() length:points_bytes options:MTLResourceStorageModeShared];
                id<MTLBuffer> status_buffer = [_device newBufferWithLength:static_cast<NSUInteger>(status.size()) options:MTLResourceStorageModeShared];
                id<MTLBuffer> err_buffer = [_device newBufferWithLength:static_cast<NSUInteger>(err.size() * sizeof(float)) options:MTLResourceStorageModeShared];

                if (prev_points_buffer == nil || next_points_buffer == nil || status_buffer == nil || err_buffer == nil)
                {
                    return;
                }

                std::memcpy(status_buffer.contents, status.data(), status.size());

                for (int level = levels; level >= 0; --level)
                {
                    id<MTLTexture> prev_tex = make_texture(prev_pyramid[level]);
                    id<MTLTexture> next_tex = make_texture(next_pyramid[level]);
                    if (prev_tex == nil || next_tex == nil)
                    {
                        return;
                    }

                    lk_params_host params {
                        static_cast<uint32_t>(prev_points.size()),
                        static_cast<float>(std::max(1, win_size.width) / 2),
                        static_cast<float>(criteria.epsilon > 0.0 ? criteria.epsilon : 0.001),
                        1.0f / static_cast<float>(1 << level),
                        static_cast<uint32_t>(((flags & cv::OPTFLOW_USE_INITIAL_FLOW) || level != levels) ? 1 : 0),
                        static_cast<uint32_t>(criteria.maxCount > 0 ? criteria.maxCount : 30)
                    };

                    id<MTLBuffer> params_buffer = [_device newBufferWithBytes:&params length:sizeof(params) options:MTLResourceStorageModeShared];
                    if (params_buffer == nil)
                    {
                        return;
                    }

                    id<MTLCommandBuffer> command_buffer = [_queue commandBuffer];
                    id<MTLComputeCommandEncoder> encoder = [command_buffer computeCommandEncoder];

                    [encoder setComputePipelineState:_pipeline];
                    [encoder setTexture:prev_tex atIndex:0];
                    [encoder setTexture:next_tex atIndex:1];
                    [encoder setBuffer:prev_points_buffer offset:0 atIndex:0];
                    [encoder setBuffer:next_points_buffer offset:0 atIndex:1];
                    [encoder setBuffer:status_buffer offset:0 atIndex:2];
                    [encoder setBuffer:err_buffer offset:0 atIndex:3];
                    [encoder setBuffer:params_buffer offset:0 atIndex:4];

                    const auto threads_per_group = std::min<NSUInteger>(static_cast<NSUInteger>(_pipeline.maxTotalThreadsPerThreadgroup), 256);
                    const MTLSize tg = MTLSizeMake(threads_per_group, 1, 1);
                    const MTLSize grid = MTLSizeMake(static_cast<NSUInteger>(prev_points.size()), 1, 1);
                    [encoder dispatchThreads:grid threadsPerThreadgroup:tg];
                    [encoder endEncoding];

                    [command_buffer commit];
                    [command_buffer waitUntilCompleted];
                }

                std::memcpy(next_points.data(), next_points_buffer.contents, points_bytes);
                std::memcpy(status.data(), status_buffer.contents, status.size());
                std::memcpy(err.data(), err_buffer.contents, err.size() * sizeof(float));
            }
        }

    private:
        bool _available = false;
        id<MTLDevice> _device = nil;
        id<MTLCommandQueue> _queue = nil;
        id<MTLComputePipelineState> _pipeline = nil;
    };

    auto backend() -> metal_pyr_lk_backend&
    {
        static metal_pyr_lk_backend instance;
        return instance;
    }
}

namespace zenslam::metal::detail
{
    auto metal_is_available() -> bool
    {
        return backend().is_available();
    }

    void metal_calc_optical_flow_pyr_lk(
        const std::vector<cv::Mat>& prev_pyramid,
        const std::vector<cv::Mat>& next_pyramid,
        const std::vector<cv::Point2f>& prev_points,
        std::vector<cv::Point2f>& next_points,
        std::vector<uchar>& status,
        std::vector<float>& err,
        const cv::Size win_size,
        const int max_level,
        const cv::TermCriteria criteria,
        const int flags,
        const double)
    {
        backend().track(
            prev_pyramid,
            next_pyramid,
            prev_points,
            next_points,
            status,
            err,
            win_size,
            max_level,
            criteria,
            flags);
    }
}
