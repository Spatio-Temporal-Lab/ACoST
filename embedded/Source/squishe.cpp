#include "../../include/algorithms.h"
#include "../../include/common.h"
#include <algorithm>

struct GPSPointWithSED {
    Point point;
    double sed;
};

AlgorithmResult SQUISHEAlgorithm::compress(const std::vector<Point>& points, 
                                            const std::vector<double>& params) {
    AlgorithmResult result;
    result.original_size = points.size();
    
    if (points.size() < 2) {
        result.indices = {0};
        result.compressed_size = 1;
        return result;
    }
    
    // SQUISH-E参数：ratio和sed阈值
    // 当ratio=1时，结果是SQUISH-E(μ)
    // 当sed=0时，结果是SQUISH-E(λ)
    double ratio = params.size() > 0 ? params[0] : 0.2;
    double sed_threshold = params.size() > 1 ? params[1] : 0.0;
    
    int max_buffer_size = static_cast<int>(ratio * points.size());
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<GPSPointWithSED> buffer;
    buffer.reserve(max_buffer_size + 1);
    buffer.push_back({points[0], 0});
    
    if (max_buffer_size > 2) {
        buffer.push_back({points[1], 0});
        
        for (size_t i = 2; i < points.size(); ++i) {
            buffer.push_back({points[i], 0});
            
            // 计算前一个点的SED
            if (buffer.size() >= 3) {
                Point segment_start = buffer[buffer.size() - 3].point;
                Point segment_end = buffer[buffer.size() - 1].point;
                double sed = TrajUtils::calcSED(
                    segment_start, buffer[buffer.size() - 2].point, segment_end);
                buffer[buffer.size() - 2].sed += sed;
            }
            
            // 缓冲区满了，移除一个点
            if (buffer.size() > static_cast<size_t>(max_buffer_size)) {
                auto start = buffer.begin();
                ++start;
                auto to_remove = buffer.end();
                
                for (auto it = buffer.begin() + 1; it != buffer.end() - 1; ++it) {
                    // 如果设置了sed阈值，优先移除超过阈值的点
                    if (sed_threshold > 0 && it->sed > sed_threshold) {
                        to_remove = it;
                        break;
                    }
                    if (to_remove == buffer.end() || it->sed < to_remove->sed) {
                        to_remove = it;
                    }
                }
                
                if (to_remove != buffer.end() && to_remove != buffer.begin() && 
                    to_remove != buffer.end() - 1) {
                    auto prev = to_remove - 1;
                    auto next = to_remove + 1;
                    
                    if (prev >= buffer.begin() + 1 && next < buffer.end() - 1) {
                        prev->sed += to_remove->sed;
                        next->sed += to_remove->sed;
                    }
                }
                
                buffer.erase(to_remove);
            }
        }
    } else {
        buffer.push_back({points[points.size() - 1], 0});
    }
    
    // 提取索引
    for (const auto& item : buffer) {
        result.indices.push_back(item.point.index);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.execution_time = static_cast<double>(duration.count());  // 微秒
    result.compressed_size = result.indices.size();
    result.compression_ratio = (double)result.compressed_size / result.original_size;
    
    return result;
}


