#include "../../include/algorithms.h"
#include "../../include/common.h"
#include <cmath>

AlgorithmResult DeadReckoningAlgorithm::compress(const std::vector<Point>& points, 
                                                  const std::vector<double>& params) {
    AlgorithmResult result;
    result.original_size = points.size();
    
    if (points.size() < 2) {
        result.indices = {0};
        result.compressed_size = 1;
        return result;
    }
    
    double eps = params[0];
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    int n = points.size();
    double max_d = 0;
    int start_idx = 0;
    
    // 计算距离和角度
    std::vector<double> distances;
    std::vector<double> angles;
    
    for (int i = 1; i < n; i++) {
        distances.push_back(TrajUtils::calcDistance(points[i-1], points[i]));
        angles.push_back(TrajUtils::calcAngle(points[i-1], points[i]));
    }
    
    std::vector<int> simplifindex;
    simplifindex.push_back(0);
    
    for (int i = 2; i < n; i++) {
        max_d += std::fabs(distances[i-1] * std::sin(angles[i-1] - angles[start_idx]));
        if (std::abs(max_d) > eps) {
            max_d = 0;
            simplifindex.push_back(i - 1);
            start_idx = i - 1;
        }
    }
    
    if (simplifindex.back() != n - 1) {
        simplifindex.push_back(n - 1);
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.execution_time = static_cast<double>(duration.count());  // 微秒
    result.indices = simplifindex;
    result.compressed_size = simplifindex.size();
    result.compression_ratio = (double)result.compressed_size / result.original_size;
    
    return result;
}


