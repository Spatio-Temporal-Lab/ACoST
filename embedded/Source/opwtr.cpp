#include "../../include/algorithms.h"
#include "../../include/common.h"

AlgorithmResult OPWTRAlgorithm::compress(const std::vector<Point>& points, 
                                          const std::vector<double>& params) {
    AlgorithmResult result;
    result.original_size = points.size();
    
    if (points.size() < 2) {
        result.indices = {0};
        result.compressed_size = 1;
        return result;
    }
    
    double epsilon = params[0];
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    std::vector<int> simplified;
    simplified.push_back(0);
    
    int e = 2;
    int originalIndex = 0;
    
    while (e < static_cast<int>(points.size())) {
        int i = originalIndex + 1;
        bool condOPW = true;
        
        while (i < e && condOPW) {
            // 使用SED而不是PED
            if (TrajUtils::calcSED(points[originalIndex], points[i], points[e]) > epsilon) {
                condOPW = false;
            } else {
                i++;
            }
        }
        
        if (!condOPW) {
            originalIndex = i;
            simplified.push_back(originalIndex);
            e = originalIndex + 2;
        } else {
            e++;
        }
    }
    
    simplified.push_back(points.size() - 1);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    result.execution_time = static_cast<double>(duration.count());  // 微秒
    result.indices = simplified;
    result.compressed_size = simplified.size();
    result.compression_ratio = (double)result.compressed_size / result.original_size;
    
    return result;
}


