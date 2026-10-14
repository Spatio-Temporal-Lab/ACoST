#define _USE_MATH_DEFINES  // 必须在 cmath 之前定义，以启用 M_PI
#define _CRT_SECURE_NO_WARNINGS  // 禁用sscanf等函数的安全警告
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <cstdio>
#include <tuple>  // 添加 tuple 头文件
#include <algorithm>  // 添加 algorithm 头文件，用于 transform 函数

// 如果编译器不支持 M_PI，手动定义
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

// ==================== GPS点类 ====================
class GPSPoint {
private:
    int U_id;        // 轨迹ID，用于分段处理
    double latitude;
    double longitude;
    long long timestamp;

public:
    GPSPoint() : U_id(0), latitude(0), longitude(0), timestamp(0) {}

    GPSPoint(int uid, double lat, double lon, long long ts)
        : U_id(uid), latitude(lat), longitude(lon), timestamp(ts) {}

    int getUid() const { return U_id; }
    double getLatitude() const { return latitude; }
    double getLongitude() const { return longitude; }
    long long getTimestamp() const { return timestamp; }

    void setUid(int uid) { U_id = uid; }
    void setLatitude(double lat) { latitude = lat; }
    void setLongitude(double lon) { longitude = lon; }
    void setTimestamp(long long ts) { timestamp = ts; }
};

// ==================== 距离计算工具类 ====================
class DistanceUtils {
public:
    // Haversine距离公式，返回单位：米
    static double haversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000.0; // 地球半径（米）
        double dLat = toRadians(lat2 - lat1);
        double dLon = toRadians(lon2 - lon1);
        double a = sin(dLat / 2) * sin(dLat / 2) +
            cos(toRadians(lat1)) * cos(toRadians(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c;
    }

private:
    static double toRadians(double degree) {
        return degree * M_PI / 180.0;
    }
};

// ==================== 抽象函数类 ====================
class AbstractFunction {
protected:
    GPSPoint Pstart;
    GPSPoint Pend;

public:
    virtual ~AbstractFunction() {}
    virtual GPSPoint estimate(long long time) = 0;
};

// ==================== 一阶贝塞尔函数 ====================
class FirstBezier : public AbstractFunction {
public:
    FirstBezier(const GPSPoint& pstart, const GPSPoint& pend) {
        this->Pstart = pstart;
        this->Pend = pend;
    }

    GPSPoint estimate(long long time) override {
        // 防止除以0
        long long timeDiff = Pend.getTimestamp() - Pstart.getTimestamp();
        if (timeDiff == 0) {
            // 起止时间相同，返回起点
            return Pstart;
        }

        double t = (double)(time - Pstart.getTimestamp()) / timeDiff;
        GPSPoint estPoint;
        double estLat = (1 - t) * Pstart.getLatitude() + t * Pend.getLatitude();
        double estLon = (1 - t) * Pstart.getLongitude() + t * Pend.getLongitude();
        estPoint.setLatitude(estLat);
        estPoint.setLongitude(estLon);
        estPoint.setTimestamp(time);
        return estPoint;
    }
};

// ==================== 二阶贝塞尔函数（简化版，不使用）====================
class SecondBezier : public AbstractFunction {
private:
    GPSPoint Pmid;

public:
    SecondBezier(const GPSPoint& pstart, const GPSPoint& pmid, const GPSPoint& pend) {
        this->Pstart = pstart;
        this->Pmid = pmid;
        this->Pend = pend;
    }

    GPSPoint estimate(long long time) override {
        // 简化：不使用二阶贝塞尔，直接使用线性插值
        // 避免复杂的时间计算问题
        long long timeDiff = Pend.getTimestamp() - Pstart.getTimestamp();
        if (timeDiff == 0) {
            return Pstart;
        }

        double t = (double)(time - Pstart.getTimestamp()) / timeDiff;
        GPSPoint estPoint;
        double estLat = (1 - t) * Pstart.getLatitude() + t * Pend.getLatitude();
        double estLon = (1 - t) * Pstart.getLongitude() + t * Pend.getLongitude();
        estPoint.setLatitude(estLat);
        estPoint.setLongitude(estLon);
        estPoint.setTimestamp(time);
        return estPoint;
    }
};

// ==================== 向量类 ====================
class Vector {
private:
    AbstractFunction* function;
    long long sTime;
    long long eTime;
    bool ownFunction; // 标记是否需要释放function

public:
    Vector() : function(nullptr), sTime(0), eTime(0), ownFunction(false) {}

    Vector(AbstractFunction* func, long long stime, long long etime, bool own = true)
        : function(func), sTime(stime), eTime(etime), ownFunction(own) {}

    ~Vector() {
        if (ownFunction && function != nullptr) {
            delete function;
        }
    }

    // 禁用拷贝构造和赋值
    Vector(const Vector&) = delete;
    Vector& operator=(const Vector&) = delete;

    void setETime(long long tEnd) { eTime = tEnd; }
    long long getETime() const { return eTime; }
    long long getSTime() const { return sTime; }

    // 扩展向量段到新的结束点（更新函数端点以避免外推误差）
    void extendTo(const GPSPoint& newEndPoint) {
        eTime = newEndPoint.getTimestamp();
        // 获取起点后，重新创建函数对象
        GPSPoint startPoint = function->estimate(sTime);
        if (ownFunction && function != nullptr) {
            delete function;
        }
        // 用起点和新端点创建新的FirstBezier，避免外推
        function = new FirstBezier(startPoint, newEndPoint);
        ownFunction = true;
    }

    GPSPoint getEstimate(long long time) const {
        return function->estimate(time);
    }

    GPSPoint getPstart() const {
        return function->estimate(sTime);
    }

    GPSPoint getPend() const {
        return function->estimate(eTime);
    }
};

// ==================== 读取GPS数据函数 ====================
// 将时间字符串转换为时间戳（毫秒）
// Geolife格式: date格式: "2008/10/23", time格式: "2:53:04"
long long parseTimestampGeolife(const string& dateStr, const string& timeStr) {
    struct tm tm = {};
    int year, month, day, hour, minute, second;

    // 解析日期: "2008/10/23"
    if (sscanf(dateStr.c_str(), "%d/%d/%d", &year, &month, &day) != 3) {
        cerr << "警告: 无法解析日期格式: " << dateStr << endl;
        return 0;
    }

    // 解析时间: "2:53:04" 或 "02:53:04"
    if (sscanf(timeStr.c_str(), "%d:%d:%d", &hour, &minute, &second) != 3) {
        cerr << "警告: 无法解析时间格式: " << timeStr << endl;
        return 0;
    }

    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    tm.tm_isdst = -1;
    time_t t = mktime(&tm);
    return (long long)t * 1000;
}

// WX_taxi和Trajectory格式: timestamp格式: "2020-07-18 09:07:17" 或 "2022-01-29 23:28:00"
long long parseTimestampFull(const string& timestampStr) {
    struct tm tm = {};
    int year, month, day, hour, minute, second;

    // 解析完整时间戳: "2020-07-18 09:07:17"
    if (sscanf(timestampStr.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) != 6) {
        cerr << "警告: 无法解析时间戳格式: " << timestampStr << endl;
        return 0;
    }

    tm.tm_year = year - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = day;
    tm.tm_hour = hour;
    tm.tm_min = minute;
    tm.tm_sec = second;
    tm.tm_isdst = -1;
    time_t t = mktime(&tm);
    return (long long)t * 1000;
}

// 从CSV文件读取GPS数据（支持多种数据集格式，支持限制点数）
// 支持的格式：
// 1. Geolife: latitude,longitude,date,time,U_id (5列)
// 2. WX_taxi: longitude,latitude,timestamp,name_id (4列)
// 3. Trajectory: longitude,latitude,timestamp,osm_way_id (4列)
vector<GPSPoint> readGPS(const string& filename, int maxPoints = -1) {
    vector<GPSPoint> points;
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "无法打开文件: " << filename << endl;
        return points;
    }

    string line;
    bool firstLine = true;
    int count = 0;
    int datasetType = -1;  // -1:未检测, 0:Geolife, 1:WX_taxi, 2:Trajectory

    while (getline(file, line)) {
        if (line.empty()) continue;

        // 跳过表头并检测数据集类型
        if (firstLine) {
            firstLine = false;
            // 检测数据集类型
            string header = line;
            transform(header.begin(), header.end(), header.begin(), ::tolower);
            if (header.find("latitude") != string::npos && header.find("date") != string::npos) {
                datasetType = 0;  // Geolife格式
                cout << "检测到数据集格式: Geolife" << endl;
            }
            else if (header.find("name_id") != string::npos) {
                datasetType = 1;  // WX_taxi格式
                cout << "检测到数据集格式: WX_taxi" << endl;
            }
            else if (header.find("osm_way_id") != string::npos) {
                datasetType = 2;  // Trajectory格式
                cout << "检测到数据集格式: Trajectory (WorldTrace)" << endl;
            }
            else {
                // 尝试根据列数判断
                stringstream ss(line);
                vector<string> fields;
                string field;
                while (getline(ss, field, ',')) {
                    fields.push_back(field);
                }
                if (fields.size() == 5) {
                    datasetType = 0;  // 假设是Geolife
                    cout << "根据列数推断数据集格式: Geolife (5列)" << endl;
                }
                else if (fields.size() == 4) {
                    datasetType = 1;  // 假设是WX_taxi或Trajectory
                    cout << "根据列数推断数据集格式: WX_taxi/Trajectory (4列)" << endl;
                }
                else {
                    cerr << "警告: 无法识别数据集格式，列数: " << fields.size() << endl;
                    cerr << "表头: " << line << endl;
                }
            }
            continue;
        }

        // 检查是否达到最大点数
        if (maxPoints > 0 && count >= maxPoints) {
            break;
        }

        stringstream ss(line);
        string field1, field2, field3, field4, field5;

        try {
            if (datasetType == 0) {
                // Geolife格式: latitude,longitude,date,time,U_id
                if (getline(ss, field1, ',') &&
                    getline(ss, field2, ',') &&
                    getline(ss, field3, ',') &&
                    getline(ss, field4, ',') &&
                    getline(ss, field5)) {

                    double latitude = stod(field1);
                    double longitude = stod(field2);
                    long long uidLong = stoll(field5);
                    int uid = (int)uidLong;
                    long long timestamp = parseTimestampGeolife(field3, field4);

                    if (timestamp == 0) {
                        cerr << "警告: 时间戳解析失败，跳过此行: " << line << endl;
                        continue;
                    }

                    points.push_back(GPSPoint(uid, latitude, longitude, timestamp));
                    count++;
                }
            }
            else if (datasetType == 1 || datasetType == 2) {
                // WX_taxi或Trajectory格式: longitude,latitude,timestamp,uid_field
                if (getline(ss, field1, ',') &&
                    getline(ss, field2, ',') &&
                    getline(ss, field3, ',') &&
                    getline(ss, field4)) {

                    double longitude = stod(field1);  // 注意：第一个是longitude
                    double latitude = stod(field2);   // 第二个是latitude
                    long long uidLong = stoll(field4);
                    int uid = (int)uidLong;
                    long long timestamp = parseTimestampFull(field3);

                    if (timestamp == 0) {
                        cerr << "警告: 时间戳解析失败，跳过此行: " << line << endl;
                        continue;
                    }

                    points.push_back(GPSPoint(uid, latitude, longitude, timestamp));
                    count++;
                }
            }
            else {
                // 未识别的格式，尝试通用解析
                vector<string> fields;
                string field;
                while (getline(ss, field, ',')) {
                    fields.push_back(field);
                }
                if (fields.size() >= 4) {
                    // 尝试解析
                    double lat = stod(fields[fields.size() - 4]);
                    double lon = stod(fields[fields.size() - 3]);
                    long long uidLong = stoll(fields[fields.size() - 1]);
                    int uid = (int)uidLong;
                    // 尝试解析时间戳
                    long long timestamp = 0;
                    if (fields.size() == 5) {
                        timestamp = parseTimestampGeolife(fields[2], fields[3]);
                    }
                    else {
                        timestamp = parseTimestampFull(fields[2]);
                    }
                    if (timestamp > 0) {
                        points.push_back(GPSPoint(uid, lat, lon, timestamp));
                        count++;
                    }
                }
            }
        }
        catch (const exception& e) {
            // 解析失败，跳过此行
            cerr << "解析行时出错: " << line << " (错误: " << e.what() << ")" << endl;
        }
    }

    file.close();
    return points;
}

// ==================== VOLTCom类 ====================
class VOLTCom {
private:
    double epsilon;
    bool debugMode;

public:
    VOLTCom(double epsi, bool debug = false) : epsilon(epsi), debugMode(debug) {}

    // 严格按照伪代码实现的extractVector算法
    vector<Vector*> extractVectorOrigin(const vector<GPSPoint>& traj) {
        vector<Vector*> comTraj;

        if (traj.size() == 0) {
            return comTraj;
        }
        else if (traj.size() == 1) {
            GPSPoint curPoint = traj[0];
            Vector* tmpVector = new Vector(
                new FirstBezier(curPoint, curPoint),
                curPoint.getTimestamp(),
                curPoint.getTimestamp()
            );
            comTraj.push_back(tmpVector);
            return comTraj;
        }

        // Algorithm 1: Vector Extraction
        for (size_t i = 0; i < traj.size(); i++) {
            GPSPoint p_i = traj[i];

            if (i == 0) {
                // Line 2-3: if p_i is first then p_i ← initial_point
                // 只存储初始点，不创建向量
                continue;
            }
            else if (i == 1) {
                // Line 4-5: else if p_i is second then vector ← createVector(p_i, R)
                Vector* vector = createVector(p_i, comTraj, traj[0]);
                comTraj.push_back(vector);
                if (debugMode) {
                    cout << "[点 " << i << "] 创建第一个向量" << endl;
                }
            }
            else {
                // Line 6-13: else
                Vector* vector_cur = comTraj.back();

                // Line 8: p'_i ← getEstimate(p_i.t, vector_cur)
                GPSPoint p_prime = vector_cur->getEstimate(p_i.getTimestamp());

                // Line 9: if sed(p_i, p'_i) < α then
                double sedCur = DistanceUtils::haversine(
                    p_i.getLatitude(), p_i.getLongitude(),
                    p_prime.getLatitude(), p_prime.getLongitude()
                );

                if (debugMode) {
                    cout << "[点 " << i << "] 误差=" << sedCur << "m, epsilon=" << epsilon << "m";
                }

                if (sedCur < epsilon) {
                    // Line 10: vector_cur.t_end ← p_i.t
                    // 修复：更新端点以避免外推误差（伪代码设计缺陷的修正）
                    vector_cur->extendTo(p_i);
                    if (debugMode) {
                        cout << " → 扩展当前向量（总段数=" << comTraj.size() << "）" << endl;
                    }
                }
                else {
                    // Line 12: vector ← createVector(p_i, R)
                    Vector* vector = createVector(p_i, comTraj, traj[0]);
                    comTraj.push_back(vector);
                    if (debugMode) {
                        cout << " → 创建新向量（总段数=" << comTraj.size() << "）" << endl;
                    }
                }
            }
        }
        return comTraj;
    }

    // Algorithm 2: Vector Generation - createVector
    Vector* createVector(const GPSPoint& p, const vector<Vector*>& R, const GPSPoint& initial_point) {
        if (R.empty()) {
            // Line 1-3: if p is second then
            // 创建first_Bézier(p, R.initial_point)
            Vector* vector = new Vector(
                new FirstBezier(initial_point, p),
                initial_point.getTimestamp(),
                p.getTimestamp()
            );
            return vector;
        }
        else {
            // Line 4-9: else
            Vector* vector_cur = R.back();

            // Line 6-7: p_start ← vector_cur.f(t_start), p_end ← vector_cur.f(t_end)
            GPSPoint p_start = vector_cur->getPstart();
            GPSPoint p_end = vector_cur->getPend();

            // Line 8-9: 创建新向量（使用FirstBezier代替SecondBezier简化）
            // 从前一段的结束点到当前点
            Vector* vector = new Vector(
                new FirstBezier(p_end, p),
                p_end.getTimestamp(),
                p.getTimestamp()
            );
            return vector;
        }
    }
};

// ==================== 误差计算和解压缩函数 ====================
// 返回: {平均误差, 最大误差, 解压缩时间(us)}
tuple<double, double, double> computeErrorsAndDecompressionTime(
    const vector<Vector*>& segments,
    const vector<GPSPoint>& points) {

    double totalError = 0.0;
    double maxError = 0.0;
    size_t count = points.size();

    // 计时解压缩过程
    auto decompressStart = chrono::high_resolution_clock::now();

    size_t segIndex = 0;
    for (const GPSPoint& p : points) {
        // 寻找覆盖该时间的向量段（假定segments按时间升序排列）
        while (segIndex < segments.size() - 1 &&
            p.getTimestamp() > segments[segIndex]->getETime()) {
            segIndex++;
        }

        // 解压缩：根据时间戳重建GPS点
        GPSPoint estimated = segments[segIndex]->getEstimate(p.getTimestamp());

        // 计算误差（单位：米）
        double error = DistanceUtils::haversine(
            p.getLatitude(), p.getLongitude(),
            estimated.getLatitude(), estimated.getLongitude()
        );

        totalError += error;
        if (error > maxError) {
            maxError = error;
        }
    }

    auto decompressEnd = chrono::high_resolution_clock::now();
    double decompressTimeUs = chrono::duration<double, micro>(decompressEnd - decompressStart).count();

    double avgError = totalError / count;
    return make_tuple(avgError, maxError, decompressTimeUs);
}

// ==================== 主函数 ====================
int main(int argc, char* argv[]) {
    string filename;
    double epsilon;
    string saveFilename;
    int maxPoints = -1;  // -1表示读取所有点
    bool debugMode = false;

    // 如果没有传入参数，使用默认值
    if (argc < 3) {
        // 使用绝对路径，确保在VS中能找到文件
        //filename = "E:\\大四\\毕设\\提交\\TestOthers\\Geolife_100k_with_timestamp.csv";
        //filename = "E:\\大四\\毕设\\提交\\TestOthers\\Trajtory_100k_with_id.csv";
        filename = "E:\\大四\\毕设\\提交\\TestOthers\\WX_taxi_100k_with_id.csv";
        epsilon = 1.0;  // Error threshold in meters
        saveFilename = "outputVOLTCom.txt";  // Use relative path
        maxPoints = -1;  // -1 means read all data (100k points)
        debugMode = false;  // Debug mode off
    }
    else {
        filename = argv[1];
        epsilon = atof(argv[2]);
        saveFilename = argv[3];
        if (argc >= 5) {
            maxPoints = atoi(argv[4]);  // 可选：限制点数
        }
        if (argc >= 6 && string(argv[5]) == "debug") {
            debugMode = true;  // 可选：调试模式
        }
    }

    // 1. 读取轨迹数据
    cout << "\n========================================" << endl;
    cout << "VOLTCom 轨迹压缩算法" << endl;
    cout << "========================================" << endl;
    cout << "读取数据文件: " << filename << endl;
    if (maxPoints > 0) {
        cout << "限制读取: 前 " << maxPoints << " 个点" << endl;
    }
    else {
        cout << "读取模式: 全部数据" << endl;
    }
    vector<GPSPoint> points = readGPS(filename, maxPoints);
    size_t originalCount = points.size();

    if (originalCount == 0) {
        cout << "未读取到任何GPS数据！" << endl;
        return 1;
    }
    cout << "读取到 " << originalCount << " 个GPS点." << endl;

    // 2. 按U_id分组轨迹段
    vector<vector<GPSPoint>> trajectorySegments;
    vector<int> segmentStartIndices;  // 记录每个段在全局points中的起始索引

    if (points.size() > 0) {
        int currentUid = points[0].getUid();
        int segmentStart = 0;

        for (size_t i = 1; i < points.size(); i++) {
            if (points[i].getUid() != currentUid) {
                // U_id改变，保存当前段
                vector<GPSPoint> segment(points.begin() + segmentStart, points.begin() + i);
                trajectorySegments.push_back(segment);
                segmentStartIndices.push_back(segmentStart);

                // 开始新段
                currentUid = points[i].getUid();
                segmentStart = (int)i;
            }
        }
        // 保存最后一段
        vector<GPSPoint> lastSegment(points.begin() + segmentStart, points.end());
        trajectorySegments.push_back(lastSegment);
        segmentStartIndices.push_back(segmentStart);
    }

    cout << "检测到 " << trajectorySegments.size() << " 个轨迹段." << endl;

    if (debugMode) {
        cout << "\n【调试模式】显示详细处理过程：" << endl;
        cout << "epsilon阈值 = " << epsilon << " 米\n" << endl;
    }

    // 3. 对每个轨迹段分别进行压缩和误差计算
    VOLTCom simplificator(epsilon, debugMode);
    auto startTime = chrono::high_resolution_clock::now();

    vector<Vector*> allSegments;  // 所有轨迹段的压缩结果
    double totalErrorSum = 0;     // 所有段的误差总和
    double globalMaxError = 0;    // 全局最大误差
    int totalErrorCount = 0;      // 所有段的误差计算点数
    double totalDecompressTimeUs = 0;  // 所有段的解压缩时间总和

    for (size_t segIdx = 0; segIdx < trajectorySegments.size(); segIdx++) {
        const vector<GPSPoint>& segment = trajectorySegments[segIdx];

        if (segment.size() < 2) {
            // 如果轨迹段点数少于2，创建单个向量
            if (segment.size() == 1) {
                GPSPoint curPoint = segment[0];
                Vector* tmpVector = new Vector(
                    new FirstBezier(curPoint, curPoint),
                    curPoint.getTimestamp(),
                    curPoint.getTimestamp()
                );
                allSegments.push_back(tmpVector);
            }
            continue;
        }

        // 对当前轨迹段进行压缩
        vector<Vector*> segmentSegments = simplificator.extractVectorOrigin(segment);

        // 计算当前段的误差
        double segAvgError, segMaxError, segDecompressTimeUs;
        tie(segAvgError, segMaxError, segDecompressTimeUs) =
            computeErrorsAndDecompressionTime(segmentSegments, segment);

        // 累计误差统计（segment.size()就是该段的点数）
        int segErrorCount = (int)segment.size();
        totalErrorSum += segAvgError * segErrorCount;
        totalErrorCount += segErrorCount;
        if (segMaxError > globalMaxError) {
            globalMaxError = segMaxError;
        }
        totalDecompressTimeUs += segDecompressTimeUs;

        // 合并压缩结果
        allSegments.insert(allSegments.end(),
            segmentSegments.begin(), segmentSegments.end());
    }

    auto endTime = chrono::high_resolution_clock::now();

    if (debugMode) {
        cout << "\n【压缩完成】" << endl;
    }

    double totalTimeUs = chrono::duration<double, micro>(endTime - startTime).count();

    // 4. 计算总体统计指标
    double globalAvgError = (totalErrorCount > 0) ? totalErrorSum / totalErrorCount : 0;
    double compressionRatio = (double)allSegments.size() / originalCount;
    double avgCompressTimePerPointUs = totalTimeUs / originalCount;
    double avgDecompressTimePerPointUs = totalDecompressTimeUs / originalCount;

    // 5. 将压缩结果写入输出文件
    ofstream outFile(saveFilename);
    if (outFile.is_open()) {
        outFile << fixed << setprecision(6);
        for (Vector* seg : allSegments) {
            GPSPoint p = seg->getPstart();
            outFile << p.getLatitude() << " " << p.getLongitude() << " "
                << p.getTimestamp() << endl;
        }
        outFile << "\n========================================\n";
        outFile << "Performance Statistics\n";
        outFile << "========================================\n";
        outFile << "Original Points: " << originalCount << endl;
        outFile << "Compressed Segments: " << allSegments.size() << endl;
        outFile << "Compression Ratio: " << compressionRatio * 100 << "%" << endl;
        outFile << "\n--- Compression ---\n";
        outFile << "Total Compression Time: " << totalTimeUs << " us" << endl;
        outFile << "Average Time per Point: " << avgCompressTimePerPointUs << " us/point" << endl;
        outFile << "\n--- Decompression ---\n";
        outFile << "Total Decompression Time: " << totalDecompressTimeUs << " us" << endl;
        outFile << "Average Time per Point: " << avgDecompressTimePerPointUs << " us/point" << endl;
        outFile << "\n--- Accuracy (Distance in meters) ---\n";
        outFile << "Error Threshold (epsilon): " << epsilon << " m" << endl;
        outFile << "Average Error: " << globalAvgError << " m" << endl;
        outFile << "Maximum Error: " << globalMaxError << " m" << endl;
        outFile << "\n--- Trajectory Segments ---\n";
        outFile << "Number of Segments: " << trajectorySegments.size() << endl;
        outFile.close();
    }

    // 6. 在控制台输出统计信息
    cout << "\n========================================" << endl;
    cout << "VOLTCom Trajectory Compression Results" << endl;
    cout << "========================================" << endl;
    cout << fixed << setprecision(6);
    cout << "\n--- Data ---" << endl;
    cout << "Original Points: " << originalCount << endl;
    cout << "Compressed Segments: " << allSegments.size() << endl;
    cout << "Compression Ratio: " << compressionRatio * 100 << "%" << endl;
    cout << "\n--- Compression Performance ---" << endl;
    cout << "Total Time: " << totalTimeUs << " us" << endl;
    cout << "Average Time per Point: " << avgCompressTimePerPointUs << " us/point" << endl;
    cout << "\n--- Decompression Performance ---" << endl;
    cout << "Total Time: " << totalDecompressTimeUs << " us" << endl;
    cout << "Average Time per Point: " << avgDecompressTimePerPointUs << " us/point" << endl;
    cout << "\n--- Accuracy (Distance in meters) ---" << endl;
    cout << "Error Threshold (epsilon): " << epsilon << " m" << endl;
    cout << "Average Error: " << globalAvgError << " m" << endl;
    cout << "Maximum Error: " << globalMaxError << " m" << endl;
    cout << "\n--- Trajectory Segments ---" << endl;
    cout << "Number of Segments: " << trajectorySegments.size() << endl;
    cout << "========================================\n" << endl;

    // 清理内存
    for (Vector* vec : allSegments) {
        delete vec;
    }

    return 0;
}

