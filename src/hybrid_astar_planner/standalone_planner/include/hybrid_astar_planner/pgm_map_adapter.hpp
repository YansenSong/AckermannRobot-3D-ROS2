/**
 * PGM 地图适配器 — 使 PGM 栅格地图兼容 Hybrid A* 的 CostmapT 接口
 *
 * 功能:
 *   1. 解析 P5 二进制 PGM 文件
 *   2. 读取 map.yaml 获取分辨率、origin
 *   3. PGM 像素值 → Nav2 标准 cost 值映射
 *      PGM 0 (黑/占据)  → OCCUPIED (254)
 *      PGM 254 (白/空闲) → FREE (0)
 *      PGM 205 (灰/未知) → UNKNOWN (255)
 *   4. 世界坐标 ↔ 栅格坐标互转
 */

#ifndef HYBRID_ASTAR_PLANNER__PGM_MAP_ADAPTER_HPP_
#define HYBRID_ASTAR_PLANNER__PGM_MAP_ADAPTER_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <cstring>
#include <iostream>
#include <cmath>
#include <algorithm>

namespace hybrid_astar_planner
{

class PGMMapAdapter
{
public:
  // ---- 必须的静态常数 (CostmapT 接口要求) ----
  static constexpr double UNKNOWN   = 255;   // 未知区域
  static constexpr double OCCUPIED  = 254;   // 占据 (碰撞检测阈值: cost >= OCCUPIED)
  static constexpr double INSCRIBED = 253;   // 内切膨胀 (波前启发式: cost < INSCRIBED)
  static constexpr double FREE      = 0;     // 空闲

  PGMMapAdapter() = default;

  /**
   * @brief 从 PGM + YAML 加载地图
   * @param pgm_path  PGM 文件路径 (P5 格式)
   * @param yaml_data YAML 解析后的内容 (需包含 resolution, origin)
   * @param width     地图宽度 (像素), 从 YAML 读取, 用于验证
   * @param height    地图高度 (像素)
   */
  bool load(const std::string & pgm_path,
            double resolution, double origin_x, double origin_y)
  {
    resolution_ = resolution;
    origin_x_   = origin_x;
    origin_y_   = origin_y;

    // ---- 解析 P5 PGM 二进制文件 ----
    std::ifstream f(pgm_path, std::ios::binary);
    if (!f) {
      std::cerr << "[PGMMapAdapter] Cannot open: " << pgm_path << "\n";
      return false;
    }

    std::string line;
    // P5 魔数
    std::getline(f, line);
    if (line != "P5") {
      std::cerr << "[PGMMapAdapter] Not a P5 PGM: " << line << "\n";
      return false;
    }

    // 跳过注释行
    while (f.peek() == '#') {
      std::getline(f, line);
    }

    // 读取宽高
    f >> width_ >> height_;
    int maxval;
    f >> maxval;
    f.get();  // 消耗换行符

    if (maxval > 255) {
      std::cerr << "[PGMMapAdapter] Unsupported maxval: " << maxval << "\n";
      return false;
    }

    // 读入像素数据 (PGM row 0 = 图像顶部)
    // 翻转 Y 轴: ROS 地图坐标系 row 0 = 底部 (origin_y), row (height-1) = 顶部
    std::vector<uint8_t> pgm_raw;
    pgm_raw.resize(width_ * height_);
    f.read(reinterpret_cast<char *>(pgm_raw.data()), width_ * height_);
    data_.resize(width_ * height_);
    for (unsigned int row = 0; row < height_; ++row) {
      unsigned int src_row = height_ - 1 - row;  // PGM 顶部 → ROS 底部
      std::memcpy(data_.data() + row * width_,
                  pgm_raw.data() + src_row * width_,
                  width_);
    }

    std::cout << "[PGMMapAdapter] Loaded " << pgm_path
              << " (" << width_ << "x" << height_ << ")\n"
              << "  resolution: " << resolution_ << " m/cell\n"
              << "  origin: [" << origin_x_ << ", " << origin_y_ << "]\n"
              << "  coverage: " << width_ * resolution_ << " x "
              << height_ * resolution_ << " m\n";

    return true;
  }

  // ================================================================
  // CostmapT 接口
  // ================================================================

  /** 世界坐标 → 栅格坐标 */
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) return false;
    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);
    return (mx < width_ && my < height_);
  }

  /** 整数栅格坐标 → 世界坐标 (取格点中心) */
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + (my + 0.5) * resolution_;
  }

  /** 连续栅格坐标 → 世界坐标 (保持精度) */
  void mapToWorldContinuous(float mx, float my, double & wx, double & wy) const
  {
    wx = origin_x_ + static_cast<double>(mx) * resolution_;
    wy = origin_y_ + static_cast<double>(my) * resolution_;
  }

  /** 获取指定栅格 cost 值 (已映射到 Nav2 约定) */
  double getCost(unsigned int mx, unsigned int my) const
  {
    if (mx >= width_ || my >= height_) return OCCUPIED;
    return pgmToCost(data_[my * width_ + mx]);
  }

  /** 获取平坦索引 cost 值 */
  double getCost(unsigned int idx) const
  {
    if (idx >= data_.size()) return OCCUPIED;
    return pgmToCost(data_[idx]);
  }

  unsigned int getSizeInCellsX() const { return width_; }
  unsigned int getSizeInCellsY() const { return height_; }

  double getResolution() const { return resolution_; }
  double getOriginX()    const { return origin_x_; }
  double getOriginY()    const { return origin_y_; }

  /** 获取原始 PGM 像素值 (用于构建 OccupancyGrid) */
  const std::vector<uint8_t> & getRawData() const { return data_; }

private:
  /** PGM 像素值 → Nav2 Cost 映射 */
  static inline double pgmToCost(uint8_t pgm_val)
  {
    // PGM P5 约定: 0=黑(占据), 254=白(空闲), 205=灰(未知)
    // Nav2  约定: 254=OCCUPIED, 0=FREE, 255=UNKNOWN
    switch (pgm_val) {
      case 0:   return OCCUPIED;   // 黑 → 占据
      case 254: return FREE;        // 白 → 空闲
      case 205: return UNKNOWN;     // 灰 → 未知
      default:  return static_cast<double>(pgm_val);  // 保持原值
    }
  }

  std::vector<uint8_t> data_;
  unsigned int width_  = 0;
  unsigned int height_ = 0;
  double resolution_   = 0.05;
  double origin_x_     = 0.0;
  double origin_y_     = 0.0;
};

}  // namespace hybrid_astar_planner

#endif  // HYBRID_ASTAR_PLANNER__PGM_MAP_ADAPTER_HPP_
