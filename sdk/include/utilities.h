#ifndef LINE_EXTRACTION_UTILITIES_H
#define LINE_EXTRACTION_UTILITIES_H

#include <cmath>
#include <vector>

#define threshold_a 6
#define distance_point(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))


namespace line_extraction {

typedef struct _POINTT {
  double x;
  double y;
} POINTT;

struct CachedData {
  std::vector<unsigned int> indices;
  std::vector<double> bearings;
};

struct RangeData {
  std::vector<double> ranges;
  std::vector<double> xs;
  std::vector<double> ys;
  void clear() {
    ranges.clear();
    xs.clear();
    ys.clear();
  }
};

struct Params {
  double bearing_var;
  double range_var;
  double least_thresh;
  double least_sq_angle_thresh;
  double least_sq_radius_thresh;
  double max_line_gap;
  double min_line_length;
  double min_range;
  double min_split_dist;
  double outlier_dist;
  double max_point_distance;//两点间最大允许分割距离
  double predict_distance;//真实点与与预测点之间的距离阈值
  unsigned int min_line_points;//一条线段包含的激光点个数
  unsigned int seed_line_points;//种子线段包含的激光点个数

};


struct PointParams {
  std::vector<double> a;
  std::vector<double> ap;
  std::vector<double> app;
  std::vector<double> b;
  std::vector<double> bp;
  std::vector<double> bpp;
  std::vector<double> c;
  std::vector<double> s;
};

//直线段信息结构体
typedef struct _line {
  double a;//直线参数
  double b;
  double c;
  int left;//直线范围
  int right;
  POINTT p1;
  POINTT p2;
  bool inte[2];
} line;

//直线方程式结构体
typedef struct _least {
  double a;
  double b;
  double c;
} least;

//}

typedef struct _point {
  double role;
  double theta;
  double m_x;
  double m_y;
  double distance;
  double m_gradient;
  bool flag;
} PoinT;


typedef struct _generate_line {
  //first point
  double x1;
  double y1;
  //end point
  double x2;
  double y2;

  double distance;
  double angle;
} gline;

inline double pi_to_pi(double angle) {
  angle = fmod(angle, 2 * M_PI);

  if (angle >= M_PI) {
    angle -= 2 * M_PI;
  }

  return angle;
}

} // namespace line_extraction

#endif
