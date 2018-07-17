#ifndef LINE_EXTRACTION_UTILITIES_H
#define LINE_EXTRACTION_UTILITIES_H

#include <cmath>
#include <vector>

#define distance_point(a1,a2,b1,b2) sqrt((a1-b1)*(a1-b1)+(a2-b2)*(a2-b2))


namespace line_feature
{

typedef struct _POINTT
{
    double x;
    double y;
}POINTT;

struct CachedData
{
  std::vector<unsigned int> indices;
  std::vector<double> bearings;
};

struct RangeData
{
  std::vector<double> ranges;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct Params
{
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
  double max_point_distance;
  double predict_distance;
  unsigned int min_line_points;
  unsigned int seed_line_points;

};


struct PointParams
{
  std::vector<double> a;
  std::vector<double> ap;
  std::vector<double> app;
  std::vector<double> b;
  std::vector<double> bp;
  std::vector<double> bpp;
  std::vector<double> c;
  std::vector<double> s;
};

typedef struct _line
{
    double a;
    double b;
    double c;
    int left;
    int right;
    POINTT p1;
    POINTT p2;
    bool inte[2];
}line;


typedef struct _least
{
    double a;
    double b;
    double c;
}least;


typedef struct _point
{
    double role;
    double theta;
    double m_x;
    double m_y;
    double distance;
    double m_gradient;
    bool flag;
}PoinT;


typedef struct _generate_line
{
    //first point
    double x1;
    double y1;
    //end point
    double x2;
    double y2;

    double distance;
    double angle;
}gline;

inline double pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}

} // namespace line_extraction

#endif
