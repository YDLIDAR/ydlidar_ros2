#ifndef _LINE_FEATURE_H_
#define _LINE_FEATURE_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include "utilities.h"


using namespace line_extraction;
namespace line_feature {

class LineFeature {
 public:
  LineFeature();
  //
  ~LineFeature();
  //子函数声明
  //设置range，每次都需要传递range消息，在主入口函数的回调函数进行
  void setCachedRangeData(const std::vector<double> &,
                          const std::vector<unsigned int> &, const RangeData &);
  //返回直线分割结果
  void extractLines(std::vector<gline> &);
  //设置参数
  void set_least_threshold(double);
  void set_min_line_length(double);
  void set_max_point_distance(double);
  void set_min_predict_distance(double);
  void set_min_line_points(unsigned int);
  void set_seed_line_points(unsigned int);
 private:
  //通过激光数据的首末索引值进行直线方程的求解
  least leastsquare(int, int, int);
  //检测种子直线
  bool detectline(const int, const int);
  //通过种子直线，复原出整条直线，并进行最后的求定
  int findline(const int);
  //整理整条直线
  void eraseline();
  //删除小于长度阈值的线段
  bool delete_short_line(const int, const int);
  //
  void endpoint_generation(std::vector<gline> &temp_line2);
 private:
  line_extraction::CachedData cs_data_;
  line_extraction::RangeData range_data_;
  line_extraction::Params params_;
  std::vector<unsigned int> point_num_;
  //线段结构体信息
  std::vector<line> m_line;
  //直线拟合中间传递变量，已设为全局变量
  line_extraction::least m_least;

  //拟合中间变量
  double mid1;
  double mid2;
  double mid3;
  double mid4;
  double mid5;
};

}//namespace line_feature
#endif

