#ifndef _LINE_FEATURE_H_
#define _LINE_FEATURE_H_

#include <iostream>
#include <stdio.h>
#include <math.h>
#include "utilities.h"


namespace line_feature
{

class LineFeature
{
	public:
		LineFeature();
		//
		~LineFeature();
        void setCachedRangeData(const std::vector<double>& , const std::vector<unsigned int>& , const RangeData& );
        void extractLines(std::vector<gline>&);

		void set_least_threshold(double);
		void set_min_line_length(double);
        void set_max_point_distance(double);
        void set_min_predict_distance(double);
		void set_min_line_points(unsigned int);
		void set_seed_line_points(unsigned int);
	private:
        least leastsquare(int,int,int);
		bool detectline(const int,const int);
        int findline(const int);
        void eraseline();
		bool delete_short_line(const int,const int);
        void endpoint_generation(std::vector<gline>&);
	private:

        CachedData cs_data_;
        RangeData range_data_;
        Params params_;
		std::vector<unsigned int> point_num_;
        std::vector<line> m_line;
        least m_least;

		double mid1;
		double mid2;
		double mid3;
		double mid4;
		double mid5;
};

}//namespace line_feature
#endif

