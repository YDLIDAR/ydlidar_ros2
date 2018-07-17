#include "line_feature.h"

namespace line_feature
{

LineFeature::LineFeature() {
    set_least_threshold(0.03);
    set_min_line_length(0.01);
    set_min_predict_distance(0.02);
    set_max_point_distance(0.5);
    set_seed_line_points(4);
    set_min_line_points(6);
	
}


LineFeature::~LineFeature() {
	
}

//set paramters

void LineFeature::set_least_threshold(double least_thresh) {
	params_.least_thresh = least_thresh;
}

void LineFeature::set_min_line_length(double min_line_length) {
	params_.min_line_length = min_line_length;
}

void LineFeature::set_max_point_distance(double max_point_dist) {
    params_.max_point_distance = max_point_dist;
}

void LineFeature::set_min_predict_distance(double predict_distance) {
	params_.predict_distance = predict_distance;
}

void LineFeature::set_min_line_points(unsigned int min_line_points) {
	params_.min_line_points = min_line_points;
}

void LineFeature::set_seed_line_points(unsigned int seed_line_points) {
	params_.seed_line_points = seed_line_points;
}



void LineFeature::setCachedRangeData(const std::vector<double>& bearings, const std::vector<unsigned int>& index, const RangeData& ranges) {
  range_data_ = ranges;
  cs_data_.indices = index;
  cs_data_.bearings = bearings;

}

least LineFeature::leastsquare(int start,int end,int fit) {
	double w1 = 0,w2 = 0,w3 = 0;
	least temp;
	double n = end - start + 1;

    if(fit == 1) {
		mid1 = 0;
		mid2 = 0;
		mid3 = 0;
		mid4 = 0; 
		mid5 = 0;
		int k = 0;
        for(k = start;k <= end;k++) {
			mid1+=range_data_.xs[k];
			mid2+=range_data_.ys[k];
			mid3+=range_data_.xs[k]*range_data_.xs[k];
			mid4+=range_data_.ys[k]*range_data_.ys[k];
			mid5+=range_data_.xs[k]*range_data_.ys[k];
		}
    } else if(fit == 2) {
        mid1+=range_data_.xs[end];
        mid2+=range_data_.ys[end];
        mid3+=range_data_.xs[end]*range_data_.xs[end];
        mid4+=range_data_.ys[end]*range_data_.ys[end];
        mid5+=range_data_.xs[end]*range_data_.ys[end];
    } else {
        mid1+=range_data_.xs[start];
        mid2+=range_data_.ys[start];
        mid3+=range_data_.xs[start]*range_data_.xs[start];
        mid4+=range_data_.ys[start]*range_data_.ys[start];
        mid5+=range_data_.xs[start]*range_data_.ys[start];

    }
	w1 = n*mid5-mid1*mid2;
	w2 = mid2*mid2-n*mid4-mid1*mid1+n*mid3;
	w3 = mid1*mid2-n*mid5;

    if(w1==0) {
		temp.a = -1;
		temp.b = 0;
		temp.c = mid1/n;
    } else {
		temp.a = (-w2+sqrt(w2*w2-4*w1*w3))/2.0/w1;
		temp.b = -1;
		temp.c = (mid2-temp.a*mid1)/n;
	}
	return temp;
}

bool LineFeature::detectline(const int start,const int num) {

	double error1 = 0;

	double error2 = 0;

    double error3 = 0;

	int k = 0;

	POINTT m_pn;
	m_pn.x = 0;
	m_pn.y = 0;

	double kp = 0;
	double theta = 0;
    for(k = start;k < start+num;k++) {
		error1 = fabs(((m_least.a)*range_data_.xs[k]+(m_least.b)*range_data_.ys[k]+m_least.c))/sqrt((1+(m_least.a)*(m_least.a)));
		
        if(error1 > params_.least_thresh) {
            return false;;
		}
		
        theta = cs_data_.bearings[k];
        if(fabs((fabs(theta) - M_PI/2))<1e-05) {
			m_pn.x = 0;
			m_pn.y = m_least.c;
        } else {
			kp = tan(theta);
			m_pn.x = (m_least.c)/(kp - m_least.a);
			m_pn.y = kp*m_pn.x;
		}
		

        if( k-1 >= start) {
            error2 = distance_point(range_data_.xs[k],range_data_.ys[k],range_data_.xs[k-1],range_data_.ys[k -1]);
        }


        error3 = distance_point(range_data_.xs[k],range_data_.ys[k],m_pn.x,m_pn.y);


        if(error3 > params_.predict_distance ||error2 > params_.max_point_distance ) {
            return false;
		}
	}

    return true;
}


int LineFeature::findline(const int index) {
	line m_temp;
    int index1 = 0;
    int index2 = 0;
	double a = 0;
	double b = 0;
	double c = 0;

	a = m_least.a;
	b = m_least.b;
	c = m_least.c;

    index2 = index + params_.seed_line_points;
	least m_result;
	m_result.a = 0;
	m_result.b = 0;
	m_result.c = 0;	

    while(true) {
        if((fabs(a*range_data_.xs[index2]+b*range_data_.ys[index2]+c)/(sqrt(1+a*a)))<params_.least_thresh) {
            m_least = leastsquare(index,index2,2);
            if(index2 < (point_num_.size() - 1)) {
                index2 = index2 + 1;
				a = m_least.a;
				b = m_least.b;
				c = m_least.c;
                continue;
            }
        }

        break;
	}
    index2 = index2-1;
    index1 = index - 1;
    while(index1 >= 0) {
        if((fabs(a*range_data_.xs[index1]+b*range_data_.ys[index1]+c)/(sqrt(1+a*a))) < params_.least_thresh) {
            m_least = leastsquare(index1,index2,3);
            if( index1 > 0 ) {
                index1 = index1 - 1;
				a = m_least.a;
				b = m_least.b;
				c = m_least.c;
                continue;
            } else {
                break;
            }
        } else {
            break;
        }
	}

    index1 = index1+1;

    m_temp.left = index1;
    m_temp.right = index2;

    m_result = leastsquare(index1,index2,1);
	m_temp.a = m_result.a;
	m_temp.b = m_result.b;
	m_temp.c = m_result.c;

    if( (index2 - index1) > params_.min_line_points) {
        if (delete_short_line(index1,index2)) {
			m_line.push_back(m_temp);
		}
        return index2;
    } else {
        return index;
	}
}

void LineFeature::eraseline() {
    if(m_line.size() < 2) {
		return;
	}

	int m = 0;
	int n = 0;
	int m_iter = 0;
	double error1 = 0;
	double error2 = 0;
	least temp_least;
	temp_least.a = 0;
	temp_least.b = 0;
	temp_least.c = 0;

	double theta_one_ = 0;
	double theta_two_ = 0;
	double theta_d_ = 0;
	std::size_t q = 0,p = 0;
	
    for(p = 0; p < m_line.size() - 1; p++) {
		m = m_line[p].right;
        for(q = p+1;q < m_line.size();q++) {
			n = m_line[q].left;
            if(m >= n) {
				theta_one_ = atan(m_line[p].a);
				theta_two_ = atan(m_line[q].a);
				
				theta_d_ = fabs(theta_one_ - theta_two_);
				
                if(( theta_d_ < 0.1 )||( theta_d_ > (M_PI - 0.1) )) {
                    int _left = std::min(m_line[p].left,m_line[q].left);
				  
				    least m_temp = leastsquare(_left,m_line[q].right,1);
				    
				    m_line[p].a = m_temp.a;
				    m_line[p].b = m_temp.b;
				    m_line[p].c = m_temp.c;
				    
				    m_line[p].left = _left;
				    m_line[p].right = m_line[q].right;
				    
				    m_line.erase(m_line.begin()+q);
				    
				    m = m_line[p].right;
				    q = q - 1;
				}
				
				
			}
		}
	}
	

    for(p = 0; p < (m_line.size() - 1); p++) {
		q = p+1;
		m = m_line[p].right;
		n = m_line[q].left;
        if( m >= n ) {
            for(m_iter = n; m_iter <= m; m_iter++) {
				error1 = fabs(((m_line[p].a)*range_data_.xs[m_iter]+(m_line[p].b)*range_data_.ys[m_iter]+m_line[p].c))/sqrt((1+(m_line[p].a)*(m_line[p].a)));
				error2 = fabs(((m_line[q].a)*range_data_.xs[m_iter]+(m_line[q].b)*range_data_.ys[m_iter]+m_line[q].c))/sqrt((1+(m_line[q].a)*(m_line[q].a)));
                if(error1 > error2) {
					break;
				}
			}
			m_line[p].right = m_iter-1;
			temp_least = leastsquare(m_line[p].left,m_line[p].right,1);
			m_line[p].a = temp_least.a;	
			m_line[p].b = temp_least.b;	
			m_line[p].c = temp_least.c;	

			m_line[q].left = m_iter;
			temp_least = leastsquare(m_line[q].left,m_line[q].right,1);
			m_line[q].a = temp_least.a;	
			m_line[q].b = temp_least.b;	
			m_line[q].c = temp_least.c;	
		}		
	}
}

bool LineFeature::delete_short_line(const int n1,const int n2) {
	
    double dis = distance_point(range_data_.xs[n1],range_data_.ys[n1],range_data_.xs[n2],range_data_.ys[n2]);
    if( dis < params_.min_line_length) {
		return false;
    } else {
		return true;
	}		
}


void LineFeature::endpoint_generation(std::vector<gline>& temp_line) {
	gline line_temp;
	std::vector<gline> output;
    POINTT endpoint1;
    POINTT endpoint2;
	int m = 0,n = 0;
    for(size_t i = 0;i < m_line.size();i++) {
		m = m_line[i].left;
		n = m_line[i].right;

        if(m_line[i].b != 0) {
			endpoint1.x = (range_data_.xs[m]/m_line[i].a + range_data_.ys[m] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
			endpoint1.y = m_line[i].a*endpoint1.x + m_line[i].c;
        } else {
			endpoint1.x = range_data_.ys[m];
			endpoint1.y = m_line[i].c/m_line[i].a;
		}
				
		line_temp.x1 = endpoint1.x;
		line_temp.y1 = endpoint1.y;

		m_line[i].p1 = endpoint1;

        if(m_line[i].b != 0) {
			endpoint2.x = (range_data_.xs[n]/m_line[i].a + range_data_.ys[n] - m_line[i].c)/(m_line[i].a + 1.0/(m_line[i].a));
			endpoint2.y = m_line[i].a*endpoint2.x + m_line[i].c;
        } else {
			endpoint2.x = range_data_.ys[n];
			endpoint2.y = m_line[i].c/m_line[i].a;
		}
				
		line_temp.x2 = endpoint2.x;
		line_temp.y2 = endpoint2.y;

		m_line[i].p2 = endpoint2;

        line_temp.angle = atan2(endpoint2.y - endpoint1.y, endpoint2.x - endpoint1.x);
        line_temp.distance = hypot(endpoint2.x - endpoint1.x,endpoint2.y - endpoint1.y);


		output.push_back(line_temp);
	}
    temp_line = output;
}


void LineFeature::extractLines(std::vector<gline>& line) {
	int line_include = 0;
	m_line.clear();
    point_num_ = cs_data_.indices;

    if(point_num_.size() < params_.min_line_points) {
		return;
	}


    for(unsigned int i = 0; i < (point_num_.size() - params_.min_line_points) ;i++) {
		m_least = leastsquare(i,i + params_.seed_line_points - 1,1);
        if(detectline(i,params_.seed_line_points)) {
            line_include = findline(i);
			i = line_include;
		}
		
	}
    eraseline();
	
    for(size_t p = 0; p < m_line.size();p++) {
        if(!delete_short_line(m_line[p].left,m_line[p].right)) {
		    m_line.erase(m_line.begin()+p);
		}
	}
	
    endpoint_generation(line);
}

}
