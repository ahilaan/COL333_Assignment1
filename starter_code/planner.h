#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <string>
#include <variant>
#include <array>
#include <numeric>
#include <cmath>
#include <chrono>
#include <deque>   
#include <random>
#include <algorithm>

using namespace std;

struct City {
    double x = 0;
    double y = 0;
};

struct Helicopter {
    int city = 0;  
    double wcap = 0;
    double dcap = 0;
    double F = 0;
    double alpha = 0;
    double kms = 0;  
};

struct Village {
    int id=0;
    double x = 0;
    double y = 0;
    int n = 0;
    int food = 0;
    int other = 0;
};

struct Delivery{
    int vil=0;        
    double x = 0;                       
    double y = 0;                      
    array<int,3> resources{0,0,0};
};

using Node = variant<City*, Delivery*>;

template<typename NodeT>
struct Trip {
    vector<NodeT> path;  
    double path_cost = 0;  
    double weight = 0;   
    double val = 0;                
};

struct State {
    double state_cost = 0;
    vector<vector<Trip<Node>>> state_table;
    deque<Delivery> delivery_pool;    
};

class Planner {
private:
    double proc_time = 0;  
    double dmax = 0;       
    double wd = 0, vd = 0, wp = 0, vp = 0, wo = 0, vo = 0;

    vector<City> cities;
    vector<Helicopter> helis;
    vector<Village> villages;

public:
    Planner() = default;

    Planner(double proc_time_,
            double dmax_,
            double wd_, double vd_,
            double wp_, double vp_,
            double wo_, double vo_)
    : proc_time(proc_time_), dmax(dmax_),
      wd(wd_), vd(vd_), wp(wp_), vp(vp_), wo(wo_), vo(vo_) {}

    void add_city(const City& c)        { cities.push_back(c); }
    void add_heli(const Helicopter& h)  { helis.push_back(h); }
    void add_vill(const Village& v)     { villages.push_back(v); }

    const vector<City>& get_cities()      const { return cities; }
    const vector<Helicopter>& get_helis()  const { return helis; }
    const vector<Village>& get_villages()  const { return villages; }

    State compute_allocation();
};

#endif // PLANNER_H
