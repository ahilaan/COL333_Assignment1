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
    int x = 0;
    int y = 0;
};

struct Helicopter {
    int city = 0;   // <-- init
    int wcap = 0;
    int dcap = 0;
    int F = 0;
    int alpha = 0;
    int kms = 0;    // <-- init
};

struct Village {
    int x = 0;
    int y = 0;
    int n = 0;
    int food = 0;
    int other = 0;
};

struct Delivery{
    Village* vil = nullptr;          // <-- init
    int x = 0;                       // <-- init
    int y = 0;                       // <-- init
    array<int,3> resources{0,0,0};
};

using Node = variant<City*, Delivery*>;

template<typename NodeT>
struct Trip {
    vector<NodeT> path;  
    int path_cost = 0;  
    int weight = 0;   
    int val = 0;                  // <-- init
};

struct State {
    int state_cost = 0;
    vector<vector<Trip<Node>>> state_table;
    deque<Delivery> delivery_pool;    // <-- stable ownership for Delivery*
};

class Planner {
private:
    int proc_time = 0;  
    int dmax = 0;       
    int wd = 0, vd = 0, wp = 0, vp = 0, wo = 0, vo = 0;

    vector<City> cities;
    vector<Helicopter> helis;
    vector<Village> villages;

public:
    Planner() = default;

    Planner(int proc_time_,
            int dmax_,
            int wd_, int vd_,
            int wp_, int vp_,
            int wo_, int vo_)
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
