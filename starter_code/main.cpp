#include <iostream>
#include <fstream>
#include <string>
#include <variant>
#include <array>
#include <numeric>
#include <cmath>
#include <chrono>
#include <random>
#include <algorithm>

#include "planner.h"

using namespace std;

int main(int argc, char** argv) {
    if (argc < 3) {
        cout << "Missing arguments\n";
        cout << "Usage:\n";
        cout << "./main <input_filename> <output_filename>\n";
        return 1;
    }
    string inputfilename(argv[1]);
    string outputfilename(argv[2]);

    ifstream ipfile(inputfilename);
    if (!ipfile) {
        cout << "No such file\n";
        return 1;
    }

    double proc_time, dmax;
    ipfile >> proc_time >> dmax;

    double wd, vd, wp, vp, wo, vo;
    ipfile >> wd >> vd >> wp >> vp >> wo >> vo;

    Planner pl(proc_time, dmax, wd, vd, wp, vp, wo, vo);

    int ncities;
    ipfile >> ncities;
    for (int i = 0; i < ncities; ++i) {
        City c;
        ipfile >> c.x >> c.y;
        pl.add_city(c);
    }

    int nvillage;
    ipfile >> nvillage;
    for (int i = 0; i < nvillage; ++i) {
        Village v;
        ipfile >> v.x >> v.y >> v.n;
        v.food  = 9 * v.n;
        v.other = v.n;
        v.id=i;
        pl.add_vill(v);
    }

    int nheli;
    ipfile >> nheli;
    for (int i = 0; i < nheli; ++i) {
        Helicopter h;
        int city_id;                         
        ipfile >> city_id >> h.wcap >> h.dcap >> h.F >> h.alpha;
        h.city = city_id - 1;               
        h.kms  = 0;                      
        pl.add_heli(h);
    }

    State opt = pl.compute_allocation();   

    for (int i = 0; i < (int)opt.state_table.size(); ++i) {
        const auto& trips = opt.state_table[i];
        cout << (i + 1) << " " << trips.size() << "\n";

        for (const auto& trip : trips) {
            // print pickup numbers (sum of resources on this trip)
            int total_d=0, total_p=0, total_o=0;
            for (auto& node : trip.path) {
                if (auto pd = get_if<Delivery*>(&node)) {
                    total_d += (*pd)->resources[0];
                    total_p += (*pd)->resources[1];
                    total_o += (*pd)->resources[2];
                }
            }
            cout << total_d << " " << total_p << " " << total_o;

            // count villages visited
            int vcount = 0;
            for (auto& node : trip.path) 
                if (holds_alternative<Delivery*>(node)) vcount++;
            cout << " " << vcount;

            // print village id + drops
            for (auto& node : trip.path) {
                if (auto pd = get_if<Delivery*>(&node)) {
                    int vid = (*pd)->vil;
                    cout << " " << (vid+1) << " "
                        << (*pd)->resources[0] << " "
                        << (*pd)->resources[1] << " "
                        << (*pd)->resources[2];
                }
            }
            cout << "\n";
        }
        cout << "-1\n";
    }
    cout<<opt.state_cost<<endl;

    return 0;
}
