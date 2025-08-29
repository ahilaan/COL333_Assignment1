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

    int proc_time, dmax;
    ipfile >> proc_time >> dmax;

    int wd, vd, wp, vp, wo, vo;
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
        pl.add_vill(v);
    }

    int nheli;
    ipfile >> nheli;
    for (int i = 0; i < nheli; ++i) {
        Helicopter h;
        int city_id;                         // <-- added
        ipfile >> city_id >> h.wcap >> h.dcap >> h.F >> h.alpha;
        h.city = city_id - 1;                // <-- assume input is 1-based; drop -1 if 0-based
        h.kms  = 0;                          // <-- ensure initialized
        pl.add_heli(h);
    }

    State opt = pl.compute_allocation();     // <-- State, not vector

    /*ofstream outfile(outputfilename);
    if (!outfile) {
        cout << "Cannot open output file\n";
        return 1;
    }*/

    // --- minimal output: per-heli trip count, then each trip’s deliveries ---
    // Format example:
    // <heli_index> <num_trips>
    // <num_deliveries_in_trip_1>
    // x y d p o
    // ...
    // -1
    // <num_deliveries_in_trip_2>
    // ...
    cout<<"lol"<<endl;
    for (int i = 0; i < (int)opt.state_table.size(); ++i) {
        const auto& trips = opt.state_table[i];
        cout << (i + 1) << " " << trips.size() << "\n";

        for (const auto& trip : trips) {
            // count deliveries in path (skip cities)
            int dcount = 0;
            for (const auto& node : trip.path) {
                if (holds_alternative<Delivery*>(node)) ++dcount;
            }
            cout << dcount << "\n";

            for (const auto& node : trip.path) {
                if (auto pd = get_if<Delivery*>(&node)) {
                    Delivery* d = *pd;
                    // Print village coordinates and resources (order: d, p, o as per pkts indices 0,1,2)
                    cout << d->x << " " << d->y << " "
                            << d->resources[0] << " "
                            << d->resources[1] << " "
                            << d->resources[2] << "\n";
                }
            }
            cout << -1 << "\n"; // trip terminator
        }
    }

    // If your checker expects only trips (no trailing text), you can remove this line:
    // outfile << "OK\n";

    return 0;
}
