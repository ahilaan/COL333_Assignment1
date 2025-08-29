#include <fstream>
#include <iostream>
#include <variant>
#include <array>
#include <numeric>
#include <cmath>
#include <chrono>
#include <deque>
#include <random>
#include <algorithm>

using namespace std;

#include "planner.h"

vector<array<int,3>> pkts; // pkts ordered in v/w order desc

// ---------- small helpers ----------
static inline int dist(int x1,int y1,int x2,int y2){
    return (int)std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
static inline pair<int,int> xy(const Node& n){
    if (auto pc = std::get_if<City*>(&n))     return { (*pc)->x, (*pc)->y };
    auto pd = std::get_if<Delivery*>(&n);
    return { (*pd)->x, (*pd)->y };
}

// ---------- packet allocator (mutating real village & trip) ----------
Delivery packet_allocator(Village* v, Helicopter* h, Trip<Node>* t){
    Delivery d;
    d.vil = v;
    d.x = v->x;
    d.y = v->y;

    for (auto arr : pkts){
        int i = arr[0], val = arr[1], wt = arr[2];

        int remaining_heli_cap = h->wcap - t->weight;
        int remaining_vil_cap  = (i <= 1 ? v->food : v->other);

        if (wt > remaining_heli_cap || remaining_vil_cap == 0) continue;

        int num = std::min(remaining_heli_cap / wt, remaining_vil_cap);

        if (i <= 1) v->food  -= num;
        else        v->other -= num;

        t->weight += (num * wt);
        t->val    += (num * val);
        d.resources[i] = num;
    }
    return d;
}

// ---------- LOCAL SEARCH STEP (modifies s IN-PLACE) ----------
tuple<int,int> best_neighbour(State& s,
                              vector<Village>& villages,
                              vector<Helicopter>& helis,
                              vector<City>& cities,
                              int dmax)
{
    int n = (int)s.state_table.size();
    int m = (int)villages.size();

    int mx = 0;
    int bestH = -1, bestT = -1, bestP = -1;
    int bestI = -1;
    bool place_in_existing = true;
    Delivery bestD{}; // local copy; will emplace into pool before storing pointer

    for (int i = 0; i < m; i++){ // candidate village
        const Village& vil3 = villages[i];

        for (int j = 0; j < n; j++){ // choose helicopter
            // try inserting into each existing trip
            for (int k = 0; k < (int)s.state_table[j].size(); k++){
                auto& trip = s.state_table[j][k];

                for (int l = 1; l < (int)trip.path.size(); l++){
                    auto [x1,y1] = xy(trip.path[l]);     // node after insertion
                    auto [x2,y2] = xy(trip.path[l-1]);   // node before insertion
                    int x3 = vil3.x, y3 = vil3.y;        // candidate

                    int d12 = dist(x1,y1,x2,y2);
                    int d23 = dist(x2,y2,x3,y3);
                    int d31 = dist(x3,y3,x1,y1);
                    int delta = d31 + d23 - d12;

                    if (trip.path_cost + delta > helis[j].dcap) continue;
                    if (helis[j].kms + delta > dmax) continue;

                    // evaluate packets without touching real village: use a copy
                    Trip<Node> temp; temp.weight = trip.weight; temp.val = 0;
                    Village vcopy = villages[i];
                    Delivery deli = packet_allocator(&vcopy, &helis[j], &temp);
                    if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

                    int gain = temp.val - delta;
                    if (gain > mx){
                        mx = gain;
                        bestH = j; bestT = k; bestP = l; bestI = i;
                        bestD = deli;
                        place_in_existing = true;
                    }
                }
            }

            // also try starting a fresh trip for heli j
            int round = 2 * dist(cities[helis[j].city].x, cities[helis[j].city].y,
                                 villages[i].x, villages[i].y);
            if (helis[j].kms + round > dmax || round > helis[j].dcap) continue;

            Trip<Node> t; t.weight = 0; t.val = 0;
            Village vcopy = villages[i];
            Delivery deli = packet_allocator(&vcopy, &helis[j], &t);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            int gain = t.val - round;
            if (gain > mx){
                mx = gain;
                bestH = j; bestI = i;
                bestD = deli;
                place_in_existing = false;
            }
        }
    }

    if (mx == 0) return { -1, s.state_cost };

    if (place_in_existing){
        auto& trip = s.state_table[bestH][bestT];

        // recompute delta for chosen position
        auto [x1,y1] = xy(trip.path[bestP]);
        auto [x2,y2] = xy(trip.path[bestP-1]);
        int x3 = bestD.x, y3 = bestD.y;

        int d12 = dist(x1,y1,x2,y2);
        int d23 = dist(x2,y2,x3,y3);
        int d31 = dist(x3,y3,x1,y1);
        int delta = d31 + d23 - d12;

        // OWN the delivery in this State; store pointer into path
        s.delivery_pool.push_back(std::move(bestD));
        Delivery* dptr = &s.delivery_pool.back();

        trip.path.insert(trip.path.begin() + bestP, Node(dptr));
        trip.path_cost += delta;

        // apply real packet allocation (mutate real village & trip)
        Trip<Node> apply_tmp; apply_tmp.weight = trip.weight; apply_tmp.val = 0;
        Delivery applied = packet_allocator(&villages[bestI], &helis[bestH], &apply_tmp);
        (void)applied; // applied resources already applied via allocator

        trip.weight = apply_tmp.weight;
        trip.val   += apply_tmp.val;
        helis[bestH].kms += delta;

        s.state_cost += (apply_tmp.val - delta);
    } else {
        // start a new trip
        Trip<Node> t;
        t.path.push_back(&cities[helis[bestH].city]);

        s.delivery_pool.push_back(std::move(bestD));
        Delivery* dptr = &s.delivery_pool.back();

        t.path.push_back(Node(dptr));
        t.path.push_back(&cities[helis[bestH].city]);

        t.path_cost = 2 * dist(cities[helis[bestH].city].x, cities[helis[bestH].city].y,
                               dptr->x, dptr->y);

        Trip<Node> apply_tmp; apply_tmp.weight = 0; apply_tmp.val = 0;
        Delivery applied = packet_allocator(dptr->vil, &helis[bestH], &apply_tmp);
        (void)applied;

        t.weight = apply_tmp.weight;
        t.val    = apply_tmp.val;

        helis[bestH].kms += t.path_cost;
        s.state_cost += (t.val - t.path_cost);

        s.state_table[bestH].push_back(std::move(t));
    }

    return { 0, s.state_cost };
}

// ---------- RANDOM RESTART ----------
State random_restart(vector<Village>& villages,
                     vector<Helicopter>& helis,
                     vector<City>& cities,
                     int dmax)
{
    State s;
    s.state_cost = 0;
    s.state_table.clear();
    s.state_table.resize(helis.size());

    int n = (int)villages.size();
    vector<int> perm(n);
    iota(perm.begin(), perm.end(), 0);

    static mt19937 rng(std::random_device{}());
    shuffle(perm.begin(), perm.end(), rng);

    for (int i : perm){
        uniform_int_distribution<int> pick_heli(0, (int)helis.size()-1);
        int h = pick_heli(rng);
        cout<<h<<endl;
        if (s.state_table[h].empty()){
            cout<<"hi"<<endl;
            int round = 2 * dist(cities[helis[h].city].x, cities[helis[h].city].y,
                                  villages[i].x, villages[i].y);
            if (helis[h].kms + round > dmax || round > helis[h].dcap) continue;

            Trip<Node> t; t.weight = 0; t.val = 0;
            Delivery deli = packet_allocator(&villages[i], &helis[h], &t);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            // own delivery and store pointer
            s.delivery_pool.push_back(std::move(deli));
            Delivery* dptr = &s.delivery_pool.back();

            t.path.push_back(&cities[helis[h].city]);
            t.path.push_back(Node(dptr));
            t.path.push_back(&cities[helis[h].city]);
            t.path_cost = round;

            helis[h].kms += round;

            s.state_table[h].push_back(std::move(t));
            continue;
        }

        // append into last trip if feasible
        Trip<Node>& t = s.state_table[h].back();

        Node lastNode = t.path.back();
        auto [cx,cy] = xy(lastNode); // city at end
        Node prevNode = t.path[t.path.size()-2];
        auto [vx,vy] = xy(prevNode); // last delivery

        int d12 = dist(cx,cy,vx,vy);
        int d23 = dist(villages[i].x, villages[i].y, vx, vy);
        int d31 = dist(villages[i].x, villages[i].y, cx, cy);
        int delta = d31 + d23 - d12;

        if (!((t.path_cost + delta > helis[h].dcap) || (helis[h].kms + delta > dmax))){
            Delivery deli = packet_allocator(&villages[i], &helis[h], &t);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            // move last city, insert delivery*, put city back
            Node last = t.path.back();
            t.path.pop_back();

            s.delivery_pool.push_back(std::move(deli));
            Delivery* dptr = &s.delivery_pool.back();

            t.path.push_back(Node(dptr));
            t.path.push_back(last);

            helis[h].kms += delta;
            t.path_cost  += delta;
        } else {
            int round = 2 * dist(cities[helis[h].city].x, cities[helis[h].city].y,
                                  villages[i].x, villages[i].y);
            if (helis[h].kms + round > dmax || round > helis[h].dcap) continue;

            Trip<Node> t1; t1.weight = 0; t1.val = 0;
            Delivery deli = packet_allocator(&villages[i], &helis[h], &t1);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            s.delivery_pool.push_back(std::move(deli));
            Delivery* dptr = &s.delivery_pool.back();

            t1.path.push_back(&cities[helis[h].city]);
            t1.path.push_back(Node(dptr));
            t1.path.push_back(&cities[helis[h].city]);
            t1.path_cost = round;

            helis[h].kms += round;
            s.state_table[h].push_back(std::move(t1));
        }
    }
    return s;
}

// ---------- MAIN LOOP ----------
State Planner::compute_allocation(){
    // order pkts by value/weight ratio
    if (vd/(double)wd > vp/(double)wp && vd/(double)wd > vo/(double)wo){
        pkts.push_back({0,vd,wd});
        if (vp/(double)wp > vo/(double)wo){
            pkts.push_back({1,vp,wp});
            pkts.push_back({2,vo,wo});
        } else {
            pkts.push_back({2,vo,wo});
            pkts.push_back({1,vp,wp});
        }
    } else if (vp/(double)wp > vd/(double)wd && vp/(double)wp > vo/(double)wo){
        pkts.push_back({1,vp,wp});
        if (vd/(double)wd > vo/(double)wo){
            pkts.push_back({0,vd,wd});
            pkts.push_back({2,vo,wo});
        } else {
            pkts.push_back({2,vo,wo});
            pkts.push_back({0,vd,wd});
        }
    } else {
        pkts.push_back({2,vo,wo});
        if (vd/(double)wd > vp/(double)wp){
            pkts.push_back({0,vd,wd});
            pkts.push_back({1,vp,wp});
        } else {
            pkts.push_back({1,vp,wp});
            pkts.push_back({0,vd,wd});
        }
    }

    using namespace std::chrono;
    auto start = high_resolution_clock::now();

    State s = random_restart(villages, helis, cities, dmax);
    cout<<s.state_cost<<endl;
    while (true){
        auto [flag, val] = best_neighbour(s, villages, helis, cities, dmax);
        if (flag == -1){
            s = random_restart(villages, helis, cities, dmax);
        }
        auto now = high_resolution_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - start).count();
        // NOTE: if your proc_time is already milliseconds, keep as-is.
        // If it's minutes per the PDF, use: proc_time * 60'000LL
        if (elapsed >= (long long)proc_time) break;
    }
    return s;
}
