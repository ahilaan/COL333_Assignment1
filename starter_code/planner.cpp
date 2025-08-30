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
#include <tuple>

using namespace std;

#include "planner.h"

#include <unordered_map>

// ======================== cloning helper (unchanged) ========================
static State clone_state_rebind(const State& s) {
    State o;
    o.state_cost = s.state_cost;
    o.state_table.resize(s.state_table.size());
    o.delivery_pool = s.delivery_pool;

    std::unordered_map<const Delivery*, Delivery*> mp;
    mp.reserve(o.delivery_pool.size());
    for (size_t i = 0; i < o.delivery_pool.size(); i++) {
        mp[ &s.delivery_pool[i] ] = &o.delivery_pool[i];
    }

    for (size_t h = 0; h < s.state_table.size(); h++) {
        for (const auto& trip : s.state_table[h]) {
            Trip<Node> nt;
            nt.path_cost = trip.path_cost;
            nt.weight    = trip.weight;
            nt.val       = trip.val;
            nt.path.reserve(trip.path.size());

            for (const auto& node : trip.path) {
                if (auto pc = std::get_if<City*>(&node)) {
                    nt.path.push_back(*pc); 
                } else if (auto pd = std::get_if<Delivery*>(&node)) {
                    nt.path.push_back(mp[*pd]); 
                }
            }
            o.state_table[h].push_back(std::move(nt));
        }
    }
    return o;
}

vector<tuple<int,double,double>> pkts; // (index, value, weight) ordered in v/w order desc

// ---------- small helpers ----------
static inline double dist(double x1,double y1,double x2,double y2){
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
static inline pair<double,double> xy(const Node& n){
    if (auto pc = std::get_if<City*>(&n))     return { (*pc)->x, (*pc)->y };
    auto pd = std::get_if<Delivery*>(&n);
    return { (*pd)->x, (*pd)->y };
}

// ---------- packet allocator (mutating real village & trip) ----------
Delivery packet_allocator(Village* v, Helicopter* h, Trip<Node>* t) {
    Delivery d;
    d.vil = v;
    d.x = v->x;
    d.y = v->y;

    for (auto [i, val, wt] : pkts) {
        double remaining_heli_cap = h->wcap - t->weight;
        int remaining_vil_cap = (i <= 1 ? v->food : v->other);

        if (wt > remaining_heli_cap || remaining_vil_cap == 0) 
            continue;

        int num = min((int)floor(remaining_heli_cap / wt), remaining_vil_cap);

        if (i <= 1)
            v->food -= num;
        else
            v->other -= num;

        t->weight += (num * wt);
        t->val    += (num * val);
        d.resources[i] = num;
    }

    return d;
}

// ---------- LOCAL SEARCH STEP (greedy; unchanged) ----------
tuple<int,double> best_neighbour(State& s,
                              vector<Village>& villages,
                              vector<Helicopter>& helis,
                              vector<City>& cities,
                              double dmax)
{
    int n = (int)s.state_table.size();
    int m = (int)villages.size();

    double mx = 0;
    int bestH = -1, bestT = -1, bestP = -1;
    int bestI = -1;
    bool place_in_existing = true;
    Delivery bestD{}; 

    for (int i = 0; i < m; i++){ // candidate village
        const Village& vil3 = villages[i];

        for (int j = 0; j < n; j++){ // choose helicopter
            // try inserting into each existing trip
            for (int k = 0; k < (int)s.state_table[j].size(); k++){
                auto& trip = s.state_table[j][k];

                for (int l = 1; l < (int)trip.path.size(); l++){
                    auto [x1,y1] = xy(trip.path[l]);    
                    auto [x2,y2] = xy(trip.path[l-1]);   
                    double x3 = vil3.x, y3 = vil3.y;      

                    double d12 = dist(x1,y1,x2,y2);
                    double d23 = dist(x2,y2,x3,y3);
                    double d31 = dist(x3,y3,x1,y1);
                    double delta = d31 + d23 - d12;

                    if (trip.path_cost + delta > helis[j].dcap) continue;
                    if (helis[j].kms + delta > dmax) continue;

                    Trip<Node> temp; temp.weight = trip.weight; temp.val = 0;
                    Village vcopy = villages[i];
                    Delivery deli = packet_allocator(&vcopy, &helis[j], &temp);
                    if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

                    double gain = temp.val - helis[j].alpha*delta;
                    if (gain > mx){
                        mx = gain;
                        bestH = j; bestT = k; bestP = l; bestI = i;
                        deli.vil = &villages[i];                 
                        bestD = std::move(deli);
                        place_in_existing = true;
                    }
                }
            }

            // also try starting a fresh trip
            double round = 2 * dist(cities[helis[j].city].x, cities[helis[j].city].y,
                                 villages[i].x, villages[i].y);
            if (helis[j].kms + round > dmax || round > helis[j].dcap) continue;

            Trip<Node> t; t.weight = 0; t.val = 0;
            Village vcopy = villages[i];
            Delivery deli = packet_allocator(&vcopy, &helis[j], &t);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            double gain = t.val - (helis[j].F+helis[j].alpha*round);
            if (gain > mx){
                mx = gain;
                bestH = j; bestI = i;
                deli.vil = &villages[i];               
                bestD = std::move(deli);
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
        double x3 = bestD.x, y3 = bestD.y;

        double d12 = dist(x1,y1,x2,y2);
        double d23 = dist(x2,y2,x3,y3);
        double d31 = dist(x3,y3,x1,y1);
        double delta = d31 + d23 - d12;

        s.delivery_pool.push_back(std::move(bestD));
        Delivery* dptr = &s.delivery_pool.back();

        trip.path.insert(trip.path.begin() + bestP, Node(dptr));
        trip.path_cost += delta;

        Trip<Node> apply_tmp; apply_tmp.weight = trip.weight; apply_tmp.val = 0;
        Delivery applied = packet_allocator(&villages[bestI], &helis[bestH], &apply_tmp);
        (void)applied; 

        trip.weight = apply_tmp.weight;
        trip.val   += apply_tmp.val;
        helis[bestH].kms += delta;

        s.state_cost += (apply_tmp.val - helis[bestH].alpha*delta);
    } else {
        // start a new trip
        Trip<Node> t;
        t.path.push_back(Node(&cities[helis[bestH].city]));

        s.delivery_pool.push_back(std::move(bestD));
        Delivery* dptr = &s.delivery_pool.back();

        t.path.push_back(Node(dptr));
        t.path.push_back(Node(&cities[helis[bestH].city]));

        t.path_cost = 2 * dist(cities[helis[bestH].city].x, cities[helis[bestH].city].y,
                               dptr->x, dptr->y);

        Trip<Node> apply_tmp; apply_tmp.weight = 0; apply_tmp.val = 0;
        Delivery applied = packet_allocator(dptr->vil, &helis[bestH], &apply_tmp);
        (void)applied;

        t.weight = apply_tmp.weight;
        t.val    = apply_tmp.val;

        helis[bestH].kms += t.path_cost;
        s.state_cost += (t.val - (helis[bestH].F + helis[bestH].alpha * t.path_cost));

        s.state_table[bestH].push_back(std::move(t));
    }

    return { 0, s.state_cost };
}

// ---------- RANDOM RESTART (unchanged) ----------
State random_restart(vector<Village>& villages,
                     vector<Helicopter>& helis,
                     vector<City>& cities,
                     double dmax)
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
        if (s.state_table[h].empty()){
            double round = 2 * dist(cities[helis[h].city].x, cities[helis[h].city].y,
                                  villages[i].x, villages[i].y);
            if (helis[h].kms + round > dmax || round > helis[h].dcap) continue;

            Trip<Node> t; t.weight = 0; t.val = 0;
            Delivery deli = packet_allocator(&villages[i], &helis[h], &t);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            // own delivery and store pointer
            s.delivery_pool.push_back(std::move(deli));
            Delivery* dptr = &s.delivery_pool.back();

            t.path.push_back(Node(&cities[helis[h].city]));
            t.path.push_back(Node(dptr));
            t.path.push_back(Node(&cities[helis[h].city]));
            t.path_cost = round;

            helis[h].kms += round;

            s.state_table[h].push_back(std::move(t));
            s.state_cost += (s.state_table[h].back().val
                            - (helis[h].F + helis[h].alpha * s.state_table[h].back().path_cost));
            continue;
        }

        // append into last trip if feasible
        Trip<Node>& t = s.state_table[h].back();

        Node lastNode = t.path.back();
        auto [cx,cy] = xy(lastNode); // city at end
        Node prevNode = t.path[t.path.size()-2];
        auto [vx,vy] = xy(prevNode); // last delivery

        double d12 = dist(cx,cy,vx,vy);
        double d23 = dist(villages[i].x, villages[i].y, vx, vy);
        double d31 = dist(villages[i].x, villages[i].y, cx, cy);
        double delta = d31 + d23 - d12;

        if (!((t.path_cost + delta > helis[h].dcap) || (helis[h].kms + delta > dmax))){
            double old_val=t.val;
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
            double gain=t.val-old_val;
            s.state_cost += (gain - helis[h].alpha * delta);

        } else {
            double round = 2 * dist(cities[helis[h].city].x, cities[i].x, cities[helis[h].city].y,
                                  villages[i].y);
            round = 2 * dist(cities[helis[h].city].x, cities[helis[h].city].y,
                             villages[i].x, villages[i].y); // fix accidental line above if any merge

            if (helis[h].kms + round > dmax || round > helis[h].dcap) continue;

            Trip<Node> t1; t1.weight = 0; t1.val = 0;
            Delivery deli = packet_allocator(&villages[i], &helis[h], &t1);
            if (deli.resources[0]==0 && deli.resources[1]==0 && deli.resources[2]==0) continue;

            s.delivery_pool.push_back(std::move(deli));
            Delivery* dptr = &s.delivery_pool.back();

            t1.path.push_back(Node(&cities[helis[h].city]));
            t1.path.push_back(Node(dptr));
            t1.path.push_back(Node(&cities[helis[h].city]));
            t1.path_cost = round;

            helis[h].kms += round;
            s.state_table[h].push_back(std::move(t1));
            const auto& last = s.state_table[h].back();
            s.state_cost += (last.val - (helis[h].F + helis[h].alpha * last.path_cost));
        }
    }
    return s;
}

// ====================== SA: random insertion proposal =======================
// Tries ONE random feasible insertion (into existing path or as new trip).
// On success, it *applies* the change and returns {true, delta_objective}.
// The caller must snapshot and restore (s, villages, helis) if it decides to reject.
static bool sa_random_insertion(State& s,
                                vector<Village>& villages,
                                vector<Helicopter>& helis,
                                vector<City>& cities,     // made non-const
                                double dmax,
                                mt19937& rng,
                                double& delta_obj_out)
{
    if (villages.empty() || helis.empty()) return false;

    uniform_int_distribution<int> pick_vil(0, (int)villages.size()-1);
    uniform_int_distribution<int> pick_heli(0, (int)helis.size()-1);
    uniform_real_distribution<double> coin(0.0,1.0);

    const int tries = 20; // a few shots to find a feasible move
    for (int attempt = 0; attempt < tries; ++attempt) {
        int vi = pick_vil(rng);
        int hi = pick_heli(rng);

        // decide: insert into existing vs start new
        bool can_new = true;
        bool can_existing = !s.state_table[hi].empty();

        bool choose_existing = can_existing && (coin(rng) < 0.7); // bias to existing
        if (!can_existing && !can_new) continue;
        if (!can_existing) choose_existing = false;

        if (choose_existing) {
            // pick random trip & position
            uniform_int_distribution<int> pick_trip(0, (int)s.state_table[hi].size()-1);
            int ti = pick_trip(rng);
            auto& trip = s.state_table[hi][ti];
            if ((int)trip.path.size() < 2) continue;

            uniform_int_distribution<int> pick_pos(1, (int)trip.path.size()-1);
            int pos = pick_pos(rng);

            // compute incremental distance
            auto [x1,y1] = xy(trip.path[pos]);
            auto [x2,y2] = xy(trip.path[pos-1]);
            double x3 = villages[vi].x, y3 = villages[vi].y;

            double d12 = dist(x1,y1,x2,y2);
            double d23 = dist(x2,y2,x3,y3);
            double d31 = dist(x3,y3,x1,y1);
            double delta_km = d31 + d23 - d12;

            if (trip.path_cost + delta_km > helis[hi].dcap) continue;
            if (helis[hi].kms + delta_km > dmax) continue;

            // simulate pack on a copy to check if we deliver anything
            Trip<Node> temp; temp.weight = trip.weight; temp.val = 0;
            Village vcopy = villages[vi];
            Delivery deli_sim = packet_allocator(&vcopy, &helis[hi], &temp);
            if (deli_sim.resources[0]==0 && deli_sim.resources[1]==0 && deli_sim.resources[2]==0) continue;

            // Apply to REAL structures
            Delivery real_deli = deli_sim; // will overwrite vil pointer just below
            real_deli.vil = &villages[vi];

            s.delivery_pool.push_back(std::move(real_deli));
            Delivery* dptr = &s.delivery_pool.back();

            trip.path.insert(trip.path.begin() + pos, Node(dptr));
            trip.path_cost += delta_km;

            Trip<Node> apply_tmp; apply_tmp.weight = trip.weight; apply_tmp.val = 0;
            Delivery applied = packet_allocator(&villages[vi], &helis[hi], &apply_tmp);
            (void)applied;

            double gain_val = apply_tmp.val;
            trip.weight = apply_tmp.weight;
            trip.val   += apply_tmp.val;
            helis[hi].kms += delta_km;

            double delta_obj = gain_val - helis[hi].alpha * delta_km;
            s.state_cost += delta_obj;

            delta_obj_out = delta_obj;
            return true;
        } else {
            // start a new trip
            double round = 2 * dist(cities[helis[hi].city].x, cities[helis[hi].city].y,
                                    villages[vi].x, villages[vi].y);
            if (helis[hi].kms + round > dmax || round > helis[hi].dcap) continue;

            Trip<Node> t; t.weight = 0; t.val = 0;
            Village vcopy = villages[vi];
            Delivery deli_sim = packet_allocator(&vcopy, &helis[hi], &t);
            if (deli_sim.resources[0]==0 && deli_sim.resources[1]==0 && deli_sim.resources[2]==0) continue;

            // Apply to REAL structures
            Delivery real_deli = deli_sim;
            real_deli.vil = &villages[vi];

            Trip<Node> new_trip;
            new_trip.path.push_back(Node(&cities[helis[hi].city]));

            s.delivery_pool.push_back(std::move(real_deli));
            Delivery* dptr = &s.delivery_pool.back();

            new_trip.path.push_back(Node(dptr));
            new_trip.path.push_back(Node(&cities[helis[hi].city]));
            new_trip.path_cost = round;

            Trip<Node> apply_tmp; apply_tmp.weight = 0; apply_tmp.val = 0;
            Delivery applied = packet_allocator(dptr->vil, &helis[hi], &apply_tmp);
            (void)applied;

            new_trip.weight = apply_tmp.weight;
            new_trip.val    = apply_tmp.val;

            helis[hi].kms += round;
            s.state_table[hi].push_back(std::move(new_trip));

            double delta_obj = apply_tmp.val - (helis[hi].F + helis[hi].alpha * round);
            s.state_cost += delta_obj;

            delta_obj_out = delta_obj;
            return true;
        }
    }
    return false; // no feasible move found
}

// ====================== MAIN LOOP with Simulated Annealing ==================
State Planner::compute_allocation(){
    // packet types in value/weight priority (as before)
    pkts.clear();
    pkts.emplace_back(1,vp,wp);
    pkts.emplace_back(0,vd,wd);
    pkts.emplace_back(2,vo,wo);

    using namespace std::chrono;
    static mt19937 rng(std::random_device{}());
    uniform_real_distribution<double> U(0.0,1.0);

    const long long total_ms = (long long)proc_time * 60000LL;
    auto start = high_resolution_clock::now();

    // --- initialize from a random restart to start SA at a reasonable point
    // Also reset per-village inventory and heli kms to "fresh day" before seeding.
    for(auto &v:villages){
        v.food  = 9*(v.n);
        v.other = v.n;
    }
    for(auto &h:helis){
        h.kms = 0;
    }
    State s = random_restart(villages, helis, cities, dmax);

    State opt = clone_state_rebind(s);
    double mx  = s.state_cost;

    // --- temperature schedule (exponential cooling, data-driven T0)
    // We adapt T0 online from the running mean |delta| during the first few dozen accepted proposals.
    double T0 = 0.0;
    double Tend = 0.0;
    int    t0_samples = 0;
    double mean_abs_delta = 0.0;

    // Small helper to compute current temperature based on elapsed time
    auto temperature = [&](long long elapsed_ms) -> double {
        if (T0 <= 0.0) return 1.0; // until we have samples
        double frac = std::max(0.0, std::min(1.0, (double)elapsed_ms / (double)total_ms));
        // Exponential from T0 down to Tend
        double T = T0 * std::pow(Tend / T0, frac);
        return std::max(T, Tend);
    };

    // We will interleave occasional greedy improvement to intensify search.
    const int GREEDY_PERIOD = 250;   // every N SA proposals, run greedy climb once
    int sa_steps = 0;

    // --- main time-bounded loop
    while (true) {
        auto now = high_resolution_clock::now();
        long long elapsed = duration_cast<milliseconds>(now - start).count();
        if (elapsed >= total_ms) break;

        // Occasionally try a greedy hop upward (keeps your old operator in play)
        if (sa_steps % GREEDY_PERIOD == 0) {
            auto [flag, val] = best_neighbour(s, villages, helis, cities, dmax);
            if (val > mx) { mx = val; opt = clone_state_rebind(s); }
            if (flag == -1) {
                // refresh inventories and kms and reseed
                for(auto &v:villages){ v.food = 9*(v.n); v.other = v.n; }
                for(auto &h:helis){ h.kms = 0; }
                s = random_restart(villages, helis, cities, dmax);
                if (s.state_cost > mx) { mx = s.state_cost; opt = clone_state_rebind(s); }
            }
        }

        // SA move: snapshot current state + resources so we can rollback if we reject
        State s_backup = clone_state_rebind(s);
        auto villages_backup = villages;
        auto helis_backup = helis;

        double delta = 0.0;
        bool moved = sa_random_insertion(s, villages, helis, cities, dmax, rng, delta);

        if (!moved) {
            // If we couldn't find a feasible random move after a few tries, random-restart
            for(auto &v:villages){ v.food = 9*(v.n); v.other = v.n; }
            for(auto &h:helis){ h.kms = 0; }
            s = random_restart(villages, helis, cities, dmax);
            if (s.state_cost > mx) { mx = s.state_cost; opt = clone_state_rebind(s); }
            ++sa_steps;
            continue;
        }

        // Set/refresh temperature parameters based on observed deltas
        double absd = std::abs(delta);
        if (absd > 0.0) {
            // incremental mean of |delta|
            ++t0_samples;
            mean_abs_delta += (absd - mean_abs_delta) / (double)t0_samples;
            // Once we have some stats, set T0 and Tend once (or keep them gentle-updating)
            if (t0_samples == 20) {
                T0   = std::max(1.0, 3.0 * mean_abs_delta);    // hot enough to accept many early
                Tend = std::max(1e-3, 0.01 * T0);              // cool endpoint
            }
        }

        // Decide acceptance
        double T = temperature(elapsed);
        bool accept = (delta >= 0.0);
        if (!accept) {
            double prob = std::exp(delta / std::max(T, 1e-9)); // delta<0
            static uniform_real_distribution<double> U01(0.0,1.0);
            if (U01(rng) < prob) accept = true;
        }

        if (!accept) {
            // rollback everything
            s = std::move(s_backup);
            villages = std::move(villages_backup);
            helis = std::move(helis_backup);
        } else {
            // keep, and update best
            if (s.state_cost > mx) {
                mx = s.state_cost;
                opt = clone_state_rebind(s);
            }
        }

        ++sa_steps;
    }

    return opt;
}
