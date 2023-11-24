#pragma once
#include <cstdlib>
#include <utility>
#include "boost/heap/pairing_heap.hpp"

namespace Routing {

// we will use it for both high level and low level search
struct State {

    int pos;
    int orient;
    float g;
    float h;
    float f;

    float max_congestion;

    State * prev;
    
    State(int pos, int orient, float g, float h, float max_congestion, State * prev) : pos(pos), orient(orient), g(g), h(h), f(g+h), max_congestion(max_congestion), prev(prev) {}

    void copy(const State * s) {
        pos = s->pos;
        orient = s->orient;
        g = s->g;
        h = s->h;
        f = s->f;
        max_congestion = s->max_congestion;
        prev = s->prev;
    }


    struct Compare {
        bool operator()(const State * s1, const State * s2) const {
            if (s1->f != s2->f){
                if (s1->max_congestion != s2->max_congestion) {
                    return s1->max_congestion > s2->max_congestion;
                }
                return s1->h > s2->h;
            }
            return s1->f > s2->f;
        }
    };

    struct Hash {
        std::size_t operator()(const State * s) const {
            size_t h = std::hash<int>()((s->pos<<2)+(s->orient<<1));
            return h;        
        }
    };

    struct Equal {
        bool operator()(const State * s1, const State * s2) const {
            return s1->pos == s2->pos && s1->orient == s2->orient;
        }
    };

    bool closed=false;
    boost::heap::pairing_heap<State*, boost::heap::compare<State::Compare>>::handle_type open_list_handle;


};

}