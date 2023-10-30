#pragma once
#include <cstdlib>
#include <utility>
#include "boost/heap/pairing_heap.hpp"
#include "LaCAM2/SUO/Search/Constants.h"

namespace SUO {

namespace TemporalSpatial {

struct State {
    int pos;
    int orient;
    int t;
    int g;
    int h;
    int f;
    State * prev;

    State(): pos(-1), orient(-1), t(-1), g(-1), h(0), f(-1), prev(nullptr), closed(false) {};
    // State(int pos): pos(pos), orient(-1), t(-1), g(-1), h(0), f(-1), prev(nullptr), closed(false) {};
    // State(int pos, int t): pos(pos), orient(-1), t(t), g(-1), h(0), f(-1), prev(nullptr), closed(false) {};
    State(int pos, int orient, int t): pos(pos), orient(orient), t(t), g(-1), h(0), f(-1), prev(nullptr), closed(false) {};
    // State(int pos, int t, int g, int h, State * prev): pos(pos), orient(-1), t(t), g(g), h(h), f(g+h), prev(prev), closed(false) {};
    State(int pos, int orient, int t, int g, int h, State * prev): pos(pos), orient(orient), t(t), g(g), h(h), f(g+h), prev(prev), closed(false) {};

    void copy(const State * s) {
        pos = s->pos;
        orient = s->orient;
        t = s->t;
        g = s->g;
        h = s->h;
        f = s->f;
        prev = s->prev;
    }
    struct StateCompare {
        bool operator()(const State * s1, const State * s2) const {
            if (s1->f == s2->f){
                // if (s1->h == s2->h) {
                //     return rand()%2==0;
                // }
                return s1->h > s2->h;
            }
            return s1->f > s2->f;
        }
    };

    struct StateHash {
        std::size_t operator()(const State * s) const {
            // TODO(rivers): we may replace the hash here!
            size_t h = std::hash<int>()(s->t*MAX_STATE+s->pos*MAX_ORIENT+s->orient);
            return h;
        }
    };

    struct StateEqual {
        bool operator()(const State * s1, const State * s2) const {
            return s1->pos == s2->pos && s1->orient==s2->orient && s1->t==s2->t;
        }
    };

    bool closed;
    boost::heap::pairing_heap<State*, boost::heap::compare<State::StateCompare> >::handle_type open_list_handle;
    // int heap_index;

};

}

}