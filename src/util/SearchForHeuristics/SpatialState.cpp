#include "util/SearchForHeuristics/SpatialState.h"

namespace RIVERS {

namespace SPATIAL {

bool is_better(const State * s1, const State * s2) {
    if (s1->f == s2->f){
        if (s1->h == s2->h) {
            return rand()%2==0;
        }
        return s1->h < s2->h;
    }
    return s1->f < s2->f; 
}

}

}