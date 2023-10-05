#pragma once
#include <iostream>
#include <random>
#include <cstring>
#include "LaCAM2/SUO/Search/TemporalSpatialState.h"

namespace SUO {

namespace TemporalSpatial {
    
class CostMap {
public:
    int max_x;
    int max_y;
    int max_size;
    int max_t;

    float * data;

    CostMap(int max_x, int max_y, int max_t): max_x(max_x), max_y(max_y), max_t(max_t), max_size(max_x*max_y) {
        data = new float[max_t*max_size];
        clear();
    }

    ~CostMap() {
        delete [] data;
    }

    inline float & operator[](int index) {
        return data[index];
    }

    inline float & operator()(int pos, int t) {
        return data[t*max_size+pos];
    }

    void clear() {
        memset(data, 0, sizeof(float)*max_t*max_size);
    }

    void copy(const CostMap & other) {
        // TODO(rivers): this is a large array, should not be copied often
        if (max_x!=other.max_x || max_y!=other.max_y || max_t!=other.max_t) {
            std::cerr<<"Error: copying cost map with different shape"<<std::endl;
            exit(-1);
        }

        memcpy(data, other.data, sizeof(float)*max_t*max_size);
    }

    void update(const std::vector<std::pair<State,float> > & deltas) {
        for (const auto & delta: deltas) {
            (*this)(delta.first.pos,delta.first.t) += delta.second;
        }
    }

};

}

}