#pragma once
#include "LaCAM2/SUO/Search/SpatialState.h"
#include <iostream>
#include <random>
#include <cstring>

namespace SUO {

namespace Spatial {
    
class CostMap {
public:
    int max_x;
    int max_y;

    float * data;

    CostMap(int max_x, int max_y): max_x(max_x), max_y(max_y) {
        data = new float[max_x*max_y];
        clear();
    }

    ~CostMap() {
        delete [] data;
    }

    inline float & operator[](int index) {
        return data[index];
    }

    void clear() {
        memset(data, 0, sizeof(float)*max_x*max_y);
    }

    void copy(const CostMap & other) {
        // TODO(rivers): this is a large array, should not be copied often
        if (max_x!=other.max_x || max_y!=other.max_y) {
            std::cerr<<"Error: copying cost map with different shape"<<std::endl;
            exit(-1);
        }

        memcpy(data, other.data, sizeof(float)*max_x*max_y);
    }

    void update(const std::vector<std::pair<int,float> > & deltas) {
        for (const auto & delta: deltas) {
            int index = delta.first;
            float cost = delta.second;
            data[index] += cost;
        }
    }

};

class OpenList {
public:
    State ** heap;

    inline bool is_better(const State * s1, const State * s2) {
        if (s1->f == s2->f){
            // if (s1->h == s2->h) {
                // the random here cause slower?
                // return rng()%2==0;
            // }
            return s1->h < s2->h;
        }
        return s1->f < s2->f; 
    }

    int max_size;
    int size;
    std::mt19937 rng;

    OpenList(int _max_size): max_size(_max_size), size(0),rng(0) {
        heap = new State * [max_size];
    };

    ~OpenList(){
        delete [] heap;
    };

    inline void swap(State *s, State * p) {
        int i = s->heap_index;
        int j = p->heap_index;
        heap[i] = p;
        heap[j] = s;
        s->heap_index = j;
        p->heap_index = i;
    }

    inline void move_up(State *s) {
        int heap_index = s->heap_index;
        while (heap_index>0) {
            int parent_heap_index = (heap_index-1)/2;
            if (is_better(heap[parent_heap_index], heap[heap_index])) {
                break;
            }
            swap(s, heap[parent_heap_index]);
            heap_index = parent_heap_index;
        }
    }

    inline void move_down(State * s) {
        int heap_index = s->heap_index;
        while (heap_index*2+1<size) {
            int child_heap_index = heap_index*2+1;
            if (child_heap_index+1<size && is_better(heap[child_heap_index+1], heap[child_heap_index])) {
                ++child_heap_index;
            }
            if (is_better(heap[heap_index], heap[child_heap_index])) {
                break;
            }
            swap(s, heap[child_heap_index]);
            heap_index = child_heap_index;
        }

    }

    void push(State * s) {
        // if (full()) {
        //     std::cerr<<"the heap is full!"<<std::endl;
        //     exit(-1);
        // }

        heap[size] = s;
        s->heap_index = size;
        ++size;
        move_up(s);
    }

    State * pop() {
        // if (empty()) {
        //     std::cerr<<"the heap is empty!"<<std::endl;
        //     exit(-1);
        // }

        State * ret = heap[0];
        --size;
        heap[0] = heap[size];
        heap[0]->heap_index = 0;
        move_down(heap[0]);

        return ret;
    }

    State * top() {
        return heap[0];
    }

    void increase(State * s) {
        move_up(s);
    }   

    inline bool empty() {
        return size==0;
    }

    inline bool full() {
        return size==max_size;
    }

    void clear() {
        size = 0;
    }

};

}

}