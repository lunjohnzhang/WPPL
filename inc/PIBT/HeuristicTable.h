#pragma once
#include "common.h"
#include "SharedEnv.h"
#include <omp.h>
#include <chrono>
#include "RHCR/interface/CompetitionActionModel.h"
#include "boost/filesystem.hpp"
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include "util/Dev.h"
#include "util/Timer.h"
#include "util/MyLogger.h"
#include "boost/format.hpp"

class HeuristicTable {
public:

    const SharedEnvironment & env;
    CompetitionActionModelWithRotate action_model;
    // loc1, loc2
    unsigned short * main_heuristics;
    // loc1, loc2, orient1, orient2
    char * sub_heuristics;
    int * empty_locs;
    int * loc_idxs; 
    int n_orientations=4;
    size_t loc_size=0;
    size_t state_size;
    bool consider_rotation=true;

    // divide by 16 in case of overlow
    const uint MAX_HEURISTIC=UINT_MAX/16;

    HeuristicTable(SharedEnvironment * _env, bool consider_rotation=true):env(*_env),action_model(_env),consider_rotation(consider_rotation){
        if (consider_rotation) {
            n_orientations=4;
        } else {
            n_orientations=1;
        }

        loc_size=0;
        for (int i=0;i<env.map.size();++i) {
            if (!env.map[i]) {
                ++loc_size;
            }
        }

        empty_locs = new int[loc_size];
        loc_idxs = new int[env.map.size()];

        g_logger.debug("number of empty locations: {}", loc_size);
        std::fill(loc_idxs,loc_idxs+env.map.size(),-1);

        int loc_idx=0;
        for (int loc=0;loc<env.map.size();++loc) {
            if (!env.map[loc]) {
                empty_locs[loc_idx]=loc;
                loc_idxs[loc]=loc_idx;
                ++loc_idx;
            }
        }
        ONLYDEV(assert(loc_idx==loc_size);)

        state_size = loc_size*n_orientations;
        main_heuristics = new unsigned short[loc_size*loc_size];
        if (consider_rotation)
            sub_heuristics = new char[state_size*state_size];
    };

    ~HeuristicTable() {
        delete [] empty_locs;
        delete [] loc_idxs;
        delete [] main_heuristics;
        if (consider_rotation)
            delete [] sub_heuristics;
    }

    void compute_heuristics(){
        g_logger.debug("[start] Compute heuristics.");
        g_timer.record_p("heu/compute_start");

        int n_threads=omp_get_max_threads();
        cout<<"number of threads used for heuristic computation: "<<n_threads<<endl;
        unsigned short * values = new unsigned short[n_threads*n_orientations*state_size];
        bool * visited = new bool[n_threads*n_orientations*state_size];
        State * queues = new State[n_threads*n_orientations*state_size];

        cerr<<"created"<<endl;


        int ctr=0;
        int step=100;
        auto start = std::chrono::steady_clock::now();
        #pragma omp parallel for
        for (int loc_idx=0;loc_idx<loc_size;++loc_idx)
		{
            int thread_id=omp_get_thread_num();

            int s_idx=thread_id*n_orientations*state_size;
            compute_heuristics(loc_idx,values+s_idx,visited+s_idx,queues+s_idx);
            
            #pragma omp critical
            {
                ++ctr;
                if (ctr%step==0){
                    auto end = std::chrono::steady_clock::now();
                    double elapse=std::chrono::duration<double>(end-start).count();
                    double estimated_remain=elapse/ctr*(loc_size-ctr);
                    cout<<ctr<<"/"<<loc_size<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
                }
            }
		}

        delete [] values;
        delete [] visited;
        delete [] queues;

        g_timer.record_d("heu/compute_start","heu/compute_end","heu/compute");

        g_logger.debug("[end] Compute heuristics. (duration: {:.3f})", g_timer.get_d("heu/compute"));
    }

    inline void _push(State * queue, State &s, int & e_idx) {
        queue[e_idx]=s;
        e_idx+=1;
    }

    inline State _pop(State * queue, int & s_idx) {
        State & s = queue[s_idx];
        s_idx+=1;
        return s;
    }

    inline bool _empty(int s_idx, int e_idx) {
        return s_idx==e_idx;
    }

    void compute_heuristics(
        int start_loc_idx,
        unsigned short * values, 
        bool * visited,
        State * queue
    ) {
        std::fill(values,values+n_orientations*state_size,USHRT_MAX);
        std::fill(visited,visited+n_orientations*state_size,false);

        int start_loc=empty_locs[start_loc_idx];

        for (int start_orient=0;start_orient<n_orientations;++start_orient) {
            // this are indexs used for queues.
            int s_idx=start_orient*state_size;
            int e_idx=start_orient*state_size;

            // start
            State state(start_loc,0,start_orient);
            ONLYDEV(assert(loc_idxs[state.location]>=0);)
            // start_orient,loc,orient
            size_t idx=start_orient*state_size+loc_idxs[state.location]*n_orientations+state.orientation;
            // because this is uniform-cost bfs, we know the first time is the best time
            values[idx]=state.timestep;
            visited[idx]=true;
            _push(queue,state,e_idx);

            int ctr=0;
            while (!_empty(s_idx,e_idx)) {
                State state=_pop(queue,s_idx);
                ++ctr;
                int c=0;

                vector<State> neighbors;
                if (consider_rotation){
                    neighbors=action_model.get_state_neighbors(state,false);
                } else {
                    neighbors=action_model.get_loc_neighbors(state,false);
                }

                for (auto & next: neighbors){
                    ++c;
                    ONLYDEV(assert(loc_idxs[state.location]>=0);)
                    size_t idx=start_orient*state_size+loc_idxs[next.location]*n_orientations+next.orientation;
                    if (!visited[idx]) {
                        values[idx]=next.timestep;
                        assert(next.timestep==state.timestep+1 && next.timestep<USHRT_MAX);
                        visited[idx]=true;
                        _push(queue,next,e_idx);
                    }
                }
            }
        }


        if (consider_rotation) {
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                unsigned short min_val=USHRT_MAX;
                for (int start_orient=0;start_orient<n_orientations;++start_orient) {
                    for (int orient=0;orient<n_orientations;++orient) {
                        size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                        // cerr<<"helo"<<values[value_idx]<<endl;
                        if (values[value_idx]<min_val) {
                            min_val = values[value_idx];
                        }
                    }
                }
                // cerr<<start_loc_idx<<" "<<loc_idx<<" "<<min_val<<endl;
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                main_heuristics[main_idx]=min_val;
                for (int start_orient=0;start_orient<n_orientations;++start_orient) {
                    for (int orient=0;orient<n_orientations;++orient) {
                        size_t value_idx=start_orient*state_size+loc_idx*n_orientations+orient;
                        size_t sub_idx=((start_loc_idx*loc_size+loc_idx)*n_orientations+start_orient)*n_orientations+orient;
                        sub_heuristics[sub_idx]=char(values[value_idx]-min_val);
                    }
                }
            }
        } else {
            for (int loc_idx=0;loc_idx<loc_size;++loc_idx) {
                size_t main_idx=start_loc_idx*loc_size+loc_idx;
                main_heuristics[main_idx]=values[loc_idx];
            }
        }

    }
    
    // TODO(hj) add check
    inline uint get(int loc1, int loc2) {
        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=loc_idx1*loc_size+loc_idx2;
        return main_heuristics[idx];
    }

    inline uint get(int loc1, int orient1, int loc2) {
        if (!consider_rotation) {
            cerr<<"no valid to use this func if not consider rotation"<<endl;
            exit(-1);
        }

        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=((loc_idx1*loc_size+loc_idx2)*n_orientations+orient1)*n_orientations;
        char min_v=CHAR_MAX;
        for (int i=0;i<n_orientations;++i) {
            if (sub_heuristics[idx+i]<min_v) {
                min_v=sub_heuristics[idx+i];
            }
        }
        return min_v+main_heuristics[loc_idx1*loc_size+loc_idx2];
    } 

    inline uint get(int loc1, int orient1, int loc2, int orient2) {
        if (!consider_rotation) {
            cerr<<"no valid to use this func if not consider rotation"<<endl;
            exit(-1);
        }
        int loc_idx1=loc_idxs[loc1];
        int loc_idx2=loc_idxs[loc2];

        if (loc_idx1==-1 || loc_idx2==-1) {
            return MAX_HEURISTIC;
        }

        size_t idx=((loc_idx1*loc_size+loc_idx2)*n_orientations+orient1)*n_orientations+orient2;
        return sub_heuristics[idx]+main_heuristics[loc_idx1*loc_size+loc_idx2];
    }


    void preprocess() {

        string fname=env.map_name.substr(0,env.map_name.size()-4);
        string folder=env.file_storage_path;
        if (folder[folder.size()-1]!=boost::filesystem::path::preferred_separator){
            folder+=boost::filesystem::path::preferred_separator;
        }
        string fpath;
        if (consider_rotation) {
            fpath=folder+fname+"_heuristics_v2.gz";
        } else {
            fpath=folder+fname+"_heuristics_no_rotation_v2.gz";
        }

        if (boost::filesystem::exists(fpath)) {
            load(fpath);
        } else {
            compute_heuristics();
            save(fpath);
        }

    }

    void save(const string & fpath) {
        g_logger.debug("[start] Save heuristics to {}.", fpath);
        g_timer.record_p("heu/save_start");

        std::ofstream fout;
        fout.open(fpath,std::ios::binary|std::ios::out);

        boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
        outbuf.push(boost::iostreams::zlib_compressor());
        outbuf.push(fout);
        
        std::ostream out(&outbuf);

        // save loc size
        out.write((char *)&loc_size,sizeof(int));

        // save empty locs
        out.write((char*)empty_locs,sizeof(int)*loc_size);

        // save main heuristics
        out.write((char *)main_heuristics,sizeof(unsigned short)*loc_size*loc_size);

        // save sub heuristics
        if (consider_rotation)
            out.write((char *)sub_heuristics,sizeof(char)*state_size*state_size);

        boost::iostreams::close(outbuf);
        fout.close();
        
        g_timer.record_d("heu/save_start","heu/save_end","heu/save");

        g_logger.debug("[end] Save heuristics to {}. (duration: {:.3f})", fpath, g_timer.get_d("heu/save"));
    }

    void load(const string & fpath) {
        g_logger.debug("[start] load heuristics from {}.",fpath);
        g_timer.record_p("heu/load_start");
        std::ifstream fin;
        fin.open(fpath,std::ios::binary|std::ios::in);

        boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
        inbuf.push(boost::iostreams::zlib_decompressor());
        inbuf.push(fin);

        std::istream in(&inbuf);

        // load loc size
        int _loc_size;
        in.read((char *)&_loc_size,sizeof(int));

        // check loc size
        if (_loc_size!=loc_size) {
            cerr<<"the sizes of empty locations don't match!"<<endl;
            exit(-1);
        }

        // load empty locs
        int * _empty_locs=new int[loc_size];
        in.read((char *)_empty_locs,sizeof(int)*loc_size);

        // check empty locs
        for (auto i=0;i<loc_size;++i) {
            if (_empty_locs[i]!=empty_locs[i]) {
                cerr<<"the empty locations don't match!"<<endl;
                exit(-1);
            }
        }

        // load main heurisitcs
        in.read((char *)main_heuristics,sizeof(unsigned short)*loc_size*loc_size);
        
        // load sub heuristics
        if (consider_rotation)
            in.read((char *)sub_heuristics,sizeof(char)*state_size*state_size);

        g_timer.record_d("heu/load_start","heu/load_end","heu/load");

        g_logger.debug("[end] load heuristics from {}. (duration: {:.3f})",fpath,g_timer.get_d("heu/load"));
    }
};