#include "RHCR/interface/CompetitionGraph.h"
#include "common.h"
#include <omp.h>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <chrono>
#include <cstring>
#include <algorithm>

namespace RHCR {

// this contructor directly convert Grid to CompetitionGraph
CompetitionGraph::CompetitionGraph(const Grid & grid){
    map_name=grid.map_name;
    
    rows=grid.rows;
    cols=grid.cols;

    move[0] = 1; // move east
    move[1] = -cols; // move north 
    move[2] = -1; // move west
    move[3] = cols; // move south

    int n_grids=rows*cols;
    // set up grid types and weights
    types.resize(n_grids);
    weights.resize(n_grids);

    for (int idx=0;idx<rows*cols;++idx) {
        weights[idx].resize(5,WEIGHT_MAX);
        if (grid.map[idx]) {
            // obstacle
            types[idx]="Obstacle";
        } else {
            types[idx]="Travel";
            // cost of wait
            weights[idx][4] = 1;
        }
    }

    for (int idx=0;idx<rows*cols;++idx) {
        if (types[idx] == "Obstacle")
        {
            continue;
        }
        // cost of feasible move
        for (int dir=0;dir<4;++dir) {
            if (0 <= idx + move[dir] && idx + move[dir] < cols * rows && 
                get_Manhattan_distance(idx, idx + move[dir]) <= 1 && 
                types[idx + move[dir]] != "Obstacle") {
                weights[idx][dir] = 1;  
            }         
        }
    }
}

bool CompetitionGraph::load_map(string fname){
    // TODO
    return true;
}

void CompetitionGraph::preprocessing(bool consider_rotation){
    // currently only support this setting.
    assert(consider_rotation);

    std::cout << "*** PreProcessing map ***" << std::endl;
	clock_t t = std::clock();
	this->consider_rotation = consider_rotation;
	std::string fname;
	if (consider_rotation)
		fname = map_name + "_rotation_heuristics_table.gz";
	else
		fname = map_name + "_heuristics_table.gz";
	std::ifstream myfile(fname.c_str(),std::ios::binary|std::ios::in);
	bool succ = false;
	if (myfile.is_open())
	{
		succ = load_heuristics_table(myfile);
		myfile.close();
	}
	if (!succ)
	{
        // use parallel execution here
        vector<int> idxs;
        int total=0;
		for (int idx=0;idx<rows*cols;++idx)
		{
            if (types[idx]!="Obstacle"){
			    ++total;
                idxs.push_back(idx);
                heuristics[idx]=vector<double>(this->size(),DBL_MAX);
            }
		}        
        // int step=100;
        // int ctr=0;
        // double s=clock();
		// for (int idx=0;idx<rows*cols;++idx)
		// {
        //     if (types[idx]!="Obstacle"){
		// 	    heuristics[idx] = compute_heuristics(idx);
        //         ++ctr;
        //         if (ctr%step==0){
        //             double elapse=(clock()-s)/CLOCKS_PER_SEC;
        //             double estimated_remain=elapse/ctr*(total-ctr);
        //             cout<<ctr<<"/"<<total<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s."<<endl;
        //         }
        //     }

		// }

        int n_threads=omp_get_max_threads();               
        cout<<"number of threads used for heuristic computation: "<<n_threads<<endl;
        int n_directions=4;
        // maybe let's just use c way to speed up?
        double * lengths = new double[n_threads*this->size()*n_directions];
        bool * visited = new bool[n_threads*this->size()*n_directions];
        State * queues = new State[n_threads*this->size()*n_directions];

        int ctr=0;
        int step=100;
        auto start = std::chrono::steady_clock::now();
        #pragma omp parallel for
        for (int i=0;i<idxs.size();++i)
		{
            int thread_id=omp_get_thread_num();

            int idx=idxs[i];
            int s_idx=thread_id*this->size()*n_directions;
            compute_heuristics(idx,lengths+s_idx,visited+s_idx,queues+s_idx,heuristics[idx],n_directions);
            
            #pragma omp critical
            {
                ++ctr;
                if (ctr%step==0){
                    auto end = std::chrono::steady_clock::now();
                    double elapse=std::chrono::duration<double>(end-start).count();
                    double estimated_remain=elapse/ctr*(total-ctr);
                    cout<<ctr<<"/"<<total<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s.  estimated total time: "<<(estimated_remain+elapse)<<"s."<<endl;
                }
            }
		}

        delete [] lengths;
        delete [] visited;
        delete [] queues;
        
		save_heuristics_table(fname);
	}

	double runtime = double(std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

void push(State * queue, State & s, int & e_idx){
    queue[e_idx]=s;
    e_idx+=1;
}

State pop(State * queue, int & s_idx){
    State s = queue[s_idx];
    s_idx+=1;
    return s;
}

inline bool empty(int s_idx, int e_idx) {
    return s_idx==e_idx;
}

// TODO: use r-value?
void CompetitionGraph::compute_heuristics(
    int root_location,
    double * lengths,
    bool * visited,
    State * queue,
    vector<double> & result,
    const int n_directions
 ){

    // just initialize inside this function
    std::fill(lengths,lengths+this->size()*n_directions,DBL_MAX);
    std::fill(visited,visited+this->size()*n_directions,false);
    int s_idx=0;
    int e_idx=0;

    for (int d=0;d<n_directions;++d){
        State s(root_location,0,d);
        int idx=s.location*n_directions+s.orientation;
        lengths[idx]=s.timestep;
        visited[idx]=true;
        push(queue,s,e_idx);
    }
    
    while (!empty(s_idx,e_idx)){
        const State & prev_s = pop(queue,s_idx);
        for (auto & s : get_reverse_neighbors(prev_s)){
            // if (root_location==2318){
            //     cerr<<"s:"<<s<<endl;
            // }
            s.timestep=prev_s.timestep+1;
            int idx=s.location*n_directions+s.orientation;
            if (get_weight(prev_s.location,s.location)<WEIGHT_MAX && !visited[idx]){
                lengths[idx]=s.timestep;
                visited[idx]=true;
                push(queue,s,e_idx);
            }
        }
    }

    for (int i=0;i<this->size();++i) {
        for (int d=0;d<n_directions;++d){
            result[i]=min(result[i],lengths[i*n_directions+d]);
        }
        if (result[i]>USHRT_MAX && result[i]<DBL_MAX){
            cerr<<"err:"<<result[i]<<endl;
            assert(!"failed");
        }
    }
}

bool CompetitionGraph::load_heuristics_table(std::ifstream& myfile)
{
    // TODO: we need to test the loading speed for compressed & uncompressed version.
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::gzip_decompressor());
    inbuf.push(myfile);

    std::istream in(&inbuf);

    // load table size.
    // N is the size of the heuristic table, M is the size of the map
    int N,M;
    in.read((char*)&N,sizeof(int));
    in.read((char*)&M,sizeof(int));
    assert(M==this->size());

    // load non-obstacle locations
    vector<int> locs;
    for (int i=0;i<N;++i){
        int loc;
        in.read((char*)&loc,sizeof(int));
        locs.push_back(loc);
        heuristics[loc]=std::vector<double>(M,DBL_MAX);
    }

    // load dists from one non-obstacle location to another
    // NOTE(hj): currently, the heuristic table in the memory is still N*M size double-type.
    for (int loc1:locs){
        for (int loc2:locs){
            if (loc1<=loc2){
                unsigned short v;
                in.read((char*)&v,sizeof(unsigned short));
                double h;
                if (v==USHRT_MAX){
                    h=DBL_MAX;
                } else {
                    h=(double) v;
                }
                heuristics[loc1][loc2]=h;
                heuristics[loc2][loc1]=h;
            }
        }
    }

    // boost::char_separator<char> sep(",");
    // boost::tokenizer< boost::char_separator<char> >::iterator beg;
    // std::string line;
    
    // getline(in, line); //skip "table_size"
    // getline(in, line);
    // boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    // beg = tok.begin();
	// int N = atoi ( (*beg).c_str() ); // read number of cols
	// beg++;
	// int M = atoi ( (*beg).c_str() ); // read number of rows
	// if (M != this->size())
	//     return false;
	// for (int i = 0; i < N; i++)
	// {
	// 	getline (in, line);
    //     int loc = atoi(line.c_str());
    //     getline (in, line);        
    //     boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	//     beg = tok.begin();
    //     std::vector<double> h_table(this->size());
    //     for (int j = 0; j < this->size(); j++)
    //     {
    //         h_table[j] = atof((*beg).c_str());

    //         // jh: why this line? I'll remove it.
    //         // if (h_table[j] >= INT_MAX && types[j] != "Obstacle")
    //         //     types[j] = "Obstacle";
    //         beg++;
    //     }
    //     heuristics[loc] = h_table;
    // }

    boost::iostreams::close(inbuf);
	return true;
}


void CompetitionGraph::save_heuristics_table(std::string fname)
{
    std::ofstream myfile;
	myfile.open(fname,std::ios::binary|std::ios::out);

    boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
    outbuf.push(boost::iostreams::gzip_compressor());
    outbuf.push(myfile);
    
    std::ostream out(&outbuf);

    // save table size.
    int size;
    size=heuristics.size();
    out.write((char*)&size,sizeof(int));
    size=this->size();
    out.write((char*)&size,sizeof(int));

    // save non-obstacle locations
    vector<int> locs;
    for (int idx=0;idx<types.size();++idx){
        if (types[idx]!="Obstacle"){
            out.write((char*)&idx,sizeof(int));
            locs.push_back(idx);
        }
    }

    // save dists from one non-obstacle location to another
    for (int loc1:locs){
        for (int loc2:locs){
            // NOTE: we assume symmetry.
            if (loc1<=loc2){
                double h=heuristics[loc1][loc2];
                unsigned short v;
                if (h==DBL_MAX){
                    v=USHRT_MAX;
                }
                else if (h>USHRT_MAX && h<DBL_MAX){
                    auto & G=*this;
                    // TODO(hj): this is a protection. if not true, we probably need to consider other data type.
                    cerr<<"h value("<<h<<") from loc1("<<G.get_row(loc1)<<","<<G.get_col(loc1)<<") to loc2("<<G.get_row(loc2)<<","<<G.get_col(loc2)<<") exceeds the unsigned short limit("<<USHRT_MAX<<")!"<<endl;
                    // NOTE(hj): assert only works when DBEUG is set! probably need better assertion and logging tools.
                    assert(!"failed");
                    v=USHRT_MAX;
                } else{
                    v=(unsigned short)h;
                }

                out.write((char*)&v,sizeof(unsigned short));
            }
        }
    }

    boost::iostreams::close(outbuf);
    myfile.close();

    // std::ofstream myfile;
	// myfile.open (fname);

    // boost::iostreams::filtering_streambuf<boost::iostreams::output> outbuf;
    // outbuf.push(boost::iostreams::gzip_compressor());
    // outbuf.push(myfile);
    
    // std::ostream out(&outbuf);

	// out << "table_size" << std::endl << 
    //     heuristics.size() << "," << this->size() << std::endl;
	// for (auto h_values: heuristics) 
	// {
    //     out << h_values.first << std::endl;
	// 	for (double h : h_values.second) 
	// 	{
    //         out << h << ",";
	// 	}
	// 	out << std::endl;
	// }

    // boost::iostreams::close(outbuf);
	// myfile.close();

}

}