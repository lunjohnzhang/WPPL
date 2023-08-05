#include "RHCR/interface/CompetitionGraph.h"
#include "common.h"
#include <omp.h>

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
		fname = map_name + "_rotation_heuristics_table.txt";
	else
		fname = map_name + "_heuristics_table.txt";
	std::ifstream myfile(fname.c_str());
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

        int ctr=0;
        int step=100;
        double s=clock();
        #pragma omp parallel for
        for (int i=0;i<idxs.size();++i)
		{
            if (i==0) {
                int nthreads=omp_get_num_threads();
                cerr<<"number of threads used for heuristic computation: "<<nthreads<<endl;
            }

            int idx=idxs[i];
            heuristics[idx] = compute_heuristics(idx);

            #pragma omp critical
            {
                ++ctr;
                if (ctr%step==0){
                    double elapse=(clock()-s)/CLOCKS_PER_SEC;
                    double estimated_remain=elapse/ctr*(total-ctr);
                    cout<<ctr<<"/"<<total<<" completed in "<<elapse<<"s. estimated time to finish all: "<<estimated_remain<<"s."<<endl;
                }
            }
		}
        
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

// TODO: use r-value?
vector<double> CompetitionGraph::compute_heuristics(int root_location){

    const int n_directions=4;

    // 4 for each direction
    // TODO: just use int
    vector<vector<double> > lengths(this->size(),vector<double>(n_directions,DBL_MAX));
    vector<vector<bool> > visited(this->size(),vector<bool>(n_directions,false));

    std::queue<State> q;

    for (int d=0;d<n_directions;++d){
        State s(root_location,0,d);
        lengths[s.location][s.orientation]=s.timestep;
        visited[s.location][s.orientation]=true;
        q.push(s);
    }
    
    while (!q.empty()){
        const State & prev_s = q.front();
        q.pop();

        for (auto & s : get_reverse_neighbors(prev_s)){
            s.timestep=prev_s.timestep+1;
            if (get_weight(prev_s.location,s.location)<=WEIGHT_MAX && !visited[s.location][s.orientation]){
                lengths[s.location][s.orientation]=s.timestep;
                visited[s.location][s.orientation]=true;
                q.push(s);
            }
        }
    }

    vector<double> res(this->size(),DBL_MAX);

    for (int i=0;i<this->size();++i) {
        for (int d=0;d<n_directions;++d){
            res[i]=min(res[i],lengths[i][d]);
        }
    }

    return res;
}

bool CompetitionGraph::load_heuristics_table(std::ifstream& myfile)
{
    boost::char_separator<char> sep(",");
    boost::tokenizer< boost::char_separator<char> >::iterator beg;
    std::string line;
    
    getline(myfile, line); //skip "table_size"
    getline(myfile, line);
    boost::tokenizer< boost::char_separator<char> > tok(line, sep);
    beg = tok.begin();
	int N = atoi ( (*beg).c_str() ); // read number of cols
	beg++;
	int M = atoi ( (*beg).c_str() ); // read number of rows
	if (M != this->size())
	    return false;
	for (int i = 0; i < N; i++)
	{
		getline (myfile, line);
        int loc = atoi(line.c_str());
        getline (myfile, line);        
        boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	    beg = tok.begin();
        std::vector<double> h_table(this->size());
        for (int j = 0; j < this->size(); j++)
        {
            h_table[j] = atof((*beg).c_str());
            // jh: why this line? I'll remove it.
            // if (h_table[j] >= INT_MAX && types[j] != "Obstacle")
            //     types[j] = "Obstacle";
            beg++;
        }
        heuristics[loc] = h_table;
    }
	return true;
}


void CompetitionGraph::save_heuristics_table(std::string fname)
{
    std::ofstream myfile;
	myfile.open (fname);
	myfile << "table_size" << std::endl << 
        heuristics.size() << "," << this->size() << std::endl;
	for (auto h_values: heuristics) 
	{
        myfile << h_values.first << std::endl;
		for (double h : h_values.second) 
		{
            myfile << h << ",";
		}
		myfile << std::endl;
	}
	myfile.close();
}

}