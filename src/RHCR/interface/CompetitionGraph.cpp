#include "RHCR/interface/CompetitionGraph.h"
#include "common.h"

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

    for (int i=0;i<rows;++i) {
        for (int j=0;j<cols;++j) {
            int idx=cols*i+j;
            weights[idx].resize(5,WEIGHT_MAX);
            if (grid.map[idx]) {
                // obstacle
                types[idx]="Obstacle";
            } else {
                types[idx]="Travel";
                // cost of wait
                weights[idx][4] = 1;

                // cost of feasible move
                for (int dir=0;dir<4;++dir) {
                    if (0 <= i + move[dir] && i + move[dir] < cols * rows && 
                        get_Manhattan_distance(i, i + move[dir]) <= 1 && 
                        types[i + move[dir]] != "Obstacle") {
                        weights[i][dir] = 1;  
                    }         
                }
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
		for (int idx=0;idx<rows*cols;++idx)
		{
            if (types[idx]!="Obstacle"){
			    heuristics[idx] = compute_heuristics(idx);
            }
		}
		save_heuristics_table(fname);
	}

	double runtime = (std::clock() - t) / CLOCKS_PER_SEC;
	std::cout << "Done! (" << runtime << " s)" << std::endl;
}

}