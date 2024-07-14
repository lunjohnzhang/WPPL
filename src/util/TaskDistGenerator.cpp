#include "util/TaskDistGenerator.h"


using namespace std;

// Function to generate a Gaussian distribution
vector<vector<double>> getGaussian(int h, int w, int center_h, int center_w, double sigma = 0.5) {
    vector<double> x0(w);
    vector<double> y0(h);
    for (int i = 0; i < w; ++i) x0[i] = -5 + i * 10.0 / (w - 1);
    for (int i = 0; i < h; ++i) y0[i] = -5 + i * 10.0 / (h - 1);

    vector<vector<double>> z(h, vector<double>(w, 0));
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            double x_diff = (x0[c] - x0[center_w]) * (x0[c] - x0[center_w]);
            double y_diff = (y0[r] - y0[center_h]) * (y0[r] - y0[center_h]);
            z[r][c] = exp(-(x_diff / (2 * sigma * sigma) + y_diff / (2 * sigma * sigma)));
        }
    }
    return z;
}

// Function to generate a vector of distances
vector<double> generateVecEDist(const vector<vector<double>>& map_e, const vector<vector<double>>& dist_e) {
    vector<double> vec_e_dist;
    int h = map_e.size();
    int w = map_e[0].size();
    
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            if (map_e[r][c] != 1.0) continue;
            vec_e_dist.push_back(dist_e[r][c]);
        }
    }
    return vec_e_dist;
}


void readMap(const Grid& map, vector<vector<double>>& map_e, vector<vector<double>>& map_w){
    map_e.resize(map.rows, vector<double>(map.cols, 0));
    map_w.resize(map.rows, vector<double>(map.cols, 0));
    for (auto e_id: map.end_points){
        int r = e_id / map.cols;
        int c = e_id % map.cols;
        map_e[r][c] = 1;
    }
    for (auto w_id: map.agent_home_locations){
        int r = w_id / map.cols;
        int c = w_id % map.cols;
        map_w[r][c] = 1;
    }
}

// Function to generate task and agent distribution
void generateTaskAndAgentDist(const Grid& map, mt19937 MT, vector<double>& w_dist, vector<double>& e_dist) {
    vector<vector<double>> map_e, map_w;

    readMap(map, map_e, map_w);
    
    int num_w = 0;
    for (const auto& row : map_w) {
        num_w += count(row.begin(), row.end(), 1.0);
    }

    w_dist.resize(num_w, 1.0);
    
    int h = map.rows;
    int w = map.cols;

    random_device rd;
    uniform_int_distribution<> dis_h(0, h - 1);
    uniform_int_distribution<> dis_w(0, w - 1);
    
    int center_h = dis_h(MT);
    int center_w = dis_w(MT);
    
    vector<vector<double>> dist_full = getGaussian(h, w, center_h, center_w);
    vector<vector<double>> dist_e(h, vector<double>(w, 0));
    double max_val = 0;

    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            dist_e[r][c] = dist_full[r][c] * map_e[r][c];
            if (dist_e[r][c] > max_val) max_val = dist_e[r][c];
        }
    }

    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            dist_e[r][c] /= max_val; // normalize
        }
    }

    e_dist = generateVecEDist(map_e, dist_e);

    // For demonstration purposes
    // cout << "Task and Agent Distribution Generated Successfully!" << endl;
}