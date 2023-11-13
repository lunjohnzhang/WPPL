#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <array>
#include <algorithm>
#include <limits>
#include <numeric>
#include "ctpl_stl.h"
#include <future> 
#include <sstream>

/*
 * This file should receieves a map file, a weight file, and outputs path distribution file
 */

struct Node {
	std::vector<int> location;
	Node* parent = nullptr;
	/// int f = std::numeric_limits<int>::infinity();
	int g = std::numeric_limits<int>::infinity();
	bool closed = false;
	// constructor with input of location
    Node(std::vector<int> loc) : location(loc) {}
};
Node* addSuccessor(Node*& prev, int x, int y, int orient, float g){
    Node* newNode = new Node({x, y, orient});
    newNode->g = g;
    newNode->parent = prev;
    return newNode;
}

std::vector<Node*> getSuccessor(Node*& node,int& height, int& width, bool**& mapArray, std::vector<float>& map_weights){
    // TODO: implement getSuccessor for search
    std::vector<Node*> successors;
    int x = node->location[0];
    int y = node->location[1];
    int orient = node->location[2];
    float g = node->g;
    
 
}

std::vector<Node*> DykstraSearch(){
    /* TODO: implement Dykstra search
    * 1. Will be called for one start and all goal location
    * 2. Keep closed list between calls to avoid re-exploring
    * 3. Return a vector of nodes from start to goal
    */ 
}

// Function to check if a character represents a traversable space.
bool isTraversable(char ch) {
    return ch == '.';
}

// Function to read the map from a file and store it in a boolean array.
bool** readMap(const std::string& filePath, int& height, int& width) {
    std::ifstream mapFile(filePath);
    if (!mapFile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return nullptr;
    }

    std::string line;
    while (std::getline(mapFile, line)) {
        if (line.substr(0, 5) == "type ") {
            // Skip the type line.
            continue;
        } else if (line.substr(0, 7) == "height ") {
            height = std::stoi(line.substr(7));
        } else if (line.substr(0, 6) == "width ") {
            width = std::stoi(line.substr(6));
        } else if (line == "map") {
            // Start reading the map after this line.
            break;
        }
    }

    // Create a 2D boolean array to store map information.
    bool** mapArray = new bool*[height];
    for (int i = 0; i < height; ++i) {
        mapArray[i] = new bool[width];
        if (!std::getline(mapFile, line) || line.length() != static_cast<size_t>(width)) {
            std::cerr << "Map format error or unexpected end of file." << std::endl;
            for (int j = 0; j <= i; j++) {
                delete[] mapArray[j];
            }
            delete[] mapArray;
            return nullptr;
        }
        for (int j = 0; j < width; ++j) {
            mapArray[i][j] = isTraversable(line[j]);
        }
    }

    mapFile.close();
    return mapArray;
}

std::vector<float> ReadWeights(const std::string& filePath) {
    std::vector<float> weights;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filePath);
    }

    std::string line;
    return weights;
}

float get_weight(int row, int col, int action, int rows, int cols, const std::vector<float>& map_weights) {
    // Calculate the index for the location and action
    int index = (row * cols + col) * 5 + action;

    if (index < 0 || index >= map_weights.size()) {
        std::cerr << "Invalid location or action" << std::endl;
        exit(-1);
    }

    return map_weights[index];
}

void change_weights(int row, int col, int action, int rows, int cols, std::vector<float>& map_weights, float new_weight) {
    // Calculate the index for the location and action
    int index = (row * cols + col) * 5 + action;

    if (index < 0 || index >= map_weights.size()) {
        std::cerr << "Invalid location or action" << std::endl;
        exit(-1);
    }

    map_weights[index] = new_weight;
    return;
}

std::vector<std::vector<float>> get_frequency_metrics(std::vector<std::vector<int>>& travel_frequency){
    /* get a metric of map traversal frequency (current metric is standard deviation)
    * 1. get input map traversal frequency 
    * 2. outputs a map of metrics that describe how often each cell is traversed
    */
   // find mean of the map
    std::vector<std::vector<float>> metric_map(travel_frequency.size(), std::vector<float>(travel_frequency[0].size(), 0));
    int mean = 0;
    int count = 0;
    for (int i = 0; i < travel_frequency.size(); i++){
        for (int j = 0; j < travel_frequency[0].size(); j++){
            if (travel_frequency[i][j] == 0){
                continue;
            }
            mean += travel_frequency[i][j];
            count += 1;
        }
    }
    mean = mean / count;   

    // mean offset sum
    int meanOffsetSum = 0;
    for (int i = 0; i < travel_frequency.size(); i++){
        for (int j = 0; j < travel_frequency[0].size(); j++){
            if (travel_frequency[i][j] == 0){
                continue;
            }
            meanOffsetSum += pow(travel_frequency[i][j] - mean, 2);
        }
    }
    
    // find standard deviation of the map
    int standardDeviation = sqrt(meanOffsetSum / count);

    // find q value for the map
    for (int i = 0; i < travel_frequency.size(); i++){
        for (int j = 0; j < travel_frequency[0].size(); j++){
            if (travel_frequency[i][j] == 0){
                metric_map[i][j] = std::numeric_limits<float>::quiet_NaN();
            }
            else{
                metric_map[i][j] = (travel_frequency[i][j] - mean)/standardDeviation;
            }
        }
    }

    return metric_map;
}

void update_weights(std::vector<std::vector<float>> old_metrics, std::vector<std::vector<float>> new_metrics, std::vector<float>& map_weights,const int& rows, const int& cols){
    /*TODO:updates weights based on frequency metrics ::::::: May have a smarter way to update weights
    * 1. compare old frequency metrics and new frequency metrics
    * 2. update weights based on the difference
    */
    const int updateFactor = 100;
    for (int i = 0; i < new_metrics.size(); i++){
        for (int j = 0; j < new_metrics[0].size(); j++){
            if (new_metrics[i][j] == std::numeric_limits<float>::quiet_NaN()){
                continue;
            }
            else{
                // float diff = new_metrics[i][j] - old_metrics[i][j];
                for (int k = 0; k < 5; k++){
                    // need deeper thoughts on how to update weights
                    float old_weight = get_weight(i, j, k, rows, cols, map_weights);
                    // if q value is positive, increase weight, if q value is negative, decrease weight
                    float new_weight = old_weight *(1+new_metrics[i][j]/updateFactor);
                    change_weights(i, j, k, rows, cols, map_weights, new_weight);
                }
            }
        }
    }

}

// Define a function that performs the task for a single starting point
std::vector<std::vector<int>> process_starting_point(int i, int j, int k, bool**& mapArray, const int& rows, const int& cols) {
    std::vector<std::vector<int>> local_frequency(rows, std::vector<int>(cols, 0));
    // i, j, for map location, k for action
    Node* curStartnode = new Node({i, j, k});
    for (int ii = 0; ii < rows; ++ii) {
        for (int jj = 0; jj < cols; jj++) {
            if (mapArray[ii][jj] == false) {
                continue;
            }
            // ii, jj for goal location
            std::vector<int> curGoalnode = {ii, jj};
            std::vector<Node*> path = DykstraSearch();
            // add to local frequency map
            while (path.size() > 0) {
                Node* curNode = path.back();
                path.pop_back();
                local_frequency[i][j] += 1;
            }
        }  
    }
    return local_frequency;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <map_file_path>" << argv[1] << " <weight_file_path>" << std::endl;
        return 1;
    }

    std::string mapFilePath = argv[1];
    std::string weightFilePath = argv[2];

    int height = 0, width = 0;
    std::vector<std::vector<int>> travel_frequency(height, std::vector<int>(width, 0));
    bool** mapArray = readMap(mapFilePath, height, width);
    std::vector<float> map_weights = ReadWeights(weightFilePath);

    // Determine the number of threads to use
    int num_threads = std::thread::hardware_concurrency();

    int maxEpoch = 10;

    for (int epoch = 0; epoch < maxEpoch; epoch++){
        // Create a thread pool with the desired number of threads
        ctpl::thread_pool pool(num_threads);

        // Submit tasks to the thread pool
        std::vector<std::future<std::vector<std::vector<int>>>> futures;
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; j++) {
                if (mapArray[i][j] == false) {
                    continue;
                }
                for (int k = 0; k < 5; k++) {
                    futures.push_back(pool.push([i, j, k, &mapArray, &height, &width](int id) {
                        return process_starting_point(i, j, k, mapArray, height, width);
                    }));
                }
            }
        }

        // Wait for all tasks to complete
        pool.stop(true);

        // Join all the threads and accumulate the results
        for (auto& future : futures) {
            auto local_frequency = future.get();
            for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    travel_frequency[i][j] += local_frequency[i][j];
                }
            }
        }

        // get frequency metrics
        std::vector<std::vector<float>> metric_map = get_frequency_metrics(travel_frequency);

    }
    

    return 0;
}