#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <array>
#include <algorithm>
#include <limits>
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

std::vector<Node*> DykstraSearch(){
    /* TODO: implement Dykstra search
    * 1. Will be called for one start and all goal location
    * 2. Keep closed list between calls to avoid re-exploring
    * 3. Return a vector of nodes from start to goal
    */ 
}

std::vector<int> getSuccessor(){
    // TODO: implement getSuccessor for search
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

float get_weight(int row, int col, int action, int rows, int cols, const std::vector<float> map_weights) {
    // Calculate the index for the location and action
    int index = (row * cols + col) * 5 + action;

    if (index < 0 || index >= map_weights.size()) {
        std::cerr << "Invalid location or action" << std::endl;
        exit(-1);
    }

    return map_weights[index];
}

void change_weights(int row, int col, int action, int rows, int cols, std::vector<float> map_weights, float new_weight) {
    // Calculate the index for the location and action
    int index = (row * cols + col) * 5 + action;

    if (index < 0 || index >= map_weights.size()) {
        std::cerr << "Invalid location or action" << std::endl;
        exit(-1);
    }

    map_weights[index] = new_weight;
    return;
}

std::vector<float> get_frequency_metrics(){
    /* TODO: get a metric of map traversal frequency 
    * 1. get input map traversal frequency 
    * 2. outputs a map of metrics that describe how often each cell is traversed
    */
}

std::vector<float> update_weights(){
    /*TODO:updates weights based on frequency metrics 
    * 1. compare old frequency metrics and new frequency metrics
    * 2. update weights based on the difference
    */
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map_file_path>" << std::endl;
        return 1;
    }

    std::string mapFilePath = argv[1];
    int height = 0, width = 0;
    std::vector<std::vector<int>> travel_frequency(height, std::vector<int>(width, 0));
    bool** mapArray = readMap(mapFilePath, height, width);
    for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; j++) {
        if (mapArray[i][j] == false) {
            continue;
        }
    for (int k = 0; k < 5; k++) {
        // i, j, for map location, k for action
        Node* curStartnode = new Node({i, j, k});
        for (int ii = 0; ii < height; ++ii) {
        for (int jj = 0; jj < width; jj++) {
            if (mapArray[ii][jj] == false) {
                continue;
            }
            // ii, jj for goal location
            std::vector<int> curGoalnode = {ii, jj};
            std::vector<Node*> path = DykstraSearch();
            // add to frequency map
            while (path.size() > 0) {
                Node* curNode = path.back();
                path.pop_back();
                travel_frequency[i][j] += 1;
            }
        }  
        }
    }
    }
    
    // for (int i = 0; i < height; ++i) {
    //     delete[] mapArray[i];
    // }
    // delete[] mapArray;

    return 0;
}