#include <MiniDNN.h>
#include <Eigen/Dense>

using namespace MiniDNN;

struct NetworkConfig{
    int n_in_chan=10;
    int n_hid_chan=32;
    int kernel_size=5;
};

extern NetworkConfig network_config;

class CNNNetwork{
    private:
        Network net;
        int param_num=0;
    public:
        CNNNetwork(){}
        CNNNetwork(int map_h, int map_w){
            this->build_model(map_h, map_w, network_config.n_in_chan, network_config.kernel_size, network_config.n_hid_chan);
        }
        void build_model(int map_h, int map_w, int nc, int kernel_size, int n_hid_chan) {
            // Initial Conv2D Layer
            int input_h = map_h+kernel_size-1;
            int input_w = map_w+kernel_size-1;
            Layer* layer1 = new Convolutional<ReLU>(input_h, input_w, nc, n_hid_chan, kernel_size, kernel_size);
            this->net.add_layer(layer1);

            Layer* layer2 = new Convolutional<ReLU>(map_h, map_w, n_hid_chan, n_hid_chan, 1, 1);
            this->net.add_layer(layer2);

            Layer* layer3 = new Convolutional<ReLU>(map_h, map_w, n_hid_chan, 5, 1, 1);
            this->net.add_layer(layer3);

            this->net.init();

            for (auto layer: this->net.get_layers()){
                int pn = (layer->get_parameters()).size();
                this->param_num+=pn;
            }
            
            std::cout << "network params num = "<< this->param_num <<std::endl;
        }

        std::vector<float> forward(std::vector<int>& input_vec){
            // input vec -> mat
            // normalize
            // forward
            // mat -> vec
            std::cout << "not implement yet!" << std::endl;
            exit(1);
        }

        void set_params(std::vector<double>& new_params) {
            std::cout<<"in MiniDNNMLPNetwork::set_params"<<std::endl;
            if (this->param_num!=new_params.size()){
                std::cout << "error! net params is = [" <<this->param_num<<"] but receive ["<<new_params.size()<<"]!" <<std::endl;
                exit(1);
            }
            auto layers = this->net.get_layers();
            int param_start = 0;
            std::vector<std::vector<double>> collected_params;
            for (auto layer: layers){
                int pn = (layer->get_parameters()).size();
                std::vector<double> layer_params(new_params.begin()+param_start, new_params.begin()+param_start+pn);
                collected_params.push_back(layer_params);
                param_start += pn;
            }
            this->net.set_parameters(collected_params);
        }

};