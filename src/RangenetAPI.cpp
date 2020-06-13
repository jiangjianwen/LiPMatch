#include "RangenetAPI.hpp"


RangenetAPI::RangenetAPI(const std::string model_path){
  std::string backend = "tensorrt";
  // initialize a network
  net = cl::make_net(model_path, backend);
}


std::vector<std::vector<float>> RangenetAPI::infer(const std::vector<float>& scan,
                                                   const uint32_t num_points){
  return net->infer(scan, num_points);
}
