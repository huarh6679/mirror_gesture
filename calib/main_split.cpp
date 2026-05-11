#include "split_stereo.hpp"

using namespace stereo_toolkit;

int main(int argc, char* argv[])
{   
    if(argc < 2){
        std::cout << "need more parameters ." << std::endl;
        exit(1);
    }

    std::string value = argv[1];
    std::cout << "value: " << value << std::endl;

    std::string yaml_file = argv[1];
    YAML::Node config_node = YAML::LoadFile(yaml_file);

    StereoSplit stereo_split_obj(config_node);
    if(stereo_split_obj.Run()){
        cout << "Split Success ." << endl;
    }

    return 0;
}
