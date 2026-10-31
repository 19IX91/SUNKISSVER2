#include "r1_tracking_core.h"
#include <iomanip>
#include <iostream>

template<class V> void print_vector(const V& v) {
    std::cout<<'[';for(int i=0;i<v.size();++i){if(i)std::cout<<',';std::cout<<v[i];}std::cout<<']';
}
int main(int argc,char** argv) {
    try {
        if(argc!=2)throw std::runtime_error("Usage: r1_tracking_offline /path/to/params/deploy.yaml");
        r1_tracking::Engine engine(argv[1]);std::cout<<std::setprecision(9);std::string line;
        while(std::getline(std::cin,line)) {
            try {
                auto n=YAML::Load(line);auto s=r1_tracking::parse_state(n);
                if(n["align"] && n["align"].as<bool>())engine.align(s);
                auto obs=engine.observe(n["frame"].as<size_t>(),s,r1_tracking::vector<24>(n["last_action"]));
                auto action=engine.infer(obs);auto target=engine.targets(action);
                std::cout<<"{\"obs\":";print_vector(obs);std::cout<<",\"actions\":";print_vector(action);
                std::cout<<",\"targets\":";print_vector(target);std::cout<<"}"<<std::endl;
            } catch(const std::exception& e) {
                YAML::Emitter escaped;escaped<<YAML::DoubleQuoted<<e.what();
                std::cout<<"{\"error\":"<<escaped.c_str()<<"}"<<std::endl;
            }
        }
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
