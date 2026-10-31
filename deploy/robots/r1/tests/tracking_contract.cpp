#include "r1_tracking_estimate.h"
#include <iostream>

int main() {
    using namespace r1_tracking;
    try {
        PoseFeed feed(.1);bool refused=false;
        try{feed.require_fresh(1000000000);}catch(const std::exception&){refused=true;}
        require(refused,"Missing feed must fail closed");
        auto valid=YAML::Load(R"({"version":1,"frame":"pelvis","stamp_ns":1000000000,"position_w":[0,0,0.7],"quaternion_wxyz":[1,0,0,0],"linear_velocity_b":[0.1,0,0]})");
        require(feed.accept(valid,1010000000),"Fresh valid estimate must pass");feed.require_fresh(1099999999);
        require(!feed.accept(valid,1020000000),"Duplicate timestamps must fail");
        for(const std::string& field:{"frame","quaternion_wxyz","linear_velocity_b","position_w"}) {
            auto bad=YAML::Clone(valid);bad["stamp_ns"]=1020000000;
            if(field=="frame")bad[field]="camera";
            else if(field=="quaternion_wxyz")bad[field]=std::vector<float>{0,0,0,0};
            else if(field=="linear_velocity_b")bad[field]=std::vector<float>{0,0};
            else bad[field]=std::vector<float>{NAN,0,0};
            require(!feed.accept(bad,1030000000),"Invalid estimate must fail");
        }
        auto bad=YAML::Clone(valid);bad["stamp_ns"]=1100000000;
        require(!feed.accept(bad,1090000000),"Future timestamps must fail");
        bad["stamp_ns"]=1000000001;require(!feed.accept(bad,1200000000),"Old data must fail");
        refused=false;try{feed.require_fresh(1100000001);}catch(const std::exception&){refused=true;}
        require(refused,"Invalid packets must never refresh freshness");
        std::cout<<"Estimator contract checks passed"<<std::endl;
    }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}
}
