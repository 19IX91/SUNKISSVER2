#pragma once
#include "r1_tracking_core.h"

namespace r1_tracking {
// Only ordered, finite and fresh observations are accepted on the local feed.
// This parses estimates; it is not an estimator or IMU integrator.
class PoseFeed {
public:
    explicit PoseFeed(double max_age_s):max_age_ns_(int64_t(max_age_s*1e9)) {
        require(max_age_ns_>0,"Invalid estimator age limit");
    }
    bool accept(const YAML::Node& n,int64_t now) {
        try {
            require(n["version"].as<int>()==1 && n["frame"].as<std::string>()=="pelvis","Wrong estimator protocol/frame");
            auto stamp=n["stamp_ns"].as<int64_t>();
            require(stamp>stamp_ && stamp<=now && now-stamp<=max_age_ns_,"Stale, future or reordered estimate");
            auto p=vector<3>(n["position_w"]),v=vector<3>(n["linear_velocity_b"]);
            auto q=quaternion(n["quaternion_wxyz"]);
            position=p;velocity=v;orientation=q;stamp_=stamp;valid_=true;return true;
        }catch(const std::exception&){return false;}
    }
    void require_fresh(int64_t now) const {
        require(valid_ && now>=stamp_ && now-stamp_<=max_age_ns_,"Fresh pelvis odometry required; missing/stale feed");
    }
    Eigen::Vector3f position,velocity;
    Eigen::Quaternionf orientation;
private:
    int64_t max_age_ns_,stamp_=0;
    bool valid_=false;
};
}
