#include "arrow_projection/PathProjection.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "path_projection_node");

    PathProjection p;
    p.spin();

    return 0;
}