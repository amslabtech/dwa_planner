#include "detect_stack/detect_stack.h"

DetectStack::DetectStack():local_nh("~")
{
    local_nh.param("HZ"

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_stack");
