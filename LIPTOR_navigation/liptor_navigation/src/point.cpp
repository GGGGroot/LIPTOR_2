#include <ros/ros.h>
#include <geometry_msgs/Point.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "point");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<geometry_msgs::Point>("RubbishCoordinate", 10);
    
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        printf("point\n");
        geometry_msgs::Point point;
        point.x = 2.1;
        point.y = 0.5;
        point.z = -0.00143;
        pub.publish(point);

        loop_rate.sleep();

    }

    return 0;
}
