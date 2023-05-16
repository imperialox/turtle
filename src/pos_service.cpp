#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tb3_projet_LAKHAL_MEZOUAR/pos_service.h>

using namespace tb3_projet_LAKHAL_MEZOUAR;
using namespace ros;
using namespace std;

bool pos_service_callback(pos_service::Request &req,
                          pos_service::Response &res) {

  // destination_x = req.x;
  // destination_y = req.y;

  ROS_INFO("requete recu avec les arguments x  =  %f et y =  %f ", req.x,
           req.y);
  res.retour = true;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pos_service");
  ros::NodeHandle nh;

  ServiceServer service =
      nh.advertiseService("pos_service", pos_service_callback);

  ROS_INFO("mon service is ready !");

  spin();
}