#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <tb3_projet_LAKHAL_MEZOUAR/MoveAction.h>

using namespace tb3_projet_LAKHAL_MEZOUAR;
using namespace actionlib;
using namespace ros;

typedef SimpleActionServer<MoveAction> Server;

void execute(const MoveGoalConstPtr &goal, Server *as) {

  MoveResult res;
  MoveFeedback fb;

  ROS_INFO("goal = %f ! ", goal->pos_x);

  int count = 0;
  ros::Rate loop_rate(1); // 1 hz

  bool success = true;
  while (count < goal->pos_x) {
    if (as->isPreemptRequested()) // check that preempt has not been requested
                                  // by the client
    {
      ROS_INFO("The goal has been cancelled/preempted");
      as->setPreempted(); // set_preempted function will signals that the//
                          // action has been preempted by user request.
      success = false;
      break;
    }

    count++;
    fb.pos_x_intermediaire = count;
    ROS_INFO("POS intermediaire = %f", fb.pos_x_intermediaire);
    as->publishFeedback(fb);

    loop_rate.sleep();
  }

  if (success) {
    res.reached = true;
    ROS_INFO("objectif atteint !");
    as->setSucceeded(res);
  }

  // A compléter (voir slide serveur d'action 2/2)
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "serveur_action");
  ros::NodeHandle nh;

  Server server(nh, "moving", boost::bind(&execute, _1, &server), false);

  server.start();

  ROS_INFO("serveur demarre et en attente d'un goal ! ");

  spin();

  return 0;
}