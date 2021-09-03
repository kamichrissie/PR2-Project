#include "laser_line_extraction/base_controller.h"
#include "laser_line_extraction/line_extraction_ros.h"
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_line_extraction/line_extraction.h"
#include "laser_line_extraction/line.h"
#include "laser_line_extraction/ellipse.h"
#include <tf2/utils.h>


using namespace line_extraction;

// this callback will be invoked periodically by ros::Timer loop_timer
void timerCallback(BaseController* controller, const ros::TimerEvent&)
{
  controller->loop();
  //
}

void goalCallback(BaseController* controller, const geometry_msgs::PoseStampedConstPtr& goal)
{
  geometry_msgs::Pose2D goal_2d;
  goal_2d.x = goal->pose.position.x;
  goal_2d.y = goal->pose.position.y;
  goal_2d.theta = tf2::getYaw(goal->pose.orientation);
  controller->setGoal(goal_2d);
}

void goalCallback2(BaseController* controller, double goalx, double goaly)
{
  geometry_msgs::Pose2D goal_2d;
  goal_2d.x = goalx;
  goal_2d.y = goaly;
  goal_2d.theta = 0;//orientation
  controller->setGoal(goal_2d);
}


/*EllipseCoordinates::EllipseCoordinates(double px, double py, double palpha, double pr1, double pr2){

  dx =px;
  dy = py;
  alpha = palpha;
  r1 = pr1;
  r2 = pr2;

}*/


void runBaseController(int nwid, int nhei){

  ros::NodeHandle node_handle;

  // controller WITHOUT modulator
  BaseController controller(&node_handle);

  geometry_msgs::Pose2D pd;
  pd.x = nwid;
  pd.y = nhei;
  pd.theta = 1;
  controller.setGoal(pd);

  ros::Subscriber goal_subscriber = node_handle.subscribe<geometry_msgs::PoseStamped> (
        "move_base_simple/goal", 10,
        boost::bind(goalCallback, &controller, _1));


  ros::Timer loop_timer = node_handle.createTimer(ros::Duration(0.05),
                                                  boost::bind(timerCallback, &controller, _1));

  ros::spin();


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_movement");

  ros::NodeHandle node_handle;

  // controller WITHOUT modulator
  BaseController controller(&node_handle);


  /*LineExtractionROS myLine;
  std::vector<EllipseCoordinates> currentellipses;
  currentellipses = myLine.ellipses_pass();*/

  std::vector<EllipseCoordinates> currentellipses;

  //-----------------------------------------
  // NOTE: if you want to use the modulator, you should create controller like this instead:
  // std::vector<Ellipse> ellipses;
  // push_back() your ellipses here
  // best practice: use smart pointers
  // EllipseModulator modulator(ellipses);
  // BaseController controller(&node_handle, &modulator);
  //-----------------------------------------

  // will subscribe to this
  // http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack#A2D_Nav_Goal

  // you need to use boost::bind to add an extra argument:
  // https://answers.ros.org/question/12045/how-to-deliver-arguments-to-a-callback-function/

  ros::Subscriber goal_subscriber = node_handle.subscribe<geometry_msgs::PoseStamped> (
        "move_base_simple/goal", 10,
        boost::bind(goalCallback, &controller, _1));



  //ros::Subscriber ellipse_subscriber = node_handle.subscribe<line_extraction::Ellipse> ("ellipses", 10, ellipseCallback());

  /*ros::Subscriber ellipse_subscriber =  node_handle.subscribe<geometry_msgs::PoseStamped> (
        "line_extraction_ros/",   line_publisher_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 1));
*/


  // this will call the controller loop periodically.
  // Much better than your own loop.
  // http://wiki.ros.org/roscpp/Overview/Timers
  ros::Timer loop_timer = node_handle.createTimer(ros::Duration(0.05),
                                                  boost::bind(timerCallback, &controller, _1));

  ros::spin();

  return 0;
}
