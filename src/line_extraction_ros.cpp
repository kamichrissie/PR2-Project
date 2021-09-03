#include "laser_line_extraction/line_extraction_ros.h"
#include "laser_line_extraction/ellipse.h"
#include "laser_line_extraction/drive_base.h"
#include "ellipse.cpp"
//#include "laser_line_extraction/modulation.cpp"
#include "laser_line_extraction/base_controller.h"
#include "base_controller.cpp"
#include "test_base_controller.cpp"
#include "drive_base.cpp"
#include <cmath>
#include <ros/console.h>
//#include <move_group.h>


namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LineExtractionROS::LineExtractionROS(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
  nh_(nh),
  nh_local_(nh_local),
  data_cached_(false)
{
  loadParameters();
  line_publisher_ = nh_.advertise<laser_line_extraction::LineSegmentList>("line_segments", 1);
  scan_subscriber_ = nh_.subscribe(scan_topic_, 1, &LineExtractionROS::laserScanCallback, this);
  if (pub_markers_)
  {
    marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("line_markers", 1);
  }
}

LineExtractionROS::~LineExtractionROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////



void LineExtractionROS::run()
{

  ROS_DEBUG_STREAM("LineExROS - run function");
  // Extract the lines
  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  // Populate message
  laser_line_extraction::LineSegmentList msg;
  populateLineSegListMsg(lines, msg);
  


  //line_extraction::Ellipse calculated_ellipse_ = line_extraction::Ellipse(px,py,alpha_ellipse,r2,r1);


  //setGoal();
  ROS_DEBUG("TEST!1\n");
  runBaseController(3,3);
  ROS_DEBUG("TEST!2\n");
  runBaseController(5,5);
  ROS_DEBUG("TEST!3\n");
  //runMyBaseController(3,4.5);
/*

/*
std::vector<EllipseCoordinates> LineExtractionROS::ellipses_pass(const std::vector<Line> &lines){

  std::vector<EllipseCoordinates> all_ellipses;// = new std::vector<EllipseCoordinates>;

  for(std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); cit++){
    double startx = cit -> getStart()[0];
    double starty = cit -> getStart()[1];
    double endx = cit -> getEnd()[0];
    double endy = cit -> getEnd()[1];
    geometry_msgs::Point p_start;
    geometry_msgs::Point testpoint;
    geometry_msgs::Point p_end;
    p_start.x = startx;
    p_start.y = starty;
    p_start.z = 0;

    double distx = (endx- startx)/2.;
    double disty = (endy- starty)/2.;
    double r2 = sqrt(pow(distx,2) + pow(disty,2));
    // Choose radius!!!
    double r1 = 0.4;

    //10°-Schritte
    for(int i=0; i < 18; i++){
      double alpha = 10. * i;
      testpoint.x = (startx + distx) + r1 * cos(alpha * 180/M_PI);
      testpoint.y = (startx + distx) + r2 * sin(alpha * 180/M_PI);
      testpoint.z = 0;
      double alpha_ellipse = atan(disty/distx);

      EllipseCoordinates ellipse(testpoint.x, testpoint.y, alpha_ellipse, r2, r1);
      all_ellipses.push_back(ellipse);
      //all_ellipses[i] = ellipses_coordinates(testpoint.x, testpoint.y, alpha_ellipse, r2, r1);
    }
  }

    return all_ellipses;
    ROS_DEBUG_NAMED("test_only", "HEllo TEST!");

}
*/

}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;
  bool pub_markers;

  nh_local_.param<std::string>("frame_id", frame_id, "base_laser_link");
  frame_id_ = "base_laser_link"; //frame_id;
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<std::string>("scan_topic", scan_topic, "scan");
  scan_topic_ = scan_topic;
  ROS_DEBUG("scan_topic: %s", scan_topic_.c_str());

  nh_local_.param<bool>("publish_markers", pub_markers, false);
  pub_markers_ = pub_markers;
  ROS_DEBUG("publish_markers: %s", pub_markers ? "true" : "false");

  // Parameters used by the line extraction algorithm

  double bearing_std_dev, range_std_dev, least_sq_angle_thresh, least_sq_radius_thresh,
      max_line_gap, min_line_length, min_range, min_split_dist, outlier_dist;
  int min_line_points;

  nh_local_.param<double>("bearing_std_dev", bearing_std_dev, 1e-3);
  line_extraction_.setBearingVariance(bearing_std_dev * bearing_std_dev);
  ROS_DEBUG("bearing_std_dev: %f", bearing_std_dev);

  nh_local_.param<double>("range_std_dev", range_std_dev, 0.02);
  line_extraction_.setRangeVariance(range_std_dev * range_std_dev);
  ROS_DEBUG("range_std_dev: %f", range_std_dev);

  nh_local_.param<double>("least_sq_angle_thresh", least_sq_angle_thresh, 1e-4);
  line_extraction_.setLeastSqAngleThresh(least_sq_angle_thresh);
  ROS_DEBUG("least_sq_angle_thresh: %f", least_sq_angle_thresh);
  
  nh_local_.param<double>("least_sq_radius_thresh", least_sq_radius_thresh, 1e-4);
  line_extraction_.setLeastSqRadiusThresh(least_sq_radius_thresh);
  ROS_DEBUG("least_sq_radius_thresh: %f", least_sq_radius_thresh);

  nh_local_.param<double>("max_line_gap", max_line_gap, 0.4);
  line_extraction_.setMaxLineGap(max_line_gap);
  ROS_DEBUG("max_line_gap: %f", max_line_gap);

  nh_local_.param<double>("min_line_length", min_line_length, 0.5);
  line_extraction_.setMinLineLength(min_line_length);
  ROS_DEBUG("min_line_length: %f", min_line_length);

  nh_local_.param<double>("min_range", min_range, 0.4);
  line_extraction_.setMinRange(min_range);
  ROS_DEBUG("min_range: %f", min_range);

  nh_local_.param<double>("min_split_dist", min_split_dist, 0.05);
  line_extraction_.setMinSplitDist(min_split_dist);
  ROS_DEBUG("min_split_dist: %f", min_split_dist);

  nh_local_.param<double>("outlier_dist", outlier_dist, 0.05);
  line_extraction_.setOutlierDist(outlier_dist);
  ROS_DEBUG("outlier_dist: %f", outlier_dist);

  nh_local_.param<int>("min_line_points", min_line_points, 9);
  line_extraction_.setMinLinePoints(static_cast<unsigned int>(min_line_points));
  ROS_DEBUG("min_line_points: %d", min_line_points);

  ROS_DEBUG("*************************************");
}


///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::populateLineSegListMsg(const std::vector<Line> &lines,
                                               laser_line_extraction::LineSegmentList &line_list_msg)
{
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    laser_line_extraction::LineSegment line_msg;
    line_msg.angle = cit->getAngle();
    line_msg.radius = cit->getRadius();
    line_msg.covariance = cit->getCovariance();
    line_msg.start = cit->getStart();
    line_msg.end = cit->getEnd();
    line_list_msg.line_segments.push_back(line_msg);
  }
  line_list_msg.header.frame_id = frame_id_;
  line_list_msg.header.stamp = ros::Time::now();
}

void LineExtractionROS::populateMarkerMsg(const std::vector<Line> &lines, 
                                          visualization_msgs::Marker &marker_msg){

  marker_msg.ns = "line_extraction";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::POINTS;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.scale.x = 0.1;
  marker_msg.scale.y = 0.1;
  marker_msg.scale.z = 0.1;
  marker_msg.color.r = 1.0;
  marker_msg.color.g = 0.0;
  marker_msg.color.b = 0.0;
  marker_msg.color.a = 1.0;
  for (std::vector<Line>::const_iterator cit = lines.begin(); cit != lines.end(); ++cit)
  {
    double startx = cit->getStart()[0];
    double starty = cit->getStart()[1];
    double endx = cit->getEnd()[0];
    double endy = cit->getEnd()[1];
    geometry_msgs::Point p_start;
    p_start.x = startx;
    p_start.y = starty;
    p_start.z = 0;
    marker_msg.points.push_back(p_start);

    ROS_DEBUG_STREAM("Test" << "popMarkerMsg");

    double distx = (endx - startx)/2;
    double disty = (endy - starty)/2;
    double r2 = sqrt(pow(distx,2) + pow(disty,2));
    double r1 = 0.4;
    geometry_msgs::Point testpoint;
    for(int i = 0; i < 18; i++){
      double alpha = 10. * i;
      testpoint.x = (startx + distx) + r1 * cos(alpha* 180/M_PI);
      testpoint.y = (starty + disty) + (r2+0.2) * sin(alpha* 180/M_PI);
      testpoint.z = 0;
      marker_msg.points.push_back(testpoint);
      ROS_DEBUG_STREAM("Testpoint" << i <<": " << testpoint.x << " "<< testpoint.y <<"\n");
    }

    testpoint.x = startx + distx;
    testpoint.y = starty + disty;
    testpoint.z = 0;
    marker_msg.points.push_back(testpoint);


    double px = startx + distx/2;
    double py = starty + disty/2;
    double alpha_ellipse = atan(disty/distx);

    //How to save it as Ellipse-type?
    //line_extraction::Ellipse calculated_ellipse_ = line_extraction::Ellipse(px,py,alpha_ellipse,r2,r1);

    geometry_msgs::Point p_end;
    p_end.x = cit->getEnd()[0];
    p_end.y = cit->getEnd()[1];
    p_end.z = 0;
    marker_msg.points.push_back(p_end);
  }
  marker_msg.header.frame_id = frame_id_;
  marker_msg.header.stamp = ros::Time::now();
  //done in RUN-fct
  //marker_publisher_.publish(marker_msg);

}


/*
void LineExtractionROS::populateEllipseMarkers(const std::vector<Line> &lines,
                                          visualization_msgs::Marker &marker_msg){


}*/

///////////////////////////////////////////////////////////////////////////////
// Calculate Ellipses to pass them to modulation
///////////////////////////////////////////////////////////////////////////////





///////////////////////////////////////////////////////////////////////////////
// Cache data on first LaserScan message received
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::cacheData(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  std::vector<double> bearings, cos_bearings, sin_bearings;
  std::vector<unsigned int> indices;
  const std::size_t num_measurements = std::ceil(
        (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  for (std::size_t i = 0; i < num_measurements; ++i)
  {
    const double b = scan_msg->angle_min + i * scan_msg->angle_increment;
    bearings.push_back(b);
    cos_bearings.push_back(cos(b));
    sin_bearings.push_back(sin(b));
    indices.push_back(i);
  }

  line_extraction_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  ROS_DEBUG("Data has been cached.");


}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LineExtractionROS::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  if (!data_cached_)
  {
    cacheData(scan_msg);
    data_cached_ = true;
  }

  std::vector<double> scan_ranges_doubles(scan_msg->ranges.begin(), scan_msg->ranges.end());
  line_extraction_.setRangeData(scan_ranges_doubles);


  std::vector<Line> lines;
  line_extraction_.extractLines(lines);

  //set Ellipses
  /*for(int i =0; i < lines.size(); i++){
    double startx = lines[i].getStart()[0];
    double starty = lines[i].getStart()[1];
    double endx = lines[i].getEnd()[0];
    double endy = lines[i].getEnd()[1];
    double x = startx + (endx-startx)/2.;
    double y = starty + (endy-starty)/2.;
    double r1 = sqrt(pow(((endx-startx)/2.),2) + pow(((endy-starty)/2.),2));
    double alpha = 0.;

    Ellipse ellipse_t(x,y,alpha,r1,0.4); // choose radius!

    ROS_DEBUG_STREAM("Ellipse:" << i << ellipse_t.getPPoint()[0] << ellipse_t.getPPoint()[1] << "\n");
  }
*/

  //laser_line_extraction::LineSegmentList msg;
  //populateLineSegListMsg(lines, msg);
  // Publish the lines
  //line_publisher_.publish(msg);


  // Also publish markers if parameter publish_markers is set to true
  //if (pub_markers_)
  {
    visualization_msgs::Marker marker_msg;
    populateMarkerMsg(lines, marker_msg);
   // marker_publisher_.publish(marker_msg);
  }


}

} // namespace line_extraction

