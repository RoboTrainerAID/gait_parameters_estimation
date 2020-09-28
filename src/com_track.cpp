#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cmath>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <iirob_filters/kalman_filter.h>
#include <gait_parameters_estimation/comass.h>

#define  KEYPOINT_HEAD 0
#define  KEYPOINT_SPINE 1
#define  KEYPOINT_SHOULDER_R 2
#define  KEYPOINT_ELBOW_R 3
#define  KEYPOINT_WRIST_R 4
#define  KEYPOINT_SHOULDER_L 5
#define  KEYPOINT_ELBOW_L 6
#define  KEYPOINT_WRIST_L 7
#define  KEYPOINT_SPINE_BASE 8
#define  KEYPOINT_HIP_R 9
#define  KEYPOINT_KNEE_R 10
#define  KEYPOINT_FOOT_R 11
#define  KEYPOINT_HIP_L 12
#define  KEYPOINT_KNEE_L 13
#define  KEYPOINT_FOOT_L 14
#define  KEYPOINT_EYE_R 15
#define  KEYPOINT_EYE_L 16
#define  KEYPOINT_EAR_R 17
#define  KEYPOINT_EAR_L 18
#define  KEYPOINT_TOE_BIG_L 19
#define  KEYPOINT_TOE_SMALL_L 20
#define  KEYPOINT_HEEL_L 21
#define  KEYPOINT_TOE_BIG_R 22
#define  KEYPOINT_TOE_SMALL_R 23
#define  KEYPOINT_HEEL_R 24


typedef iirob_filters::MultiChannelKalmanFilter<double> KalmanFilter;

KalmanFilter* kFilter_com;
ros::Time previous_T_kal;
geometry_msgs::TransformStamped transformStamped;
ros::Publisher pubCOM;
ros::Publisher pubCOM_point;
ros::Publisher pubHead;
ros::Publisher pubPelvis;
ros::Publisher pubTrunk;
ros::Publisher publuarm;
ros::Publisher pubruarm;
ros::Publisher publlarm;
ros::Publisher pubrlarm;
ros::Publisher pubrhip;
ros::Publisher publhip;
ros::Publisher pubrwrist;
ros::Publisher publwrist;
std::vector<double> CoM(6, 0.0);

//weights taken from Winter anthropometry table
//head com (0), upper arm distal (1) (shoulder), lower arm distal (2) (elbow) ,trunk distal (3), trunk com (4), upper arm com (5), lower arm com (6), pelvis Com (7)
std::vector<double> w = { .081, 0.564, 0.682,0.65, .355, 0.028, 0.022, 0.142};

std::vector<double> calc_limb_com(geometry_msgs::PointStamped higher_weighted, geometry_msgs::PointStamped lower_weighted, double weight) {
    std::vector<double> out(3,0.0);   
    
    out[0] = weight * higher_weighted.point.x + (1 - weight) * lower_weighted.point.x;
    out[1] = weight * higher_weighted.point.y + (1 - weight) * lower_weighted.point.y;
    out[2] = weight * higher_weighted.point.z + (1 - weight) * lower_weighted.point.z;
    
    return out;
}

void tf_point(const geometry_msgs::PolygonStamped& body_points, geometry_msgs::PointStamped& tf_joint, int num_joint) {

  geometry_msgs::PointStamped joint;
  joint.header.stamp = body_points.header.stamp;
  joint.header.frame_id = body_points.header.frame_id;

  joint.point.x = body_points.polygon.points[num_joint].x;
  joint.point.y = body_points.polygon.points[num_joint].y;
  joint.point.z = body_points.polygon.points[num_joint].z;
  tf2::doTransform(joint, tf_joint, transformStamped);
}

//read data points given by body detection topic
void body_points_cb(geometry_msgs::PolygonStamped body_points) {

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  try {
      transformStamped = tfBuffer.lookupTransform("base_link", "camera_body_rgb_optical_frame", ros::Time(0), ros::Duration(2));
  } catch (tf2::TransformException &ex) {
      ROS_WARN("No Transformation for Camera found %s", ex.what());
  }

  std::cout << "received Polygon " << std::endl;
  geometry_msgs::PolygonStamped body_points_tf;
  std::vector<geometry_msgs::PointStamped> joints;
  std::vector<geometry_msgs::PointStamped> CoMs;

  int N = body_points.polygon.points.size();
  for(int i = 0; i < N; ++i) {
      geometry_msgs::PointStamped tf_joint;
      tf_point(body_points, tf_joint, i);
      joints.push_back(tf_joint);
  }
    
  //holds total weight of available calculated coms  
  double total_weight = 0.0;
  std::vector<double> pelvis(3, 0.0);
  pelvis[0] = std::nan("");
  std::vector<double> trunkCOM(3, 0.0);
  // 'larm' lower arm, 'uarm' = upper arm
  std::vector<double> llarmCom(3, 0.0);
  std::vector<double> rlarmCom(3, 0.0);
  std::vector<double> luarmCom(3, 0.0);
  std::vector<double> ruarmCom(3, 0.0);
  std::vector<bool> used_joints(8,false);
  
  //each joint must be checked for Nan, if nan point is not used
  if (!std::isnan(joints[KEYPOINT_HIP_R].point.x) && !std::isnan(joints[KEYPOINT_HIP_L].point.x))
  {    
    pelvis = calc_limb_com(joints[KEYPOINT_HIP_R], joints[KEYPOINT_HIP_L], 0.5);
    used_joints[7] = true;
    total_weight += w[7];

    geometry_msgs::PointStamped pelvP;
    pelvP.header.stamp = body_points.header.stamp;
    pelvP.header.frame_id = "base_link";

    pelvP.point.x = pelvis[0];
    pelvP.point.y = pelvis[1];
    pelvP.point.z = pelvis[2];

    pubPelvis.publish(pelvP);
    publhip.publish(joints[KEYPOINT_HIP_L]);
    pubrhip.publish(joints[KEYPOINT_HIP_R]);
  }
  else if (!std::isnan(joints[KEYPOINT_HIP_R].point.x))
  {
    pelvis[0] = joints[KEYPOINT_HIP_R].point.x;
    pelvis[1] = joints[KEYPOINT_HIP_R].point.y; 
    pelvis[2] = joints[KEYPOINT_HIP_R].point.z;
    used_joints[7] = true; 
    total_weight += w[7];

    pubrhip.publish(joints[KEYPOINT_HIP_R]);
  }
  else if (!std::isnan(joints[KEYPOINT_HIP_L].point.x))
  {
    pelvis[0] = joints[KEYPOINT_HIP_L].point.x;
    pelvis[1] = joints[KEYPOINT_HIP_L].point.y; 
    pelvis[2] = joints[KEYPOINT_HIP_L].point.z; 
    used_joints[7] = true;
    total_weight += w[7];

    publhip.publish(joints[KEYPOINT_HIP_L]);
  }

  if (!std::isnan(pelvis[0]) && !std::isnan(joints[KEYPOINT_HEAD].point.x))
  {
    trunkCOM[0] = w[3] * pelvis[0] + (1 - w[3]) * joints[KEYPOINT_HEAD].point.x;
    trunkCOM[1] = w[3] * pelvis[1] + (1 - w[3]) * joints[KEYPOINT_HEAD].point.y;
    trunkCOM[2] = w[3] * pelvis[2] + (1 - w[3]) * joints[KEYPOINT_HEAD].point.z;
    used_joints[5] = true;
    total_weight += w[4];

    geometry_msgs::PointStamped trunkP;
    trunkP.header.stamp = body_points.header.stamp;
    trunkP.header.frame_id = "base_link";

    trunkP.point.x = trunkCOM[0];
    trunkP.point.y = trunkCOM[1];
    trunkP.point.z = trunkCOM[2];
    
    pubTrunk.publish(trunkP);
  }
  else if (!std::isnan(pelvis[0]) && !!std::isnan(joints[KEYPOINT_SHOULDER_L].point.x) && !!std::isnan(joints[KEYPOINT_SHOULDER_R].point.x))
  {
    std::vector<double> sh_middle(3, 0.0);
    sh_middle = calc_limb_com(joints[KEYPOINT_SHOULDER_L], joints[KEYPOINT_SHOULDER_R], 0.5);
  
    trunkCOM[0] = w[3] * pelvis[0] + (1 - w[3]) * sh_middle[0];
    trunkCOM[1] = w[3] * pelvis[1] + (1 - w[3]) * sh_middle[1];
    trunkCOM[2] = w[3] * pelvis[2] + (1 - w[3]) * sh_middle[2];
    used_joints[5] = true;
    total_weight += w[4];
  }
  if (!std::isnan(joints[KEYPOINT_HEAD].point.x))
  {
    used_joints[0] = true;
    total_weight += w[0];

    geometry_msgs::PointStamped headP;
    headP.header.stamp = body_points.header.stamp;
    headP.header.frame_id = "base_link";

    headP.point.x = joints[KEYPOINT_HEAD].point.x;
    headP.point.y = joints[KEYPOINT_HEAD].point.y;
    headP.point.z = joints[KEYPOINT_HEAD].point.z;
    
    pubHead.publish(headP);
  }
  if (!std::isnan(joints[KEYPOINT_ELBOW_R].point.x) && !std::isnan(joints[KEYPOINT_WRIST_R].point.x))
  {
    rlarmCom = calc_limb_com(joints[KEYPOINT_ELBOW_R], joints[KEYPOINT_WRIST_R], w[3]);
    used_joints[2] = true;
    total_weight += w[6];

    geometry_msgs::PointStamped rlarmP;
    rlarmP.header.stamp = body_points.header.stamp;
    rlarmP.header.frame_id = "base_link";

    rlarmP.point.x = rlarmCom[0];
    rlarmP.point.y = rlarmCom[1];
    rlarmP.point.z = rlarmCom[2];

    pubrlarm.publish(rlarmP);
    pubrwrist.publish(joints[KEYPOINT_WRIST_R]);
  }
  if (!std::isnan(joints[KEYPOINT_ELBOW_L].point.x) && !std::isnan(joints[KEYPOINT_WRIST_L].point.x))
  {
    llarmCom = calc_limb_com(joints[KEYPOINT_ELBOW_L], joints[KEYPOINT_WRIST_L], w[2]);
    used_joints[1] = true;
    total_weight += w[6];

    geometry_msgs::PointStamped llarmP;
    llarmP.header.stamp = body_points.header.stamp;
    llarmP.header.frame_id = "base_link";

    llarmP.point.x = llarmCom[0];
    llarmP.point.y = llarmCom[1];
    llarmP.point.z = llarmCom[2];

    publlarm.publish(llarmP);
    publwrist.publish(joints[KEYPOINT_WRIST_L]);
  }
  if (!std::isnan(joints[KEYPOINT_SHOULDER_L].point.x) && !std::isnan(joints[KEYPOINT_ELBOW_L].point.x))
  {
    luarmCom = calc_limb_com(joints[KEYPOINT_SHOULDER_L], joints[KEYPOINT_ELBOW_L], w[1]);
    used_joints[3] = true;
    total_weight += w[5];

    // std::cout << "Left Elbow z : " << joints[KEYPOINT_SHOULDER_L].point.z << std::endl;
    // std::cout << "luarmcom z : " << luarmCom[2] << std::endl;

    geometry_msgs::PointStamped luarmP;
    luarmP.header.stamp = body_points.header.stamp;
    luarmP.header.frame_id = "base_link";

    luarmP.point.x = luarmCom[0];
    luarmP.point.y = luarmCom[1];
    luarmP.point.z = luarmCom[2];

    publuarm.publish(luarmP);
  }
  if (!std::isnan(joints[KEYPOINT_SHOULDER_R].point.x) && !std::isnan(joints[KEYPOINT_ELBOW_R].point.x))
  {
    ruarmCom = calc_limb_com(joints[KEYPOINT_SHOULDER_R], joints[KEYPOINT_ELBOW_R], w[1]);
    used_joints[4] = true;
    total_weight += w[5];

    geometry_msgs::PointStamped ruarmP;
    ruarmP.header.stamp = body_points.header.stamp;
    ruarmP.header.frame_id = "base_link";

    ruarmP.point.x = ruarmCom[0];
    ruarmP.point.y = ruarmCom[1];
    ruarmP.point.z = ruarmCom[2];

    pubruarm.publish(ruarmP);
  }
  //If total weight is zero --> No Limbs found;
  if (total_weight > 0.0) {
    double x,y,z;

    // std::cout << "Total weight is : " << total_weight << std::endl;
    // std::cout << "adjusted weight for ruarm is : " << (w[5] / total_weight);
    // std::cout << "ruarm.x : " << ruarmCom[0] << "luarm.x : " << luarmCom[0] << std::endl;
    // std::cout << "rlarm.x : " << rlarmCom[0] << "llarm.x : " << llarmCom[0] << std::endl;
    // std::cout << "Head.x : " << joints[KEYPOINT_HEAD].point.x << std::endl;
    // std::cout << "Trunk.x : " << trunkCOM[0] << std::endl;
    x = ruarmCom[0] * used_joints[4] * (w[5] / total_weight) + luarmCom[0] * used_joints[3] * (w[5] / total_weight);
    x += llarmCom[0] * used_joints[1] * (w[6] / total_weight) + rlarmCom[0] * used_joints[2] * (w[6] / total_weight);
    if (!std::isnan(joints[KEYPOINT_HEAD].point.x))
      x += joints[KEYPOINT_HEAD].point.x * used_joints[0] * (w[0] / total_weight);
    x += trunkCOM[0] * used_joints[5] * (w[4] / total_weight);
    x += used_joints[7] * pelvis[0] * (w[7] / total_weight);

    // std::cout << "ruarm.y : " << ruarmCom[1] << "luarm.y : " << luarmCom[1] << std::endl;
    // std::cout << "rlarm.y : " << rlarmCom[1] << "llarm.y : " << llarmCom[1] << std::endl;
    // std::cout << "Head.y : " << joints[KEYPOINT_HEAD].point.y << std::endl;
    // std::cout << "Trunk.y : " << trunkCOM[1] << std::endl;
    y = ruarmCom[1] * used_joints[4] * (w[5] / total_weight) + luarmCom[1] * used_joints[3] * (w[5] / total_weight);
    y += llarmCom[1] * used_joints[1] * (w[6] / total_weight) + rlarmCom[1] * used_joints[2] * (w[6] / total_weight);
    if (!std::isnan(joints[KEYPOINT_HEAD].point.y))
      y += joints[KEYPOINT_HEAD].point.y * used_joints[0] * (w[0] / total_weight);
    y += trunkCOM[1] * used_joints[5] * (w[4] / total_weight);
    y += used_joints[7] * pelvis[1] * (w[7] / total_weight);

    // std::cout << "ruarm.z : " << ruarmCom[2] << "luarm.z : " << luarmCom[2] << std::endl;
    // std::cout << "rlarm.z : " << rlarmCom[2] << "llarm.z : " << llarmCom[2] << std::endl;
    // std::cout << "Head.z : " << joints[KEYPOINT_HEAD].point.z << std::endl;
    // std::cout << "Trunk.z : " << trunkCOM[2] << std::endl;
    z = ruarmCom[2] * used_joints[4] * (w[5] / total_weight) + luarmCom[2] * used_joints[3] * (w[5] / total_weight);
    z += llarmCom[2] * used_joints[1] * (w[6] / total_weight) + rlarmCom[2] * used_joints[2] * (w[6] / total_weight);
      if (!std::isnan(joints[KEYPOINT_HEAD].point.y))
    z += joints[KEYPOINT_HEAD].point.z * used_joints[0] * (w[0] / total_weight);
    z += trunkCOM[2] * used_joints[5] * (w[4] / total_weight);
    z += used_joints[7] * pelvis[2] * (w[7] / total_weight);

    CoM[0] = x;
    CoM[1] = y;
    CoM[2] = z;

    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
    {
     if (!kFilter_com->isInitializated())
     {
      std::vector<double> init_pos;
      init_pos.push_back(CoM[0]);
      init_pos.push_back(CoM[1]);
      init_pos.push_back(CoM[2]);
      init_pos.push_back(0);
      init_pos.push_back(0);
      init_pos.push_back(0);
      if (!kFilter_com->configure(init_pos)) ROS_ERROR("KalmanFilter  couldn't configure!");
      previous_T_kal = ros::Time::now();
     }
     else
     {
      double delta_t = (ros::Time::now() - previous_T_kal).toSec();
      previous_T_kal = ros::Time::now();

      std::vector<double> data_in;
      data_in.push_back(x);
      data_in.push_back(y);
      data_in.push_back(z);

      kFilter_com->update(data_in, CoM, delta_t, true);

      std::cout << "com after kf size : "<< CoM.size() << "CoM velocity x: ? " << CoM[3] << "CoM velocity z: ? " << CoM[5] << std::endl;
     }

      gait_parameters_estimation::comass msg;
      geometry_msgs::PointStamped msg_point;
      msg.header.stamp = body_points.header.stamp;
      msg.header.frame_id = "base_link";
      msg.point.x = CoM[0];
      msg.point.y = CoM[1];
      msg.point.z = CoM[2];
      msg.velocity.linear.x = CoM[3];
      msg.velocity.linear.y = CoM[4];
      msg.velocity.linear.z = CoM[5];

      msg_point.header = msg.header;
      msg_point.point = msg.point;

      pubCOM.publish(msg);
      pubCOM_point.publish(msg_point);
   }
    std::cout << "CoM x: " << x << " y : " << y << " z : " << z << std::endl;
    std::cout << "KF : CoM x: " << CoM[0] << " y : " << CoM[1] << " z : " << CoM[2] << std::endl;
  }



}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gait_upper_body");
  ros::NodeHandle nh;


  kFilter_com = new KalmanFilter();

  ros::Subscriber sub = nh.subscribe ("/human_body_detection/points", 10, body_points_cb);
  pubCOM = nh.advertise<gait_parameters_estimation::comass>("/gait/CoM", 5);
  pubCOM_point = nh.advertise<geometry_msgs::PointStamped>("/gait/CoM_point", 5);
  pubHead = nh.advertise<geometry_msgs::PointStamped>("/Head", 5);
  pubPelvis = nh.advertise<geometry_msgs::PointStamped>("/Pelvis", 5);
  pubTrunk = nh.advertise<geometry_msgs::PointStamped>("/Trunk", 5);
  publuarm = nh.advertise<geometry_msgs::PointStamped>("/luarm", 5);
  pubruarm = nh.advertise<geometry_msgs::PointStamped>("/ruarm", 5);
  publlarm = nh.advertise<geometry_msgs::PointStamped>("/llarm", 5);
  pubrlarm = nh.advertise<geometry_msgs::PointStamped>("/rlarm", 5);
  publhip = nh.advertise<geometry_msgs::PointStamped>("/lhip", 5);
  pubrhip = nh.advertise<geometry_msgs::PointStamped>("/rhip", 5);
  pubrwrist = nh.advertise<geometry_msgs::PointStamped>("/rwrist", 5);
  publwrist = nh.advertise<geometry_msgs::PointStamped>("/lwrist", 5);
  ros::spin();
  return 0;
}
