#include <cmath>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

#include <string>
#include <json-c/json.h>
#include "UDPBroadcast.h"

void handleDebugMessages(const std::string &msg) {ROS_DEBUG(" [VIVE] %s",msg.c_str());}
void handleInfoMessages(const std::string &msg) {ROS_INFO(" [VIVE] %s",msg.c_str());}
void handleErrorMessages(const std::string &msg) {ROS_ERROR(" [VIVE] %s",msg.c_str());}


class vive_entry
{
public:
    int vive_id;
    int type_id;
    int event_type;
    double time;
    double pose[3][4];
};

class VIVEnode
{
  public:
    VIVEnode(int rate);
    ~VIVEnode();
    bool Init();
    void Run();
    void Shutdown();
    bool setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  private:
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    std::vector<double> world_offset_;
    double world_yaw_;

    int getDeviceMatrix(int i, double m[3][4]);
    void vr_update();
    
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;
    ros::ServiceServer set_origin_server_;

    UDPBroadcast ub_;
    std::vector<vive_entry> ve_;
    int click_id_;
};

VIVEnode::VIVEnode(int rate)
  : loop_rate_(rate)
  , nh_()
  , tf_broadcaster_()
  , tf_listener_()
  , world_offset_({0, 0, 0})
  , world_yaw_(0)
  , ub_(8002)
  , click_id_(0)
{
    ve_.resize(16);
  nh_.getParam("/vive/world_offset", world_offset_);
  nh_.getParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

  set_origin_server_ = nh_.advertiseService("/vive/set_origin", &VIVEnode::setOriginCB, this);

  //~ twist1_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist1", 10);
  //~ twist2_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist2", 10);

  return;
}

VIVEnode::~VIVEnode()
{
    return;
}

bool VIVEnode::Init()
{
    return true;
}

void VIVEnode::Shutdown()
{
    // empty
}

int VIVEnode::getDeviceMatrix(int i, double m[3][4])
{
    memcpy(m,ve_[i].pose,12*sizeof(double));

    for (int r=0;r<3;r++)
        for (int c=0;c<4;c++)
        {
            //printf("m[%d][%d]=%f %f\n", r,c,m[r][c], ve_[i].pose[r][c]);
        }
    return ve_[i].type_id;
}

void VIVEnode::vr_update()
{
    std::string s="";
    while ((s = ub_.listen(10)) != "")
    {
        //ROS_INFO("udp:'%s'", s.c_str());
        json_object * jobj = json_tokener_parse(s.c_str());
        if (!jobj) continue;
        vive_entry v;
        v.event_type = 0;
        json_object_object_foreach(jobj, key, val) {
            enum json_type type = json_object_get_type(val);
            if (strcmp(key, "vive_id") == 0) v.vive_id = json_object_get_int(val);
            if (strcmp(key, "type_id") == 0) v.type_id = json_object_get_int(val);
            if (strcmp(key, "eventtype") == 0)
            {
                v.event_type = json_object_get_int(val);
            }
            if (strcmp(key, "time") == 0)    v.time = json_object_get_double(val);
            if (strcmp(key, "pose") == 0)
            {
                array_list* jobj_row = json_object_get_array(val);                
                int rows = json_object_array_length(val);
                if (rows != 3) continue;
                for (int r=0;r<rows;r++)
                {
                    array_list* jobj_col = json_object_get_array((json_object*)jobj_row->array[r]);
                    int cols = json_object_array_length((json_object*)(jobj_row->array[r]));
                    if (cols != 4) continue;
                    for (int c=0;c<cols;c++)
                    {
                        v.pose[r][c] = json_object_get_double((json_object*)(jobj_col->array[c]));
                    }
                    //array_list_free(jobj_col);
                }
                //array_list_free(jobj_row);
            }
        }
        if (v.time==0 || v.type_id==0) continue;
        if (v.event_type == 201) // Button release
        {
            double tf_matrix[3][4];
            int dev_type = getDeviceMatrix(v.vive_id, tf_matrix);
            if (dev_type == 0) continue;
            
            tf::Transform tf;
            tf.setOrigin(tf::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));
            
            tf::Quaternion quat;
            tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                                     tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                                     tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
            
            rot_matrix.getRotation(quat);
            tf.setRotation(quat);
            
            if (abs(click_id_)>15) click_id_=0;
            tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "click"+std::to_string(click_id_++)));
            ROS_INFO("Click at:[%.3f,%.3f,%.3f]", tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]);
            continue;
        }
        if (v.event_type > 0) continue;
        ve_[v.vive_id] = v;
    }
}

bool VIVEnode::setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  double tf_matrix[3][4];
  int index = 1, dev_type;
  while (dev_type != 2) 
  {
    dev_type = getDeviceMatrix(index++, tf_matrix);
  }
  if (dev_type == 0) 
  {
    ROS_WARN(" [VIVE] Coulnd't find controller 1.");
    return false;
  }

  tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                           tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                           tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
  tf::Vector3 c_z;
  c_z = rot_matrix*tf::Vector3(0,0,1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(tf::Vector3(0,0,1).dot(c_z)) + M_PI_2;
  if (c_z[0] < 0) new_yaw = -new_yaw;
  world_yaw_ = -new_yaw;

  tf::Vector3 new_offset;
  tf::Matrix3x3 new_rot;
  new_rot.setRPY(0, 0, world_yaw_);
  new_offset = new_rot*tf::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  world_offset_[0] = new_offset[0];
  world_offset_[1] = new_offset[1];
  world_offset_[2] = new_offset[2];

  nh_.setParam("/vive/world_offset", world_offset_);
  nh_.setParam("/vive/world_yaw", world_yaw_);
  ROS_INFO(" [VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

  return true;
}

void VIVEnode::Run()
{
  double tf_matrix[3][4];

  while (ros::ok())
  {
    // do stuff
    vr_update();

    int hmd_count = 1;
    int controller_count = 1;
    int tracker_count = 1;
    int lighthouse_count = 1;
    for (int i=0; i<6; i++)
    {
      int dev_type = getDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0) continue;

      tf::Transform tf;
      tf.setOrigin(tf::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));

      tf::Quaternion quat;
      tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
                               tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
                               tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

      rot_matrix.getRotation(quat);
      tf.setRotation(quat);

      // It's a HMD
      if (dev_type == 1)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "hmd"+std::to_string(hmd_count++)));
      }
      // It's a controller
      if (dev_type == 2)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "controller"+std::to_string(controller_count++)));
      }
      // It's a tracker
      if (dev_type == 3)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "tracker"+std::to_string(tracker_count++)));
      }
      // It's a lighthouse
      if (dev_type == 4)
      {
        tf_broadcaster_.sendTransform(tf::StampedTransform(tf, ros::Time::now(), "world_vive", "lighthouse"+std::to_string(lighthouse_count++)));
      }

    }

    // Publish corrective transform
    tf::Transform tf_world;
    tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    tf::Quaternion quat_world;
    quat_world.setRPY(M_PI/2, 0, world_yaw_);
    tf_world.setRotation(quat_world);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world, ros::Time::now(), "world", "world_vive"));


    ros::spinOnce();
    loop_rate_.sleep();
  }
}

// Main
int main(int argc, char** argv){
  ros::init(argc, argv, "vive_node_udp");

  VIVEnode nodeApp(20);

  if (!nodeApp.Init())
  {
    nodeApp.Shutdown();
    return 1;
  }

  nodeApp.Run();

  nodeApp.Shutdown();

  return 0;
};
