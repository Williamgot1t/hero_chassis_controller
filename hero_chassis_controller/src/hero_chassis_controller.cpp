#include <hero_chassis_controller/hero_chassis_controller.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace hero_chassis_controller {

HeroChassisController::HeroChassisController()
{}

HeroChassisController::~HeroChassisController()
{
 
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
  
  std::string joint_name;
  if (!n.getParam("left1_joint", joint_name))
   {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());               //从参数服务器 得到四个轴的joint名称
    return false;
  }


  left1_joint_ = robot->getHandle(joint_name);
  
  if (!n.getParam("left2_joint", joint_name))
   {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  left2_joint_ = robot->getHandle(joint_name);
  
  
  if (!n.getParam("right1_joint", joint_name))
   {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());
    return false;
  }


  right1_joint_ = robot->getHandle(joint_name);
  
  
  if (!n.getParam("right2_joint", joint_name))
   {
    ROS_ERROR("No joint given (namespace: %s)", n.getNamespace().c_str());               //获取完毕
    return false;
  }


  right2_joint_ = robot->getHandle(joint_name);
  

  
  
  
  if (!pid1_controller_.init(ros::NodeHandle(n, "pid1")))
    return false;
  if (!pid2_controller_.init(ros::NodeHandle(n, "pid2")))                          //从参数服务器中 得到四个轴的pid参数
    return false;
  if (!pid3_controller_.init(ros::NodeHandle(n, "pid3")))
    return false;
  if (!pid4_controller_.init(ros::NodeHandle(n, "pid4")))
    return false; 
  
  
  
  n.getParam("wheelBase", wheelBase);                                                     //从参数服务器中获取底盘的参数
  n.getParam("trackWidth", trackWidth);
  n.getParam("radius", radius);
  
  g_odomPub = n.advertise<nav_msgs::Odometry>("/odom",10);
  velSub = n.subscribe("/cmd_vel",1,&HeroChassisController::velCallBack,this);            //速度订阅和 里程发布
  
  g_odomBroadcaster = new tf2_ros::TransformBroadcaster();
  
  
  lastTime = ros::Time::now();
  
  ROS_INFO("HeroChassisController");
  return true;
}

void HeroChassisController::velCallBack(const geometry_msgs::TwistConstPtr msg){          //设置回调函数获取速度信息

    g_vel = *msg;    
}


void HeroChassisController::starting(const ros::Time& time)
{ 
  pid1_controller_.reset();
  pid2_controller_.reset();                                                               // pid初始化
  pid3_controller_.reset();
  pid4_controller_.reset();
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{

 
  calWheelSpeed(period);       //运动学逆解                                                //把update改成正逆运动学的计算

 
  solve();                     //计算运动学正解 计算历程

}

void HeroChassisController::solve(){                                        

    double dx; 
    double dy;                                                                           //计算运动学正解 计算历程
    double dth; 

    
    double dt = (ros::Time::now() - lastTime).toSec();
    lastTime = ros::Time::now();
    if(dt > 2){
         //时间异常
         return;
    }
    

    double v1 = right1_joint_.getVelocity();
    double v2 = left1_joint_.getVelocity();
    double v3 = left2_joint_.getVelocity();
    double v4 = right2_joint_.getVelocity();
    
    dx = radius*0.25*(v1+v2+v3+v4);
    dy = radius*0.25*(-v3+v4+v2-v1);
    dth = radius*(-v3+v4-v2+v1)/((trackWidth/2+wheelBase/2)*4);
    
    x += dx*cos(th)*dt - dy*sin(th)*dt;
    y += dx*sin(th)*dt + dy*cos(th)*dt; 
    th += dth*dt;
    
    
    tf2::Quaternion q;
    q.setRPY(0,0,th);
    
    nav_msgs::Odometry odom;                                                             //发布里程信息需要把角度变为四元数
    odom.child_frame_id = "base_link"; 
    odom.header.frame_id = "odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation.w = q.getW();
    odom.pose.pose.orientation.x = q.getX();
    odom.pose.pose.orientation.y = q.getY();
    odom.pose.pose.orientation.z = q.getZ();
    
    odom.twist.twist.linear.x = dx;
    odom.twist.twist.linear.y = dy;
    odom.twist.twist.angular.z = dth;
    g_odomPub.publish(odom);
    
    
 
    geometry_msgs::TransformStamped odomTrans;                                          //发布tf变换
    odomTrans.child_frame_id = "base_link";
    odomTrans.header.frame_id = "odom";
    odomTrans.header.stamp  = ros::Time::now();
    odomTrans.transform.rotation.w = q.getW();
    odomTrans.transform.rotation.x = q.getX();
    odomTrans.transform.rotation.y = q.getY();
    odomTrans.transform.rotation.z = q.getZ();
    
    odomTrans.transform.translation.x = x;
    odomTrans.transform.translation.y = y;
    odomTrans.transform.translation.z = 0;
    
    g_odomBroadcaster->sendTransform(odomTrans);
  
}
void HeroChassisController::calWheelSpeed(const ros::Duration& period){                     //运动学逆解  



                 
  command_[0] = (g_vel.linear.x + g_vel.linear.y - g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; 
  command_[1] = (g_vel.linear.x - g_vel.linear.y - g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius;                    //通过逆解分别计算四个轮的速度
  command_[2] = (g_vel.linear.x - g_vel.linear.y + g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius;        
  command_[3] = (g_vel.linear.x + g_vel.linear.y + g_vel.angular.z*(wheelBase/2+trackWidth/2))/radius; 
  
  
  double error = command_[0] - left1_joint_.getVelocity();
  double commanded_effort = pid1_controller_.computeCommand(error, period);
  left1_joint_.setCommand(commanded_effort);
  
  error = command_[1] - left2_joint_.getVelocity();
  commanded_effort = pid2_controller_.computeCommand(error, period);                                          //将误差和速度通过PID运算后，把速度发送给各个Joint关节
  left2_joint_.setCommand(commanded_effort);
  
  error = command_[2] - right1_joint_.getVelocity();
  commanded_effort = pid3_controller_.computeCommand(error, period);
  right1_joint_.setCommand(commanded_effort);
  
  error = command_[3] - right2_joint_.getVelocity();
  commanded_effort = pid4_controller_.computeCommand(error, period);
  right2_joint_.setCommand(commanded_effort);
  
}

} // namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
