#include "first_challenge/first_challenge.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz_", hz_, {10});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void GetRPY(const geometry_msgs::Quaternion &q,double &roll,double &pitch,double &yaw){
    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    old_odometry_ = odometry_;
    odometry_ = *msg;
    dx_ = odometry_.pose.pose.position.x - old_odometry_.pose.pose.position.x;
    sumx_+=dx_;

    old_yaw = yaw;
    GetRPY(odometry_.pose.pose.orientation,roll,pitch,yaw);
    if(yaw*old_yaw<0) dtheta_=0.0;
    else dtheta_=fabs(yaw-old_yaw);
    sum_theta_+=dtheta_;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}
void FirstChallenge::stop()
{
    cmd_vel_.mode=11;
    cmd_vel_.cntl.linear.x=0.0;
    cmd_vel_.cntl.angular.z=0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}
void FirstChallenge::turn()
{
    cmd_vel_.mode=11;
    cmd_vel_.cntl.linear.x=0.0;
    cmd_vel_.cntl.angular.z=0.7;

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::run()
{

    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.1;
    cmd_vel_.cntl.angular.z = 0.0;

    pub_cmd_vel_.publish(cmd_vel_);
}



void FirstChallenge::show_odom()
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

void FirstChallenge::show_scan()
{
    if(laser_.ranges.size()==0){
        return;
    }
    range = 0;
    int Lnum=(int)((0-laser_.angle_min)/laser_.angle_increment);
    std::cout<<"Lnum"<<Lnum<<std::endl;

    for (int i = -5; i <6 ; i++) {
        range+= laser_.ranges[Lnum+i];
    }
    range/=11;
    // ROS_INFO_STREAM("scan: min: %f", range_min);
    std::cout << "scan: range:" << range << std::endl;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    sumx_=0;
    odometry_.pose.pose.position.x=0;
    old_yaw=0;
    yaw=0;
    sum_theta_=0;

    while(ros::ok())
    {
        std::cout<<"range"<<range<<std::endl;
        if(sumx_<0.05){
            run();
        }else if(sum_theta_<=2*M_PI){
            turn();
        }else if(range>=0.5){
            run();
        }else{
            stop();
        }
       // printf("%lf",4*M_PI/3/laser_.angle_increment);
        show_odom();
        show_scan();


        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
