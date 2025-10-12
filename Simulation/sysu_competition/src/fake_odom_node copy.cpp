#include "cmd_to_odom.h"
#include "cmd_to_odom_ugv.h"
#include <random>

using namespace cmd_to_odom_ns;

#define MAX_NUM 40
int swarm_num_uav,swarm_num_ugv;
bool manual_init_pos;
int preset_init_pos_flag;
gazebo_msgs::ModelState model_state;

cmd_to_odom fake_odom_uav[MAX_NUM];
Eigen::Vector3d init_pos_uav[MAX_NUM];
double init_yaw_uav[MAX_NUM];
Eigen::Vector3d pos_uav[MAX_NUM];

cmd_to_odom_ugv fake_odom_ugv[MAX_NUM];
Eigen::Vector3d init_pos_ugv[MAX_NUM];
double init_yaw_ugv[MAX_NUM];
Eigen::Vector3d pos_ugv[MAX_NUM];

random_device rd;
default_random_engine eng(rd()); 
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;

float time_tank;
ros::Publisher gazebo_state_pub;
void get_preset_pos_uav(int i);
void get_preset_pos_ugv(int i);
void gazebo_pub_cb(const ros::TimerEvent &e);
void safety_check_cb(const ros::TimerEvent &e);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "fake_odom_node");
    ros::NodeHandle nh("~");

    nh.param("fake_odom/swarm_num_uav", swarm_num_uav, 0);
    nh.param("fake_odom/swarm_num_ugv", swarm_num_ugv, 0);
    nh.param("fake_odom/manual_init_pos", manual_init_pos, false);
    nh.param("fake_odom/preset_init_pos_flag", preset_init_pos_flag, 1);

    unsigned int seed = rd();
    // unsigned int seed = 2433201515;
    cout << GREEN<< "random seed=" << seed << TAIL<< endl;
    eng.seed(seed);

    sleep(5.0);

    for(int i = 0; i<swarm_num_uav;i++)
    {
        fake_odom_uav[i].init(nh, i+1);

        // 设置无人机初始位置
        if(manual_init_pos)
        {
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_x", init_pos_uav[i][0], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_y", init_pos_uav[i][1], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_z", init_pos_uav[i][2], 0.0);
            nh.param("fake_odom/uav" + to_string(i+1) + "_init_yaw", init_yaw_uav[i], 0.0);    
        }else
        {
            // 得到预设的初始位置
            get_preset_pos_uav(i);
        }
        fake_odom_uav[i].set_init_pos(init_pos_uav[i], init_yaw_uav[i]);
    }

    for(int i = 0; i<swarm_num_ugv;i++)
    {
        fake_odom_ugv[i].init(nh, i+1);

        // 设置无人机初始位置
        if(manual_init_pos)
        {
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_x", init_pos_ugv[i][0], 0.0);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_y", init_pos_ugv[i][1], 0.0);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_z", init_pos_ugv[i][2], 0.08);
            nh.param("fake_odom/ugv" + to_string(i+1) + "_init_yaw", init_yaw_ugv[i], 0.0);    
        }else
        {
            // 得到预设的初始位置
            get_preset_pos_ugv(i);
        }
        fake_odom_ugv[i].set_init_pos(init_pos_ugv[i], init_yaw_ugv[i]);
    }

    sleep(2.0);

    time_tank = 0.0;

    gazebo_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    ros::Timer safety_check_timer = nh.createTimer(ros::Duration(0.2), safety_check_cb);
    ros::Timer gazebo_pub_timer = nh.createTimer(ros::Duration(0.1), gazebo_pub_cb);

    ros::spin();

    return 0;
}

void gazebo_pub_cb(const ros::TimerEvent &e)
{
    for(int i = 0; i<swarm_num_uav; i++)
    {
        model_state = fake_odom_uav[i].get_model_state();
        
        gazebo_state_pub.publish(model_state);

        sleep(0.001);
    }

    for(int i = 0; i<swarm_num_ugv; i++)
    {
        model_state = fake_odom_ugv[i].get_model_state();
        
        gazebo_state_pub.publish(model_state);

        sleep(0.001);
    }

    // Case2 发布移动目标位置
    if(preset_init_pos_flag == 2 && swarm_num_uav == 40)
    {
        time_tank = time_tank + 0.1;
        float linear_vel = 0.15;
        float circle_radius = 0.8;
        float omega =  fabs(linear_vel / circle_radius);
    
        const float angle = time_tank * omega;
        const float cos_angle = cos(angle);
        const float sin_angle = sin(angle);

        // 轨迹                    
        model_state.model_name = "tank";
        model_state.pose.position.x = circle_radius * cos_angle + -6.0;
        model_state.pose.position.y = circle_radius * sin_angle + -9.0;
        model_state.pose.position.z = 0.0;
        model_state.pose.orientation.x = 0.0;
        model_state.pose.orientation.y = 0.0;
        model_state.pose.orientation.z = 0.0;
        model_state.pose.orientation.w = 1.0;
        model_state.reference_frame = "ground_plane::link";    
        gazebo_state_pub.publish(model_state);
        model_state.model_name = "aruco_tank";
        model_state.pose.position.z = 0.40;
        gazebo_state_pub.publish(model_state);
    }
}


void safety_check_cb(const ros::TimerEvent &e)
{
    for(int i = 0; i<swarm_num_uav; i++)
    {
        pos_uav[i] = fake_odom_uav[i].get_uav_pos();
    }

    for(int i = 0; i < swarm_num_uav; i++)
    {
        for(int j =i+1; j < swarm_num_uav; j++)
        {
            double distance = (pos_uav[i]-pos_uav[j]).norm();
            if(distance < 0.5)
            {

                cout << RED  << "collision : uav[ " <<  i << " ] and uav[ "  << j << " ], distance is " << distance << " [m]."<< TAIL << endl;
            }
        }
    }

    for(int i = 0; i<swarm_num_ugv; i++)
    {
        pos_ugv[i] = fake_odom_ugv[i].get_ugv_pos();
    }

    for(int i = 0; i < swarm_num_ugv; i++)
    {
        for(int j =i+1; j < swarm_num_ugv; j++)
        {
            double distance = (pos_ugv[i]-pos_ugv[j]).norm();
            if(distance < 0.4)
            {

                cout << RED  << "collision : ugv[ " <<  i << " ] and ugv[ "  << j << " ], distance is " << distance << " [m]."<< TAIL << endl;
            }
        }
    }


}

void get_preset_pos_ugv(int i)
{
    int ugv_id = i+1;

    if(preset_init_pos_flag == 2 && swarm_num_ugv == 2)
    {
        init_pos_ugv[1][0] = 0.0;
        init_pos_ugv[1][1] = 0.0;
        init_pos_ugv[1][2] = 0.0;
        init_yaw_ugv[1] = 0.0;

        init_pos_ugv[2][0] = -4.5;
        init_pos_ugv[2][1] = 0;
        init_pos_ugv[2][2] = 0.0;
        init_yaw_ugv[2] = 0.0;
    }else if(preset_init_pos_flag == 2 && swarm_num_ugv == 20)
    {
        if(ugv_id <= 10)
        {
            init_pos_ugv[i][0] = 0;
            init_pos_ugv[i][1] = -4*ugv_id + 22;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 0.0;
        }else 
        {
            init_pos_ugv[i][0] = -15;
            init_pos_ugv[i][1] = -4*ugv_id+62;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 0.0;
        }
    }else if(preset_init_pos_flag == 3 && swarm_num_ugv == 2)
    {
        init_pos_ugv[1][0] = 4.0;
        init_pos_ugv[1][1] = 0.0;
        init_pos_ugv[1][2] = 0.0;
        init_yaw_ugv[1] = 3.14;

        init_pos_ugv[2][0] = -4.0;
        init_pos_ugv[2][1] = 0;
        init_pos_ugv[2][2] = 0.0;
        init_yaw_ugv[2] = 0.0;
    }else if(preset_init_pos_flag == 3 && swarm_num_ugv == 10)
    {
        if(ugv_id <= 5)
        { 
            rand_x = uniform_real_distribution<double>(16 , 20);
            init_pos_ugv[i][0] = rand_x(eng);
            init_pos_ugv[i][1] = -8*ugv_id + 24;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 3.14;
        }else 
        {
            rand_x = uniform_real_distribution<double>(-16, -20);
            init_pos_ugv[i][0] = rand_x(eng);
            init_pos_ugv[i][1] = -8*ugv_id+64;
            init_pos_ugv[i][2] = 0.0;
            init_yaw_ugv[i] = 0.0;
        }
    }else
    {
        init_pos_ugv[i][0] = 0.0;
        init_pos_ugv[i][1] = 0.0;
        init_pos_ugv[i][2] = 0.0;
        init_yaw_ugv[i] = 0.0;
        cout << RED  << "Wrong preset_init_pos_flag (ugv)."<< TAIL << endl;
    }
    
}

void get_preset_pos_uav(int i)
{
    int uav_id = i+1;
    // case 1
    if(preset_init_pos_flag == 1)
    {
        if(uav_id%2==1)
        {
            init_pos_uav[i][0] = 0.5 * uav_id;
            init_pos_uav[i][1] = 0.0;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }else
        {
            init_pos_uav[i][0] = -0.5 * (uav_id - 1);
            init_pos_uav[i][1] = 0.0;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else if(preset_init_pos_flag == 2 && swarm_num_uav == 8)
    {
        if(uav_id <= 4)
        {
            init_pos_uav[i][0] = 0.5;
            init_pos_uav[i][1] = 3.0 - 1.2*uav_id;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }else 
        {
            init_pos_uav[i][0] = -4.0;
            init_pos_uav[i][1] = 3.0 - 1.2*(uav_id - 4);
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else if(preset_init_pos_flag == 2 && swarm_num_uav == 40)
    {
        if(uav_id <= 20)
        {
            init_pos_uav[i][0] = 1;
            init_pos_uav[i][1] = -2*uav_id+21;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }else 
        {
            init_pos_uav[i][0] = -14;
            init_pos_uav[i][1] = -2*uav_id+61;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else if(preset_init_pos_flag == 3 && swarm_num_uav == 8)
    {
        if(uav_id <= 4)
        {
            init_pos_uav[i][0] = 4.0;
            init_pos_uav[i][1] = 3.0 - 1.2*uav_id;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 3.14;
        }else 
        {
            init_pos_uav[i][0] = -4.0;
            init_pos_uav[i][1] = 3.0 - 1.2*(uav_id - 4);
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else if(preset_init_pos_flag == 3 && swarm_num_uav == 40)
    {
        if(uav_id <= 20)
        {
            rand_x = uniform_real_distribution<double>(14, 16);
            init_pos_uav[i][0] = rand_x(eng);
            init_pos_uav[i][1] = -2*uav_id+21;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 3.14;
        }else 
        {
            rand_x = uniform_real_distribution<double>(-14, -16);
            init_pos_uav[i][0] = rand_x(eng);
            init_pos_uav[i][1] = -2*uav_id+61;
            init_pos_uav[i][2] = 0.0;
            init_yaw_uav[i] = 0.0;
        }
    }else
    {
        cout << RED  << "Wrong preset_init_pos_flag."<< TAIL << endl;
    }
}