# include "rrtPlanner.hpp"
# include "openrave-core.h"
# include "iostream"
# include "cstring"
# include "string"
# include <boost/thread/thread.hpp>
# include <boost/bind.hpp>
# include <eigen3/Eigen/Eigen>
# include <openrave/planningutils.h>

rrtPlanner                              planner;

void setViewer(OpenRAVE::EnvironmentBasePtr penv, const std::string& viewername)
{
    std::cout<<"Opening the GUI!!!"<< std::endl;
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(penv, viewername);
    BOOST_ASSERT(!!viewer);
    penv->Add(viewer);
    bool showgui = true;
    viewer->main(showgui);
}

void setWeighted()
{
    std::cout << "Now we are setting the weights" << std::endl;
    planner.setWeightedVector();
}

void setWeighted(std::vector<OpenRAVE::dReal> weighted_vector)
{
    std::cout << "Now we are setting the weights" << std::endl;
    planner.setWeightedVector(weighted_vector);
}

void setStepSize(OpenRAVE::dReal step_size)
{
    std::cout<<"The step size is  "<<step_size<<std::endl;
    planner.setStepSize(step_size);
}

void setBais(int bais)
{
    std::cout<<"The bais is "<<bais<<std::endl;
    planner.setBais(bais);
}

void setJointlimits()
{
    std::cout<<"Now we are setting the joint limits!!!"<<std::endl;
    planner.setJointLimits();
}

void setRobot(OpenRAVE::RobotBasePtr robot)
{
    std::cout<<"Now we are setting the robot!!!"<<std::endl;
    planner.setRobot(robot);
}

void buildTree()
{
    std::cout<<"Now we are initialized the tree!!!"<<std::endl;
    planner.plan(planner.getStart());
}

void setStart(std::vector<OpenRAVE::dReal> start_config)
{
    std::cout<< "The start configration is [" << start_config[0];
    for(unsigned int i = 1; i < start_config.size();i++)
    std::cout<<","<<start_config[i];
    std::cout<< "]"<<std::endl;

    planner.setStart(start_config);
}

void setGoal(std::vector<OpenRAVE::dReal> goal_config)
{
    std::cout<< "The goal configration is [" << goal_config[0];
    for(unsigned int i = 1; i < goal_config.size();i++)
    std::cout<<","<<goal_config[i];
    std::cout<< "]"<<std::endl;

    planner.setGoal(goal_config);
}

void setEnv(OpenRAVE::EnvironmentBasePtr p_env)
{
    std::cout<<"We are setting environment!!!"<<std::endl;
    planner.setEnv(p_env);
}

void setJointIndices()
{
    std::cout<<"We are setting indices!!!"<<std::endl;
    planner.setJointIndices();
}

void setJointnames(std::vector<std::string> joint_names)
{
    std::cout<< "The joint names are [" << joint_names[0];
    for(unsigned int i = 1; i < joint_names.size();i++)
    std::cout<<","<<joint_names[i];
    std::cout<< "]"<<std::endl;

    planner.setJointNames(joint_names);
}

void action(OpenRAVE::EnvironmentBasePtr p_env, OpenRAVE::RobotBasePtr robot)
{
    std::vector<OpenRAVE::dReal> start_config, goal_config;  
    std::vector<std::string> joints_names;
    std::vector<OpenRAVE::dReal>      weighted_vector;

    //startconfig = [-0.15,0.075,-1.008,0,0,0,0]

    start_config.push_back(-0.15);
    start_config.push_back(0.075);
    start_config.push_back(-1.008);
    start_config.push_back(0);
    start_config.push_back(0);
    start_config.push_back(0);
    start_config.push_back(0);

    //goalconfig = [0.449,-0.201,0,0,0,0,0]

    goal_config.push_back(0.449);
    goal_config.push_back(-0.201);
    goal_config.push_back(0);
    goal_config.push_back(0);
    goal_config.push_back(0);
    goal_config.push_back(0);
    goal_config.push_back(0);

    setStart(start_config);
    setGoal(goal_config);

    setEnv(p_env);
    setStepSize(STEP_SIZE);

    setRobot(robot);

    joints_names.push_back("l_shoulder_pan_joint");
    joints_names.push_back("l_shoulder_lift_joint");
    joints_names.push_back("l_elbow_flex_joint");
    joints_names.push_back("l_upper_arm_roll_joint");
    joints_names.push_back("l_forearm_roll_joint");
    joints_names.push_back("l_wrist_flex_joint");
    joints_names.push_back("l_wrist_roll_joint");
    setJointnames(joints_names);

//    weights = [0.491099,
//                0.0870576,
//                0.0408578,
//                0.0649884,
//                0.03328,
//                0.0508196,
//                0.0145721,]



    setJointIndices();

    setJointlimits();

//    weighted_vector.push_back(0.491099);
//    weighted_vector.push_back(0.0870576);
//    weighted_vector.push_back(0.0408578);
//    weighted_vector.push_back(0.0649884);
//    weighted_vector.push_back(0.03328);
//    weighted_vector.push_back(0.0508196);
//    weighted_vector.push_back(0.0145721);
    weighted_vector.push_back(7);
    weighted_vector.push_back(6);
    weighted_vector.push_back(5);
    weighted_vector.push_back(4);
    weighted_vector.push_back(3);
    weighted_vector.push_back(2);
    weighted_vector.push_back(1);
    setWeighted(weighted_vector);

//    setWeighted();

    setBais(BAIS);

    buildTree();

}

void tuckArms(OpenRAVE::EnvironmentBasePtr p_env, OpenRAVE::RobotBasePtr robot)
{
    std::vector<std::string>        joints_names;
    std::vector<OpenRAVE::dReal>    joints_values;
    std::vector<int>                joints_indices;
    std::vector<OpenRAVE::dReal>    v;

    joints_names.push_back("l_shoulder_lift_joint");
    joints_names.push_back("l_elbow_flex_joint");
    joints_names.push_back("l_wrist_flex_joint");
    joints_names.push_back("r_shoulder_lift_joint");
    joints_names.push_back("r_elbow_flex_joint");
    joints_names.push_back("r_wrist_flex_joint");

    joints_values.push_back(1.29023451);
    joints_values.push_back(-2.32099996);
    joints_values.push_back(-0.69800004);
    joints_values.push_back(1.27843491);
    joints_values.push_back(-2.32100002);
    joints_values.push_back(-0.69799996);

    for(unsigned int i = 0; i < joints_names.size(); i++)
    {
        joints_indices.push_back(robot->GetJointIndex(joints_names.at(i)));
    }
    robot->SetActiveDOFs(joints_indices);
    robot->SetActiveDOFValues(joints_values);
    robot->GetDOFValues(v);
    robot->GetController()->SetDesired(v);
}

int main(int argc, char *argv[])
{    

    OpenRAVE::EnvironmentBasePtr            p_env;
    std::vector<OpenRAVE::RobotBasePtr>     robots;
    OpenRAVE::RobotBasePtr                  robot;
    std::string                             viewername = "qtcoin";

    OpenRAVE::RaveInitialize(true);
    p_env = OpenRAVE::RaveCreateEnvironment();
    p_env->Reset();

    boost::thread   thviewer(boost::bind(setViewer, p_env, viewername));

    p_env->Load("scenes/hw3.env.xml");

    p_env->GetRobots(robots);
    robot =  robots[0];

    tuckArms(p_env,robot);

    action(p_env,robot);

    thviewer.join();

    p_env->Destroy();

    exit(0);
}