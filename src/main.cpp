# include "rrtPlanner.hpp"
# include "openrave-core.h"
# include "iostream"
# include "cstring"
# include "string"
# include <boost/thread/thread.hpp>
# include <boost/bind.hpp>
# include <eigen3/Eigen/Eigen>
# include <openrave/planningutils.h>


rrtPlanner								planner;

void setViewer(OpenRAVE::EnvironmentBasePtr penv, const std::string& viewername)
{
	std::cout<<"Opening the GUI!!!"<< std::endl;
	OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(penv, viewername);
	BOOST_ASSERT(!!viewer);
	penv->Add(viewer);
	bool showgui = true;
	viewer->main(showgui);
}

void raiseArm(OpenRAVE::EnvironmentBasePtr &p_env, OpenRAVE::RobotBasePtr &robot)
{
	std::vector<OpenRAVE::dReal> init_position;

	init_position = {1.29023451,
						-2.32099996,
						-0.69800004,
						1.27843491,
						-2.32100002,
						-0.69799996};

	robot->SetActiveDOFValues(init_position);
}

void setArmInit(OpenRAVE::EnvironmentBasePtr &p_env, OpenRAVE::RobotBasePtr &robot)
{
	std::vector<std::string>    joints;
	std::vector<int>            joints_indecies;

	joints = {"l_shoulder_lift_joint",
			"l_elbow_flex_joint",
			"l_wrist_flex_joint",
			"r_shoulder_lift_joint",
			"r_elbow_flex_joint",
			"r_wrist_flex_joint"};


	for(unsigned int i = 0; i<joints.size();i++)
	{
		joints_indecies.push_back(robot->GetJointIndex(joints[i]));
	}

	robot->SetActiveDOFs(joints_indecies);
	raiseArm(p_env,robot);
}

void setWeighted(std::vector<double> weighted_vector)
{
	std::cout<< "The weighted vector is [" << weighted_vector[0];
	for(unsigned int i = 1; i < weighted_vector.size();i++)
	std::cout<<","<<weighted_vector[i];
	std::cout<< "]"<<std::endl;

	planner.setWeightedVector(weighted_vector);
}

void setStepSize(double step_size)
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

void setRobot()
{
	std::vector<OpenRAVE::RobotBasePtr>     robots;

	OpenRAVE::RobotBasePtr                  robot;

	planner.getEnvPtr()->GetRobots(robots);

	robot = robots[0];

	planner.setRobot(robot);
}

void buildTree()
{
	std::cout<<"Now we are initialized the tree!!!"<<std::endl;
	for (unsigned int i = 0; i < LENGTH; i++)
		std::cout << planner.getStart().getConfigAt(i) << " - ";
	std::cout << std::endl;
	for (int i = 0; i < LENGTH; i++)
		std::cout << planner.getGoal().getConfigAt(i)<< " - ";
	std::cout << std::endl;
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

void setJointnames(std::vector<std::string> joint_names)
{
	std::cout<< "The joint names are [" << joint_names[0];
	for(unsigned int i = 1; i < joint_names.size();i++)
	std::cout<<","<<joint_names[i];
	std::cout<< "]"<<std::endl;

	planner.setJointNames(joint_names);
}

int main(int argc, char *argv[])
{    

	OpenRAVE::EnvironmentBasePtr            p_env;
	std::vector<OpenRAVE::RobotBasePtr>     robots;
	OpenRAVE::RobotBasePtr                  robot;
	std::string                             viewername = "qtcoin";
	OpenRAVE::Transform                     puma_transform;



	OpenRAVE::RaveInitialize(true);
	p_env = OpenRAVE::RaveCreateEnvironment();
	p_env->Reset();

	boost::thread   thviewer(boost::bind(setViewer, p_env, viewername));

	p_env->Load("../scenes/hw3.env.xml");
	//p_env->Load("../robots/myrobot.robot.xml");

	p_env->GetRobots(robots);
	robot =  robots[0];

	setArmInit(p_env,robot);

	thviewer.join();

	p_env->Destroy();

	return 0;
}