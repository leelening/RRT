//----------------------------------------------------------------------
//      File:           rrtPlanner.cpp
//      Programmer:     Lening Li
//      Last modified:  9/23/15
//      Description:    RRT planner code
//----------------------------------------------------------------------
// Copyright (c) 2015-2016 Worcester Polytechnic Institute and Lening Li.  
// All Rights Reserved.
// 
// This file and related documentation are part of the 
// Motion Planning Homework to implement Rapidly-exploring Random Trees.
// 
// Permission to use, copy, and distribute this software and its 
// documentation is hereby granted free of charge, provided that 
// (1) it is not a component of a commercial product, and 
// (2) this notice appears in all copies of the software and
//     related documentation. 
// 
// Worcester Polytechnic Institute and the author make no representations
// about the suitability or fitness of this software for any purpose.  It is
// provided "as is" without express or implied warranty.
//----------------------------------------------------------------------

# include "rrtPlanner.hpp"

///////////////////////////////////// Here are definitions of the BiRRT   //////////////////////////////////////////////

rrtPlanner::rrtPlanner(OpenRAVE::EnvironmentBasePtr &env,
	OpenRAVE::RobotBasePtr &robot,
	std::vector<OpenRAVE::dReal> start_config,
	std::vector<OpenRAVE::dReal> goal_config,
	std::vector<std::string> joint_names):
	_p_env_rrt(env),
	_robot_rrt(robot),
	_joint_names(joint_names)
{
	_start.assginNode(start_config);
	_goal.assginNode(goal_config);

	for (unsigned int i = 0; i < _joint_names.size(); i++)
	{
		_joint_indices.push_back(_robot_rrt->GetJointIndex(_joint_names[i]));
	}

	_robot_rrt->GetDOFLimits(_minimum_limits,_maximum_limits,_joint_indices);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rrtPlanner::setStart(std::vector<OpenRAVE::dReal> start_config)
{
	_start.assginNode(start_config);
	_start.setParent(-1);
	_start.setIndex(-1);
}

void rrtPlanner::setGoal(std::vector<OpenRAVE::dReal> goal_config)
{
	_goal.assginNode(goal_config);
	_goal.setParent(-1);
	_goal.setIndex(-1);
}

void rrtPlanner::setJointIndices()
{
    for (unsigned int i = 0; i < _joint_names.size(); i++)
    {
        std::cout<<_joint_names[i]<<" - ";
        _joint_indices.push_back(_robot_rrt->GetJointIndex(_joint_names[i]));
        std::cout << _joint_indices[i] <<"   "<<std::endl;
    }
}

void rrtPlanner::setJointLimits()
{
	_robot_rrt->GetDOFLimits(_minimum_limits,_maximum_limits,_joint_indices);
    std::cout << "min limits " << std::endl;
    for (unsigned int i = 0; i < _minimum_limits.size(); i++)
    {
        std::cout<<_minimum_limits[i]<<" - ";
    }
    std::cout << std::endl << "max limits " << std::endl;
    for (unsigned int i = 0; i < _maximum_limits.size(); i++)
    {
        std::cout<<_maximum_limits[i]<<" - ";
    }
    std::cout << std::endl;
}

void rrtPlanner::setWeightedVector()
{
    std::cout << "The weights are following:" << std::endl;
    for (unsigned int i = 0 ; i < _joint_indices.size(); i ++)
    {
        std::cout << _joint_names[i]<<" - ";
        _weighted_vector.push_back(_robot_rrt->GetJoint(_joint_names[i])->GetWeight());
        std::cout << _weighted_vector[i] << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double rrtPlanner::randomRange(double range_min, double range_max)
{
    double range = std::abs(range_max - range_min);
    double div = RAND_MAX / range;
    return range_min + (rand() / div);
}

bool rrtPlanner::randConf(rrtNode &q_rand)
{
    std::vector<OpenRAVE::dReal>                    rand_vector;
	OpenRAVE::KinBody::JointPtr                     joint;

    rand_vector.resize(LENGTH);

	if(std::rand() % 100 < _basis)
	{
        q_rand = _goal;
	}
	else
	{
        for(unsigned int i = 0;i < _joint_indices.size();i++)
		{
			joint = _robot_rrt->GetJointFromDOFIndex(_joint_indices[i]);
			if (joint->IsCircular(0) == true)
			{
                rand_vector[i] = (PI) * ( (double)std::rand() / (double)RAND_MAX );
			}
			else
			{
                rand_vector[i] = _minimum_limits[i] + (_maximum_limits[i] - _minimum_limits[i]) * ( (double)std::rand() / (double)RAND_MAX );;
                if( ( rand_vector[i] < _minimum_limits[i] ) || ( rand_vector[i] > _maximum_limits[i]) )
                {
                    return false;
                }

            }
		}
        q_rand.assginNode(rand_vector);
    }
    return true;
}

void rrtPlanner::plan(rrtNode q_init)
{

    clock_t     running_start,running_finish;
    OpenRAVE::dReal      totaltime;

	running_start=clock();
    std::srand(TIME);

	rrtNode                                             q_rand;
	STATE                                               S;
	std::chrono::time_point<std::chrono::system_clock>  start;
	std::chrono::time_point<std::chrono::system_clock>  end;
	std::chrono::duration<OpenRAVE::dReal>                       elapsed_seconds;

	start = std::chrono::system_clock::now();

	init(q_init);

	for(int i = 1 ; i <= K ; ++i)
	{
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
        if (elapsed_seconds.count() > TIME_CONTROL)
        {
            std::cout<<"Sorry, I cannot find the solution!!!"<<std::endl;
            return;
        }
        bool suc = false;
        while (suc == false)
        {
            suc = randConf(q_rand);
            if(suc == false)
            std::cout << "fuck you" << std::endl;
        }
		S = (STATE)connect(q_rand);
        if(q_rand == _goal && S == Reached)
        {
            running_finish=clock();
            totaltime=(OpenRAVE::dReal)(running_finish-running_start)/CLOCKS_PER_SEC;
            std::cout<<"\nThe running time of this RRT Algorithm is "<<totaltime<<" seconds !"<<std::endl;
            path(_goal,_path);
            shortcutSmooth();
            generateTraj();
            return;
		}
	}
}

///////////////////////////////////////////////////////////
rrtNode rrtPlanner::nearestRRTNode(rrtNode &q_rand)
{
	rrtNode         q_near;

	OpenRAVE::dReal minimun  = std::numeric_limits<OpenRAVE::dReal>::max();

    for (int i = 0;i < _t.getSize();i++)
	{
        OpenRAVE::dReal dist = rho(_t.getVertixAt(i),q_rand);
        if ( dist < minimun)
		{
			q_near = _t.getVertices()[i];
            minimun = dist;
		}
	}

	return q_near;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

bool rrtPlanner::newConfig(rrtNode &q_rand,rrtNode &q_near,rrtNode &q_new)
{

    q_new.getConfigPtr()->resize(LENGTH);

	OpenRAVE::dReal step_size = _step_size;
    OpenRAVE::dReal temp;

    if ( rho(q_rand,q_near) < step_size )
        q_new.setConf(q_rand.getConfig());
    else
    {
        for (int i = 0; i < LENGTH; i++)
        {
            temp = q_near.getConfigAt(i) +  ( q_rand.getConfigAt(i) - q_near.getConfigAt(i) ) / rho(q_rand,q_near) * step_size;
            if(( temp < _minimum_limits[i]) || (temp > _maximum_limits[i]))
            {
                std::cout << "fail" << std::endl;
                std::cout << i << " - " << temp << std::endl;
                return false;
            }
            q_new.setConfAt(i, temp);
        }
    }
    if (collisionFree(q_new) == false)
        return true;
	else
		return false;
}

bool rrtPlanner::collisionFree(rrtNode &node)
{
    std::vector<OpenRAVE::KinBodyPtr>           KinBodies;
    std::vector<OpenRAVE::dReal>                dof_values;

    _p_env_rrt->GetBodies(KinBodies);
    _robot_rrt->SetActiveDOFs(_joint_indices);
    _robot_rrt->SetActiveDOFValues(node.getConfig());
    _p_env_rrt->GetCollisionChecker()->SetCollisionOptions(OpenRAVE::CO_Contacts);

    if ( _p_env_rrt->CheckCollision(_robot_rrt) == false )
    {
        for (unsigned int i = 0; i < KinBodies.size(); i++)
            if ((KinBodies[i] != _robot_rrt)&&(_p_env_rrt->CheckCollision(_robot_rrt, KinBodies[i]) == true))
            {
                return true;
            }
        return false;
    }
    return true;
}

OpenRAVE::dReal rrtPlanner::rho(rrtNode x1, rrtNode x2)
{
    OpenRAVE::dReal dis = 0;
    for(unsigned int i = 0;i < LENGTH; ++i)
    {
        dis += (x1.getConfigAt(i) - x2.getConfigAt(i)) * _weighted_vector[i] * (x1.getConfigAt(i) - x2.getConfigAt(i));
    }
    return dis;
}

int rrtPlanner::connect(rrtNode &q_rand)
{
    STATE S = Advanced;
    while(S == Advanced)
    {
        S = (STATE)extend(q_rand);
    }
    return S;
}

int rrtPlanner::extend(rrtNode &q_rand)
{
    rrtNode q_near;
    rrtNode q_new;

    q_near = nearestRRTNode(q_rand);
//    drawEndEffector(q_near,0);

    if(newConfig(q_rand, q_near, q_new) == true)
    {
        addNode(q_new);
        addEdge(q_new,q_near);
        if (q_new == q_rand)
        {
            return Reached;
        }
        else
            return Advanced;
    }
    return Trapped;
}

bool rrtPlanner::addEdge(rrtNode q_new,rrtNode q_near)
{
    int     q_new_index     = -1;
    int     q_near_index    = -1;
    bool    q_new_found     = false;
    bool    q_near_found    = false;
    for (int i = 0;i <_t.getSize();i++)
    {
        if(_t.getVertixAt(i) == q_new)
        {
           q_new_index = i;
           _t.getVertixAtPtr(i)->setIndex(i);
           q_new_found = true;
        }
        if(_t.getVertixAt(i) == q_near)
        {
            q_near_index = i;
            _t.getVertixAtPtr(i)->setIndex(i);
            q_near_found = true;
        }
        if(q_new_found == true && q_near_found == true)
        {
            break;
        }
    }
    if ((q_near_found == true)&&(q_new_found == true))
    {
        _t.getVertixAtPtr(q_new_index)->setParent(q_near_index);
        return true;
    }
    else
        return false;
}

void rrtPlanner::shortcutSmooth()
{
	clock_t running_start,running_finish;
	OpenRAVE::dReal totaltime;
	running_start=clock();

	std::cout<<"start to short cut !!!!!!!!"<<std::endl;
	int index_1, index_2;

	_shortcut_path.clear();
	_shortcut_path = _path;

    for (int i = 0; i < MAX_SHORTCUT; i++)
	{
		index_1 = _shortcut_path.size() *( (OpenRAVE::dReal)std::rand() / (OpenRAVE::dReal)RAND_MAX );
		index_2 = _shortcut_path.size() *( (OpenRAVE::dReal)std::rand() / (OpenRAVE::dReal)RAND_MAX );

		if (index_1 == index_2)
			continue;

        if (connectLine(_t.getVertixAt(index_1), _t.getVertixAt(index_2)) == true)
		{
			int parent = index_2;
			int child = index_1;

			if (index_1 < index_2)
			{
				parent = index_1;
				child = index_2;
			}

			_shortcut_path.erase(_shortcut_path.begin() + parent + 1, _shortcut_path.begin() + child);
		}

        std::cout<<"This is the "<<i<<"times iterations: "<<"The length of the shortcut path is "<<_shortcut_path.size()<<std::endl;
	}

	std::cout<<"The length of the shortcut path is "<<_shortcut_path.size()<<std::endl;
	running_finish=clock();
	totaltime=(OpenRAVE::dReal)(running_finish-running_start)/CLOCKS_PER_SEC;
	std::cout<<"\nThe running time of this Shortcut is "<<totaltime<<" seconds !"<<std::endl;
}

bool rrtPlanner::connectLine(rrtNode q_1, rrtNode q_2)
{
    rrtNode online_node;

    while (newConfig(q_1, q_2, online_node) == true)               // The basic idea is the same as the connect funtion
    {
        q_2 = online_node;
        if (q_2 == q_1)
            return true;
    }

    return false;
}

void rrtPlanner::path(rrtNode &goal, std::vector<int> &path)
{
	std::cout<<"Find the solution!!!!!!!!!!!"<<std::endl;

    _path.clear();

	rrtNode                         currentNode = goal;

	std::cout<<"The totoal number of the tree is "<<"-- "<<_t.getSize()<<std::endl;

	for(unsigned i = 0; _t.getSize();++i)
	{
		if(_t.getVertices()[i] == currentNode)
		{
			currentNode = _t.getVertices()[i];
			break;
		}
	}
	while(currentNode.getParent() != -1)
	{
		_path.push_back(currentNode.getIndex());
		currentNode = _t.getVertices()[currentNode.getParent()];
        currentNode.printNode();
	}

	std::cout<<"The length of the unshorted path is "<<_path.size()<<std::endl;
}

void rrtPlanner::drawEndEffector(rrtNode &q_near,int color_flag)
{
	static std::vector<OpenRAVE::GraphHandlePtr>    handles;
	static OpenRAVE::GraphHandlePtr                 handle;
	float                                           point[3];
	float                                           color[4];
	float                                           scale;
	if(color_flag==0)
	{
		color[0] = 1.0;
		color[1] = 0;
		color[2] = 0;
		color[3] = 0.5;
		scale = 2.0;
	}


	if(color_flag==1)
	{
		color[0] = 0;
		color[1] = 0;
		color[2] = 1.0;
		color[3] = 1.0;
		scale = 8.0;
	}

    if(color_flag == 3)
    {
        color[0] = 0;
        color[1] = 1.0;
        color[2] = 0;
        color[3] = 1.0;
        scale = 8.0;
    }

	OpenRAVE::KinBody::LinkPtr                      end_effector;
	OpenRAVE::Transform                             transform;

    _robot_rrt->SetActiveDOFValues(q_near.getConfig());

	end_effector = _robot_rrt->GetLink("l_gripper_l_finger_tip_link");
	transform = end_effector->GetTransform();

	point[0] = transform.trans.x;
	point[1] = transform.trans.y;
	point[2] = transform.trans.z;


	handle = _p_env_rrt->plot3(point,
							   3,
							   3 *sizeof(float),
							   scale,
							   color);
	handle->SetShow(true);
	handles.push_back(handle);

}

void rrtPlanner::generateTraj()
{
	std::cout<<"Generate Trajectory"<<std::endl;

	std::vector<int>                ReversedPath;

    ReversedPath.clear();

	OpenRAVE::TrajectoryBasePtr trajectory;

	trajectory = OpenRAVE::RaveCreateTrajectory(_p_env_rrt, "");

	trajectory->Init(_robot_rrt->GetActiveConfigurationSpecification());


    if(_shortcut_path.size() == 0)
    {
        for(int j = _path.size()-1;j >= 0;j--)
        {
            ReversedPath.push_back(_path[j]);
        }
    }
    else
    {
        for(int j = _shortcut_path.size()-1;j >= 0;j--)
        {
            ReversedPath.push_back(_shortcut_path[j]);
        }
    }


	for (unsigned int i = 0; i < ReversedPath.size(); i++)
	{
		drawEndEffector(_t.getVertices()[ReversedPath[i]],1);
		trajectory->Insert(i, _t.getVertices()[ReversedPath[i]].getConfig(), true);
	}

	for (unsigned int i = 0; i < _path.size(); i++)
	{
		drawEndEffector(_t.getVertices()[_path[i]],0);
	}

	OpenRAVE::planningutils::RetimeActiveDOFTrajectory(trajectory, _robot_rrt);

	_robot_rrt->GetController()->SetPath(trajectory);
}

