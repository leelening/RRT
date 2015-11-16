//----------------------------------------------------------------------
//      File:           rrtPlanner.hpp
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

# ifndef _RRTPLANNER_H_
# define _RRTPlANNER_H_

# include <math.h>
# include "vector"
# include "cstdlib" 
# include "chrono"
# include <stdlib.h>
# include "eigen3/Eigen/Eigen"
# include "limits"
# include "iostream"
# include "openrave-core.h"
# include <boost/thread/thread.hpp>
# include <boost/bind.hpp>
# include <openrave/openrave.h>
# include "openrave/planningutils.h"
# include <time.h>
# include <sstream>
# include <string>
# include "rrtNode.hpp"
# include "rrtTree.hpp"
# include "preDefined.hpp"


class rrtPlanner
{
	private:
		OpenRAVE::EnvironmentBasePtr	_p_env_rrt;
		OpenRAVE::RobotBasePtr 			_robot_rrt;
        rrtNode                         _start;
        rrtNode                         _goal;
		std::vector<OpenRAVE::dReal>    _minimum_limits;
		std::vector<OpenRAVE::dReal>    _maximum_limits;
		OpenRAVE::dReal                          _step_size;
		std::vector<std::string>        _joint_names;
		std::vector<int>                _joint_indices;
        rrtTree                         _t;

		int                             _basis;
		std::vector<int>                _path;
		std::vector<int>                _shortcut_path;
		std::vector<OpenRAVE::dReal>             _weighted_vector;

	public:
		enum STATE
		{
			Reached = 0,
			Advanced,
			Trapped
		};
        rrtPlanner(OpenRAVE::EnvironmentBasePtr     &env,
		OpenRAVE::RobotBasePtr                      &robot,
		std::vector<OpenRAVE::dReal>                startconfiguration,
		std::vector<OpenRAVE::dReal>                goalconfiguration,
		std::vector<std::string>                    joint_names);

	rrtPlanner(){}
	virtual             ~rrtPlanner(){}

	void                 setBais(int i){_basis = i;}
	void                 setWeightedVector(std::vector<OpenRAVE::dReal> weighted_vector){_weighted_vector = weighted_vector;}
	void                 setEnv(OpenRAVE::EnvironmentBasePtr &env){_p_env_rrt = env;}
	void                 setRobot(OpenRAVE::RobotBasePtr &robot){_robot_rrt = robot;}
	void                 setJointNames(std::vector<std::string> joint_names){_joint_names = joint_names;}
    void                 setStepSize(OpenRAVE::dReal step_size){_step_size = step_size;}
	void                 setJointLimits();
	void                 setStart(std::vector<OpenRAVE::dReal> start_config);
	void                 setGoal(std::vector<OpenRAVE::dReal> goal_config);

	int								getBais(){return _basis;}
	std::vector<OpenRAVE::dReal>*			getWeightedVectorPtr(){return &_weighted_vector;}
	OpenRAVE::EnvironmentBasePtr	getEnvPtr() {return _p_env_rrt;}
	OpenRAVE::RobotBasePtr          getRobotPtr(){return _robot_rrt;}
	rrtNode*  						getStartPtr(){return &_start;}
	rrtNode							getStart(){return _start;}
	rrtNode*   						getGoalPtr(){return &_goal;}
	rrtNode							getGoal(){return _goal;}
	std::vector<std::string>*		getJointNamesPtr(){return &_joint_names;}
	std::vector<std::string>		getJointName(){return _joint_names;}
	OpenRAVE::dReal                          getStepSize(){return _step_size;}
	rrtTree							getTree(){return _t;}
	rrtTree*						getTreePtr(){return &_t;}



    void	init(rrtNode &q_init)               {addNode(q_init);}
    void	addNode(rrtNode &q_new)             {_t.addNode(q_new);}
    rrtNode	nearestRRTNode(rrtNode &q_rand);
    void    randConf(rrtNode &q_rand);
    void	plan(rrtNode q_init);
	int		extend(rrtNode &q_rand);
	void	shortcutSmooth();
    bool    addEdge(rrtNode q_new,rrtNode q_near);
	bool	collisionFree(rrtNode &node);
	void	path(rrtNode &goal, std::vector<int> &path);
    void	drawEndEffector(rrtNode &q_near,int i);
	OpenRAVE::dReal	euclideanDistance(rrtNode &q_rand, rrtNode &q_near);
	void	generateTraj();
    int     connect(rrtNode &q_rand);
    bool	connectLine(rrtNode q_1, rrtNode q_2);
    OpenRAVE::dReal	rho(rrtNode x1,rrtNode x2);
	bool	newConfig(rrtNode &q_rand,rrtNode &q_near,rrtNode &q_new);

};
#endif //_RRTPLANNER_H_
