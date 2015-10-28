//----------------------------------------------------------------------
//      File:           rrtNode.hpp
//      Programmer:     Lening Li
//      Last modified:  9/23/15
//      Description:    RRT node code
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

# ifndef _RRTNODE_H_
# define _RRTNODE_H_

# include "vector"
# include <openrave/openrave.h>
# include "openrave/planningutils.h"
# include "iostream"

# define        LENGTH                      7

class rrtNode
{
public:
	rrtNode()
	{
		_parent = -1;
		_index  = -1;
	}

	virtual ~rrtNode(){}

	void assginNode(std::vector<OpenRAVE::dReal> configuration)
	{
		_con_f.resize(LENGTH);
		for(unsigned int i = 0; i < configuration.size();i++)
			_con_f[i] = configuration[i];
	}

	void operator =(const rrtNode &Node);

	bool operator ==(const rrtNode &Node);

	bool operator!=(const rrtNode &Node);

	int  printNode()
	{
		std::cout << "This rrt node is[";
		std::cout<<_con_f[0];
		for (unsigned int i = 1; i < _con_f.size(); ++i)
		{
			std::cout<<","<<_con_f[i];
		}
		std::cout << "]" << std::endl;
		return 0;
	}

	std::vector<OpenRAVE::dReal>* getConfigPtr(){return &_con_f;}

	std::vector<OpenRAVE::dReal> getConfig(){return _con_f;}

	OpenRAVE::dReal* getConfigAtPtr(int i){return &_con_f.at(i);}

	OpenRAVE::dReal getConfigAt(int i){return _con_f.at(i);}

	int getIndex(){return _index;} 

	int getParent(){return _parent;}

	void setIndex(int i){this->_index = i;}

	void setParent(int i){this->_parent = i;}

	void setConf(std::vector<OpenRAVE::dReal> config){this->_con_f = config;}

	void setConfAt(int index, OpenRAVE::dReal v){_con_f[index] = v;}

private:
		int                          _parent;
		int                          _index;
		std::vector<OpenRAVE::dReal> _con_f;					// Store the configuration of the current BiRRTNode
};
#endif //_RRTNODE_H_
