//----------------------------------------------------------------------
//      File:           rrtTree.hpp
//      Programmer:     Lening Li
//      Last modified:  9/23/15
//      Description:    RRT tree code
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

# ifndef _RRTTREE_H_
# define _RRTTREE_H_

# include "rrtNode.hpp"
# include "vector"
# include "preDefined.hpp"

class rrtTree
{
public:

    rrtTree(){}
    ~rrtTree(){}

    void                        addNode(rrtNode &v)     {_vertices.push_back(v);}

    void                        deleteNode(rrtNode &v);

    void                        printTree();

    void                        deleteNodeAt(int i)     {_vertices.erase(_vertices.begin()+i);}

    int                         getSize()               {return _vertices.size();}

    std::vector<rrtNode>*       getVerticesPtr()        {return &_vertices;}

    std::vector<rrtNode>        getVertices()           {return _vertices;}

    rrtNode                     getVertixAt(int i)      {return _vertices.at(i);}

    rrtNode*                    getVertixAtPtr(int i)   {return &_vertices.at(i);}

private:

    std::vector<rrtNode>    _vertices;

};
#endif //_RRTTREE_H_