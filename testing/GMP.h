//
//  GMP.h
//  
//
//  Created by Chris on 3/18/18.
//

#ifndef GMP_h
#define GMP_h

#include "cmd.h"
#include <argos3/core/utility/math/vector3.h>

class GMP {

public:
struct TreeNodeList {
    // location of child
    int childX;
    int childY;
    
    // next child (sibling)
    TreeNodeList* next;
    };
    
#define SWARMSIZE   (0.141+0.5)
#define OPEN_VAL    (0)
#define OBS_VAL     (1)
#define NODE_VAL    (2)

struct TreeNode {
    // indicates the status of the node define XXXX_VAL
    int val;
    
    // Cost
    float cost;
    
    // parent of node
    int fromX;
    int fromY;
    
    // children of node
    TreeNodeList* children;
    };

#define GRID            (5)
#define MAXCHILDREN     (100)
#define ENVSZ           (10.0)
#define Env2Index(X)    int((X - -EnvirX) * GRID * 2)
//                      (-5 - -5.0)*10*2 = 0
//                      (0 - -5.0)*10*2 = 100
//                      (5 - -5.0)*10*2 = 200
#define Index2Env(X)    (float(X) / 2.0 / GRID + -EnvirX)
#define MAXSOLUTIONLEN  (Env2Index(ENVSZ)*4)

public:
    GMP();
    cmd GetCmd();
    void ReadEnvironment(float X, float Y);
    void SetBoxObstacle(float x0, float y0, float xs, float ys, float theta);
    void SetCylinderObstacle(float x0, float y0, float radius);
    void SetNorthWall(float Y);
    void SetSouthWall(float Y);
    void SetEastWall(float X);
    void SetWestWall(float X);
    void SetObjSize(float X, float Y);
    void SetStart(float X, float Y);
    void SetEnd(float X, float Y);
    void ShowMap();
    void ShowSolution();
    void resetMap();
    bool GetNextNode(float* x, float* y);
    bool RRT();
    inline std::vector<argos::CVector3> GetParents() {
        return m_tParent;
        }
    inline std::vector<argos::CVector3> GetChildren() {
        return m_tChild;
        }

    inline std::vector<argos::CVector3> GetSolution() {
        return m_tSolution;
        }

private:
    float distance(int x1, int y1, int x2, int y2);
    float Cost(int x1, int y1, int x2, int y2);

    void RRTinsertNode(int atX, int atY);
    void RRTextendRRT(int randX, int randY, int* newNodeX, int* newNodeY);
    void RRTextendRRTstar(int randX, int randY, int* newX, int* newY);
    void RRTremoveEdge(int pX, int pY, int cX, int cY);
    void RRTinsertEdge(int nearX, int nearY, int newX, int newY);
    bool RRTobstacleFree(int nearX, int nearY, int newX, int newY);
    void RRTsteer(int nearX, int nearY, int randX, int randY, int* newX, int*);
    void RRTfindNearest(int randX, int randY, int* nearX, int* nearY);
    TreeNodeList* RRTnear(int newX, int newY, float radius);
    void RRTaddChild(int atX, int atY, int X, int Y);
    
    float cost(int x, int y) {
        return map[y][x].cost;
        }

    void parent(int x, int y, int *parentX, int *parentY) {
        *parentX = map[y][x].fromX;
        *parentY = map[y][x].fromY;
        }

    void setChildren(int x, int y, TreeNodeList* c) {
        map[y][x].children = c;
        }

    TreeNodeList* children(int x, int y) {
        return map[y][x].children;
        }

    void ShowNode(int x, int y);
    void GetParentandChild(int x, int y);
    void Getsolution(int x, int y);

    float EnvirX;
    float EnvirY;
    TreeNode map[int(ENVSZ*2)*5*2 + 1][int(ENVSZ*2)*5*2 + 1];
    int endX;
    int endY;
    float oSzX;
    float oSzY;
    int atIndexX;
    int atIndexY;
    std::vector<argos::CVector3> m_tParent;
    std::vector<argos::CVector3> m_tChild;
    std::vector<argos::CVector3> m_tSolution;
    int startX;
    int startY;

};

#endif /* GMP_h */
