//
//  GMP.cpp
//  
//
//  Created by Chris on 3/18/18.
//

#include "GMP.h"
#include <math.h>
#include <argos3/core/utility/logging/argos_log.h>
    
GMP::GMP() {}

float GMP::distance(int x1, int y1, int x2, int y2) {
    float dx = x2-x1;
    float dy = y2-y1;
    
    return sqrt(dx*dx+dy*dy);
    }

float GMP::Cost(int x1, int y1, int x2, int y2) {
    return distance(x1, y1, x2, y2);
    }

cmd GMP::GetCmd() {
    cmd c;
    return c;
    }

void GMP::SetObjSize(float X, float Y) {
    oSzX = X;
    oSzY = Y;
    }

void GMP::SetStart(float X, float Y) {
    endX = atIndexX = Env2Index(X);
    endY = atIndexY = Env2Index(Y);
    }

void GMP::SetEnd(float X, float Y) {
    startX = Env2Index(X);
    startY = Env2Index(Y);
    }

void GMP::ShowMap() {
    int row;
    int col;
    for (row = Env2Index(EnvirY); row >= 0; row--) {
        argos::LOG << "[ROW:" << row << "]  ";
        for (col = 0; col < Env2Index(EnvirX); col++) {
            int v = map[row][col].val;
            if (v != OPEN_VAL) {
                argos::LOG << map[row][col].val;
                }
            else {
                argos::LOG << " ";
                }
            }
            argos::LOG << std::endl;
        }
    }

void GMP::resetMap() {
    atIndexX = endX;
    atIndexY = endY;
    }

void GMP::GetParentandChild(int x, int y) {

    TreeNodeList* ch = map[y][x].children;
    while (ch) {
        m_tParent.push_back(argos::CVector3(Index2Env(x), Index2Env(y), 0));
        m_tChild.push_back(argos::CVector3(Index2Env(ch->childX), Index2Env(ch->childY), 0));
        GetParentandChild(ch->childX, ch->childY);
        ch = ch->next;
        }
    }

void GMP::Getsolution(int startX, int startY) {
    int row = endY;
    int col = endX;
    int x;
    
    int n = 0;
    while (((col != startX) || (row != startY)) && (++n < MAXSOLUTIONLEN)) {
        m_tSolution.push_back(argos::CVector3(Index2Env(col), Index2Env(row), 0));
        x = map[row][col].fromX;
        row = map[row][col].fromY;
        col = x;
        }
    if ((col != startX) || (row != startY)) {
        argos::LOG << "Maximum solution nodes reached while searching solution" << std::endl;
        }
    else {
        m_tSolution.push_back(argos::CVector3(Index2Env(col), Index2Env(row), 0));
        }
    }

bool GMP::GetNextNode(float* x, float* y) {
    int row = atIndexY;
    int col = atIndexX;
    int xt;
    bool solution;
    
    if ((col != startX) || (row != startY)) {
        xt = map[row][col].fromX;
        row = atIndexY = map[row][col].fromY;
        col = atIndexX = xt;
        *x = Index2Env(col);
        *y = Index2Env(row);
        solution = true;
        }
    else {
        solution = false;
        }
    return solution;
    }

void GMP::ShowSolution() {
    int row = endY;
    int col = endX;
    int x;
    
    argos::LOG << "<====== Solution ======>" << std::endl;
    argos::LOG << "X:" << col << " Y:" << row << " ";
    argos::LOG << "[X:" << Index2Env(col) << " Y:" << Index2Env(row) << "]" << std::endl;
    int n = 0;
    while (((col != startX) || (row != startY)) && (++n < MAXSOLUTIONLEN)) {
        x = map[row][col].fromX;
        row = map[row][col].fromY;
        col = x;
        argos::LOG << "X:" << col << " Y:" << row << " ";
        argos::LOG << "[X:" << Index2Env(col) << " Y:" << Index2Env(row) << "]" << std::endl;
        }
    if ((col != startX) || (row != startY)) {
        argos::LOG << "Maximum solution nodes reached while searching solution" << std::endl;
        }
    }

void GMP::ShowNode(int x, int y) {
    argos::LOG << "Node X:" << x << " Y:" << y;
    int v = map[y][x].val;
    if (v == OPEN_VAL) {
        argos::LOG << " OPEN" << std::endl;
        }
    else if (v == OBS_VAL) {
        argos::LOG << " OBSTACLE" << std::endl;
        }
    else if (v == NODE_VAL) {
        argos::LOG << " NODE" << std::endl;
        argos::LOG << "     Parent X:" << map[y][x].fromX << " Y:" << map[y][x].fromY << std::endl;
        argos::LOG << "     Children:" << std::endl;
        TreeNodeList* c = map[y][x].children;
        int cn = 0;
        while ((c != NULL) && (++cn < MAXCHILDREN)) {
            argos::LOG << "          Child X:" << c->childX << " Y:" << c->childY << std::endl;
            c = c->next;
            }
        if (c != NULL) {
            argos::LOG << "Maximum children reached during children search" << std::endl;
            }
        }
    else {
        argos::LOG << " UNKNOWN:" << v << std::endl;
        }
    }

void GMP::RRTaddChild(int atX, int atY, int X, int Y) {
    TreeNodeList* child = new TreeNodeList;
    child->childX = X;
    child->childY = Y;
    child->next = NULL;

    if (map[atY][atX].children == NULL) {
        map[atY][atX].children = child;
        // DEBUG argos::LOG << "first child at X:" << X << " Y:" << Y << std::endl;
        }
    else {
        TreeNodeList* p = map[atY][atX].children;
        int c = 0;
        while ((p->next != NULL) && (++c < MAXCHILDREN)) {
            p = p->next;
            }
        if (p->next == NULL) {
            p->next = child;
            // DEBUG argos::LOG << "additional child at X:" << X << " Y:" << Y << std::endl;
            }
        else {
            argos::LOG << "Maximum children reached during children search" << std::endl;
            }
        }
    }

void GMP::RRTinsertNode(int newX, int newY) {
    // Add new node to tree
    map[newY][newX].cost = 0.0;
    map[newY][newX].val = NODE_VAL;
    map[newY][newX].fromX = -1;
    map[newY][newX].fromY = -1;
    map[newY][newX].children = NULL;
    }

void GMP::RRTsteer(int nearX, int nearY, int randX, int randY, int* newX, int* newY) {
    float dx = randX - nearX;
    float dy = randY - nearY;
    float l = sqrt(dx*dx + dy*dy);
    float theta = atan2(dy, dx);
    int step = 2;
    
    if (l > step) {
        l = step;
        }
    
    *newX = int(nearX+l*cos(theta));
    *newY = int(nearY+l*sin(theta));    // CHG 3/28
    }

void GMP::RRTfindNearest(int randX, int randY, int* nearX, int* nearY) {
    int row;
    int col;
    float distance;
    float nearestDistance = 999999999;

    *nearX = -1;
    *nearY = -1;

    for (row = 0; row < Env2Index(EnvirY); row++) {
        for (col = 0; col < Env2Index(EnvirX); col++) {
            if (map[row][col].val == NODE_VAL) {
                distance = sqrt(pow(randY-row, 2)+pow(randX-col, 2));
                if (distance < nearestDistance) {
                    *nearY = row;
                    *nearX = col;
                    nearestDistance = distance;
                    }
                }
            }
        }
    if (*nearX == -1) {
        argos::LOG << "Error locating nearest node to X:" << randX << " Y:" << randY << std::endl;
        }
    }

GMP::TreeNodeList* GMP::RRTnear(int newX, int newY, float radius) {
    GMP::TreeNodeList *N = NULL;
    int row;
    int col;
    float distance;
    int num = 0;

    for (row = 0; row < Env2Index(EnvirY); row++) {
        for (col = 0; col < Env2Index(EnvirX); col++) {
            if (map[row][col].val == NODE_VAL) {
                distance = sqrt(pow(newY-row, 2)+pow(newX-col, 2));
                if (distance < radius) {
                    TreeNodeList *nearby = new TreeNodeList;
                    num++;
                    nearby->childX = col;
                    nearby->childY = row;
                    if (N != NULL) {
                        nearby->next = N;
                        }
                    else {
                        nearby->next = NULL;
                        }
                    N = nearby;
                    }
                }
            }
        }
    // DEBUG argos::LOG << "Found " << num << " nearby." << std::endl;
    return N;
    }

void GMP::RRTremoveEdge(int pX, int pY, int cX, int cY) {
    bool done = false;
    
    // reset cost
    map[cY][cX].cost = 0;
    
    // reset parent
    map[cY][cX].fromX = 0;
    map[cY][cX].fromY = 0;
    
    // remove child
    TreeNodeList* prev = NULL;
    TreeNodeList* c = children(pX, pY);
    while ((!done) && (c != NULL)) {
        if ((c->childX != cX) || (c->childY != cY)) {
            prev = c;
            c = c->next;
            }
        else {
            if (c == children(pX, pY)) {
                // first child in list
                setChildren(pX, pY, c->next);
                }
            else {
                prev->next = c->next;
                }
            delete c;
            done = true;
            }
        }
    if (!done) {
        argos::LOG << "Child not found during removal" << std::endl;
        }
    }

void GMP::RRTinsertEdge(int nearX, int nearY, int newX, int newY) {
    map[newY][newX].cost = map[nearY][nearX].cost + Cost(nearX, nearY, newX, newY);
    map[newY][newX].fromX = nearX;
    map[newY][newX].fromY = nearY;
    RRTaddChild(nearX, nearY, newX, newY);
    }

bool GMP::RRTobstacleFree(int nearX, int nearY, int newX, int newY) {
    bool oFree = true;
    if ((newX == nearX) && (newY == nearY)) {
        oFree = false;
        }
    else if (map[newY][newX].val == OBS_VAL) {
        oFree = false;
        }
    return oFree;
    }

void GMP::RRTextendRRT(int randX, int randY, int* newX, int* newY) {
    *newX = -1;
    *newY = -1;

    if (map[randY][randX].val == OPEN_VAL) {
        // DEBUG argos::LOG << "qrand X:" << randX << " Y:" << randY << std::endl;
        
        // Find nearest node
        int nearX;
        int nearY;
        RRTfindNearest(randX, randY, &nearX, &nearY);
        // DEBUG argos::LOG << "qnear X:" << nearX << " Y:" << nearY << std::endl;
        
        // Find new node in direction from nearest to random node
        RRTsteer(nearX, nearY, randX, randY, newX, newY);
        // DEBUG argos::LOG << "qnew X:" << newX << " Y:" << newY << std::endl;
        
        // If new <> nearest, add node
        if (RRTobstacleFree(nearX, nearY, *newX, *newY)) {
            RRTinsertNode(*newX, *newY);
            RRTinsertEdge(nearX, nearY, *newX, *newY);
            }
        }
    else {
        // DEBUG argos::LOG << ".";
        }
    }

void GMP::RRTextendRRTstar(int randX, int randY, int* newX, int* newY) {
    float radius = 5;
    
    *newX = -1;
    *newY = -1;

    if (map[randY][randX].val == OPEN_VAL) {
        // DEBUG argos::LOG << "qrand X:" << randX << " Y:" << randY << std::endl;
        
        // Find nearest node
        int nearX;
        int nearY;
        RRTfindNearest(randX, randY, &nearX, &nearY);
        // DEBUG argos::LOG << "qnear X:" << nearX << " Y:" << nearY << std::endl;
        
        // Find new node in direction from nearest to random node
        RRTsteer(nearX, nearY, randX, randY, newX, newY);
        // DEBUG argos::LOG << "qnew X:" << newX << " Y:" << newY << std::endl;
        
        // If new <> nearest, add node
        if (RRTobstacleFree(nearX, nearY, *newX, *newY)) {
            RRTinsertNode(*newX, *newY);

            // find node closest (least costly) to new node
            int minX = nearX;
            int minY = nearY;
            int num = 0;
            float minCost = cost(nearX, nearY) + Cost(nearX, nearY, *newX, *newY);
            // DEBUG argos::LOG << "Starting cost " << minCost << std::endl;
            TreeNodeList* N = RRTnear(*newX, *newY, radius);
            TreeNodeList* N1 = N;
            while (N != NULL) {
                if (RRTobstacleFree(N->childX, N->childY, *newX, *newY)) {
                    float nCost = cost(N->childX, N->childY) + Cost(N->childX, N->childY, *newX, *newY);
                    // DEBUG argos::LOG << "Nearby node cost " << nCost << std::endl;
                    if (nCost < minCost) {
                        minCost = nCost;
                        minX = N->childX;
                        minY = N->childY;
                        // DEBUG argos::LOG << "Found closer node" << std::endl;
                        }
                    }
                num++;
                N = N->next;
                }
            // DEBUG argos::LOG << "Searched " << num << " nearby." << std::endl;

            RRTinsertEdge(minX, minY, *newX, *newY);

            // Rewire nearby nodes
            N = N1;
            while (N != NULL) {
                if (!((N->childX == minX) && (N->childY == minY))) {
                    if ((RRTobstacleFree(N->childX, N->childY, *newX, *newY))
                        && (cost(N->childX, N->childY) >
                            (cost(*newX, *newY)+Cost(*newX, *newY, N->childX, N->childY)))) {
                        int parentX;
                        int parentY;
                        parent(N->childX, N->childY, &parentX, &parentY);
                        RRTremoveEdge(parentX, parentY, N->childX, N->childY);
                        RRTinsertEdge(*newX, *newY, N->childX, N->childY);
                        // DEBUG argos::LOG << "Rewired node" << std::endl;
                        }
                    }
                N = N->next;
                }
            }
        }
    else {
        // DEBUG argos::LOG << ".";
        }
    }

bool GMP::RRT() {
    int maxTries;
    bool solutionFound;
    int i;
    
    // Setup contraints
    maxTries = 10000000; //EnvirX * EnvirY;
    solutionFound = false;
    
    // Add start node to solution
    RRTinsertNode(startX, startY);
    
    // Loop maximum number of times
    i = 0;
    while ((i < maxTries) && (!solutionFound)) {
        int randX;
        int randY;
        
        // Get random sample
        randX = int((double(rand()) / RAND_MAX) * Env2Index(EnvirX));
        randY = int((double(rand()) / RAND_MAX) * Env2Index(EnvirY));

        int newX;
        int newY;
        RRTextendRRTstar(randX, randY, &newX, &newY);
        
        if (newX != -1) {
            if ((newX == endX) && (newY == endY)) {
                solutionFound = true;
                }
            }
        i++;
        }
    
    GetParentandChild(startX, startY);
    Getsolution(startX, startY);
    return solutionFound;
    }

void GMP::SetCylinderObstacle(float x0, float y0, float radius) {
    float circum;
    float ang;
    float x;
    float y;

    radius = radius + SWARMSIZE;
    while (radius > (1/GRID)) {
        circum = 2.0*ARGOS_PI*radius;
        ang = 0.0;
        while (ang < 2.0*ARGOS_PI) {
            x = radius*cos(ang);
            y = radius*sin(ang);
            // DEBUG argos::LOG << "Cylinder X:" << x << " Y:" << y << std::endl;
            map[Env2Index(y+y0)][Env2Index(x+x0)].val = OBS_VAL;
            ang = ang + 2.0*ARGOS_PI / (circum * GRID * 2.0);
            }
        radius = radius - 1.0 / (GRID * 3.0);
        }
    }

void GMP::SetBoxObstacle(float x0, float y0, float xs, float ys, float theta) {
    float row;
    float col;
    float x1;
    float y1;
    float x2;
    float y2;
    float x;
    float y;
    float t;
    
    // Increase object size by size of the swarm diameter
    xs = xs + SWARMSIZE;
    ys = ys + SWARMSIZE;
    x1 = x0 - xs / 2.0;
    x2 = x0 + xs / 2.0;
    y1 = y0 - ys / 2.0;
    y2 = y0 + ys / 2.0;
    
    argos::LOG << "Obstacle xs:" << xs << " ys:" << ys << " x1:" << x1 << " y1:" << y1 << " x2:" << x2 << " y2:" << y2 << std::endl;
    row = Env2Index(y1);
    while (row <= Env2Index(y2)) {
        col = Env2Index(x1);
        while (col <= Env2Index(x2)) {
            x = Index2Env(col) - x0;
            y = Index2Env(row) - y0;
            t = x * cos(theta) - y * sin(theta);
            y = x * sin(theta) + y * cos(theta);
            x = t + x0;
            y = y + y0;
            map[Env2Index(y)][Env2Index(x)].val = OBS_VAL;
            col = col + 0.5;
            }
        row = row + 0.5;
        }
    }

void GMP::SetNorthWall(float Y) {
    int row;
    int col;
    
    // DEBUG argos::LOG << "NorthWall:" << Y << " (" << Env2Index(Y) << ")" << std::endl;
    
    // Adjust wall location by swarm size
    Y = Y - SWARMSIZE / 2;
    
    for (row = Env2Index(EnvirY); row > Env2Index(Y); row--) {
        for (col = 0; col < Env2Index(EnvirX); col++) {
            map[row][col].val = OBS_VAL;
            }
        }
    }

void GMP::SetSouthWall(float Y) {
    int row;
    int col;
    
    // DEBUG argos::LOG << "SouthWall:" << Y << " (" << Env2Index(Y) << ")" << std::endl;

    // Adjust wall location by swarm size
    Y = Y + SWARMSIZE / 2;
    
    for (row = 0; row < Env2Index(Y); row++) {
        for (col = 0; col < Env2Index(EnvirX); col++) {
            map[row][col].val = OBS_VAL;
            }
        }
    }

void GMP::SetEastWall(float X) {
    int row;
    int col;
    
    // DEBUG argos::LOG << "EastWall:" << X << " (" << Env2Index(X) << ")" << std::endl;
    
    // Adjust wall location by swarm size
    X = X - SWARMSIZE / 2;
    
    for (row = 0; row < Env2Index(EnvirY); row++) {
        for (col = Env2Index(EnvirX); col > Env2Index(X); col--) {
            map[row][col].val = OBS_VAL;
            }
        }
    }

void GMP::SetWestWall(float X) {
    int row;
    int col;
    
    // DEBUG argos::LOG << "WestWall:" << X << " (" << Env2Index(X) << ")" << std::endl;

    // Adjust wall location by swarm size
    X = X + SWARMSIZE / 2;
    
    for (row = 0; row < Env2Index(EnvirY); row++) {
        for (col = 0; col < Env2Index(X); col++) {
            map[row][col].val = OBS_VAL;
            }
        }
    }

void GMP::ReadEnvironment(float X, float Y) {
    int row;
    int col;
    
    // Range check size, reset to limit if needed
    if (X > ENVSZ) {
        argos::LOG << "Arena size not supported (X > " << ENVSZ << "):" << X <<  " reset to limit" << std::endl;
        X = ENVSZ;
        }
    if (Y > ENVSZ) {
        argos::LOG << "Arena size not supported (Y > " << ENVSZ << "):" << Y <<  " reset to limit" << std::endl;
        Y = ENVSZ;
        }

    // Clear Nodes
    EnvirX = X;
    EnvirY = Y;
    for (row = 0; row < Env2Index(Y); row++) {
        for (col = 0; col < Env2Index(X); col++) {
            map[row][col].val = OPEN_VAL;
            map[row][col].children = NULL;
            map[row][col].fromX = 0;
            map[row][col].fromY = 0;
            }
        }
    }

