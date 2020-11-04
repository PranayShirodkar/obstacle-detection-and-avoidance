#ifndef NODE_H
#define NODE_H
#include <stdio.h>
#include <iostream>
#include <math.h>

class node
{
public:
    node(int xp, int yp, int d, int p) : xPos(xp), yPos(yp), level(d), priority(p) {}
    int getxPos() const { return xPos; }
    int getyPos() const { return yPos; }
    int getLevel() const { return level; }
    int getPriority() const { return priority; }
    void updatePriority(const int & xDest, const int & yDest, int VD);
    void updateLevel(const int & i);
    const int & estimate(const int & xDest, const int & yDest) const;

private:
    // node position
    int xPos;
    int yPos;

    // g(n), total distance travelled to reach this node
    int level;

    // priority=level+remaining distance estimate
    // when this value is smaller, it means higher priority or shorter path
    int priority;
};
#endif
