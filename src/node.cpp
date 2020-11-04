#include "node.hpp"

void node::updatePriority(const int & xDest, const int & yDest, int VD)
{
    if (VD)
        priority = level; // dijkstra: f(n) = g(n), and h(n) = 0
    else
        priority = level + estimate(xDest, yDest)*10; // A*: f(n) = g(n) + h(n)
}

void node::updateLevel(const int & i)
{
    // takes into account that a diagonal path is sqrt(2) longer than a horizontal/vertical path
    level += (i % 2 == 0 ? 10 : 14);
}

const int & node::estimate(const int & xDest, const int & yDest) const
{
    // calculate h(n), the total distance remaining to the goal
    static int xd, yd, d;
    xd = xDest - xPos;
    yd = yDest - yPos;

    d = sqrt(xd*xd + yd*yd); // Euclidian Distance

    //d = abs(xd) + abs(yd); // Manhattan distance

    //d = max(abs(xd), abs(yd)); // Chebyshev distance
    return(d);
}
