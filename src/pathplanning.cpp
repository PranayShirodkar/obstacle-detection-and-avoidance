#include <ros/ros.h>
// library includes
#include <math.h>
#include <string>
#include <iomanip>
#include <queue>
#include <ctime>
#include <vector>
#include <opencv2/opencv.hpp>
// local includes
#include "node.hpp"

using namespace std;

const int MAP_SIZE = 50;
int xStart = 1, yStart = 1, xGoal = MAP_SIZE - 2, yGoal = MAP_SIZE - 2;
const int IMAGE_SIZE = 600;
static bool VD = 1; //Voronoi Dijsktra if true, A*  if false
static int obstaclestate = 1; //select which obstacle map to use

static int Map[MAP_SIZE][MAP_SIZE] = {0};
static int dir_map[MAP_SIZE][MAP_SIZE]; // map of directions
const int dir = 8; // 8 possible directions from any node
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1}; //direction vectors, in x and y axis
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};
static int closed_nodes_map[MAP_SIZE][MAP_SIZE] = {0}; // map of closed (tried-out) nodes
static int open_nodes_map[MAP_SIZE][MAP_SIZE] = {0}; // map of open (not-yet-tried) nodes
static double totalpathcost = 0;

struct Obstacle
{
    // for Voronoi Diagram to use
    int obsnum;
    int x;
    int y;
};

vector<Obstacle> Obstacles;

bool operator<(const node & a, const node & b)
{
    // Determines priority (in the priority queue)
    return a.getPriority() > b.getPriority();
}

void printMap(int Map[MAP_SIZE][MAP_SIZE])
{
    std::string line;
    for(int j = 0; j < MAP_SIZE; j++)
    {
        for(int i = 0; i < MAP_SIZE; i++)
            line.push_back('0' + Map[j][i]);
        ROS_INFO("%s",line.c_str());
        line.clear();
    }
}

void colourImage(cv::Mat mapvis, int x, int y, int b, int g, int r)
{
    // this function colours the image, allowing us to visualise the map and path
    int factor = IMAGE_SIZE/MAP_SIZE;
    for (int i = 0; i < factor; i++)
    {
        for (int j = 0; j < factor; j++)
        {
            mapvis.at<cv::Vec3b>(x*factor+i,y*factor+j)[0] = b;
            mapvis.at<cv::Vec3b>(x*factor+i,y*factor+j)[1] = g;
            mapvis.at<cv::Vec3b>(x*factor+i,y*factor+j)[2] = r;
        }
    }
}

void placeRect(int xTopLeft, int yTopLeft, int height, int width, int obsnumber, cv::Mat mapvis)
{
    // set entire rectangle as obstacle, add to vector of Obstacles, 1 on map, colour mapvis black
    Obstacle temp;
    temp.obsnum = obsnumber;
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            temp.x = xTopLeft+i; temp.y = yTopLeft+j;
            Obstacles.push_back(temp);
            Map[temp.x][temp.y] = 1;
            colourImage(mapvis, temp.x, temp.y, 0, 0, 0);
        }
    }
}

void borderObstacles(int obsnumber, cv::Mat mapvis)
{
    // sets borders of map as an obstacle
    int p = MAP_SIZE;
    Obstacle temp;
    temp.obsnum = obsnumber;
    for (int i = 0; i < p; i++)
    {
        temp.x = i; temp.y = 0;
        Obstacles.push_back(temp);
        Map[temp.x][temp.y] = 1;
        colourImage(mapvis, temp.x, temp.y, 0, 0, 0);
        temp.x = i; temp.y = p-1;
        Obstacles.push_back(temp);
        Map[temp.x][temp.y] = 1;
        colourImage(mapvis, temp.x, temp.y, 0, 0, 0);
    }
    for (int i = 1; i < p-1; i++)
    {
        temp.x = 0; temp.y = i;
        Obstacles.push_back(temp);
        Map[temp.x][temp.y] = 1;
        colourImage(mapvis, temp.x, temp.y, 0, 0, 0);
        temp.x = p-1; temp.y = i;
        Obstacles.push_back(temp);
        Map[temp.x][temp.y] = 1;
        colourImage(mapvis, temp.x, temp.y, 0, 0, 0);
    }
}

void generateSparseRectangleObstacles(cv::Mat mapvis)
{
    //sets Sparse rectangle Obstacles, Start, Goal & borders
    int p = MAP_SIZE;
    Obstacle temp;
    temp.obsnum = 1; temp.x = xStart; temp.y = yStart;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 0, 255);
    temp.obsnum = 2; temp.x = xGoal; temp.y = yGoal;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 255, 0);
    placeRect(3*p/4, 2*p/5, p/12, p/2, 3, mapvis);
    placeRect(2*p/5, p/3, p/10, p/2, 4, mapvis);
    placeRect(p/10, p/8, 3*p/5, p/12, 5, mapvis);
    borderObstacles(6, mapvis);
}

void generateDenseRectangleObstacles(cv::Mat mapvis)
{
    //sets dense rectangle Obstacles, Start, Goal & borders
    int p = MAP_SIZE;
    Obstacle temp;
    temp.obsnum = 1; temp.x = xStart; temp.y = yStart;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 0, 255);
    temp.obsnum = 2; temp.x = xGoal; temp.y = yGoal;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 255, 0); 
    placeRect(p/15, 5*p/6, p/4, p/12, 3, mapvis);//3
    placeRect(p/11, 11*p/20, p/10, p/5, 4, mapvis);//4
    placeRect(p/9, 2*p/5, p/4, p/15, 5, mapvis);//5
    placeRect(p/8, p/18, p/10, p/4, 6, mapvis);//6
    placeRect(7*p/20, 4*p/5, p/12, p/7, 7, mapvis);//7
    placeRect(p/4, 25*p/40, 9*p/20, p/12, 8, mapvis);//8
    placeRect(3*p/7, 6*p/20, p/10, p/4, 9, mapvis);//9
    placeRect(p/3, p/7, p/3, p/8, 10, mapvis);//10
    placeRect(p/2, 4*p/5, p/6, p/9, 11, mapvis);//11
    placeRect(7*p/10, 9*p/10, p/5, p/15, 12, mapvis);//12
    placeRect(8*p/10, 21*p/40, p/12, 3*p/10, 13, mapvis);//13
    placeRect(13*p/20, p/3, p/5, p/11, 14, mapvis);//14
    placeRect(8*p/10, p/12, p/10, p/6, 15, mapvis);//15
    borderObstacles(16, mapvis); //16
}

void generateShoreObstacles(cv::Mat mapvis)
{
    //sets shore/jetty like rectangle Obstacles, Start, Goal & borders
    int p = MAP_SIZE;
    Obstacle temp;
    temp.obsnum = 1; temp.x = xStart; temp.y = yStart;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 0, 255);
    temp.obsnum = 2; temp.x = xGoal; temp.y = yGoal;
    Obstacles.push_back(temp);
    Map[temp.x][temp.y]=1;
    colourImage(mapvis, temp.x, temp.y, 0, 255, 0);
    placeRect(16*p/20, p/3, p/15, (p-p/3)-1, 3, mapvis);//3
    placeRect(3*p/20, 1, p/15, 2*p/3, 4, mapvis);//4
    borderObstacles(5, mapvis); //5
}

void generateObstacles(int obstaclestate, cv::Mat mapvis)
{
    if (obstaclestate == 1)
        generateSparseRectangleObstacles(mapvis);
    else if (obstaclestate == 2)
        generateDenseRectangleObstacles(mapvis);
    else if (obstaclestate == 3)
        generateShoreObstacles(mapvis);
}

double euclidDist(int x0, int y0, int x1, int y1)
{
    return sqrt(double(((x1-x0)*(x1-x0))+((y1-y0)*(y1-y0))));
}

void VoronoiDiagram(cv::Mat mapvis) // converts Map and mapvis into VD, showing only Voronoi edges that are not to close to any obstacle as traversable
{
    for (int i = 0; i < MAP_SIZE; i++)
    {
        for (int j = 0; j < MAP_SIZE; j++)
        {
            // for every point in Map
            if (Map[i][j] == 0) // if it is not an obstacle
            {
                double min_dist = INT_MAX;
                int min_dist_obsnum = -1;
                bool isVoronoiEdge = false;
                for (int k = 0; k < Obstacles.size(); k++)
                {
                    // find distance to every element in Obstacles
                    double current_dist = euclidDist(i, j, Obstacles[k].x, Obstacles[k].y);
                    double threshold1 = current_dist*0.3;
                    double threshold2 = current_dist*0.5;

                    if (current_dist < min_dist && Obstacles[k].obsnum == min_dist_obsnum)
                    {
                        // update min_dist if it is the same obstacle
                        min_dist = current_dist;
                        min_dist_obsnum = Obstacles[k].obsnum;
                        isVoronoiEdge = false;
                    }
                    else if (current_dist < (min_dist-threshold1))
                    {
                        // update min_dist if a new nearer obstacle is found that is nearer by more than threshold1
                        min_dist = current_dist;
                        min_dist_obsnum = Obstacles[k].obsnum;
                        isVoronoiEdge = false;
                    }
                    else if (abs(current_dist-min_dist) < threshold2 && Obstacles[k].obsnum != min_dist_obsnum)
                    {
                        // if the difference in min_dist to two different obstacles is below threshold2, then we have a Voronoi Edge
                        //(assuming that the Edge is not less than 3m to any obstacle)
                        isVoronoiEdge = true;
                        if (current_dist < 3) //minimum 3m distance between USV and any possible path
                        {
                            isVoronoiEdge = false;                      
                            break;
                        }
                    }
                }
                if (!isVoronoiEdge && min_dist_obsnum != 1 && min_dist_obsnum != 2)
                {
                    // Edges that are not Voronoi edges, and are not closest to Start obstacle nor Goal obstacle, are not traversable.
                    // set to grey on mapvis and 7 on map
                    Map[i][j] = 7;
                    colourImage(mapvis, i, j, 200, 200, 200);
                }
            }
        }
    }
}


void setintMapandMapvis(string path, cv::Mat mapvis)
{
    //set start and goal on Map and mapvis image
    Map[xStart][yStart] = 2;
    Map[xGoal][yGoal] = 4;

    int pathlen = path.length();
    if(path.empty())
        ROS_INFO("NO Path found!");

    else if(pathlen > 0)// follow the path and set Map and mapvis image
    {
        int x = xStart, y = yStart;
        for(int i = 0; i < pathlen; i++)
        {
            char c = path.at(i);
            int j = atoi(&c); 
            x = x + dx[j];
            y = y + dy[j];
            Map[x][y] = 3;
            colourImage(mapvis, x, y, 0, 0+(255*i/pathlen), 255*(pathlen-i)/pathlen); //colour the path from red to green, from Start to Goal
        }
    }
}

// A-star algorithm.
// The output is a string of indices for the dx and dy arrays that define the path
string pathFind( const int & xStart, const int & yStart, const int & xGoal, const int & yGoal )
{
    Map[xStart][yStart] = 0; Map[xGoal][yGoal] = 0;
    static priority_queue<node> pq[2]; // queue of open nodes (yet to be tried) in order of highest priority to lowest priority (highest priority is lowest priority value)
    static int pqindex = 0; // pq index
    static node* pnode, * cnode; //template node pointers for node manipulation of parent node and child node respectively
    static int x, y; // position of current node

    pnode = new node(xStart, yStart, 0, 0); // create the start node and push onto queue of open nodes
    pnode->updatePriority(xGoal, yGoal, VD);
    pq[pqindex].push(*pnode);
    open_nodes_map[xStart][yStart] = pnode->getPriority(); // mark it on the open nodes map

    // A*/dijsktra search
    while(!pq[pqindex].empty())
    {
        // top of pq is the open node with highest priority or lowest priority value
        x = pq[pqindex].top().getxPos(); y = pq[pqindex].top().getyPos();

        pnode = new node(x, y, pq[pqindex].top().getLevel(), pq[pqindex].top().getPriority());

        pq[pqindex].pop(); // remove the node from pq
        open_nodes_map[x][y] = 0; // mark it on the open nodes map and closed nodes map
        closed_nodes_map[x][y] = 1;

        // end condition, stop path finding
        //if((*pnode).estimate(xGoal, yGoal) == 0)
        if(x == xGoal && y == yGoal) 
        {
            // generate the path in reverse from goal to start by following the directions
            string path = "";
            while(!(x == xStart && y == yStart))
            {
                int i = dir_map[x][y];
                char c = '0' + (i + dir/2)%dir;
                path = c+path;
                x += dx[i];
                y += dy[i];
            }
            totalpathcost = pnode->getLevel();
            // call destructor
            delete pnode;
            // empty the remaining nodes in pq
            while(!pq[pqindex].empty()) 
            {
                pq[pqindex].pop();
            }
            return path;
        }

        // generate moves (child nodes) in all possible directions
        for(int i=0; i < dir; i++)
        {
            int xdx = x + dx[i]; int ydy = y + dy[i]; // position of child node

            if(xdx > -1 && xdx < MAP_SIZE && ydy > -1 && ydy < MAP_SIZE && Map[xdx][ydy] == 0 && closed_nodes_map[xdx][ydy] == 0)
            {
                // if all conditions are met, generate a valid child node
                cnode = new node( xdx, ydy, pnode->getLevel(), pnode->getPriority());
                cnode->updateLevel(i);
                cnode->updatePriority(xGoal, yGoal, VD);

                if(open_nodes_map[xdx][ydy] == 0)
                {
                    // if this is a new unexplored node, add it to open nodes map and pq
                    open_nodes_map[xdx][ydy] = cnode->getPriority();
                    pq[pqindex].push(*cnode);
                    // mark its parent node direction
                    dir_map[xdx][ydy] = (i + dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy] > cnode->getPriority())
                {
                    // if this is an already explored node, and it has higher priority, update the priority info
                    open_nodes_map[xdx][ydy] = cnode->getPriority();
                    dir_map[xdx][ydy]=(i + dir/2)%dir;

                    // replace the node with the better version
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqindex].top().getxPos() == xdx && pq[pqindex].top().getyPos() == ydy))
                    {
                        pq[1 - pqindex].push(pq[pqindex].top());
                        pq[pqindex].pop();
                    }
                    pq[pqindex].pop(); // remove the wanted node
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqindex].size() > pq[1 - pqindex].size()) 
                    {
                        pqindex=1 - pqindex;
                    }
                    while(!pq[pqindex].empty())
                    {
                        pq[1 - pqindex].push(pq[pqindex].top());
                        pq[pqindex].pop();
                    }
                    pqindex=1 - pqindex;
                    pq[pqindex].push(*cnode); // add the better node instead
                }
                delete cnode; // call destructor
            }
        }
        delete pnode; // call destructor 
    }
    return std::string(); // size zero, no route found
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pathplanning");
    ros::Time::init();
    cv::Mat mapvis(IMAGE_SIZE,IMAGE_SIZE, CV_8UC3, cv::Scalar(255,150,0)); //b,g,r//colour of water
    cv::namedWindow( "Planned Path", CV_WINDOW_AUTOSIZE );

    generateObstacles(obstaclestate, mapvis); //select which obstaclestate, colours mapvis with obstacles in black
    cv::imshow( "Planned Path", mapvis);
    cv::waitKey(0); 

    ros::Time Voronoibegin = ros::Time::now();
    if (VD)
        VoronoiDiagram(mapvis);
    ros::Time Voronoiend = ros::Time::now();
    cv::imshow( "Planned Path", mapvis);
    cv::waitKey(0);

    ros::Time pathfindbegin = ros::Time::now();
   string path=pathFind(xStart, yStart, xGoal, yGoal); // get the path, measure the beginning and end time
   ros::Time pathfindend = ros::Time::now();

    setintMapandMapvis(path, mapvis); // sets map and mapvis with path, red to green

    //printMap(Map);
    cv::imshow( "Planned Path", mapvis);
    if (VD)
        cout << "Voronoi execution time: " << Voronoiend-Voronoibegin << "s" << endl;
    cout << "Path Planning execution time: " << pathfindend-pathfindbegin << "s" << endl;
    cout << "Map Length per side: " << MAP_SIZE << "metres" << endl;    
    cout << "Total path length: " << (totalpathcost/10) << "metres" << endl;

    cv::waitKey(0);
    cv::destroyWindow("Planned Path");

    return 0;
}
