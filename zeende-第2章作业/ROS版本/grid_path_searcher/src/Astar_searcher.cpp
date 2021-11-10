#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   //地图下边界
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);
    //地图上边界
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    //各个方向栅格数量
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    //三维栅格个数
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    
    //开辟存放栅格的内存
    data = new uint8_t[GLXYZ_SIZE];
    //将数字以单个字节逐个拷贝的方式放到指定的内存中去
    //void *memset(void *s,int c,size_t n) 将已开辟内存空间 s 的首 n 个字节的值设为值 c。
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));


    // 分配栅格地图
    //GridNode 表示,存储了节点的坐标、g(n)、f(n)值、父节点指针等信息。
    GridNodeMap = new GridNodePtr ** [GLX_SIZE]; //三级指针 --->整个地图 解引用后是坐标为x的每个面的“数组”
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];//二级指针 --->坐标为x的每个面的“数组”  解引用后是坐标为x y的每个列的“数组”
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];//指针 --->坐标为x,y的每个列的“数组” 解引用后是坐标为x y z的每个栅格
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);  //把栅格编号转换为地图坐标
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);//--->用栅格编号、地图坐标初始化
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    /*
     * 拓展节点函数
    STEP 4: finish AstarPathFinder::AstarGetSucc
    please write your code below
    */
    Eigen::Vector3i current_index = currentPtr->index;
    int current_x = current_index[0];
    int current_y = current_index[1];
    int current_z = current_index[2];
//    cout<<MAGENTA<<" debug------- "<<WHITE<<endl;
//    int current_x = currentPtr->index(0);
//    int current_y = currentPtr->index(1);
//    int current_z = currentPtr->index(2);
    int neighbor_x,neighbor_y,neighbor_z;
    for(int i = -1; i <= 1; i++){
        for(int j = -1; j <= 1; j++){
            for(int k = -1; k <= 1; k++){
                if((i==0)&&(j==0)&&(k==0)) continue;
                neighbor_x = current_x + i;
                neighbor_y = current_y + j;
                neighbor_z = current_z + k;

                //防止超出范围
                if( (neighbor_x<0) || (neighbor_y<0) ||(neighbor_z<0) || (neighbor_x>GLX_SIZE-1) || (neighbor_y>GLY_SIZE-1) ||(neighbor_z>GLZ_SIZE-1) )
                    continue;
                //是否障碍物
                if(isOccupied(neighbor_x,neighbor_y,neighbor_z))
                    continue;
                //判断是否被弹出了
                GridNodePtr neighbor_ptr = GridNodeMap[neighbor_x][neighbor_y][neighbor_z];
                if(neighbor_ptr->id == -1)
                    continue;

                neighborPtrSets.push_back(neighbor_ptr);
                double cost = std::sqrt(std::pow(i,2)+std::pow(j,2)+std::pow(k,2));
                edgeCostSets.push_back(cost);
            }
        }
    }
}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *启发式函数
    STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    please write your code below
    */
    int heuristic_fun = Diagonal;

    double h(0.);
    Eigen::Vector3i this_index =  node1->index;
    Eigen::Vector3i end_index =  node2->index;
    double dx = abs((double)(this_index(0) - end_index(0)));
    double dy = abs((double)(this_index(1) - end_index(1)));
    double dz = abs((double)(this_index(2) - end_index(2)));
    switch(heuristic_fun)
    {
        case Diagonal:{
            double distance[3] = {dx,dy,dz};
            std::sort(distance,distance+3);
            //TODO 计算距离
            h = (std::sqrt(3.0)-std::sqrt(2.0))*distance[0] + (std::sqrt(2.0)-1)*distance[1]+distance[2];
            //h = distance[0] + distance[1] + distance[2] +(std::sqrt(3.0)-3) * distance[0] + (std::sqrt(2.0)-2)*distance[1];
            break;}

        case Manhattan:{
            h = dx + dy + dz;
            break;}
        case Euclidean:{
            h = std::sqrt((std::pow(dx,2.0) + std::pow(dy,2.0)+std::pow(dz,2.0)));
            break;}
        case Dijkstra:h = 0.; break;

        default:
            break;
    }

    if(Use_Tie_Breaker){
        double p = 1.0/25.;
        h = h *(1.0+p);
    }
    return h;
}

//A*搜索算法函数
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{
    //开始计算A*的时间
    cout<<BLUE<<"A* Finder start"<<WHITE<<endl;
    ros::Time time_1 = ros::Time::now();    

    //记录起点和终点
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    //计算点在实际栅格地图位置
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //初始化起点和终点的index和坐标
//    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
//    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
//
    GridNodePtr startPtr = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr   = GridNodeMap[end_idx(0)][end_idx(1)][end_idx(2)];

    //清空multimap<>
    openSet.clear();
    //定义弹出的节点和拓展节点
    GridNodePtr currentPtr  = nullptr;
    GridNodePtr neighborPtr = nullptr;

    //把起点丢入openlist 计算g f
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);

    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( {startPtr -> fScore, startPtr });
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    */
    //定义拓展的队列和移动cost
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        //弹出第一个节点
        currentPtr = openSet.begin()->second;
        currentPtr->id = -1;//标记已弹出
        openSet.erase(openSet.begin());//弹出


        //STEP 4: finish AstarPathFinder::AstarGetSucc
        // 拓展周边
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);
        /*
        STEP 5:  For all unexpanded neigbors "m" of node "n",
        */
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            Judge if the neigbors have been expanded
            please write your code below
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            double gh = currentPtr -> gScore + edgeCostSets[i];
            double fh = gh + getHeu(neighborPtr, endPtr);
            //如果没有被访问过
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                */

                neighborPtr->gScore = gh;
                neighborPtr->fScore = fh;
                neighborPtr->cameFrom = currentPtr; //添加父节点信息
                openSet.insert({fh,neighborPtr});
                // 如果目标出现 结束
                if( currentPtr->index == goalIdx ){
                    ros::Time time_2 = ros::Time::now();
                    terminatePtr = currentPtr;
                    ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );
                    return;
                }
                else{
                    neighborPtr->id = 1;
                    continue;
                }

            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                */
                if(neighborPtr->gScore > gh)
                {//覆盖
                    neighborPtr->gScore = gh;
                    neighborPtr->fScore = fh;
                    neighborPtr->cameFrom = currentPtr;
                }

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                continue;
            }
        }      
    }
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */

    GridNodePtr father_ptr = terminatePtr;
    while(father_ptr->cameFrom != NULL )
    {
        gridPath.push_back(father_ptr);
        father_ptr = father_ptr->cameFrom;
    }

    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}