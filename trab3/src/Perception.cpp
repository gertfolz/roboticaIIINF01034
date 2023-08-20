#include "Perception.h"
#include <unistd.h>
#include <iostream>

#include <queue>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/////////////////////////////////////
/// CONSTRUTOR e FUNCOES PUBLICAS ///
/////////////////////////////////////

Perception::Perception(ros::NodeHandle& n):
    nh_(n)
{
    started_ = false;
    directionOfNavigation_ = 0.0;
    validDirection_=false;

    // Initialize transform listener
    tfListener_ = new tf2_ros::TransformListener(tfBuffer_);

    // Initialize publishers
    pub_mapOccType_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_occ_types", 1);
    pub_mapPlanType_ = nh_.advertise<nav_msgs::OccupancyGrid>("/mapa_plan_types", 1);
    pub_directionOfNavigation_ = nh_.advertise<geometry_msgs::PoseStamped>("/direction_navigation", 1);
 
    // Initialize subscribers
    sub_gridmap_ = nh_.subscribe("/mapa_laser_HIMM", 1, &Perception::receiveGridmap, this);
}

bool Perception::hasValidDirection()
{
    if(validDirection_==false)
        std::cout << "Direcao de navegacao nao calculada" << std::endl;
    return validDirection_;
}

double Perception::getDirectionOfNavigation()
{
    return directionOfNavigation_;
}

/////////////////////////////////////
/// Callback dos topicos de MAPA  ///
/////////////////////////////////////

void Perception::receiveGridmap(const nav_msgs::OccupancyGrid::ConstPtr &value)
{
    // STRUCTURE OF nav_msgs::OccupancyGrid
    // # This represents a 2-D grid map, in which each cell represents the probability of occupancy.
    // Header header 
    // 
    // # MetaData for the map
    //   # The time at which the map was loaded
    //   time map_load_time
    //   # The map resolution [m/cell]
    //   float32 resolution
    //   # Map width [cells]
    //   uint32 width
    //   # Map height [cells]
    //   uint32 height
    //   # The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
    //   geometry_msgs/Pose origin
    // MapMetaData info
    //
    // # The map data, in row-major order, starting with (0,0).  
    // # Occupancy probabilities are in the range [0,100].  Unknown is -1.
    // # OBS: implemented in c++ with std::vector<u_int8>
    // int8[] data

    if(started_==false){

        // At the first time, initialize all variables and maps
        numCellsX_ = value->info.width;
        numCellsY_ = value->info.height;

        float cellSize = value->info.resolution;

        mapWidth_ = numCellsX_*cellSize;
        mapHeight_ = numCellsY_*cellSize;
        scale_ = 1.0/cellSize;

        occupancyTypeGrid_.resize(numCellsX_*numCellsY_,OCC_UNEXPLORED);
        
        planningTypeGrid_.resize(numCellsX_*numCellsY_,PLAN_INVALID);
        fValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        gValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        hValueGrid_.resize(numCellsX_*numCellsY_,DBL_MAX);
        parentGrid_.resize(numCellsX_*numCellsY_,-1);

        minKnownX_ = numCellsX_-1;
        minKnownY_ = numCellsY_-1;
        maxKnownX_ = maxKnownY_ = 0;

        started_=true;
    }

    // Copy the occupancy grid map to the occupancyTypeGrid_ (which will be modified next)
    for(unsigned int i=0; i<numCellsX_*numCellsY_; i++)
        occupancyTypeGrid_[i] = value->data[i];

    // Classify cells
    updateGridKnownLimits();
    updateCellsClassification();

    // Get cell in free space closest to the robot position
    // This is required because the robot may be near obstacles, 
    // in regions where the planning is not performed
    Pose2D robotPose = getCurrentRobotPose();
    int robotIndexInFreeSpace = getNearestFreeCell(robotPose);

    // Select center of nearest viable frontier
    int nearestFrontierIndex = clusterFrontiersAndReturnIndexOfClosestOne(robotIndexInFreeSpace);
    if(nearestFrontierIndex != -1){

        // Compute A*
        // first - compute heuristic in all cells (euclidian distance to the goal)
        computeHeuristic(nearestFrontierIndex);
        // second - compute the A* algorithm
        int goal = computeShortestPathToFrontier(robotIndexInFreeSpace);

        // Printing the index of the goal cell, must be the same as 'nearestFrontierIndex'
        std::cout << "goal " << goal << std::endl; //

        // Mark path cells for vizualization
        markPathCells(goal);

        // Compute direction of navigation based on the path
        double yaw = computeDirectionOfNavigation(robotIndexInFreeSpace, goal);
        directionOfNavigation_ = normalizeAngleDEG(RAD2DEG(yaw)-robotPose.theta);
        validDirection_=true;

        // Update and publish direction of navigation
        msg_directionOfNavigation_.header = value->header;
        msg_directionOfNavigation_.pose.position.x = robotPose.x;
        msg_directionOfNavigation_.pose.position.y = robotPose.y;
        msg_directionOfNavigation_.pose.position.z = 0;
        tf2::Quaternion quat_tf;
        quat_tf.setRPY( 0, 0, yaw );
        msg_directionOfNavigation_.pose.orientation=tf2::toMsg(quat_tf);
        pub_directionOfNavigation_.publish(msg_directionOfNavigation_);

    }else{
        validDirection_=false;
    }

    // Publish occupancyTypeGrid_ and planningTypeGrid_
    msg_occTypes_.header = value->header;
    msg_occTypes_.info = value->info;
    msg_occTypes_.data = occupancyTypeGrid_;
    pub_mapOccType_.publish(msg_occTypes_);

    msg_planTypes_.header = value->header;
    msg_planTypes_.info = value->info;
    msg_planTypes_.data = planningTypeGrid_;
    pub_mapPlanType_.publish(msg_planTypes_);
}

//////////////////////////////////////////////////////////////////
/// FUNCOES DE PLANEJAMENTO DE CAMINHOS - A SEREM PREENCHIDAS  ///
//////////////////////////////////////////////////////////////////

void Perception::updateCellsClassification()
{
    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'occupancyTypeGrid_'
    /// e atualize os valores, marcando como condição de contorno se for o caso

    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;
    /// ex: atualizando uma celula como ocupada
    ///    occupancyTypeGrid_[i] = OCC_OCCUPIED;

    /// grid na entrada (valor inteiro): 
    /// - celulas desconhecidas = -1 
    /// - celulas conhecidas com ocupação variando de 0 a 100

    /// grid na saida (valor inteiro):
    /// - celulas desconhecidas                = OCC_UNEXPLORED   (definido como -1)
    /// - celulas de obstaculo                 = OCC_OCCUPIED     (definido como 100)
    /// - celulas livres vizinhas a obstaculos = OCC_NEAROBSTACLE (definido como 90)
    /// - celulas de fronteira                 = OCC_FRONTIER     (definido como 30)
    /// - demais celulas livres                = OCC_FREE         (definido como 50)

    // Dica
    // 1) Marcar obstaculos
    // 2) Marcar celulas vizinhas a obstaculos considerando 'dangerZoneWidth' celulas
    int dangerZoneWidth = 5;
    // 3) Marcar fronteiras (ignorando OCC_OCCUPIED e OCC_NEAROBSTACLE)
    // 4) Marcar restantes, que nao sao inexploradas, como livre

    // Loop over all cells in the known area of the grid
    for (int x = minKnownX_; x <= maxKnownX_; x++) {
        for (int y = minKnownY_; y <= maxKnownY_; y++) {
            bool closeToObstacle = false;
            bool isFrontier = false;
            int i = x + y * numCellsX_;
            
            if (occupancyTypeGrid_[i] != -1) {
                for (int nx = -dangerZoneWidth; nx <= dangerZoneWidth; nx++){
                        for (int ny = -dangerZoneWidth; ny <= dangerZoneWidth; ny++){
                            int k =  (x+nx) + (y+ny) * numCellsX_;
                            if(occupancyTypeGrid_[k]==100 && (k!=i))
                                closeToObstacle = true;
                        }
                }
                for (int nx = -1; nx <= 1; nx++){
                            for (int ny = -1; ny <= 1; ny++){
                                int j =  (x+nx) + (y+ny) * numCellsX_;
                                if(occupancyTypeGrid_[j]==-1 && (j!=i))
                                    isFrontier = true;
                            }
                }
                if (occupancyTypeGrid_[i] >= 100) {
                    occupancyTypeGrid_[i] = OCC_OCCUPIED;
                }
                // If the cell is free but close to an obstacle, mark it as near obstacle
                else if (occupancyTypeGrid_[i] < 100 && closeToObstacle==true) {
                    occupancyTypeGrid_[i] = OCC_NEAROBSTACLE;
                }
                // If the cell is free and next to an unexplored cell, mark it as a frontier
                else if (isFrontier==true)
                    occupancyTypeGrid_[i] = OCC_FRONTIER;
                // If the cell is free and not next to an unexplored cell, mark it as free
                else {
                    occupancyTypeGrid_[i] = OCC_FREE;
                }
            }
        }
    }
}

void Perception::computeHeuristic(int goalIndex)
{
    int goalX = goalIndex % numCellsX_;
    int goalY = goalIndex / numCellsX_;

    /// TODO:
    /// varra as celulas dentro dos limites conhecidos do mapa 'planningTypeGrid_'
    /// e atualize os valores das medidas f, g, h e pi das celulas validas
    
    /// coordenada x das celulas vai de minKnownX_ a maxKnownX_
    /// coordenada y das celulas vai de minKnownY_ a maxKnownY_
    /// compute o indice da celula no grid usando:
    ///    int i = x + y*numCellsX_;

    /// Dica: uma celula 'i' eh valida se (planningTypeGrid_[i] != PLAN_INVALID)

    /// A atualizacao deve seguir as seguintes regras:
    ///   fValueGrid_[i] e gValueGrid_[i] - valores f e g: recebem DBL_MAX (equivalente ao infinito)
    ///   parentGrid_[i] - valor pi (indicando o pai da celula): recebe -1, pois a priori nao aponta para ninguem. 
    ///   hValueGrid_[i] - valor h - distancia para o goal
        // Loop over all cells in the known area of the grid
    for (int x = minKnownX_; x <= maxKnownX_; x++) {
        for (int y = minKnownY_; y <= maxKnownY_; y++) {
            // Compute the index of the cell in the grid
            int i = x + y * numCellsX_;
            if (planningTypeGrid_[i] != PLAN_INVALID) {
                // Update the f and g values to infinity
                fValueGrid_[i] = DBL_MAX;
                gValueGrid_[i] = DBL_MAX;

                // Update the pi value to -1
                parentGrid_[i] = -1;

                // Update the h value to the Euclidean distance to the goal
                int dx = x - goalX;
                int dy = y - goalY;
                hValueGrid_[i] = sqrt(dx * dx + dy * dy);
            }
        }
    }

}

// offset para os 8 vizinhos
// uso, i-esimo vizinho (nx,ny) da posicao (x,y):
//      int nx = x+offset[i][0];
//      int ny = y+offset[i][1];
int offset[8][2] = {{-1,  1}, { 0,  1}, { 1,  1}, { 1,  0}, { 1, -1}, { 0, -1}, {-1, -1}, {-1,  0}};

// custo de distancia para os 8 vizinhos
// uso, atualizando custo do i-esimo vizinho
//      int id_celula  =  x +  y*numCellsX_;
//      int id_vizinho = nx + ny*numCellsX_;
//      gValueGrid_[id_vizinho] = gValueGrid_[id_celula] + cost[i];
double cost[8] = {sqrt(2), 1, sqrt(2), 1, sqrt(2), 1, sqrt(2), 1};

int Perception::computeShortestPathToFrontier(int robotCellIndex)
{
    int rx = robotCellIndex % numCellsX_;
    int ry = robotCellIndex / numCellsX_;

    /// TODO:
    /// Computar o algoritmo A Star usando os valores em hValueGrid_ 
    /// e atualizando os valores em fValueGrid_, gValueGrid_ e parentGrid_

    /// Ao fim deve retornar o indice da celula de goal, encontrada na busca. Ou -1 se nao encontrar
    int goal = -1;

    /// Sugestao: usar a fila de prioridades abaixo
    /// onde o primeiro elemento do par eh o f-value e o segundo elemento eh o indice da celula
    std::priority_queue< std::pair<double, int> , std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>> > pq;

    /// Exemplo: insercao na fila
    //      std::pair<double, int> vizinho;
    //      vizinho.first = fValueGrid_[id_vizinho];
    //      vizinho.second = id_vizinho;
    //      pq.push(vizinho);

    /// Exemplo: remocao da fila
    //      std::pair<double, int> celula = pq.top();
    //      pq.pop();
    //      int id_celula = celula.second;

    /// O algoritmo comeca da posicao do robo
    gValueGrid_[robotCellIndex] = 0;

    std::pair<double, int> inicio;
    inicio.first = fValueGrid_[robotCellIndex];
    inicio.second = robotCellIndex;

    pq.push(inicio);

    /// Completar algoritmo A Star, consultando a fila enquanto ela nao estiver vazia
    ///     while(!pq.empty())
        while(!pq.empty()) {
        std::cout<<pq.size();
        usleep(1000);
        std::pair<double, int> currentCell = pq.top();
        pq.pop();
        int id_currentCell = currentCell.second;
        int cx = id_currentCell % numCellsX_;
        int cy = id_currentCell / numCellsX_;
        

        // If the current cell is a frontier, set it as the goal and break the loop
        if (planningTypeGrid_[id_currentCell] == PLAN_GOALS) {
            goal = id_currentCell;
            std::cout<<"aq";
        }

        // Loop over the 8 neighbors of the current cell
        for (int i=0; i<8; i++){
                int nx = cx+offset[i][0];
                int ny = cy+offset[i][1];
                

                // // Skip cells outside the known area of the grid
                // if (nx < minKnownX_ || nx > maxKnownX_ || ny < minKnownY_ || ny > maxKnownY_) {
                //     continue;
                // }

                int id_neighbor = nx + ny * numCellsX_;

                // Skip invalid cells
                if (planningTypeGrid_[id_neighbor] != PLAN_INVALID) {
                    // Compute the cost to reach the neighbor through the current cell
                    double dist = gValueGrid_[id_currentCell] + cost[i];

                    // If this is a shorter path to the neighbor, update its g and f values and set the current cell as its parent
                    if (dist < gValueGrid_[id_neighbor]) {
                        gValueGrid_[id_neighbor] = dist;
                        fValueGrid_[id_neighbor] = dist + hValueGrid_[id_neighbor];
                        parentGrid_[id_neighbor] = id_currentCell;

                        // Add the neighbor to the queue
                        std::pair<double, int> neighbor;
                        neighbor.first = fValueGrid_[id_neighbor];
                        neighbor.second = id_neighbor;
                        pq.push(neighbor);
                        std::cout<<"aq2";

                    }
                }
            }
        }
    return goal;
}

////////////////////////////////////////////////////////
/// FUNCOES AUXILIARES PARA PLANEJAMENTO DE CAMINHOS ///
////////////////////////////////////////////////////////

// Given the explored area, update the following variables: minKnownX_, maxKnownX_, minKnownY_, maxKnownY_
void Perception::updateGridKnownLimits()
{
    for(int x=0; x<numCellsX_; x++){
        for(int y=0; y<numCellsY_; y++){
            int i = x + y*numCellsX_;
            if(occupancyTypeGrid_[i]!=-1)
            {
                if(x<minKnownX_) minKnownX_ = x;
                if(y<minKnownY_) minKnownY_ = y;
                if(x>maxKnownX_) maxKnownX_ = x;
                if(y>maxKnownY_) maxKnownY_ = y;
            }
        }
    }
}

// Groups all frontier cells in clusters, and keep the ones with size greater than 'minFrontierSize' 
// Then selects the centers of each of the frontiers
// Next, choose the one closest to the robot's position as the goal
// Returns the index of the cell associated with the center of the selected frontier
int Perception::clusterFrontiersAndReturnIndexOfClosestOne(int robotCellIndex)
{
    frontierCentersIndices.clear();

    int width=1;
    int minFrontierSize = 3;

    // Check occupancyTypeGrid_ and set PLAN_GOALS in planningTypeGrid_
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            if(occupancyTypeGrid_[i] == OCC_FRONTIER) 
                planningTypeGrid_[i] = PLAN_GOALS;      // Frontier cells are goals
            else if(occupancyTypeGrid_[i] == OCC_FREE) 
                planningTypeGrid_[i] = PLAN_REGULAR;    // Free cells are regular cells (where path can be computed)
            else
                planningTypeGrid_[i] = PLAN_INVALID;    // Remaining cells are invalid for planning
        }
    }

    // Group all neighboring goal cells
    for(int x=minKnownX_; x<=maxKnownX_; x++){
        for(int y=minKnownY_; y<=maxKnownY_; y++){
            int i = x + y*numCellsX_;

            // detect a goal cell that is not MARKED yet
            if(planningTypeGrid_[i] == PLAN_GOALS){
                planningTypeGrid_[i] = PLAN_MARKEDGOALS;

                std::vector<unsigned int> frontier;

                float centerx = 0, centery = 0;
                float count = 0;

                // mark all neighbor goal cells
                // breadth-first search using a queue
                std::queue<int> q;
                q.push(i);
                while(!q.empty())
                {
                    int c = q.front();
                    q.pop();
                    frontier.push_back(c);

                    int cx = c % numCellsX_;
                    int cy = c / numCellsX_;
                    centerx += cx;
                    centery += cy;
                    count++;

                    for(int nx=cx-width;nx<=cx+width;nx++){
                        for(int ny=cy-width;ny<=cy+width;ny++){
                            int ni = nx + ny*numCellsX_;
                            if(planningTypeGrid_[ni] == PLAN_GOALS){
                                planningTypeGrid_[ni] = PLAN_MARKEDGOALS;
                                q.push(ni);
                            }
                        }
                    }
                }

                // keep frontiers that are larger than minFrontierSize
                if(count > minFrontierSize){
                    centerx /= count;
                    centery /= count;
 
                    // find cell closest to frontier center
                    float minDist=FLT_MAX;
                    int closest=-1;

                    for(unsigned int k=0;k<frontier.size();k++){
                        int fx = frontier[k] % numCellsX_;
                        int fy = frontier[k] / numCellsX_;

                        float dist = sqrt(pow(fx-centerx,2.0)+pow(fy-centery,2.0));
                        if(dist < minDist){
                            minDist = dist;
                            closest = frontier[k];
                        }
                    }

                    // add center of frontier to list of Goals
                    frontierCentersIndices.push_back(closest);

                }else{

                    // ignore small frontiers
                    for(unsigned int k=0;k<frontier.size();k++){
                        planningTypeGrid_[frontier[k]] = PLAN_REGULAR;
                    }
                }
            }
        }
    }

    // These are the filtered frontiers (that are not too small)
    std::cout << "Number of frontiers: " << frontierCentersIndices.size() << std::endl;
    for(unsigned int k=0;k<frontierCentersIndices.size();k++){
        planningTypeGrid_[frontierCentersIndices[k]] = PLAN_GOALS;
    }

    if(frontierCentersIndices.empty())
        return -1;
    else{

        // Select nearest frontier among the filtered frontiers
        int nearestFrontierIndex=-1;
        float distance = DBL_MAX;

        int rx = robotCellIndex % numCellsX_;
        int ry = robotCellIndex / numCellsX_;
        for(int k=0;k<frontierCentersIndices.size();k++){
            int nFx = frontierCentersIndices[k] % numCellsX_;
            int nFy = frontierCentersIndices[k] / numCellsX_;
            float d = sqrt(pow(rx-nFx,2.0)+pow(ry-nFy,2.0));
            if(d < distance)
            {
                distance = d;
                nearestFrontierIndex = frontierCentersIndices[k];
            }
        }

        // Clear frontiers that were not selected
        for(int k=0;k<frontierCentersIndices.size();k++){
            if(frontierCentersIndices[k] != nearestFrontierIndex)
                planningTypeGrid_[frontierCentersIndices[k]] = PLAN_MARKEDGOALS;
        }

        return nearestFrontierIndex;
    }
}

// Mark all path cells in the 'planningTypeGrid_' after the A* Star algorithm is computed
// by checking the index of the parent of each cell starting from the goal
void Perception::markPathCells(int goal)
{
    if(goal != -1){

        int c = parentGrid_[goal];

        while(c != -1){
            planningTypeGrid_[c] = PLAN_PATH;
            c = parentGrid_[c];
        }
    }
}

// Select a path cell in a distance given by 'localGoalRadius' from the robot
// and compute the angle difference from the robot orientation to this cell
double Perception::computeDirectionOfNavigation(int robotCellIndex, int goalIndex)
{
    int rx = robotCellIndex % numCellsX_;
    int ry = robotCellIndex / numCellsX_;

    int c = goalIndex;

    double localGoalRadius = 5;

    int cx, cy;

    while(parentGrid_[c] != -1){
        cx = c % numCellsX_;
        cy = c / numCellsX_;

        double dist = sqrt(pow(cx-rx,2.0)+pow(cy-ry,2.0));
        
        if(dist < localGoalRadius){
            break;
        }
        c = parentGrid_[c];
    }

    double yaw = atan2(cy-ry,cx-rx);

    return yaw;
}

// Return robot pose
Pose2D Perception::getCurrentRobotPose()
{
    Pose2D robotPose;

    // Get robot transformation given by ODOM
    geometry_msgs::TransformStamped transformStamped;
    try{ transformStamped = tfBuffer_.lookupTransform("odom", "base_link", ros::Time(0)); }
    catch (tf2::TransformException &ex) { ROS_WARN("%s",ex.what()); }
    
    robotPose.x = transformStamped.transform.translation.x;
    robotPose.y = transformStamped.transform.translation.y;

    // Convert quaternion to euler angles
    tf2::Quaternion q4(transformStamped.transform.rotation.x,
                       transformStamped.transform.rotation.y, 
                       transformStamped.transform.rotation.z, 
                       transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m4(q4);
    double roll, pitch, yaw;
    m4.getRPY(roll,pitch,yaw);

    robotPose.theta = RAD2DEG(yaw);

    return robotPose;
}

// Return index of the free cell closest to the robot position
int Perception::getNearestFreeCell(Pose2D robot)
{
    int rx = robot.x*scale_ + numCellsX_/2;
    int ry = robot.y*scale_ + numCellsY_/2; 

    int u;

    for(int l=1; l<20; l++){

        for(int cx=rx-l; cx<=rx+l; cx++){
            u = cx + (ry+l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = cx + (ry-l)*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

        for(int cy=ry-l; cy<=ry+l; cy++){
            u = rx+l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
            u = rx-l + cy*numCellsX_;
            if(occupancyTypeGrid_[u] == OCC_FREE)
                return u;
        }

    }

    return -1;
}
