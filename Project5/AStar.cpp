#include "AStar.h"

std::vector<Node> AStar::routeSearch(std::vector<Node> nodes, cv::Mat maze)
{




    //시작 지점의 인덱스 번호
    int index = 1;

    std::vector<int> closed;
    std::vector<int> open;

    
    Node goal = nodes.at(0);

    while (1) {
        open.push_back(index);

        // index가 가지고 있는 다음 노드 수만큼 for문을 돌린다.
        std::vector<int> next_node_index = nodes.at(index).getNextNodeIndex();
        if (checkGoalNode(next_node_index))
            break;


        //만약 다음 노드로 가는게 없을경우 closed에 가장 최신것을 가지고 온다.
        if (next_node_index.size() == 0) {
            index = closed.at(closed.size()-1);
            closed.pop_back();
            continue;
        }

        

        minCostNode(&index, next_node_index, maze, goal, nodes);
        addClosedNode(index, &closed, next_node_index);
        
        


    }

    std::vector<Node> route = routeAdd(index, nodes);


    return route;
}

double AStar::caculateHuristic(Node current, Node goal, cv::Mat maze)
{
    cv::Point c_position = current.getPostion();
    cv::Point g_position = goal.getPostion();

    double obstacle_value = obstacleCalculate(c_position, g_position, maze);
    double distance = euclideanDistance(c_position, g_position);

    return distance += obstacle_value;

}

double AStar::obstacleCalculate(cv::Point x1, cv::Point x2, cv::Mat maze)
{

    //직선의 방정식 ax+b일떄를 담당하는 함수
    double a;
    double b;
    getLineEquation(&a, &b, x1, x2);
    
    //직선에 장애물 픽셀이 존재 할때마다 카운트 해주는 함수
    int count = 0;
    for (int i = x1.x+1;i < x2.x; i++) {

        int y = a * i + b;
        
        if (maze.at<uchar>(y, i) < 255)
            count++;

    }

    return count*255;
}

double AStar::euclideanDistance(cv::Point a, cv::Point b)
{
    double differ_x = a.x - b.x;
    double differ_y = a.y - b.y;


    return std::sqrt(std::pow(differ_x, 2) + std::pow(differ_y, 2));
}

void AStar::getLineEquation(double* inclination, double* constant_b, cv::Point x1, cv::Point x2)
{
    double differ_y = x1.y - (x2).y;
    double differ_x = (x1.x - (x2).x);

    *inclination = differ_y / differ_x;

    *constant_b = x1.y - x1.x * (*inclination);

}

void AStar::minCostNode(int* index, std::vector<int> next_node_index, cv::Mat maze, Node goal, std::vector<Node> nodes)
{

    double min = 0;
    double pre_min = min;
    for (int i = 0; i < next_node_index.size(); i++) {

        //저장된 다음 인덱스를 찾음
        int next_index = next_node_index.at(i);
        Node node = nodes.at(next_index);
        double F = node.getCost() + caculateHuristic(node, goal, maze);
        if (i == 0) {
            min = F;
            continue;
        }
        //최솟값을 구한다.
        min = std::min(min, F);

        if (pre_min != min)
            *index = next_index;

        pre_min = min;

    }

    //만약 index가 하나여서 비교할 것이 없으면 
    if (next_node_index.size() == 1) {
        *index = next_node_index.at(0);
    }

}

void AStar::addClosedNode(int index, std::vector<int>* closed, std::vector<int> next_node_index)
{
    //next_node의 index를 저장하는 vector에서 휴리스틱한 거리가 가장 짧은 index만 냅두고 다 closed를 진행
    for (int i = 0; i < next_node_index.size();i++) {
        if (next_node_index.at(i) != index)
            (*closed).push_back(next_node_index.at(i));
    }

}

bool AStar::checkGoalNode(std::vector<int> next_node_index)
{

    //다음 노드 index들에 목표 노드 index가 존재하면 true를 리턴
    for (int i = 0; i < next_node_index.size();i++)
        if (next_node_index.at(i) == 0) return true;
    return false;
}

std::vector<Node> AStar::routeAdd(int index, std::vector<Node> nodes)
{
    std::vector<Node> route;
    
    
    while (1) {

        
        route.push_back(nodes.at(index));
        if (index == 1)
            break;
        index = nodes.at(index).getPretNodeIndex();

    }

    return route;
}
