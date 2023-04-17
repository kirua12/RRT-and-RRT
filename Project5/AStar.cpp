#include "AStar.h"

std::vector<Node> AStar::routeSearch(std::vector<Node> nodes, cv::Mat maze)
{




    //���� ������ �ε��� ��ȣ
    int index = 1;

    std::vector<int> closed;
    std::vector<int> open;

    
    Node goal = nodes.at(0);

    while (1) {
        open.push_back(index);

        // index�� ������ �ִ� ���� ��� ����ŭ for���� ������.
        std::vector<int> next_node_index = nodes.at(index).getNextNodeIndex();
        if (checkGoalNode(next_node_index))
            break;


        //���� ���� ���� ���°� ������� closed�� ���� �ֽŰ��� ������ �´�.
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

    //������ ������ ax+b�ϋ��� ����ϴ� �Լ�
    double a;
    double b;
    getLineEquation(&a, &b, x1, x2);
    
    //������ ��ֹ� �ȼ��� ���� �Ҷ����� ī��Ʈ ���ִ� �Լ�
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

        //����� ���� �ε����� ã��
        int next_index = next_node_index.at(i);
        Node node = nodes.at(next_index);
        double F = node.getCost() + caculateHuristic(node, goal, maze);
        if (i == 0) {
            min = F;
            continue;
        }
        //�ּڰ��� ���Ѵ�.
        min = std::min(min, F);

        if (pre_min != min)
            *index = next_index;

        pre_min = min;

    }

    //���� index�� �ϳ����� ���� ���� ������ 
    if (next_node_index.size() == 1) {
        *index = next_node_index.at(0);
    }

}

void AStar::addClosedNode(int index, std::vector<int>* closed, std::vector<int> next_node_index)
{
    //next_node�� index�� �����ϴ� vector���� �޸���ƽ�� �Ÿ��� ���� ª�� index�� ���ΰ� �� closed�� ����
    for (int i = 0; i < next_node_index.size();i++) {
        if (next_node_index.at(i) != index)
            (*closed).push_back(next_node_index.at(i));
    }

}

bool AStar::checkGoalNode(std::vector<int> next_node_index)
{

    //���� ��� index�鿡 ��ǥ ��� index�� �����ϸ� true�� ����
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
