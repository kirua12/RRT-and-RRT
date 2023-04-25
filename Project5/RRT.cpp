#include "RRT.h"



RRT::RRT()
{
	m_range = 20;

	m_node_number = 250;

}



void RRT::setFirstEnd(cv::Mat maze_map)
{

	// ��������� ���������� �̸� �־��ִ� �ڵ�
	///////////////////////////////////////////////////

	//�������
	cv::Point s_position(0, maze_map.rows - 1);
	//��������
	cv::Point e_position(maze_map.cols - 1, 0);

	//���۳��� ���� ���
	Node s_node(s_position,1);
	Node e_node(e_position,0);

	m_nodes.push_back(e_node);
	m_nodes.push_back(s_node);

	//////////////////////////////////////////////////

}

void RRT::calculate(Map maze_map)
{

	
	cv::Mat maze = maze_map.GetMap();
	//����ȭ�� ���� �ʴ� original img
	cv::Mat ori_maze = maze.clone();

	setFirstEnd(maze);


	// �Ķ���� ���� ��ŭ ��带 �����ϴ� for��
	for (int i = 2; i < m_node_number - 2; i++) {

		//���ο� ���� ���� ����� ��� �ѹ��� ����ϴ� ����
		int pre_node_number;

		Node new_node = findNewNode(maze, &pre_node_number, i);

		std::vector<int> next_nodes = rrtStar(new_node, &pre_node_number,maze);
		
		m_nodes.push_back(new_node);


		//���ο� ��忡 ���尡��� ��带 ���� ���� ���� �� ���� ��忡�� ���� ���� �缳��
		m_nodes.at(pre_node_number).setNextNodeIndex(i);
		m_nodes.at(i).setPreNodeIndex(pre_node_number);
		//cost����
		m_nodes.at(i).setCost(m_nodes.at(pre_node_number));
		mapVisualization(&maze, i, next_nodes);


	}
	connectGoal(&maze);

	AStar A;
	std::vector<Node> route = A.routeSearch(m_nodes,ori_maze);
	mapRouteVisualization(&maze, route);
}



Node RRT::findNewNode(cv::Mat maze, int* pre_node_number,int index)
{
	//���� ���� ��ǥ
	cv::Point cordi;

	// ���� ��ǥ�� ��ֹ��� ã��
	while (1) {
		cordi = findRandomSample(maze.rows, maze.cols);


		//���� ���Ÿ��� ���� ����� ��带 ã��
		*pre_node_number = findPreNode(cordi);
		if (*pre_node_number == -1) continue;
		Node pre_node = m_nodes.at(*pre_node_number);

		if (checkObstacle(pre_node, maze, &cordi) == true) break;

	}

	return Node{ cordi ,index };
}

cv::Point RRT::findRandomSample(int row, int col)
{

	int random_x = rand() % col;
	int random_y = rand() % row;

	cv::Point result(random_x, random_y);

	return result;
}

bool RRT::checkObstacle(Node pre_node, cv::Mat maze, cv::Point* cordi, int star)
{
	cv::Point pre_postion = pre_node.getPostion();

	//���� ��ġ ���� ����� ��ġ�� ���� �� ����

	double differ_y = pre_postion.y - (*cordi).y;
	double differ_x = (pre_postion.x - (*cordi).x);

	double angle = std::atan(differ_y/differ_x);



	//x�� ��� �ֳ��� ���� ��� ���� �������ִ� ����
	int k = 1;

	if (pre_postion.x > (*cordi).x) {
		k = -1;
	}	

	cvtColor(maze, maze, cv::COLOR_BGR2GRAY);


	cv::Point new_point;

	//star�� 1�ϋ��� �Ϲ����� ��� 2�϶��� rrtstar
	int node_distance = m_range;
	if(star ==2)
		node_distance = euclideanDistance(pre_postion, *cordi);
	for (int i = 5; i < node_distance + 1;i++) {

		new_point.x = pre_postion.x + i * cos(angle) * k;
		new_point.y = pre_postion.y + i * sin(angle) * k;

		if (new_point.x < 0 || new_point.x >= maze.cols || new_point.y < 0 || new_point.y >= maze.rows) return false;
		if (maze.at<uchar>(new_point.y, new_point.x) < 255) return false;
	}


	//��ġ�� ����ų� ��ֹ��� ������ false�� ����
	


	(*cordi).x = new_point.x;
	(*cordi).y = new_point.y;

	//���� �����̶� ������ ���� 
	if (findPreNode(*cordi) == -1) return false;


	return true;
}

int RRT::findPreNode(cv::Point cordi)
{


	double min = 100000000;
	//���� ����� �ѹ�
	int pre_number = 0;
	double pre_min = min;

	//���� ���� �߿� �ּҰ��� ã�´�.
	for (int i = 1; i < m_nodes.size();i++) {

		min = std::min(euclideanDistance(m_nodes.at(i).getPostion(), cordi), min);

		//���� �޶����� pre_number���� ��ȯ
		if (pre_min != min)
			pre_number = i;

		pre_min = min;

	}

	//������ ���� ������ ���� ������
	if (min <= 10) return -1;

	return pre_number;

}

std::vector<Node> RRT::findNearNodes(cv::Point cordi)
{
	double min = 100000000;
	//���� ����� �ѹ�
	int pre_number = 0;
	double pre_min = min;
	std::vector<Node> near_nodes;

	//���� ���� �߿� �ּҰ��� ã�´�.
	for (int i = 1; i < m_nodes.size();i++) {

		if (euclideanDistance(m_nodes.at(i).getPostion(), cordi) <= 2*m_range) {
			near_nodes.push_back(m_nodes.at(i));
		}



	}


	return near_nodes;
}


double RRT::euclideanDistance(cv::Point a, cv::Point b)
{
	double differ_x = a.x - b.x;
	double differ_y = a.y - b.y;


	return std::sqrt(std::pow(differ_x, 2) + std::pow(differ_y, 2));
}

void RRT::mapVisualization(cv::Mat* maze, int i)
{
	cv::Point node_cordi = m_nodes.at(i).getPostion();
	int pre_node_index = m_nodes.at(i).getPretNodeIndex();
	cv::Point pre_node_cordi = m_nodes.at(pre_node_index).getPostion();
	cv::circle(*maze, node_cordi, 3, cv::Scalar(255, 0, 0), -1);

	cv::line(*maze, node_cordi, pre_node_cordi, cv::Scalar(255, 0, 0));

	cv::imshow("maze", *maze);
	cv::waitKey(30);
}


void RRT::mapVisualization(cv::Mat* maze, int i, std::vector<int> next_nodes)
{
	//���� ���� �����ְ� ���� ���� �������ش�.
	for (int j = 0; j < next_nodes.size(); j++) {
		removeLine(maze, next_nodes.at(j), i);
		mapVisualization(maze, next_nodes.at(j));
	}

	cv::Point node_cordi = m_nodes.at(i).getPostion();

	int pre_node_index = m_nodes.at(i).getPretNodeIndex();
	cv::Point pre_node_cordi = m_nodes.at(pre_node_index).getPostion();
	cv::circle(*maze, node_cordi, 3, cv::Scalar(255, 0, 0), -1);

	cv::line(*maze, node_cordi, pre_node_cordi, cv::Scalar(255, 0, 0));

	cv::imshow("maze", *maze);
	cv::waitKey(30);
}

void RRT::removeLine(cv::Mat* maze, int index, int node_index)
{
	cv::Point cordi = m_nodes.at(index).getPostion();
	int pre_index = m_nodes.at(index).getPretNodeIndex();
	cv::Point pre_cordi = m_nodes.at(pre_index).getPostion();
	cv::line(*maze, cordi, pre_cordi, cv::Scalar(255, 255, 255));
	m_nodes.at(index).setPreNodeIndex(node_index);
	m_nodes.at(index).setCost(m_nodes.at(node_index));
	m_nodes.at(node_index).setNextNodeIndex(index);
	m_nodes.at(pre_index).removeNextIndex(index);
}

void RRT::mapVisualization(cv::Mat* maze, int pre_i, int next_i)
{
	cv::Point node_cordi = m_nodes.at(next_i).getPostion();
	cv::Point pre_node_cordi = m_nodes.at(pre_i).getPostion();
	cv::circle(*maze, node_cordi, 3, cv::Scalar(255, 0, 0), -1);

	cv::line(*maze, node_cordi, pre_node_cordi, cv::Scalar(255, 0, 0));

	cv::imshow("maze", *maze);
	cv::waitKey(30);



}

void RRT::mapRouteVisualization(cv::Mat* maze, std::vector<Node> routes)
{
	for (int i = routes.size()-1; i > 0;i--) {
		cv::Point node_cordi = routes.at(i).getPostion();
		cv::Point pre_node_cordi = routes.at(i-1).getPostion();
		cv::circle(*maze, node_cordi, 3, cv::Scalar(0, 0, 255), -1);

		cv::line(*maze, node_cordi, pre_node_cordi, cv::Scalar(0, 0, 255));

		cv::imshow("maze", *maze);
		cv::waitKey(30);
	}

	
	//������ ��θ� �̾��ִ� �Լ�
	cv::Point node_cordi = routes.at(0).getPostion();
	cv::Point pre_node_cordi = m_nodes.at(0).getPostion();
	cv::circle(*maze, node_cordi, 3, cv::Scalar(0, 0, 255), -1);
	cv::circle(*maze, pre_node_cordi, 3, cv::Scalar(0, 0, 255), -1);

	cv::line(*maze, node_cordi, pre_node_cordi, cv::Scalar(0, 0, 255));

	cv::imshow("maze", *maze);
	cv::waitKey(1000);

	//cv::waitKey(0);
}

void RRT::connectGoal(cv::Mat* maze)
{
	// ������ ���
	Node goal = m_nodes.at(0);


	//�����ȿ� ��ǥ ��带 ���� ������ ã��
	for (int i = 1; i < m_nodes.size();i++) {


		double distance = euclideanDistance(m_nodes.at(i).getPostion(), goal.getPostion());

		if (distance <= 3*m_range) {
			m_nodes.at(i).setNextNodeIndex(0);
			mapVisualization(maze, i, 0);
		}
	}
}

std::vector<int> RRT::rrtStar(Node new_node, int* pre_number, cv::Mat maze)
{
	cv::Point cordi = new_node.getPostion();
	std::vector<Node> near_nodes;
	near_nodes = findNearNodes(cordi);
	// ���� cost�� ���� index�� ã��
	int min_index = findMinCost(near_nodes, cordi,maze);
	if (min_index != -1) {
		*pre_number = near_nodes.at(min_index).GetIndex();
	}

	double cost = getCost(*pre_number, cordi);


	std::vector<int> next_index;
	//�ֺ� ��忡�� �� ����� index�� �ִ��� ã�´�
	for (int i = 0; i < near_nodes.size(); i++) {
		cv::Point near_cordi = near_nodes.at(i).getPostion();
		double pre_cost = near_nodes.at(i).getCost();
		if (checkObstacle(near_nodes.at(i), maze, &cordi,2)==false)
			cost += 5000;
		double new_cost = cost + euclideanDistance(near_cordi, cordi);
		if (new_cost < pre_cost) {
			next_index.push_back(near_nodes.at(i).GetIndex());
		}
		
	}




	return next_index;
}

int RRT::findMinCost(std::vector<Node> near_nodes, cv::Point cordi, cv::Mat maze)
{

	double min_cost = 100000;
	double pre_min_cost = min_cost;
	int min_index = -1;
	//cost�� ���� ���� index�� ã�´�.
	for (int i = 0; i < near_nodes.size(); i++) {

		cv::Point near_cordi = near_nodes.at(i).getPostion();
		double cost = near_nodes.at(i).getCost() + euclideanDistance(near_cordi, cordi);
		if (checkObstacle(near_nodes.at(i), maze, &cordi,2)==false)
			cost=cost + 5000;
		min_cost = std::min(cost, min_cost);
		if (pre_min_cost != min_cost)
			min_index = i;
		pre_min_cost = min_cost;
	}

	return min_index;
}

double RRT::getCost(int pre_number, cv::Point cordi)
{

	double cost = m_nodes.at(pre_number).getCost() + euclideanDistance(m_nodes.at(pre_number).getPostion(), cordi);
	return cost;
}


