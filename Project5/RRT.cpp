#include "RRT.h"



RRT::RRT()
{
	m_range = 20;

	m_node_number = 250;

}



void RRT::setFirstEnd(cv::Mat maze_map)
{

	// 출발지점과 종료지점을 미리 넣어주는 코드
	///////////////////////////////////////////////////

	//출발지점
	cv::Point s_position(0, maze_map.rows - 1);
	//종료지점
	cv::Point e_position(maze_map.cols - 1, 0);

	//시작노드와 종료 노드
	Node s_node(s_position,1);
	Node e_node(e_position,0);

	m_nodes.push_back(e_node);
	m_nodes.push_back(s_node);

	//////////////////////////////////////////////////

}

void RRT::calculate(Map maze_map)
{

	
	cv::Mat maze = maze_map.GetMap();
	//가시화를 하지 않는 original img
	cv::Mat ori_maze = maze.clone();

	setFirstEnd(maze);


	// 파라미터 노드수 만큼 노드를 생성하는 for문
	for (int i = 2; i < m_node_number - 2; i++) {

		//새로운 노드와 가장 가까운 노드 넘버를 기록하는 변수
		int pre_node_number;

		Node new_node = findNewNode(maze, &pre_node_number, i);

		std::vector<int> next_nodes = rrtStar(new_node, &pre_node_number,maze);
		
		m_nodes.push_back(new_node);


		//새로운 노드에 가장가까운 노드를 이전 노드로 설정 및 이전 노드에서 현재 노드로 재설정
		m_nodes.at(pre_node_number).setNextNodeIndex(i);
		m_nodes.at(i).setPreNodeIndex(pre_node_number);
		//cost설정
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
	//랜덤 변수 좌표
	cv::Point cordi;

	// 랜덤 좌표에 장애물을 찾음
	while (1) {
		cordi = findRandomSample(maze.rows, maze.cols);


		//이전 노드거리와 가장 가까운 노드를 찾음
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

	//랜덤 위치 가장 가까운 위치의 기울기 와 각도

	double differ_y = pre_postion.y - (*cordi).y;
	double differ_x = (pre_postion.x - (*cordi).x);

	double angle = std::atan(differ_y/differ_x);



	//x가 어디에 있냐의 따라 양수 음수 결정해주는 변수
	int k = 1;

	if (pre_postion.x > (*cordi).x) {
		k = -1;
	}	

	cvtColor(maze, maze, cv::COLOR_BGR2GRAY);


	cv::Point new_point;

	//star가 1일떄는 일반적인 경우 2일때는 rrtstar
	int node_distance = m_range;
	if(star ==2)
		node_distance = euclideanDistance(pre_postion, *cordi);
	for (int i = 5; i < node_distance + 1;i++) {

		new_point.x = pre_postion.x + i * cos(angle) * k;
		new_point.y = pre_postion.y + i * sin(angle) * k;

		if (new_point.x < 0 || new_point.x >= maze.cols || new_point.y < 0 || new_point.y >= maze.rows) return false;
		if (maze.at<uchar>(new_point.y, new_point.x) < 255) return false;
	}


	//위치가 벗어나거나 장애물이 있으면 false를 리턴
	


	(*cordi).x = new_point.x;
	(*cordi).y = new_point.y;

	//이전 노드들이랑 가까우면 새로 
	if (findPreNode(*cordi) == -1) return false;


	return true;
}

int RRT::findPreNode(cv::Point cordi)
{


	double min = 100000000;
	//이전 노드의 넘버
	int pre_number = 0;
	double pre_min = min;

	//이전 노드들 중에 최소값을 찾는다.
	for (int i = 1; i < m_nodes.size();i++) {

		min = std::min(euclideanDistance(m_nodes.at(i).getPostion(), cordi), min);

		//값이 달라지면 pre_number값을 변환
		if (pre_min != min)
			pre_number = i;

		pre_min = min;

	}

	//범위의 절반 이하의 값이 있으면
	if (min <= 10) return -1;

	return pre_number;

}

std::vector<Node> RRT::findNearNodes(cv::Point cordi)
{
	double min = 100000000;
	//이전 노드의 넘버
	int pre_number = 0;
	double pre_min = min;
	std::vector<Node> near_nodes;

	//이전 노드들 중에 최소값을 찾는다.
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
	//이전 노드와 끊어주고 현재 노드와 연결해준다.
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

	
	//마지막 경로만 이어주는 함수
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
	// 도착지 노드
	Node goal = m_nodes.at(0);


	//범위안에 목표 노드를 가는 노드들을 찾음
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
	// 가장 cost가 적은 index를 찾음
	int min_index = findMinCost(near_nodes, cordi,maze);
	if (min_index != -1) {
		*pre_number = near_nodes.at(min_index).GetIndex();
	}

	double cost = getCost(*pre_number, cordi);


	std::vector<int> next_index;
	//주변 노드에서 더 가까운 index가 있는지 찾는다
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
	//cost가 가장 작은 index를 찾는다.
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


