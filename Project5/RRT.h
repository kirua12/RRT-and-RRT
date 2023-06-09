#pragma once
#include "Map.h"
#include "Node.h"
#include "AStar.h"
#include<cmath>
#include <cstdlib>


class RRT
{
	// 랜덤 픽셀 지정 범위
	int m_range;
	//노드의 수
	int m_node_number;

	//노드 전체를 담아두는 vector
	std::vector<Node> m_nodes;

public:
	RRT();
	// 시작과 끝을 설정해주는 코드
	void setFirstEnd(cv::Mat maze_map);
	//rrt 계산 코드
	void calculate(Map maze_map);
	//새로운 노드를 찾는 함수
	Node findNewNode(cv::Mat maze, int *pre_node_number,int index);
	//랜덤 point 생성 함수
	cv::Point findRandomSample(int row, int col);
	// 장애물 체크 함수 장애물이 있으면 false 없으면 true
	bool checkObstacle(Node pre_node, cv::Mat maze, cv::Point *cordi, int star = 1);
	// distance가 가장 가까운 node를 찾는다. 이전노드와 위치가 같거나 범위가 가까우면 -1를 리턴
	int findPreNode(cv::Point cordi);
	// distance가 가장 가까운 node들을 찾는다.
	std::vector<Node> findNearNodes(cv::Point cordi);
	//두 좌표의 거리를 계산해주는 함수
	double euclideanDistance(cv::Point a, cv::Point b);
	// 지도에 시각화 해주는 함수
	void mapVisualization(cv::Mat *maze ,int i);
	void mapVisualization(cv::Mat* maze, int pre_i, int next_i);
	void mapVisualization(cv::Mat* maze, int i, std::vector<int> next_nodes); //rrtstar 구현할때 추가된 함수
	//remove 라인
	void removeLine(cv::Mat* maze, int index, int node_index);
	//astar route를 가시화 해주는 함수
	void mapRouteVisualization(cv::Mat* maze, std::vector<Node> routes);
	//노드가 완성되었을때 마지막 골 목표와 연결해주는 함수
	void connectGoal(cv::Mat *maze);
	//RRT star 알고리즘
	std::vector<int> rrtStar(Node new_node, int* pre_number, cv::Mat maze);
	//가장 cost가 작은 index를 리턴
	int findMinCost(std::vector<Node> near_nodes, cv::Point cordi, cv::Mat maze);
	//노드가 아닌 좌표일 때 cost를 리턴해주는 함수
	double getCost(int pre_number,cv::Point cordi);

};

