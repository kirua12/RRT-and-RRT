#pragma once
#include<vector>
#include "Node.h"


class AStar
{
	//휴리스틱 계산 함수
	double caculateHuristic(Node current, Node goal,cv::Mat maze);
	//장애물 휴리스틱 거리 구하는 함수
	double obstacleCalculate(cv::Point x1, cv::Point x2, cv::Mat maze);
	//유클리디안 거리 구하는 함수
	double euclideanDistance(cv::Point a, cv::Point b);
	//직선의 방정식 구하는 함수
	void getLineEquation(double* inclination, double* constant_b,cv::Point x1, cv::Point x2);
	// 다음 노드들 중에서 가장 짧은 노드를 선택
	void minCostNode(int* index, std::vector<int> next_node_index, cv::Mat maze, Node goal, std::vector<Node> nodes);
	//closed 추가해주는 함수
	void addClosedNode(int index, std::vector<int>* closed, std::vector<int> next_node_index);
	//다음 노드 index들에 목표 노드 index가 존재하면 true를 리턴

	bool checkGoalNode(std::vector<int> next_node_index);
	//최종 노드만 받고 이전노드들로 최적경로 생성
	std::vector<Node> routeAdd(int index, std::vector<Node> nodes);
public:


	AStar() {};

	std::vector<Node> routeSearch(std::vector<Node> nodes,cv::Mat maze);
	

};

