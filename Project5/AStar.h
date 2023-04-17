#pragma once
#include<vector>
#include "Node.h"


class AStar
{
	//�޸���ƽ ��� �Լ�
	double caculateHuristic(Node current, Node goal,cv::Mat maze);
	//��ֹ� �޸���ƽ �Ÿ� ���ϴ� �Լ�
	double obstacleCalculate(cv::Point x1, cv::Point x2, cv::Mat maze);
	//��Ŭ����� �Ÿ� ���ϴ� �Լ�
	double euclideanDistance(cv::Point a, cv::Point b);
	//������ ������ ���ϴ� �Լ�
	void getLineEquation(double* inclination, double* constant_b,cv::Point x1, cv::Point x2);
	// ���� ���� �߿��� ���� ª�� ��带 ����
	void minCostNode(int* index, std::vector<int> next_node_index, cv::Mat maze, Node goal, std::vector<Node> nodes);
	//closed �߰����ִ� �Լ�
	void addClosedNode(int index, std::vector<int>* closed, std::vector<int> next_node_index);
	//���� ��� index�鿡 ��ǥ ��� index�� �����ϸ� true�� ����

	bool checkGoalNode(std::vector<int> next_node_index);
	//���� ��常 �ް� ��������� ������� ����
	std::vector<Node> routeAdd(int index, std::vector<Node> nodes);
public:


	AStar() {};

	std::vector<Node> routeSearch(std::vector<Node> nodes,cv::Mat maze);
	

};

