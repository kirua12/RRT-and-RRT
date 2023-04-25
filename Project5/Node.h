#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>



class Node
{
	// ���� ���
	int m_pre_node_index;

	// ���� ����
	std::vector<int> m_next_node_index;
	
	//��ä ��ġ
	cv::Point m_position;

	int m_current_index;
	double m_cost;

public:

	Node(cv::Point position, int index);
	Node() {};

	// �� ����Ʈ���� �Ÿ��� ����ϴ� �Լ� -> cost ���� ���
	double euclideanDistance(cv::Point a, cv::Point b);
	//rrt star�� ���� ������ �������ִ� �Լ�
	void removeNextIndex(int index);

	//set �Լ�
	void setPreNodeIndex(int pre);
	void setNextNodeIndex(int next);
	void setCost(Node pre);

	//get �Լ�
	std::vector<int> getNextNodeIndex();
	int GetIndex();
	int getPretNodeIndex();
	cv::Point getPostion();
	double getCost();


};

