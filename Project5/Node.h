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

	double cost;

public:

	Node(cv::Point position);
	Node() {};
	double euclideanDistance(cv::Point a, cv::Point b);


	//set �Լ�
	void setPreNodeIndex(int pre);
	void setNextNodeIndex(int next);
	void setCost(Node pre);

	//get �Լ�
	std::vector<int> getNextNodeIndex();
	int getPretNodeIndex();
	cv::Point getPostion();
	double getCost();


};

