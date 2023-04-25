#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <vector>



class Node
{
	// 이전 노드
	int m_pre_node_index;

	// 다음 노드들
	std::vector<int> m_next_node_index;
	
	//현채 위치
	cv::Point m_position;

	int m_current_index;
	double m_cost;

public:

	Node(cv::Point position, int index);
	Node() {};

	// 두 포인트간의 거리를 계산하는 함수 -> cost 계산시 사용
	double euclideanDistance(cv::Point a, cv::Point b);
	//rrt star시 다음 노드들을 제거해주는 함수
	void removeNextIndex(int index);

	//set 함수
	void setPreNodeIndex(int pre);
	void setNextNodeIndex(int next);
	void setCost(Node pre);

	//get 함수
	std::vector<int> getNextNodeIndex();
	int GetIndex();
	int getPretNodeIndex();
	cv::Point getPostion();
	double getCost();


};

