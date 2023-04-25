#include "Node.h"

Node::Node(cv::Point position,int index)
{
	m_position = position;
	m_cost = 0;
	m_current_index = index;


}

double Node::euclideanDistance(cv::Point a, cv::Point b)
{
	double differ_x = a.x - b.x;
	double differ_y = a.y - b.y;


	return std::sqrt(std::pow(differ_x, 2) + std::pow(differ_y, 2));
}

void Node::setPreNodeIndex(int pre)
{
	m_pre_node_index = pre;

}

void Node::setNextNodeIndex(int next)
{
	m_next_node_index.push_back(next);

}

void Node::removeNextIndex(int index)
{
	for (int i = 0;i < m_next_node_index.size();i++) {
		if(m_next_node_index.at(i)== index)
			m_next_node_index.erase(m_next_node_index.begin() + i);
	}

}

void Node::setCost(Node pre)
{

	m_cost = euclideanDistance(pre.getPostion(), m_position)+pre.getCost();
}


std::vector<int> Node::getNextNodeIndex()
{
	return m_next_node_index;
}


int Node::getPretNodeIndex()
{
	return m_pre_node_index;
}


cv::Point Node::getPostion()
{
	return m_position;
}

int Node::GetIndex()
{
	return m_current_index;
}

double Node::getCost()
{
	return m_cost;
}



