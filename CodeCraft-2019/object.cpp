#include "object.hpp"
#include <queue>
using namespace std;
GRAPH::route_type* GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed)
{
	/*
	Dijkstra求最短路径
	*/

	priority_queue<__Node*, vector<__Node*>, __Node::Compare> candidates;
	map<CROSS::id_type, bool> visited_map;				// 记录节点是否被访问过
	map<CROSS::id_type, weight_type> distance;			// 记录源节点到每个节点的开销
	p_weight = weight_map[speed];
	visited_map[from] = true;
	distance[from] = 0;
	__Node *pre_node = nullptr;							// 记录上个入visited_map的节点
	CROSS::id_type pre_index = from;					// 记录上个入visited_map的节点的cross_id
	__Node *record;
	__Node *mid;
	vector<__Node*> del_nodes;
	while (visited_map.find(to) == visited_map.end())
	{
		vector<Node*> &start = graph_map[pre_index];
		for(int i=0;i<start.size();i++)
		{
			if ()										// 判断这个边的容量是否小于等于0
			{
				weight_type cost_mid = distance[pre_index] + p_weight[start[i]->weight_idx];
				CROSS::id_type cross_id_mid = start[i]->cross_id;
				__Node *parent_mid = pre_node;
				Node *p_Node_mid = start[i];
				mid = new __Node(cost_mid, cross_id_mid, parent_mid, p_Node_mid);		//如果这个边的容量大于0，建立__Node实例, 并把它压入优先队列
				candidates.push(mid);
			}
		}
		while (visited_map.find(candidates.top()->cross_id) != visited_map.end())		//一直pop直到队顶节点没被访问过
		{
			auto temp = candidates.top()
			candidates.pop();
			delete temp;
		}
		record = candidates.top();						// 记录这个队顶节点, 以供回溯使用
		del_nodes.push_back(record);
		candidates.pop();
		visited_map[record->cross_id] = true;			// 记录为已访问
		distance[record->cross_id] = record->cost;		// 记录到这个节点的开销
		pre_node = record;								// 修改父节点信息
		pre_index = record->cross_id;
		
	}
	auto answer = new vector<CROSS::id_type>;			// answer这个vector存储的是从目的节点到源节点的边的地址的序列
	while (record != nullptr)
	{
		answer->push_back(record->p_Node);				// 从目的节点开始存路径节点(Node)的地址
		record = record->parent;
	}
	for (int i = 0; i < del_nodes.size(); i++)
	{
		delete del_nodes[i];
	}
	//这边各个点容量减少一

	return answer;
	
}