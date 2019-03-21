#include "object.hpp"
#include <queue>
using namespace std;
GRAPH::route_type* GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed)
{
	/*
	Dijkstra�����·��
	*/

	priority_queue<__Node*, vector<__Node*>, __Node::Compare> candidates;
	map<CROSS::id_type, bool> visited_map;				// ��¼�ڵ��Ƿ񱻷��ʹ�
	map<CROSS::id_type, weight_type> distance;			// ��¼Դ�ڵ㵽ÿ���ڵ�Ŀ���
	p_weight = weight_map[speed];
	visited_map[from] = true;
	distance[from] = 0;
	__Node *pre_node = nullptr;							// ��¼�ϸ���visited_map�Ľڵ�
	CROSS::id_type pre_index = from;					// ��¼�ϸ���visited_map�Ľڵ��cross_id
	__Node *record;
	__Node *mid;
	vector<__Node*> del_nodes;
	while (visited_map.find(to) == visited_map.end())
	{
		vector<Node*> &start = graph_map[pre_index];
		for(int i=0;i<start.size();i++)
		{
			if ()										// �ж�����ߵ������Ƿ�С�ڵ���0
			{
				weight_type cost_mid = distance[pre_index] + p_weight[start[i]->weight_idx];
				CROSS::id_type cross_id_mid = start[i]->cross_id;
				__Node *parent_mid = pre_node;
				Node *p_Node_mid = start[i];
				mid = new __Node(cost_mid, cross_id_mid, parent_mid, p_Node_mid);		//�������ߵ���������0������__Nodeʵ��, ������ѹ�����ȶ���
				candidates.push(mid);
			}
		}
		while (visited_map.find(candidates.top()->cross_id) != visited_map.end())		//һֱpopֱ���Ӷ��ڵ�û�����ʹ�
		{
			auto temp = candidates.top()
			candidates.pop();
			delete temp;
		}
		record = candidates.top();						// ��¼����Ӷ��ڵ�, �Թ�����ʹ��
		del_nodes.push_back(record);
		candidates.pop();
		visited_map[record->cross_id] = true;			// ��¼Ϊ�ѷ���
		distance[record->cross_id] = record->cost;		// ��¼������ڵ�Ŀ���
		pre_node = record;								// �޸ĸ��ڵ���Ϣ
		pre_index = record->cross_id;
		
	}
	auto answer = new vector<CROSS::id_type>;			// answer���vector�洢���Ǵ�Ŀ�Ľڵ㵽Դ�ڵ�ıߵĵ�ַ������
	while (record != nullptr)
	{
		answer->push_back(record->p_Node);				// ��Ŀ�Ľڵ㿪ʼ��·���ڵ�(Node)�ĵ�ַ
		record = record->parent;
	}
	for (int i = 0; i < del_nodes.size(); i++)
	{
		delete del_nodes[i];
	}
	//��߸�������������һ

	return answer;
	
}