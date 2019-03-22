#include "object.hpp"

#include <queue> // Ѱ·�㷨��ʹ�� priority_queue

using namespace std;

/********* class GRAPH  *********/

GRAPH::idx_type GRAPH::Node::node_count = 0;

void GRAPH::add_node(ROAD::id_type road_id, CROSS::id_type from, CROSS::id_type to,
	ROAD::capacity_type capacity, bool isDuplex)
{
	Node *p_Node = new Node(to, road_id);
	graph_map[from].push_back(p_Node);
	capacity_vec.push_back(capacity);

	if (isDuplex) {
		p_Node = new Node(from, road_id);
		graph_map[to].push_back(p_Node);
		capacity_vec.push_back(capacity);
	}
}


void GRAPH::add_weight_accord_to_speed(CAR::speed_type speed)
{
	//vector<Node*>& tmp_vec = graph_map[23];
}


GRAPH::route_type & GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed, CAR *car, int global_time)
{
	/**
	* @brief Dijkstra�����·��
	*
	*/
	priority_queue<__Node *, vector<__Node *>, __Node::Compare> candidates;
	map<CROSS::id_type, bool> visited_map;     // ��¼�ڵ��Ƿ񱻷��ʹ�
	map<CROSS::id_type, weight_type> distance; // ��¼Դ�ڵ㵽ÿ���ڵ�Ŀ���
	p_weight = weight_map[speed];

	visited_map[from] = true;
	distance[from] = 0;
	__Node *pre_node = nullptr;      // ��¼�ϸ���visited_map�Ľڵ�
	CROSS::id_type pre_index = from; // ��¼�ϸ���visited_map�Ľڵ��cross_id
	__Node *record;
	__Node *mid;
	vector<__Node *> del_nodes;
	while (visited_map.find(to) == visited_map.end()) {
		vector<Node *> &start = graph_map[pre_index];

		for (int i = 0; i < start.size(); i++) {
			// �ж�����ߵ������Ƿ�С�ڵ���0, �������ߵ���������0������__Nodeʵ��, ������ѹ�����ȶ���
			if (capacity_vec[start[i]->capacity_idx] >= 0) {
				weight_type cost_mid = distance[pre_index] + p_weight[start[i]->weight_idx];
				CROSS::id_type cross_id_mid = start[i]->cross_id;
				__Node *parent_mid = pre_node;
				Node *p_Node_mid = start[i];
				mid = new __Node(cost_mid, cross_id_mid, parent_mid, p_Node_mid);
				candidates.push(mid);
			}
		}

		// һֱpopֱ���Ӷ��ڵ�û�����ʹ�
		while (visited_map.find(candidates.top()->cross_id) != visited_map.end()) {
			auto temp = candidates.top();
			candidates.pop();
			delete temp;
		}
		record = candidates.top();                  // ��¼����Ӷ��ڵ�, �Թ�����ʹ��
		del_nodes.push_back(record);
		candidates.pop();
		visited_map[record->cross_id] = true;      // ��¼Ϊ�ѷ���
		distance[record->cross_id] = record->cost; // ��¼������ڵ�Ŀ���
		pre_node = record;                         // �޸ĸ��ڵ���Ϣ
		pre_index = record->cross_id;
	}
	auto answer = new vector<CROSS::id_type>; // answer���vector�洢���Ǵ�Ŀ�Ľڵ㵽Դ�ڵ�ıߵĵ�ַ������

	while (record != nullptr) {
		//-----------------------------
		//��������
		capacity_vec[record->p_Node->capacity_idx] -= CAPACITY_FACTOR;
		//��¼�ڵ㵽�ó����Ķ�����
		car->past_nodes.push((CAR::Past_node(record->p_Node, record->cost + global_time)));
		//---------------------------

		answer->push_back(record->p_Node);    // ��Ŀ�Ľڵ㿪ʼ��·���ڵ�(Node)�ĵ�ַ
		record = record->parent;

	}

	for (int i = 0; i < del_nodes.size(); i++) {
		delete del_nodes[i];
	}
	//��߸�������������һ

	return *answer;
}


void release_capacity(std::vector<CAR*>& car_running, int global_time, GRAPH *graph)
{
	CAR::Past_node node;
	for (vector<CAR*>::iterator car = car_running.begin(); car != car_running.end(); car++)
	{
		for (int i = (*car)->past_nodes.size() - 1; i >= 0; i--) {
			node = (*car)->past_nodes.top();
			if (node.arrive_time < global_time)
			{
				(*car)->past_nodes.pop();
				graph->capacity_vec[node.node->capacity_idx] += CAPACITY_FACTOR;
				break;
			}
		}
		if ((*car)->past_nodes.size() == 0)
			car_running.erase(car);
	}
}