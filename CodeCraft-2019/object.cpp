#include "object.hpp"

#include <queue> // 寻路算法中使用 priority_queue

using namespace std;

/********* class GRAPH  *********/

GRAPH::idx_type GRAPH::Node::node_count = 0;

void GRAPH::add_node(ROAD::id_type road_id, CROSS::id_type from, CROSS::id_type to,
                     ROAD::capacity_type capacity, bool isDuplex)
{
    Node *p_Node = new Node(to, road_id);
    graph_map[from].push_back(p_Node);
    capacity_vec.push_back(capacity);

    if(isDuplex) {
        p_Node = new Node(from, road_id);
        graph_map[to].push_back(p_Node);
        capacity_vec.push_back(capacity);
    }
}


void GRAPH::add_weight_accord_to_speed(CAR::speed_type speed)
{
    //vector<Node*>& tmp_vec = graph_map[23];
}


GRAPH::route_type* GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed)
{

}
