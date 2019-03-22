#include "lib/object.hpp"

using namespace std;

/********* class GRAPH  *********/

GRAPH::idx_type GRAPH::Node::node_count = 0;

GRAPH::GRAPH(int reserve_road_count){
    p_weight = nullptr;
    capacity_vec.reserve(reserve_road_count);
}
GRAPH::~GRAPH(){
    // free weigth_map
    // free graph_map
}

void GRAPH::add_node(ROAD::id_type road_id, CROSS::id_type from, CROSS::id_type to, ROAD::speed_type speed,
                     ROAD::capacity_type capacity, bool isDuplex)
{
    graph_map[from].push_back(new Node(to, road_id, speed));
    capacity_vec.push_back(capacity);

    if(isDuplex) {
        graph_map[to].push_back(new Node(from, road_id, speed));
        capacity_vec.push_back(capacity);
    }
}


void GRAPH::add_weight_accord_to_speed(CAR::speed_type speed)
{
    //vector<Node*>& tmp_vec = graph_map[23];
}


GRAPH::route_type & GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed)
{
    /**
     * @brief Dijkstra求最短路径
     * 
     */
    p_weight = weight_map[speed];       // 根据车速重定向 p_weight

    auto answer = new vector<Node *>; // answer这个vector存储的是从目的节点到源节点的边的地址的序列

    priority_queue<__Node *, vector<__Node *>, __Node::Compare>  candidates;
    map<CROSS::id_type, bool>                                    visited_map;  // 记录节点是否被访问过

    visited_map.insert(pair<CROSS::id_type, bool>(from, true));   // 将起点标记为已访问

    // 创建一个 cost为0， cross_id 为 from， 且 parent 和 p_Node 均为 nullptr 的初始 __Node 节点 
    __Node *record = new __Node(0, from, nullptr, nullptr);

    vector<__Node *> del_nodes;

    bool success = true;

    while (true) {
        for (auto node : graph_map[record->cross_id]) {
            // 如果这个节点代表的边没有被访问过 且 边的容量大于0，建立__Node实例, 并把它压入优先队列
            if(visited_map.find(node->cross_id) == visited_map.end() && capacity_vec[node->info_idx] > 0) {
                candidates.push(new __Node(record->cost + p_weight[node->info_idx], node->cross_id, record, node));
            }
        }

        // 一直pop直到队顶节点没被访问过
        while (visited_map.find(candidates.top()->cross_id) != visited_map.end() ) {
            delete candidates.top();
            candidates.pop();
            if(candidates.empty()) {
                success = false;
                break;
            }
        }
        if(!success)
            return *answer;

        record = candidates.top();                  // 记录这个队顶节点, 以供回溯使用
        candidates.pop();

        del_nodes.push_back(record);
        
        //visited_map[record->cross_id] = true;      // 记录为已访问

        // todo modify flag
    }


    while (record != nullptr) {
        answer->push_back(record->p_Node);    // 从目的节点开始存路径节点(Node)的地址
        record = record->parent;
    }

    for_each(del_nodes.begin(), del_nodes.end(), [](__Node *val) -> void {delete val;});
    //这边各个点容量减少一

    return *answer;
}
