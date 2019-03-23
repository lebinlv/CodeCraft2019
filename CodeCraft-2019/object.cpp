#include "lib/object.hpp"

#define ROAD_VECTOR_RESERVE   64     // 为 寻路函数返回的vector 预分配的空间
using namespace std;

/********* class GRAPH  *********/

GRAPH::idx_type GRAPH::Node::node_count = 0;
vector<GRAPH::Node*> GRAPH::Node::pNode_vec;

//GRAPH::weight_type weight[300];   for debug 

GRAPH::GRAPH(int reserve_node_count){
    p_weight = nullptr;
    //capacity_vec.reserve(reserve_road_count);
    Node::pNode_vec.reserve(reserve_node_count);
}


GRAPH::~GRAPH(){
    // free weigth_map
    for_each(weight_map.begin(), weight_map.end(),
             [](const pair<CAR::speed_type, weight_type*> & val)->void{delete val.second;});
    // free graph_map
}

void GRAPH::add_node(ROAD::id_type road_id, CROSS::id_type from, CROSS::id_type to, ROAD::length_type length,
                     ROAD::speed_type speed, ROAD::capacity_type capacity, bool isDuplex)
{
    graph_map[from].push_back(new Node(to, road_id, length, speed, capacity));
    //capacity_vec.push_back(capacity);

    if(isDuplex) {
        graph_map[to].push_back(new Node(from, road_id, length, speed, capacity));
        //capacity_vec.push_back(capacity);
    }
}


void GRAPH::add_weight_accord_to_speed(CAR::speed_type speed)
{
    weight_type *p = new weight_type[Node::node_count];
    //weight_type *p = weight;          // for debug
    weight_type *p_back = p;
    for (auto pNode : Node::pNode_vec){
        *p_back = weight_type(pNode->length) / min(speed, pNode->max_speed);
        p_back++;
    }
    weight_map.insert(pair<CAR::speed_type, weight_type*>(speed, p));
}


void GRAPH::update_weights(bool speed_detect_array[], int size)
{
    for(int i=0; i<size; i++){
        if(speed_detect_array[i]){
            weight_type *p = new weight_type[Node::node_count];
            //weight_type *p = weight;   // for debug
            weight_type *p_back = p;
            for(auto pNode : Node::pNode_vec){
                *p_back = weight_type(pNode->length)/min(ROAD::speed_type(i), pNode->max_speed);
                p_back++;
            }
            weight_map.insert(pair<CAR::speed_type, weight_type*>(i, p));
        }
    }
}


GRAPH::route_type & GRAPH::get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed)
{
    /**
     * @brief Dijkstra求最短路径
     * 
     */
    p_weight = weight_map[speed];       // 根据车速重定向 p_weight

    auto answer = new route_type;   // 存储的是从目的节点到源节点的节点指针序列
    answer->reserve(ROAD_VECTOR_RESERVE); // 预分配空间

    priority_queue<__Node *, vector<__Node *>, __Node::Compare>  candidates;
    map<CROSS::id_type, __Node *>                                visited_map;  // 记录节点是否被访问过

    // 创建一个 cost为0， cross_id 为 from， 且 parent 和 p_Node 均为 nullptr 的初始 __Node 节点
    __Node *record = new __Node(0, from, nullptr, nullptr);

    visited_map.insert(pair<CROSS::id_type, __Node *>(from, record));   // 将起点标记为已访问

    while (record->cross_id != to) {
        for (auto node : graph_map[record->cross_id]) {
            // 如果这个节点代表的边没有被访问过 且 边的容量大于0，建立__Node实例, 并把它压入优先队列
            if(visited_map.find(node->cross_id) == visited_map.end() && node->capacity > 0) {
                candidates.push(new __Node(record->cost + p_weight[node->info_idx], node->cross_id, record, node));
            }
        }

        // 一直pop直到队顶节点没被访问过
        while (!candidates.empty() && visited_map.find(candidates.top()->cross_id) != visited_map.end() ) {
            delete candidates.top();      // 释放节点
            candidates.pop();
        }

        if(candidates.empty()) return *answer;

        record = candidates.top();                  // 记录这个队顶节点, 以供回溯使用
        candidates.pop();

        visited_map.insert(pair<CROSS::id_type, __Node *>(from, record)); // 将当前节点标记为已访问
    }

    // 上面的while循环结束时，record指向目的节点，接下来从record开始回溯，获得完整路径
    while (record->parent != nullptr) {
        answer->push_back(record->p_Node);    // 从目的节点开始存路径节点(Node)的地址
        record = record->parent;
    }

    // 释放空间
    for_each(visited_map.begin(), visited_map.end(),
             [](const pair<CROSS::id_type, __Node *> & val) -> void {delete val.second;});
    //这边各个点容量减少一

    return *answer;
}
