#include "lib/object.hpp"

#define ROAD_VECTOR_RESERVE   64     // 为寻路函数返回的vector预分配的空间，建议为路口的数目的一半
using namespace std;

/********* class GRAPH  *********/

int              GRAPH::Node::node_count = 0;
vector<ROAD *>   GRAPH::Node::pRoad_vec;    //std::vector<Node *>

//double weight[300];   //for debug

GRAPH::Node::Node(int _cross_id, ROAD *_pRoad) : cross_id(_cross_id), pRoad(_pRoad)
{
    capacity = _pRoad->ini_capacity;
    info_idx = node_count++;
    pRoad_vec.push_back(_pRoad);
}


GRAPH::GRAPH(int reserve_node_count)
{
    p_weight = nullptr;
    Node::pRoad_vec.reserve(reserve_node_count);
}


GRAPH::~GRAPH()
{
    // free weigth_map
    for(auto val:weight_map){delete val.second;}

    // free graph_map
    for(auto val:graph_map){
        for(auto node:val.second){delete node;}
    }
}


void GRAPH::add_node(ROAD* pRoad)
{
    graph_map[pRoad->from].push_back(new Node(pRoad->to, pRoad));

    if(pRoad->isDuplex) {
        graph_map[pRoad->to].push_back(new Node(pRoad->from, pRoad));
    }
}


void GRAPH::add_weights(int speed)
{
    double *p = new double[Node::node_count];
    //double *p = weight;          // for debug
    double *p_back = p;

    for (auto pRoad : Node::pRoad_vec){
        *p_back = double(pRoad->length) / min(speed, pRoad->max_speed);
        p_back++;
    }
    weight_map.insert(pair<int, double*>(speed, p));
}


void GRAPH::add_weights(bool speed_detect_array[], int size)
{
    for(int i=0; i<size; i++){
        if(speed_detect_array[i]){
            double *p = new double[Node::node_count];
            //double *p = weight;   // for debug
            double *p_back = p;
            for(auto pRoad : Node::pRoad_vec){
                *p_back = double(pRoad->length)/min(i, pRoad->max_speed);
                p_back++;
            }
            weight_map.insert(pair<int, double*>(i, p));
        }
    }
}


GRAPH::route_type * GRAPH::get_least_cost_route(CAR* car, int global_time)
{
    /**
     * @brief Dijkstra求最短路径
     * 
     */
    p_weight = weight_map[car->speed];       // 根据车速重定向 p_weight

    auto answer = new route_type;   // 存储的是从目的节点到源节点的节点指针序列
    answer->reserve(ROAD_VECTOR_RESERVE); // 预分配空间

    priority_queue<__Node *, vector<__Node *>, __Node::Compare>  candidates;
    unordered_map<int, __Node *>                                 visited_map;  // 记录节点是否被访问过

    // 创建一个 cost为0， cross_id 为 from， 且 parent 和 p_Node 均为 nullptr 的初始 __Node 节点
    __Node *record = new __Node(0, car->from, nullptr, nullptr);

    visited_map.insert(pair<int, __Node *>(record->cross_id, record)); // 将起始节点标记为已访问

    bool not_find_new_node;
    while (record->cross_id != car->to) {

        for (auto node : graph_map[record->cross_id]) {
            // 如果这个节点代表的边没有被访问过 且 边的容量大于0，建立__Node实例, 并把它压入优先队列
            if(visited_map.find(node->cross_id) == visited_map.end() && node->capacity > ROAD_VALID_THREHOLD) {
                candidates.push(new __Node(record->cost + p_weight[node->info_idx], node->cross_id, record, node));
            }
        }

        // 一直pop直到队顶节点没被访问过
        not_find_new_node = true;
        while (!candidates.empty()) {
            // 取队列顶节点
            record = candidates.top();

            // 如果在visited_map中没有找到该节点，即该节点未被访问过，则退出
            if(visited_map.find(record->cross_id) == visited_map.end()) {
                candidates.pop();
                visited_map.insert(pair<int, __Node *>(record->cross_id, record)); // 将当前节点标记为已访问
                not_find_new_node = false;
                break;
            }

            // 否则就弹出顶点并释放资源
            delete record;
            candidates.pop();
        }

        if(not_find_new_node) {
            for(auto val : visited_map){delete val.second;}  // 释放 visited_map中的 __Node*
            return answer;
        }
    }
    car->start_time = global_time;
    // 上面的while循环结束时，record指向目的节点，接下来从record开始回溯，获得完整路径
    while (record->parent != nullptr) {

        record->p_Node->capacity -= CAPACITY_FACTOR; //容量减少
        car->past_nodes.push(new CAR::Past_node(record->p_Node, record->cost + global_time)); //记录节点到该车辆的对象中

        answer->push_back(record->p_Node);    // 从目的节点开始存路径节点(Node)的地址
        record = record->parent;
    }

    // 释放空间
    while(!candidates.empty()) { delete candidates.top(); candidates.pop();}  // 释放优先队列中的 __Node*

    for(auto val : visited_map){delete val.second;}  // 释放 visited_map中的 __Node*

    return answer;
}


//容量释放与相关车辆删除
void release_capacity(std::list<CAR*>& car_running, int global_time)
{
    CAR::Past_node* node;
    for (auto car = car_running.begin(); car!=car_running.end();) {
        node = (*car)->past_nodes.top();
        if (node->arrive_time < global_time){
            //已经到达这个节点，恢复容量并且将它从路径stack中删除
            node->node->capacity += CAPACITY_FACTOR;
            (*car)->past_nodes.pop();
            delete node;
        }
        
        if ((*car)->past_nodes.empty()){
            car_running.erase(car++);
        }else {
            car++;
        }
    }
}
// 答案写入接口
void write_to_file(vector<GRAPH::Node*> * tem_vec, CAR * car, std::ofstream &fout)
{
    fout << '(' << car->id << ", " << car->start_time;
    for(auto val:*tem_vec){fout << ", " << val->pRoad->id;}
    fout << ")\n";
}
