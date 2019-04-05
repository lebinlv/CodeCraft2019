#include "lib/object.hpp"

#define ROAD_VECTOR_RESERVE   64     // 为寻路函数返回的vector预分配的空间，建议为路口的数目的一半
using namespace std;

extern map_type<int, ROAD*> roadMap;
//extern map_type<int, uint8_t> turnMap;
extern vector<CROSS*> crossVec;


/*************************** class Container *****************************/
Container::Container(int _id, int _channel, int _length):
                     id(_id), channel(_channel), length(_length),
                     from(nullptr), to(nullptr)
{
    carInChannel = new container_t [channel];
    for(int i=0; i<channel; ++i) {
        carInChannel[i].reserve(length);
    }
}

int Container::push_back(CAR *pCar)
{
    //int carSpeed = min(pCar->speed, pCar->route.back()->maxSpeed); // 计算pCar在当前道路上的可行驶速度
    int s2 = max(0, pCar->nextSpeed - pCar->currentIdx);          // 计算出 pCar 在此道路上的可行距离 s2
    int new_idx = length - s2;

    CAR *pre_car;
    int pre_car_idx;

    // 逐车道遍历
    for (int i = 0; i < channel; i++) {
        // 如果当前车道为空
        if(carInChannel[i].empty()) {
            pCar->enterNewRoad(new_idx, i);
            carInChannel[i].push_back(pCar);
            return SUCCESS;
        }

        pre_car = carInChannel[i].back();
        pre_car_idx = pre_car->currentIdx;

        if (pre_car->state == CAR::END) { // 如果该车道最后一辆车状态为 END
            if (pre_car_idx < length-1) { // 如果该车道还有空间
                pCar->enterNewRoad((new_idx>pre_car_idx ? new_idx:pre_car_idx+1), i);
                carInChannel[i].push_back(pCar);
                return SUCCESS;
            }
        } else { // 如果最后一辆车是 WAIT
            if (new_idx > pre_car_idx) { // 如果不被阻挡，则成功进入并标记为END
                pCar->enterNewRoad(new_idx, i);
                carInChannel[i].push_back(pCar);
                return SUCCESS;
            } // End of 如果不被阻挡
            return length-pre_car_idx-1; // 被阻挡则返回该车道剩余空间大小
        } // End of 如果最后一辆车是WAIT
    } // End of 逐车道遍历
    return FULL_LOAD;
}


bool Container::pop()
{
    if (priCar.empty()) return false;

    // 获取车辆所在车道id
    auto &channelVec = carInChannel[priCar.top()->preChannel];

    // 删除该车
    channelVec.erase(channelVec.begin());
    priCar.pop();

    return true;
}

// TODO: 调用路径规划函数为能出路口的车寻路
void Container::dispatchCarInChannel(int channel_idx)
{
    auto & car_vec = carInChannel[channel_idx];
    int len = car_vec.size();

    if(len==0) return;    // 如果车道为空，则无需调度

    CAR *temp_car = car_vec.front(); // 取离出口最近的一辆车
    if(temp_car->state == CAR::END) return; // 

    CAR::CAR_STATE pre_car_state = CAR::WAIT;
    int pre_car_idx = temp_car->currentIdx;
    int new_idx = pre_car_idx - temp_car->currentSpeed; // 计算新坐标

    if(new_idx < 0){  // 若 new_idx<0，则该车有可能出路口
        //TODO: getNextRoad(temp_car);

        // 因为规划了下条道路，所以计算出该车在下条道路的可行距离s2以判断该车能否出路口
        int s2 = max(0, temp_car->nextSpeed - temp_car->currentIdx);
        if (s2) {
            priCar.push(temp_car);
        } else { // 若s2为0， 说明该车不能出路口
            temp_car->currentIdx = 0; // 则移动该车至路口处，
            pre_car_idx = 0;
            temp_car->state = CAR::END; // 并将该车标记为END
            pre_car_state = CAR::END;
        }
    } else { // 否则（即new_idx>=0） 该车无法出路口
        temp_car->currentIdx = new_idx; // 则移动该车
        pre_car_idx = new_idx;
        temp_car->state = CAR::END;     // 并标记为END
        pre_car_state = CAR::END;
    }

    if(pre_car_state == CAR::WAIT) return;

    // 第一辆车已处理过，接下来从第二辆开始遍历剩余车辆
    for(int i=1; i<len; i++) {
        temp_car = car_vec[i];
        if(temp_car->state == CAR::END) return;

        new_idx = temp_car->currentIdx - temp_car->currentSpeed;

        if(new_idx > pre_car_idx) { // 如果不受阻挡
            temp_car->currentIdx = new_idx; // 则移动该车
            pre_car_idx = new_idx;
            temp_car->state = CAR::END; // 并标记为 END
        } else { // 如果受到阻挡
            pre_car_idx += 1;
            temp_car->currentIdx = pre_car_idx; //移动该车
            temp_car->state = CAR::END;
        }
    } // End of 遍历剩余车辆
}

void Container::updateWhenNextRoadFull(int channel_idx)
{
    int pre_car_idx = -1, new_idx; // 计算新坐标

    // 遍历车辆
    for(auto temp_car : carInChannel[channel_idx]) {
        if(temp_car->state == CAR::END) return;

        new_idx = temp_car->currentIdx - temp_car->currentSpeed;
        if(new_idx > pre_car_idx) { // 如果不受阻挡
            temp_car->currentIdx = new_idx; // 则移动该车
            pre_car_idx = new_idx;
            temp_car->state = CAR::END; // 并标记为 END
        } else { // 如果受到阻挡
            pre_car_idx += 1;
            temp_car->currentIdx = pre_car_idx; //移动该车
            temp_car->state = CAR::END;
        }
    } // End of 遍历剩余车辆
}

// TODO: 调用路径规划函数为能出路口的车寻路
void Container::dispatchCarInChannelFirst(int channel_idx)
{
    auto & car_vec = carInChannel[channel_idx];
    int len = car_vec.size();

    // 如果车道为空，则无需调度
    if(len==0) return;

    CAR *temp_car = car_vec.front(); // 取离出口最近的一辆车
    int new_idx = temp_car->currentIdx - temp_car->currentSpeed; // 计算新坐标

    CAR::CAR_STATE pre_car_state = CAR::END;
    int pre_car_idx = 0;

    if(new_idx < 0){  // 若 new_idx<0，则该车有可能出路口
        if(temp_car->getNewRoad == false) { // 若没有为该车分配过道路，则调用路径规划函数为其分配道路
            //TODO: getNextRoad(temp_car);
            
            // 因为规划了下条道路，所以计算出该车在下条道路的可行距离s2以判断该车能否出路口
            int s2 = max(0, temp_car->nextSpeed - temp_car->currentIdx);
            if (s2) { // 如果 s2 不为0， 说明该车可出路口
                pre_car_idx = temp_car->currentIdx; // 不移动该车
                temp_car->state = CAR::WAIT;        // 并将该车标记为WAIT
                pre_car_state = CAR::WAIT;

                priCar.push(temp_car); // 该车能出路口，所以将其放入出路口优先队列
            }
            else {// s2为0， 说明该车不能出路口
                temp_car->currentIdx = 0; // 则移动该车至路口处，
                temp_car->state = CAR::END; // 并将该车标记为END
            }
        }
        else { // 若已经为该车分配过道路，（这种情况下该车的 currentIdx 必然为0）
            temp_car->state = CAR::WAIT;
            pre_car_state = CAR::WAIT;

            priCar.push(temp_car); // 该车能出路口，所以将其放入出路口优先队列
        }
    } else { // 否则（即new_idx>=0） 该车无法出路口
        temp_car->currentIdx = new_idx; // 则移动该车
        pre_car_idx = new_idx;
        temp_car->state = CAR::END;     // 并标记为END
    }

    // 第一辆车已处理过，接下来从第二辆开始遍历剩余车辆
    for(int i=1; i<len; i++){
        temp_car = car_vec[i];
        new_idx = temp_car->currentIdx - temp_car->currentSpeed;
        switch (pre_car_state) {
            case CAR::WAIT: // 如果前车状态为 WAIT
                if (new_idx > pre_car_idx) {  // 如果不受前车阻挡
                    temp_car->currentIdx = new_idx; // 则移动该车
                    pre_car_idx = new_idx;
                    temp_car->state = CAR::END; // 并标记为 END
                    pre_car_state = CAR::END;
                } else { // 如果受前车阻挡
                    pre_car_idx = temp_car->currentIdx; // 则不移动该车
                    temp_car->state = CAR::WAIT; //并标记为 WAIT
                }
                break;
            case CAR::END: // 如果前车状态为 END
                if(new_idx > pre_car_idx) { // 如果不受阻挡
                    temp_car->currentIdx = new_idx; // 则移动该车
                    pre_car_idx = new_idx;
                    temp_car->state = CAR::END; // 并标记为 END
                } else { // 如果受到阻挡
                    pre_car_idx += 1;
                    temp_car->currentIdx = pre_car_idx; //移动该车
                    temp_car->state = CAR::END;
                }
                break;
        } // End of switch
    } // End of 遍历剩余车辆
}
/*************************** end of class Container *****************************/



/*************************** class ROAD  *****************************/
ROAD::ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex) : 
           id(_id), length(_length), maxSpeed(_speed), channel(_channel),
           from(_from), to(_to), isDuplex(_isDuplex)
{
    capacity = _length * _channel;
    forward = new Container(_id, _channel, _length);
    backward = _isDuplex ? new Container(_id, _channel, _length) : nullptr;
}
/*************************** end of class ROAD *****************************/



/*************************** class CROSS *****************************/
CROSS::CROSS(int crossId, int roadId[]):id(crossId)
{
    Container *enter[4] = {nullptr, nullptr, nullptr, nullptr};
    Container *away[4] = {nullptr, nullptr, nullptr, nullptr};

    for(int i=0; i<4; ++i) {
        if(roadId[i] > 0) {
            auto val = roadMap[roadId[i]]->getContainer(id);
            enter[i] = val.first;
            away[i] = val.second;
        }
    }

    Container *temp;
    int real_idx, weight_idx;
    for(int i=0; i<4; ++i) {
        temp = enter[i];
        if(temp) {
            for(uint8_t j=i+1; j<i+4; ++j) {
                real_idx = j&3; // j&3 == j%4
                weight_idx = (j-i)%3;
                temp->opposite[weight_idx] = enter[real_idx];
                if(away[real_idx]) {
                    temp->turn_to[weight_idx] = away[real_idx];
                    turnMap.insert(pair<int, uint8_t>(((roadId[i])*10000+roadId[real_idx]),weight_idx));
                }
            }
            temp->to = this;
            enterRoadVec.push_back(temp);
        }
        if(away[i]) {
            away[i]->from = this;
            awayRoadVec.push_back(away[i]);
        }
    }

    sort(enterRoadVec.begin(), enterRoadVec.end(), [](Container *a, Container *b)->bool{return a->id < b->id;});
    garage.reserve(GARAGE_RESERVE_SIZE);
}
/*************************** end of class CROSS *****************************/

/********* class GRAPH  *********/
int              GRAPH::Node::node_count = 0;
vector<ROAD *>   GRAPH::Node::pRoad_vec;    //std::vector<Node *>

//double weight[300];   //for debug

GRAPH::Node::Node(int _cross_id, ROAD *_pRoad) : cross_id(_cross_id), pRoad(_pRoad)
{
    capacity = _pRoad->capacity;
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
    // free weight_map
    for(auto &val:weight_map){delete val.second;}

    // free graph_map
    for(auto &val:graph_map){
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
        *p_back = double(pRoad->length) / min(speed, pRoad->maxSpeed);
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
                *p_back = double(pRoad->length)/min(i, pRoad->maxSpeed);
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
            if(visited_map.find(node->cross_id) == visited_map.end() && node->capacity > 1) {
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
            for(auto &val : visited_map){delete val.second;}  // 释放 visited_map中的 __Node*
            return answer;
        }
    }
    car->startTime = global_time;
    // 上面的while循环结束时，record指向目的节点，接下来从record开始回溯，获得完整路径
    while (record->parent != nullptr) {

        answer->push_back(record->p_Node);    // 从目的节点开始存路径节点(Node)的地址
        record = record->parent;
    }

    // 释放空间
    while(!candidates.empty()) { delete candidates.top(); candidates.pop();}  // 释放优先队列中的 __Node*

    for(auto &val : visited_map){delete val.second;}  // 释放 visited_map中的 __Node*

    return answer;
}


GRAPH::route_type * GRAPH::get_least_cost_route(int from, int to, int speed)
{
    /**
     * @brief Dijkstra求最短路径
     * 
     */
    p_weight = weight_map[speed];       // 根据车速重定向 p_weight

    auto answer = new route_type;   // 存储的是从目的节点到源节点的节点指针序列
    answer->reserve(ROAD_VECTOR_RESERVE); // 预分配空间

    priority_queue<__Node *, vector<__Node *>, __Node::Compare>  candidates;
    unordered_map<int, __Node *>                                 visited_map;  // 记录节点是否被访问过

    // 创建一个 cost为0， cross_id 为 from， 且 parent 和 p_Node 均为 nullptr 的初始 __Node 节点
    __Node *record = new __Node(0, from, nullptr, nullptr);
    visited_map.insert(pair<int, __Node *>(record->cross_id, record)); // 将当前节点标记为已访问
    bool not_find_new_node;
    while (record->cross_id != to) {

        for (auto node : graph_map[record->cross_id]) {
            // 如果这个节点代表的边没有被访问过 且 边的容量大于0，建立__Node实例, 并把它压入优先队列
            if(visited_map.find(node->cross_id) == visited_map.end() && node->capacity > 0) {
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
            for(auto &val : visited_map){delete val.second;}  // 释放 visited_map中的 __Node*
            return answer;
        }
    }
    
    // 上面的while循环结束时，record指向目的节点，接下来从record开始回溯，获得完整路径
    while (record->parent != nullptr) {
        answer->push_back(record->p_Node);    // 从目的节点开始存路径节点(Node)的地址
        record = record->parent;
    }

    // 释放空间
    while(!candidates.empty()){
        delete candidates.top();
        candidates.pop();
    }
    for(auto &val : visited_map){delete val.second;}
    //这边各个点容量减少一

    return answer;
}
