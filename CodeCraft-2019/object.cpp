#include "lib/object.hpp"

using namespace std;

extern map_type<int, ROAD*> roadMap;
extern map_type<int, CROSS *> crossMap;
extern bool speedDetectArray[SPEED_DETECT_ARRAY_LENGTH];


/*************************** class Container *****************************/
int Container::containerCount = 0;
Container::Container(int _roadId, int _channel, int _length, int _maxSpeed, int _crossId):
                     roadId(_roadId), channel(_channel), length(_length), maxSpeed(_maxSpeed),
                     nextCrossId(_crossId), capacity(_channel * _length), 
                     from(nullptr), to(nullptr)
{
    carInChannel = new container_t [channel];
    for(int i=0; i<channel; ++i) {
        carInChannel[i].reserve(length);
    }
    infoIdx = containerCount++;
    GRAPH::containerVec.push_back(this);
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
    forward = new Container(_id, _channel, _length, _speed, to);
    backward = _isDuplex ? new Container(_id, _channel, _length, _speed, from) : nullptr;
}
/*************************** end of class ROAD *****************************/



/*************************** class CROSS *****************************/
map_type<int, uint8_t> CROSS::turnMap;
int CROSS::crossCount = 0;

CROSS::CROSS(int crossId, int roadId[]):id(crossId)
{
    Container *enter[4] = {nullptr, nullptr, nullptr, nullptr};
    Container *away[4] = {nullptr, nullptr, nullptr, nullptr};

    for(uint8_t i=0; i<4; ++i) {
        if(roadId[i] > 0) {
            auto val = roadMap[roadId[i]]->getContainer(id);
            enter[i] = val.first;
            away[i] = val.second;
        }
    }

    Container *temp;
    int real_idx, weight_idx;
    for(uint8_t i=0; i<4; ++i) {
        temp = enter[i];
        if(temp) {
            for(uint8_t j=i+1; j<i+4; ++j) {
                real_idx = j&3; // j&3 == j%4
                weight_idx = (j-i)%3;
                temp->opposite[weight_idx] = enter[real_idx];
                if(away[real_idx]) {
                    temp->turn_to[weight_idx] = away[real_idx];
                    turnMap.insert(pair<int, uint8_t>(MERGE(roadId[i], roadId[real_idx]),weight_idx));
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

    sort(enterRoadVec.begin(), enterRoadVec.end(), [](Container *a, Container *b)->bool{return a->roadId < b->roadId;});
    garage.reserve(GARAGE_RESERVE_SIZE);
    crossCount++;
}

struct Node
{
    float cost;
    int crossId;
    Container *first;

    Node(int _cost, int _id, Container * _first):cost(_cost), crossId(_id), first(_first){}

    struct Compare{
        inline bool operator()(Node *a, Node *b){return a->cost > b->cost;}
    };
};

void CROSS::updateRouteTable()
{
    for(auto car_speed : GRAPH::speedDetectResultVec) {
        for(auto val : awayRoadVec){
            updateRouteTableInternall(car_speed, val->roadId);
        }
        updateRouteTableInternall(car_speed, 0);
    }
}

void CROSS::updateRouteTableInternall(int speed, int removeRoadId)
{
#if __DEBUG_MODE__
    // 路由表的索引： uint32_t    speed*1e8 + removeRoadId*1e4 + 目的crossId
    uint32_t temp_idx = speed*1e8 + removeRoadId*1e4;
#else
    // 路由表的索引： uint32_t    | speed 6bit(0~63) | removeRoadId 14bit(0~16383) | 目的crossId 12bit(0~4095) |
    uint32_t temp_idx = (speed<<26) | (removeRoadId<<12);
#endif

    float *pWeight = GRAPH::costMap[speed]; // 根据车速重定向 pWeight
    int count = 1; // 记录已经经过的节点数,如果等于总路口数则退出
    bool not_find_new_node;

    priority_queue<Node *, vector<Node *>, Node::Compare> candidates;
    unordered_map<int, Node *> visited_map; // 记录节点是否被访问过

    Node *record = new Node(0, id, nullptr);
    visited_map.insert(pair<int, Node *>(id, record)); // 将起始节点标记为已访问


  /* 特殊对待第一次向外辐射 */
    for (auto node : awayRoadVec){
        if(node->roadId != removeRoadId) candidates.push(new Node(pWeight[node->infoIdx], node->nextCrossId, node));
    }
    record = candidates.top(); // 取队列顶节点
    candidates.pop();
    visited_map.insert(pair<int, Node *>(record->crossId, record)); // 将当前节点标记为已访问

#if __DEBUG_MODE__
    routeTable.insert(pair<uint32_t, routeInfo_t >(temp_idx+record->crossId, routeInfo_t(record->first, record->cost)));
#else
    routeTable.insert(pair<uint32_t, routeInfo_t>(temp_idx|record->crossId, routeInfo_t(record->first, record->cost)));
#endif

    count++;
  /* 第一次辐射结束 */


    while (count < crossCount)
    {
        for (auto node : crossMap[record->crossId]->awayRoadVec){
            // 如果这个节点代表的边没有被访问过，建立Node实例, 并把它压入优先队列
            if (visited_map.find(node->nextCrossId) == visited_map.end()) {
                candidates.push(new Node(record->cost + pWeight[node->infoIdx], node->nextCrossId, record->first));
            }
        }

        // 一直pop直到队顶节点没被访问过
        not_find_new_node = true;
        while (!candidates.empty()) {
            record = candidates.top(); // 取队列顶节点

            // 如果在visited_map中没有找到该节点，即该节点未被访问过，则退出while
            if (visited_map.find(record->crossId) == visited_map.end()) {
                candidates.pop();
                // 将未被访问到的节点写入routeMap和visited_map
                visited_map.insert(pair<int, Node *>(record->crossId, record)); // 将当前节点标记为已访问

            #if __DEBUG_MODE_
                routeTable.insert(pair<uint32_t, routeInfo_t >(temp_idx+record->crossId, routeInfo_t(record->first, record->cost)));
            #else
                routeTable.insert(pair<uint32_t, routeInfo_t>(temp_idx|record->crossId, routeInfo_t(record->first, record->cost)));
            #endif
                not_find_new_node = false;
                count++;
                break;
            }
            // 否则就弹出顶点并释放资源
            delete record;
            candidates.pop();
        }

        if (not_find_new_node) {
            for (auto &val : visited_map) { delete val.second;}  // 释放 visited_map中的 Node*
            break;
        }
    }

    // 释放空间
    while(!candidates.empty()){ delete candidates.top(); candidates.pop(); } // 释放优先队列中的 Node*
    for (auto &val : visited_map){delete val.second;} // 释放 visited_map中的 Node*
}
/*************************** end of class CROSS *****************************/



/*************************** class GRAPH *****************************/
vector<Container *> GRAPH::containerVec;
unordered_map<uint8_t, float *> GRAPH::costMap;
vector<uint8_t> GRAPH::speedDetectResultVec;
void GRAPH::calculateCostMap()
{
    for(int i=0; i<SPEED_DETECT_ARRAY_LENGTH; i++){
        if(speedDetectArray[i]){
            float *pCost = new float [Container::containerCount];
            float *temp = pCost;
            for(auto val : containerVec){
                *temp = float(val->length)/(min(val->maxSpeed, i));
                temp++;
            }
            costMap.insert(pair<uint8_t, float *>(i, pCost));
            speedDetectResultVec.push_back(i);
        }
    }
}
/*************************** end of class GRAPH *****************************/