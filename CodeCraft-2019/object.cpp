#include <cfloat>

#include "lib/object.hpp"


/*
*调参区间
*/
static float init_prob = 0.9;  //得到的最优路径的初始概率
static float max_factor = 0.9; //用于道路容量计算的最大因子，也就是车数量>factor*容量，则初始概率为0
static float alpha = 0.7;
/*
*/

using namespace std;

extern map_type<int, ROAD*> roadMap;
extern map_type<int, CROSS *> crossMap;
extern bool speedDetectArray[SPEED_DETECT_ARRAY_LENGTH];


/*************************** class Container *****************************/
int Container::containerCount = 0;
Container::Container(int _roadId, int _channel, int _length, int _maxSpeed, int _crossId):
                     roadId(_roadId), channel(_channel), length(_length), maxSpeed(_maxSpeed),
                     nextCrossId(_crossId), capacity(_channel * _length), 
                     startCross(nullptr), endCross(nullptr)
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
        //为车辆分配路线
        searchRoad(temp_car);

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
            // 为车辆规划路线
            searchRoad(temp_car);
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

// TODO: 预置车辆？？？？
void Container::searchRoad(CAR* car)
{
    int speed=car->speed;
    int destination = car->to;

    int count=0;//计数器，统计可以转向的路口数量

    bool not_right_crowded=false;//右侧道路是否不拥塞
    bool not_left_crowded=false;//左侧道路是否不拥塞
    bool not_straight_crowded=false;//直行道路是否不拥塞
    float distance_left = FLT_MAX, distance_right = FLT_MAX, distance_straight = FLT_MAX;

    // right[0]  left[1]  stright[2]
    // bool is_crowded[3] = {true, true, true};    //初值全都堵
    // float next_distance[3] = {FLT_MAX, FLT_MAX, FLT_MAX};

    auto &route_info = endCross->lookUp(speed, roadId, destination);
    Container *best_road= route_info.first;//最优的路

    // TODO: 预置车辆？？？？
    if(car->isPreset)//preset car does not change anything
        return;

    else if(car->isPrior) {//prior car just use its shortest route
        car->nextSpeed = min(car->speed, best_road->maxSpeed);
        car->getNewRoad = true;
        (car->route).push_back(best_road);
        return;
    }

    // for ordinary car
    else {
        // Container *next_container;
        // for(int i=0; i<3; i++) {
        //     next_container = turnTo[i];
        //     if(next_container){ // 如果存在
        //         next_distance[i] = next_container->endCross->lookUp(speed, next_container->roadId, destination).second;
        //         next_distance[i] += float(next_container->length)/min(speed, next_container->maxSpeed);

        //         is_crowded[i] = (next_container->size())>(max_factor*next_container->capacity);
        //         next_container->probability = is_crowded[i]? 0:1;
        //         count++;
        //     }
        // }

        Container* right=turnTo[0];
        if(right)//存在右边的路
        {
            distance_right = endCross->lookUp(speed, right->roadId, car->to).second;
            right->probability=1;
            count++;//计数有效的路数量
            not_right_crowded=(right->size())<( max_factor*right->capacity);//判断这条路是否拥塞
            if( !not_right_crowded ) right->probability=0;//拥塞了概率就是0
        }

        Container* left=turnTo[1];
        if(left){
            left->probability=1;
            count++;
            not_left_crowded=left->size()<( max_factor*left->capacity) ;
            if(!not_left_crowded) left->probability=0;
        }

        Container* straight=turnTo[2];
        if(straight){
            straight->probability=1;
            count++;
            not_straight_crowded=straight->size()<( max_factor*straight->capacity);
            if( !not_straight_crowded) straight->probability=0;
        }

        switch(count)
        {
            //有三条有效路径
            case 3:
            {
                //最有路是左边的路
                if(best_road->roadId==left->roadId)
                {
                    best_road->probability=min(0.7,best_road->probability);//最优路幅值概率
                    if(distance_right<distance_straight)//根据另外两条路的长短来给这两条路幅值概率
                    {
                        right->probability=min(0.2,right->probability);
                        straight->probability=min(0.1,straight->probability);
                    }
                    else if(distance_right == distance_straight)
                    {
                        right->probability=min(0.15,right->probability);
                        straight->probability=min(0.15,straight->probability);
                    }
                    else
                    {
                        right->probability=min(0.1,right->probability);
                        straight->probability=min(0.2,straight->probability);
                    }
                    
                }
                else if(best_road->roadId==right->roadId)//最有路是右边的路
                {
                    best_road->probability=min(0.7,best_road->probability);
                    if(distance_left<distance_straight)
                    {
                        left->probability=min(0.2,left->probability);
                        straight->probability=min(0.1,straight->probability);
                    }
                    else if(distance_left == distance_straight)
                    {
                        left->probability=min(0.15,left->probability);
                        straight->probability=min(0.15,straight->probability);
                    }
                    else
                    {
                        left->probability=min(0.1,left->probability);
                        straight->probability=min(0.2,straight->probability);
                    }
                }
                else//最有路是直行的路
                {
                    best_road->probability=min(0.7,best_road->probability);
                    if(distance_right<distance_left)
                    {
                        right->probability=min(0.2,right->probability);
                        left->probability=min(0.1,left->probability);
                    }
                    else if(distance_right == distance_left)
                    {
                        right->probability=min(0.15,right->probability);
                        left->probability=min(0.15,left->probability);
                    }
                    else
                    {
                        right->probability=min(0.1,right->probability);
                        left->probability=min(0.2,left->probability);
                    }
                }
            }
            break;
            //有两条有效路径
            case 2:
            {
                if(!(left))//左边道路无效
                {
                    if(distance_right<distance_straight)//根据另外两条路的长短来给这两条路幅值概率
                    {
                        right->probability=min(0.8,right->probability);
                        straight->probability=min(0.2,straight->probability);
                    }
                    else if(distance_right == distance_straight)
                    {
                        right->probability=min(0.5,right->probability);
                        straight->probability=min(0.5,straight->probability);
                    }
                    else
                    {
                        right->probability=min(0.2,right->probability);
                        straight->probability=min(0.8,straight->probability);
                    }
                }
                else if(!(right))//右边道路无效
                {
                    if(distance_left<distance_straight)
                    {
                        left->probability=min(0.8,left->probability);
                        straight->probability=min(0.2,straight->probability);
                    }
                    else if(distance_left == distance_straight)
                    {
                        left->probability=min(0.5,left->probability);
                        straight->probability=min(0.5,straight->probability);
                    }
                    else
                    {
                        left->probability=min(0.2,left->probability);
                        straight->probability=min(0.8,straight->probability);
                    }
                }
                else//直行无效
                {
                    if(distance_right<distance_left)
                    {
                        right->probability=min(0.8,right->probability);
                        left->probability=min(0.2,left->probability);
                    }
                    else if(distance_left==distance_straight)
                    {
                        right->probability=min(0.5,right->probability);
                        left->probability=min(0.5,left->probability);
                    }
                    else
                    {
                        right->probability=min(0.2,right->probability);
                        left->probability=min(0.8,left->probability);
                    }
                }
            }
            break;
            // 只有一条有效路径
            case 1:
            {
                best_road->probability=1;//只有一条路不是空，那么这条路肯定是路由表中的最优路
            }
            break;
        }

        Container* road=endCross->searchRoadForCar(roadId);

        if(road){
            car->nextSpeed = min(car->speed, road->maxSpeed);
            car->getNewRoad = true;
            (car->route).push_back(road);
        }
        else{
            car->nextSpeed = min(car->speed, best_road->maxSpeed);
            car->getNewRoad = true;
            (car->route).push_back(best_road);
        }
    }
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
                    temp->turnTo[weight_idx] = away[real_idx];
                    turnMap.insert(pair<int, uint8_t>(MERGE(roadId[i], roadId[real_idx]),weight_idx));
                }
            }
            temp->startCross = this;
            enterRoadVec.push_back(temp);
        }
        if(away[i]) {
            away[i]->endCross = this;
            awayRoadVec.push_back(away[i]);
        }
    }

    sort(enterRoadVec.begin(), enterRoadVec.end(), [](Container *a, Container *b)->bool{return a->roadId < b->roadId;});
    //garage.reserve(GARAGE_RESERVE_SIZE);
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
        updateRouteTableInternall(car_speed, -1);
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

Container* CROSS::searchRoadForCar(int current_road_id)
{
    double p[4]={0,0,0,0};//最终概率
    double p_total=0;
    int away_road_count = awayRoadVec.size();

    Container *temp_container;
    for(int i=0; i<away_road_count; i++)
    {
        temp_container = awayRoadVec[i];
        if(temp_container->roadId == current_road_id) continue;

        int car_total = temp_container->size();          //下一条路的车的数量
        int road_capacity = temp_container->capacity;    //下一条路的容量
        double congestion = alpha * (double)(car_total) / (double)(road_capacity);//拥塞指标计算，此处只是其中一项
        int count = 0;          //记录有几条二阶边不为空
        double target = 0;      //记录二阶边拥塞指标之和
        for(int j=0; j<3; j++)
        {
            Container* second_road = temp_container->turnTo[j];
            if(second_road)
            {
                target += (double)(second_road->size()) / (double)(second_road->capacity);
                count++;
            }
        }
        congestion += (1-alpha) * target / count;
        p[i] = temp_container->probability * (1 - congestion);
        p_total += p[i];
    }
    //如果出去的路都不能走,就直接返回空
    if(p_total == 0)    return nullptr;
    //将概率归一化
    for(int i=0; i<4; i++) p[i] /= p_total;

    double p_random = double(rand() % 10000) / 10000;
    double p_heap = 0;

    for(int i=0; i<away_road_count; i++)
    {
        if(p[i]>0){
            if(p_random>=p_heap && p_random<=p_heap+p[i]) return awayRoadVec[i];
            p_heap += p[i];
        }
    }

    for(int i=away_road_count-1; i>=0; i--)
    {
        if(p[i]) return awayRoadVec[i];
    }
}


// TODO: 处理预置车辆
void CROSS::driveCarInitList(bool is_prior,int global_time)
{
    if (is_prior) //如果is_prior是true，则只上路优先车辆
    {
        for (deque<CAR*>::iterator i =garage.begin();i!=garage.end();)
        {
            CAR *temp_car = *i;
            int speed = temp_car->speed;
            int destination = temp_car->to;
            float *cost_array = GRAPH::costMap[speed];

            // 如果计划时间晚于当前时间 或 不是优先车辆
            if ((temp_car->planTime) > global_time || (!temp_car->isPrior)) {
                ++i; continue;
            }

            else//是优先车辆
            {
                sort(awayRoadVec.begin(), awayRoadVec.end(), [cost_array, speed, destination](Container *a, Container *b) -> bool {
                    return (cost_array[a->infoIdx] + a->endCross->lookUp(speed, 0, destination).second) 
                          < (cost_array[b->infoIdx] + b->endCross->lookUp(speed, 0, destination).second);
                });

                switch(awayRoadVec.size())
                {
                    case 4:
                    {
                        awayRoadVec[0]->probability=\
                            min(0.4,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                        awayRoadVec[1]->probability=\
                            min(0.3,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                        awayRoadVec[2]->probability=\
                            min(0.2,((awayRoadVec[2]->size())<( max_factor*awayRoadVec[2]->length*awayRoadVec[2]->channel)?1.0:0.0));
                        awayRoadVec[3]->probability=\
                            min(0.1,((awayRoadVec[3]->size())<( max_factor*awayRoadVec[3]->length*awayRoadVec[3]->channel)?1.0:0.0));
                    }
                    break;
                    case 3:
                    {
                        awayRoadVec[0]->probability=\
                            min(0.7,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                        awayRoadVec[1]->probability=\
                            min(0.2,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                        awayRoadVec[2]->probability=\
                            min(0.1,((awayRoadVec[2]->size())<( max_factor*awayRoadVec[2]->length*awayRoadVec[2]->channel)?1.0:0.0));
                    }
                    break;
                    case 2:
                    {
                        awayRoadVec[0]->probability=\
                            min(0.8,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                        awayRoadVec[1]->probability=\
                            min(0.2,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                    }
                    break;
                    case 1:
                        awayRoadVec[0]->probability=1;
                    break;
                }

                Container* temp_road = searchRoadForCar(-1);

                //can not find road
                if(!temp_road) { ++i; continue; }

                //find road
                else {
                    temp_car->nextSpeed = min(speed, temp_road->maxSpeed);
                    if(temp_road->push_back(temp_car) == Container::SUCCESS )//success then delete it from garage
                    {
                        temp_car->startTime=global_time;
                        temp_car->route.push_back(temp_road);
                        garage.erase(i);
                    }    
                    else//else operate next car
                    {
                        ++i;
                        continue;
                    }
                }
            }
        }
    }
    else
    {
        for (deque<CAR*>::iterator i=garage.begin(); i!=garage.end();)
        {
            CAR *temp_car = *i;
            int speed = temp_car->speed;
            int destination = temp_car->to;
            float *cost_array = GRAPH::costMap[speed];

            //car cant go at this time ,operate next car
            if(temp_car->planTime>global_time) { i++; continue; }

            sort(awayRoadVec.begin(), awayRoadVec.end(), [cost_array, speed, destination](Container *a, Container *b) -> bool {
                return (cost_array[a->infoIdx] + a->endCross->lookUp(speed, 0, destination).second)
                      < (cost_array[b->infoIdx] + b->endCross->lookUp(speed, 0, destination).second);
            });

            switch(awayRoadVec.size())
            {
                case 4:
                {
                    awayRoadVec[0]->probability=\
                        min(0.4,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                    awayRoadVec[1]->probability=\
                        min(0.3,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                    awayRoadVec[2]->probability=\
                        min(0.2,((awayRoadVec[2]->size())<( max_factor*awayRoadVec[2]->length*awayRoadVec[2]->channel)?1.0:0.0));
                    awayRoadVec[3]->probability=\
                        min(0.1,((awayRoadVec[3]->size())<( max_factor*awayRoadVec[3]->length*awayRoadVec[3]->channel)?1.0:0.0));
                }
                break;
                case 3:
                {
                    awayRoadVec[0]->probability=\
                        min(0.7,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                    awayRoadVec[1]->probability=\
                        min(0.2,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                    awayRoadVec[2]->probability=\
                        min(0.1,((awayRoadVec[2]->size())<( max_factor*awayRoadVec[2]->length*awayRoadVec[2]->channel)?1.0:0.0));
                }
                break;
                case 2:
                {
                    awayRoadVec[0]->probability=\
                        min(0.8,((awayRoadVec[0]->size())<( max_factor*awayRoadVec[0]->length*awayRoadVec[0]->channel)?1.0:0.0));
                    awayRoadVec[1]->probability=\
                        min(0.2,((awayRoadVec[1]->size())<( max_factor*awayRoadVec[1]->length*awayRoadVec[1]->channel)?1.0:0.0));
                }
                break;
                case 1:
                    awayRoadVec[0]->probability=1;
                break;

            }

            Container* temp_road = searchRoadForCar(-1);

            //can not find road
            if(!temp_road) { ++i; continue; }

            else//find road
            {
                temp_car->nextSpeed = min(speed, temp_road->maxSpeed);
                if(temp_road->push_back(temp_car) == Container::SUCCESS )//success then delete it from garage
                {
                    temp_car->startTime=global_time;
                    temp_car->route.push_back(temp_road);
                    garage.erase(i);
                }

                else { ++i; continue; }
            }
        }
    }
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