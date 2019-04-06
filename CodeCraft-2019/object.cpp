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




int Container::get_car_sum()
{
    int sum=0;
    for (int i=0;i<this->channel;i++)
    {
        sum+=this->carInChannel[i].size();
    }
    return sum;
}
void Container::search_road(CAR* car){
    int speed=car->speed;
    //TODO:路由表需要改
    map<pair<int,int>,Container*>* temp_table = speed_router_map[speed];//路由表
    int count=0;//计数器，统计可以转向的路口数量
    int capacity=this->channel*this->length;//道路容量

    bool not_right_crowded=false;//右侧道路是否不拥塞
    bool not_left_crowded=false;//左侧道路是否不拥塞
    bool not_straight_crowded=false;//直行道路是否不拥塞
    //TODO:路由表需要改
    Container *best_road=(*temp_table)[pair<int,int>(this->to->id,car->to)];//最优的路
    if(car->isPreset)//preset car does not change anything
        return;
    else if(car->isPrior)//prior car just use its shortest route
    {
        //TODO:路由表需要改
        (car->route).push_back((*temp_table)[pair<int,int>(this->to->id,car->to)]);
        return;
    }
    else//for ordinary car
    {
        Container* right=this->turn_to[0];
        if(right)//存在右边的路
        {
            right->probability=1;
            count++;//计数有效的路数量
            not_right_crowded=(right->get_car_sum())<( max_factor*right->length*right->channel);//判断这条路是否拥塞
            if( !not_right_crowded ) right->probability=0;//拥塞了概率就是0
        }
        //下面同理
        Container* left=this->turn_to[1];
        if(left){
            left->probability=1;
            count++;
            not_left_crowded=left->get_car_sum()<( max_factor*left->length*left->channel) ;
            if(!not_left_crowded) left->probability=0;
        }
        Container* straight=this->turn_to[2];
        if(straight){
            straight->probability=1;
            count++;
            not_straight_crowded=straight->get_car_sum()<( max_factor*straight->length*straight->channel);
            if( !not_straight_crowded) straight->probability=0;
        }
        //only return 
        //==============
        //取得各个道路的下一节点到目的节点的距离
        //如果不存在去一个很大的数字
        //TODO:
        float distance_left;
        float distance_right;
        float distance_straight;
        //==============
        
        if(count==3)//有三条有效路径
        {
            //只注释第一部分，其余部分同理
            if(best_road->id==left->id)//最有路是左边的路
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
            else if(best_road->id==right->id)//最有路是右边的路
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
        else if(count==2)//only two way
        {
            //只注释一部分，其余同理
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
        else//only one way
        {
            best_road->probability=1;//只有一条路不是空，那么这条路肯定是路由表中的最优路
        }
        CROSS* cross=this->to;
        Container* road=cross->searchRoadForCar(this);
        if(!road)
        {
            (car->route).push_back(best_road);
        }
        else
            (car->route).push_back(road);
    }
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
}

Container* CROSS::searchRoadForCar(Container *current_container)
{
    double p[4]={0,0,0,0};//最终概率
    double p_total=0;
    for(int i=0; i<awayRoadVec.size(); i++)
    {
        if(current_container)
            if(awayRoadVec[i]->roadId == current_container->roadId)
                continue;
        Container* next_road = awayRoadVec[i];      //下一条路
        int car_total = next_road->size();          //下一条路的车的数量
        int road_capacity = next_road->capacity;    //下一条路的容量
        double congestion = alpha * (double)(car_total) / (double)(road_capacity);//拥塞指标计算，此处只是其中一项
        int count = 0;          //记录有几条二阶边不为空
        double target = 0;      //记录二阶边拥塞指标之和
        for(int j=0; j<4; j++)
        {
            Container* second_road = next_road->turn_to[i];
            if(second_road)
            {
                target += (double)(second_road->size()) / (double)(second_road->capacity);
                count++;
            }
        }
        congestion += (1-alpha) * target / count;
        p[i] = next_road->probability * (1 - congestion);
        p_total += p[i];
    }
    if(p_total == 0)            //如果出去的路都不能走,就直接返回空
        return nullptr;
    for(int i=0; i<4; i++)
        p[i] /= p_total;    //将概率归一化
    double p_random = double(rand() % 10000) / 10000;
    double p_heap = 0;
    for(int i=0; i<awayRoadVec.size(); i++)
    {
        if(!p[i])   continue;
        if(p_random>=p_heap && p_random<=p_heap+p[i])
            return awayRoadVec[i];
        p_heap += p[i];
    }
    for(int i=awayRoadVec.size()-1; i>=0; i++)
    {
        if(p[i])
            return awayRoadVec[i];
    }
}


void CROSS::drive_car_init_list(bool is_prior,int global_time)
{
    
    //sort => prior>start_time>id
    //注意这边的garage是已经经过排序的
    if(is_prior)//如果是优先车辆，调度优先车辆
    {
        for (deque<CAR*>::iterator i =this->garage.begin();i!=this->garage.end();)
        {
            if((*i)->planTime>global_time)//car cant go at this time ,operate next car
            {
                i++;
                continue;
            }
            if(!(*i)->isPrior)//不是优先车辆，继续遍历
            {
                i++;
                continue;
            }
            else//是优先车辆
            {
                //TODO: 排序，a的下一个路口到目的节点的开销加这条路的开销小于b的笑一个路口到目的节点的开销加上这条路的开销
                //也就是按照这个距离升序排列，第一个元素是最近的
                //cost(this->id,a->to->id)+cost(a->to->id,(*i)->to)<cost(this->id,b->to->id)+cost(b->to->id,(*i)->to)
                sort(this->awayRoadVec.begin(),this->awayRoadVec.end(),
                    [](Container* a, Container* b) { return a->to<b->to; });
                switch(this->awayRoadVec.size())
                {
                    case 4:
                        this->awayRoadVec[0]->probability=\
                            min(0.4,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                        this->awayRoadVec[1]->probability=\
                            min(0.3,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                        this->awayRoadVec[2]->probability=\
                            min(0.2,((this->awayRoadVec[2]->get_car_sum())<( max_factor*this->awayRoadVec[2]->length*this->awayRoadVec[2]->channel)?1.0:0.0));
                        this->awayRoadVec[3]->probability=\
                            min(0.1,((this->awayRoadVec[3]->get_car_sum())<( max_factor*this->awayRoadVec[3]->length*this->awayRoadVec[3]->channel)?1.0:0.0));
                        break;
                    case 3:
                        this->awayRoadVec[0]->probability=\
                            min(0.7,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                        this->awayRoadVec[1]->probability=\
                            min(0.2,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                        this->awayRoadVec[2]->probability=\
                            min(0.1,((this->awayRoadVec[2]->get_car_sum())<( max_factor*this->awayRoadVec[2]->length*this->awayRoadVec[2]->channel)?1.0:0.0));
                        break;
                    case 2:
                        this->awayRoadVec[0]->probability=\
                            min(0.8,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                        this->awayRoadVec[1]->probability=\
                            min(0.2,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                        break;
                    case 1:
                        this->awayRoadVec[0]->probability=1;
                        break;
                    default:
                        break;
                }
                Container* temp_road;
                temp_road = searchRoadForCar(nullptr);
                if(!temp_road)//can not find road
                {
                    i++;
                    continue;
                }
                else//find road
                {
                    if(temp_road->push_back((*i)) ==Container::PUSH_BACK_STATE::SUCCESS )//success then delete it from garage
                    {
                        //这边寻路成功了，只是修改了开始时间
                        //这边我不是很确定
                        (*i)->startTime=global_time;
                        garage.erase(i);
                    }    
                    else//else operate next car
                    {
                        i++;
                        continue;
                    }
                    
                }
                
            }
        }
    }
    else
    {
        for (deque<CAR*>::iterator i =this->garage.begin();i!=this->garage.end();)
        {
            if((*i)->planTime>global_time)//car cant go at this time ,operate next car
            {
                i++;
                continue;
            }
            //TODO: 排序，a的下一个路口到目的节点的开销加这条路的开销小于b的笑一个路口到目的节点的开销加上这条路的开销
            //也就是按照这个距离升序排列，第一个元素是最近的
            //cost(this->id,a->to->id)+cost(a->to->id,(*i)->to)<cost(this->id,b->to->id)+cost(b->to->id,(*i)->to)
            sort(this->awayRoadVec.begin(),this->awayRoadVec.end(),
                [](Container* a, Container* b) { return a->to<b->to; });
            switch(this->awayRoadVec.size())
            {
                case 4:
                    this->awayRoadVec[0]->probability=\
                        min(0.4,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                    this->awayRoadVec[1]->probability=\
                        min(0.3,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                    this->awayRoadVec[2]->probability=\
                        min(0.2,((this->awayRoadVec[2]->get_car_sum())<( max_factor*this->awayRoadVec[2]->length*this->awayRoadVec[2]->channel)?1.0:0.0));
                    this->awayRoadVec[3]->probability=\
                        min(0.1,((this->awayRoadVec[3]->get_car_sum())<( max_factor*this->awayRoadVec[3]->length*this->awayRoadVec[3]->channel)?1.0:0.0));
                    break;
                case 3:
                    this->awayRoadVec[0]->probability=\
                        min(0.7,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                    this->awayRoadVec[1]->probability=\
                        min(0.2,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                    this->awayRoadVec[2]->probability=\
                        min(0.1,((this->awayRoadVec[2]->get_car_sum())<( max_factor*this->awayRoadVec[2]->length*this->awayRoadVec[2]->channel)?1.0:0.0));
                    break;
                case 2:
                    this->awayRoadVec[0]->probability=\
                        min(0.8,((this->awayRoadVec[0]->get_car_sum())<( max_factor*this->awayRoadVec[0]->length*this->awayRoadVec[0]->channel)?1.0:0.0));
                    this->awayRoadVec[1]->probability=\
                        min(0.2,((this->awayRoadVec[1]->get_car_sum())<( max_factor*this->awayRoadVec[1]->length*this->awayRoadVec[1]->channel)?1.0:0.0));
                    break;
                case 1:
                    this->awayRoadVec[0]->probability=1;
                    break;

            }
            Container* temp_road;
            temp_road = searchRoadForCar(nullptr);
            if(!temp_road)//can not find road
            {
                i++;
                continue;
            }
            else//find road
            {
                if(temp_road->push_back((*i)) ==Container::PUSH_BACK_STATE::SUCCESS )//success then delete it from garage
                {
                    //这边寻路成功了，只是修改了开始时间
                    //这边我不是很确定
                    (*i)->startTime=global_time;
                    garage.erase(i);
                }    
                else//else operate next car
                {
                    i++;
                    continue;
                }
                
            }
        }
    }
    
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
