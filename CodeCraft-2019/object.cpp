#include "lib/object.hpp"

#define ROAD_VECTOR_RESERVE   64     // 为寻路函数返回的vector预分配的空间，建议为路口的数目的一半
using namespace std;


Container::Container(int channel, int length)
{
    this->channel=channel;
    this->length=length;
    carInChannel = new container_t [channel];
    for(int i=0; i<channel; ++i) {
        carInChannel[i].reserve(length);
    }
}

// TODO: Container::push_back()
inline bool Container::push_back(CAR* pCar)
{
    //当前车辆idx还没有变，同时nextroad保存的也就是当前的道路
    //需要调度完成后运行更新优先队列
    for (int i=0;i<this->channel;i++)
    {
        int s2;
        CAR* pre_car=this->getCarVec(i).back();
        if(pre_car->state==CAR::CAR_STATE::END)
        {
            if(pre_car->idx==this->length-1)
               continue;
            else
            {
                s2 = max(0, min(pCar->speed, pCar->next_road->max_speed) - pCar->idx);
                
                this->getCarVec(i).push_back(pCar);
                //这边没有从原来的vector里面把这辆车删掉
                pCar->idx=length-min(s2,this->length-1-pre_car->idx);
                pCar->state=CAR::CAR_STATE::END;
                if(pCar->idx-pre_car->idx==1)
                    pCar->v=min(pre_car->v,pCar->speed);
                else
                    pCar->v=pCar->speed;
                return true;
            }
        }
        else if(pre_car->state==CAR::CAR_STATE::WAIT)
        {
            s2 = max(0, min(pCar->speed, pCar->next_road->max_speed) - pCar->idx);
            if(s2<=this->length-1-pre_car->idx)
            {
                this->getCarVec(i).push_back(pCar);
                //这边没有从原来的vector里面把这辆车删掉
                pCar->state=CAR::CAR_STATE::END;
                pCar->idx=length-min(s2,this->length-1-pre_car->idx);
                if(pCar->idx-pre_car->idx==1)
                    pCar->v=min(pre_car->v,pCar->speed);
                else
                    pCar->v=pCar->speed;
                return true;
            }
            else
                return false;
        }
    }
    return false;
}

// TODO: Container::top()
inline CAR *Container::top()
{
    return priCar.top();
}

// TODO: Container::pop()
inline bool Container::pop()
{
    if(this->priCar.size()==0)
        return false;
    else
    {
        //得到优先队列的第一个元素
        int channel_idx=this->priCar.top()->channel_idx;
        //得到这辆车所在的车道
        vector<CAR*> & temp=this->getCarVec(channel_idx);
        //删除该车道的第一辆车
        temp.erase(temp.begin());
        this->priCar.pop();
        return true;
    }
}

void Container::update_prior_queue(int channel_idx)
{
    if(carInChannel[channel_idx].front()->state != CAR::END)
        priCar.push(carInChannel[channel_idx].front());
}


ROAD::ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex) : 
           id(_id), length(_length), max_speed(_speed), channel(_channel),
           from(_from), to(_to), isDuplex(_isDuplex)
{
    capacity = _length * _channel;
    forward = new Container(_channel, _length);
    backward = _isDuplex ? new Container(_channel, _length) : nullptr;
}

// TODO: 
void ROAD::moveOnRoad()
{
    //取得车道数量
    int channels=this->channel;
    //车道长度
    for (int i=0;i<channels;i++)
    {
        vector<CAR*> & carVec = this->backward->getCarVec(channels);
        dispatch_one_channel(carVec);
        vector<CAR*> & carVec = this->forward->getCarVec(channels);
        dispatch_one_channel(carVec);
    }
}

void dispatch_one_channel(std::vector<CAR*> & carVec)
{
    int len=carVec.size();
    CAR* pre_car=NULL;
    int s2;//the length that car can drive in next road

    for (int i=0;i<len;i++)
    {
        //car can pass 
        if(i==0)//for the
        {
            //compile algorithm, get next road 
            //===========================
            //carVec[i]->next_road=get_next_road();
            //===========================
            if(carVec[i]->v>i) 
            {
                s2 = max(0, std::min(carVec[i]->speed, carVec[i]->next_road->max_speed) - i);
                if(s2>0)
                {
                    carVec[i]->state=CAR::CAR_STATE::WAIT;
                    pre_car=carVec[i];
                    continue;
                }
                else//car canot pass cross
                {
                    carVec[i]->state=CAR::CAR_STATE::END;
                    pre_car=carVec[i];
                    carVec[i]->idx=0;
                    continue;
                }
            }
            else
            {
                carVec[i]->state=CAR::CAR_STATE::END;
                pre_car=carVec[i];
                carVec[i]->idx-=carVec[i]->v;
                continue;
            } 
        }
        else
        {
            //car can drive without front car's influence
            if(carVec[i]->v<(pre_car->idx-carVec[i]->idx))
            {
                carVec[i]->state=CAR::CAR_STATE::END;
                carVec[i]->idx-=carVec[i]->v;
                pre_car=carVec[i];
                continue;
            }
            else//car can drive with front car's influence
            {
                //if the front car is end
                if(pre_car->state==CAR::CAR_STATE::END)
                {    
                    carVec[i]->state=CAR::CAR_STATE::END;
                    carVec[i]->idx-=pre_car->idx+1;
                    carVec[i]->v=min(carVec[i]->v,pre_car->v);
                    pre_car=carVec[i];
                    continue;
                }
                //if front car is wait
                else if(pre_car->state==CAR::CAR_STATE::WAIT)
                {    
                    carVec[i]->state=CAR::CAR_STATE::WAIT;
                    pre_car=carVec[i];
                    continue;
                }
            }
            
        }
        
        
    }
}




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
            if(visited_map.find(node->cross_id) == visited_map.end() && node->capacity > ROAD_VALID_THRESHOLD) {
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
    car->start_time = global_time;
    // 上面的while循环结束时，record指向目的节点，接下来从record开始回溯，获得完整路径
    while (record->parent != nullptr) {

        record->p_Node->capacity -= car->capacity_factor; //容量减少
        car->past_nodes.push(new CAR::Past_node(record->p_Node, record->cost + global_time)); //记录节点到该车辆的对象中

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
