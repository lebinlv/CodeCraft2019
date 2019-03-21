#ifndef _OBJRCT_H_
#define _OBJECT_H_

#include <cstdint>
#include <vector>      // 寻路函数返回vector
#include <queue>       // 寻路算法中使用 priority_queue
#include <map>         // 权重字典

struct CROSS
{
    typedef uint8_t     id_type;  // 路口id类型
};

struct ROAD
{
    typedef uint16_t    id_type;
    typedef uint8_t     length_type;
    typedef uint8_t     speed_type;
    typedef uint8_t     channel_type;
    typedef int16_t     capacity_type;

    // (id,length,speed,channel,from,to,isDuplex)
    id_type             id;
    length_type         length;
    speed_type          max_speed;
    capacity_type       capacity;
    channel_type        channel;

    ROAD(){}
    ROAD(id_type _id, length_type _length, speed_type _speed, channel_type _channel):
         id(_id), length(_length), max_speed(_speed), channel(_channel) {
        capacity = _length * _channel;
    }
    ~ROAD(){}
};


struct CAR
{
    typedef uint16_t    id_type;
    typedef uint8_t     speed_type;
    typedef uint8_t     time_type;

    // (id, from, to, speed, planTime)
    id_type             id;
    CROSS::id_type      from;
    CROSS::id_type      to;
    speed_type          speed;
    time_type           plan_time;

    CAR() {}
    CAR(id_type _id, CROSS::id_type _from, CROSS::id_type _to, speed_type _speed, time_type _time):
        id(_id), from(_from), to(_to), speed(_speed), plan_time(_time) {}
    ~CAR() {}

    /** 
     * @brief 自定义优先队列的比较方法
     * 
     * 如果两辆车的计划时间相等，则先速度快的车优先级高；
     * 否则先出发的车优先级高。
     */
    struct Compare {
        bool operator()(const CAR* a, const CAR* b) {
            return (a->plan_time == b->plan_time) ?
                   (a->speed < b->speed) : (a->plan_time > b->plan_time);
        }
        bool operator()(const CAR & a, const CAR & b) {
            return (a.plan_time == b.plan_time) ?
                   (a.speed < b.speed) : (a.plan_time > b.plan_time);
        }
    };
}; 



class GRAPH
{
  public:
    typedef double                    weight_type;  // 边的权重的数据类型
    typedef uint16_t                  idx_type;     // 下标类型
    typedef std::vector<CROSS::id_type>    route_type;   // 寻最短路函数的返回数据类型

    struct Node{
        CROSS::id_type       cross_id;        // 本节点代表的路口的id
        ROAD::id_type        road_id;         // 到达本节点的路的id
        //weight_type          weight;          // 到达本节点的路的权重
        idx_type             weight_idx;      // 到达该节点的边的权重在权重数组中的下标

        Node* next_node = nullptr;

        Node(CROSS::id_type _cross_id, ROAD::id_type _road_id) : 
             cross_id(_cross_id), road_id(_road_id) {weight_idx = node_count++;}
        //Node(){}
        ~Node(){delete next_node;}

      private:
        static idx_type      node_count;      // 静态变量用于统计节点个数，初始值为0
    };

    GRAPH(){}
    ~GRAPH(){}

    /**
     * @brief 根据道路信息添加节点
     * 
     * @param road_id  道路id
     * @param from     道路起点路口id
     * @param to       道路终点路口id
     * @param isDuplex 是否双向
     */
    void add_node(ROAD::id_type road_id, CROSS::id_type from, CROSS::id_type to, bool isDuplex);

    /**
     * @brief 根据车速计算新的权重数组
     * 
     * @param speed 车速
     * @attention   必须在添加所有节点之后调用该函数
     */
    void add_weight_accord_to_speed(CAR::speed_type speed);

    /**
     * @brief Get the least cost route object
     * 
     * @param from 
     * @param to 
     * @param speed 
     * @return route_type* 
     */
    route_type* get_least_cost_route(CROSS::id_type from, CROSS::id_type to, CAR::speed_type speed);


  private:
    std::map<CAR::speed_type, weight_type*>  weight_map;  // 边对于不同速度的车，具有不同的权重
    std::map<CROSS::id_type, Node*>          graph_map;   //


    weight_type*                        p_weight;    // 每次计算最短路径之前，根据车速重定向该指针

    struct __Node{
        weight_type       cost;
        CROSS::id_type    cross_id;

        __Node *       parent;
		Node*          p_Node;

		__Node(weight_type _cost, CROSS::id_type _cross_id, __Node* _parent, Node* _p_Node) :
			cost(_cost), cross_id(_cross_id), parent(_parent), p_Node(_p_Node) {}
        ~__Node(){}

		struct Compare {};
    };

};



#endif