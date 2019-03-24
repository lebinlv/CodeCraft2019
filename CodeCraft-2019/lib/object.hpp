#ifndef _OBJRCT_H_
#define _OBJECT_H_

#include <cstdint>
#include <vector>      // 寻路函数返回vector
#include <map>         // 权重字典
#include <queue>
#include <algorithm>
#include <list>
#include <fstream>
#include <stack>

#define BATCH_SIZE 100    //每个时间段发车数量
#define CAPACITY_FACTOR 1 //容量因子
#define COST_FACTOR 1.1   //开销因子

struct ROAD
{
    // (id,length,speed,channel,from,to,isDuplex)
    int id, length, max_speed, channel, from, to;
    int ini_capacity;
    bool isDuplex;

    ROAD(){}
    ROAD(int _id, int _length, int _speed, int _channel, int _from, int _to, bool _isDuplex):
         id(_id), length(_length), max_speed(_speed), channel(_channel),
         from(_from), to(_to), isDuplex(_isDuplex) {ini_capacity = _length * _channel;}
    ~ROAD(){}
};


struct CAR;


class GRAPH
{
  public:
    struct Node{
        int                           cross_id;   // 本节点代表的路口的id
        int                           capacity;   // 到达本节点的边的容量
        int                           info_idx;   // 权重和容量信息均通过info_idx访问
        ROAD *                        pRoad;      // 通过 pRoad 获取道路长度、最大速度、车道数、是否双向等信息

        static int                    node_count; // 静态变量用于统计节点个数，初始值为0
        static std::vector<ROAD *>    pRoad_vec;  // 为了便于计算 weight_map 而添加的vector

        Node(int _cross_id, ROAD *_pRoad);
        ~Node(){}
    };

    typedef std::vector<Node *> route_type; // 寻最短路函数的返回数据类型

    GRAPH(int reserve_node_count);
    ~GRAPH();

    /**
     * @brief 根据道路信息添加节点
     * 
     * @param pRoad 指向道路的指针
     * @param cross_id 这条边的终点
     */
    void add_node(ROAD* pRoad);

    /**
     * @brief 根据车速计算新的权重数组
     * 
     * @param speed 车速
     * @attention   必须在添加所有节点之后调用该函数
     */
    void add_weights(int speed);
    void add_weights(bool speed_detect_array[], int size);

    /**
     * @brief Get the least cost route object
     * 
     * @param from 
     * @param to 
     * @param speed 
     * @return route_type& 返回 vector<Node *>，如果失败则vector为空，如果成功，逆序遍历此vector便可得到路径
     */
    route_type * get_least_cost_route(CAR* car, int global_time);


  private:
    std::map<int, double*>               weight_map;  // 边对于不同速度的车，具有不同的权重
    std::map<int, std::vector<Node*> >   graph_map;   //

    double*                              p_weight;    // 每次计算最短路径之前，根据车速重定向该指针

    struct __Node{
        double       cost;
        int          cross_id;

        __Node *     parent;   // 用于到达终点时回溯得到路径
        Node *       p_Node;

        __Node(double _cost, int _cross_id, __Node* _parent, Node* _p_Node) :
               cost(_cost), cross_id(_cross_id), parent(_parent), p_Node(_p_Node){}
        ~__Node(){}

        struct Compare{
            bool operator()(const __Node* a, const __Node* b){
                return a->cost > b->cost;
            }
        };
    };
};


struct CAR
{
    // (id, from, to, speed, planTime)
    int id, from, to, speed, plan_time;
    int					start_time;
    struct Past_node {
        GRAPH::Node* node;
        double arrive_time;
        Past_node() {}
        Past_node(GRAPH::Node* _node, double _arrive_time) :node(_node), arrive_time(_arrive_time) {}
    };
    std::stack<Past_node*>      past_nodes;

    CAR() {}
    CAR(int _id, int _from, int _to, int _speed, int _time):
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

//释放在行驶车辆占用的容量以及删除到达车辆
void release_capacity(std::list<CAR *> & car_running, int global_time);

//输出answer文件
void write_to_file(std::vector<GRAPH::Node *> * tem_vec, CAR *car, std::ofstream &fout);

#endif