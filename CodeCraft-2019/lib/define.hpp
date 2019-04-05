#ifndef _DEFINE_H_
#define _DEFINE_H_

#define __DEBUG_MODE__ 1

// 所有文件中每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
#define MAXIMUM_LENGTH_PER_LINE 50 // char line_buffer[MAXIMUM_LENGTH_PER_LINE]; fgets(line_buffer, 50, fptr);

// 为每一个路口的车库vector预分配的空间大小
#define GARAGE_RESERVE_SIZE 512 // vector<CAR*> car_vec; car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

// 为 cross vector 预分配的空间大小
#define CROSS_VECTOR_RESERVE_SIZE 256 // vector<CROSS *> crossVec; crossVec.reserve(CROSS_VECTOR_RESERVE_SIZE);

// 为图中的vector预分配空间，假设图包含n*n个路口，建议设置为 4n(n-1)
#define NODE_VECTOR_RESERVE_SIZE 360 // GRAPH graph(NODE_VECTOR_RESERVE_SIZE);

// 官方保证车辆速度小于道路长度length，因此创建一个长度为length的bool数组检测有多少种车速
#define MAXIMUM_ROAD_LENGTH 50 // 道路的最大长度

// 最大计划出发时间应小于 PLAN_TIME_DETECT_ARRAY_SIZE
#define PLAN_TIME_DETECT_ARRAY_SIZE 50 //

// 为减少磁盘IO操作次数，优化速度，为输出文件answer.txt预分配空间，单位：KB
#define OUT_FILE_RESERVE_SPACE 1024 // char out_file_buffer[OUT_FILE_RESERVE_SPACE*1024]; fout.rdbuf()->pubsetbuf(out_file_buffer, OUT_FILE_RESERVE_SPACE*1024);



#ifdef __DEBUG_MODE__
// `std::map`为有序容器，便于debug，但查找效率低于hash，正式代码中应改为`std::unordered_map`
#define map_type std::map
// 将两个 id 合并为一个全局唯一的 id
#define MERGE(a, b) (((a)*10000)+(b))
#else
// `std::map`为有序容器，便于debug，但查找效率低于hash，正式代码中应改为`std::unordered_map`
#define map_type std::unordered_map
// 将两个 id 合并为一个全局唯一的 id
#define MERGE(a, b) (((a)<<16)|(b))
#endif


#endif