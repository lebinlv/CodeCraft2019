#ifndef _DEFINE_H_
#define _DEFINE_H_

#define __DEBUG_MODE__ 1

/* 程序限制相关 */

  // 所有文件(presetAnswer.txt除外)中每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE
  #define MAXIMUM_LENGTH_PER_LINE 50 // char line_buffer[MAXIMUM_LENGTH_PER_LINE]; fgets(line_buffer, 50, fptr);

  // 用于控制车辆速度种类检测数组的长度，应 >= 最大车速+1
  #define SPEED_DETECT_ARRAY_LENGTH 50 

/* End */


/* 提高性能相关, 空间换时间 */

  // 为每一个路口的车库vector预分配的空间大小
  #define GARAGE_RESERVE_SIZE 512 // vector<CAR*> car_vec; car_vec.reserve(CAR_VECTOR_RESERVE_SIZE);

  // 为 cross vector 预分配的空间大小
  #define CROSS_VECTOR_RESERVE_SIZE 256 // vector<CROSS *> crossVec; crossVec.reserve(CROSS_VECTOR_RESERVE_SIZE);

  #define CONTAINER_VECTOR_RESERVE_SIZE 512

  #define CAR_ROUTE_VECTOR_RESERVE_SIZE 24

  // 为减少磁盘IO操作次数，优化速度，为输出文件answer.txt预分配空间，单位：KB
  #define OUT_FILE_RESERVE_SPACE 1024 // char out_file_buffer[OUT_FILE_RESERVE_SPACE*1024]; fout.rdbuf()->pubsetbuf(out_file_buffer, OUT_FILE_RESERVE_SPACE*1024);

/* END */




#if __DEBUG_MODE__
    // `std::map`为有序容器，便于debug，但查找效率低于hash，正式代码中应改为`std::unordered_map`
    #define map_type std::map

    // 将两个 `road_id` 合并为一个全局唯一的 id
    #define MERGE(a, b) (((a)*10000)+(b))

    // 路由表的索引： uint32_t    speed*1e8 + removeRoadId*1e4 + 目的crossId
    #define ROUTEID(a, b, c)((a)*1e8+(b)*1e4+(c))
#else
    // `std::map`为有序容器，便于debug，但查找效率低于hash，正式代码中应改为`std::unordered_map`
    #define map_type std::unordered_map
    // 将两个 id 合并为一个全局唯一的 id
    #define MERGE(a, b) (((a)<<16)|(b))

    // 路由表的索引： uint32_t    | speed 6bit(0~63) | removeRoadId 14bit(0~16383) | 目的crossId 12bit(0~4095) |
    #define ROUTEID(a, b, c) ( (((a)<<26)|((b)<<12)) | (c) )
#endif


#endif