#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include "object.hpp"

#define MAXIMUM_LENGTH_PER_LINE 50 // 每行字符个数不超过 MAXIMUM_LENGTH_PER_LINE

using namespace std;

int main(int argc, char *argv[])
{
    // // 生成一个10000行的文件用于测试
    // ofstream out("test.txt", ios::out);
    // for(int i=0; i<2000000; i++) {
    //     out << "(10003, 16, 34, 4, 3)\n";
    // }
    // out.close();

    ios::sync_with_stdio(false);
    cout << "Begin" << endl;

    if(argc < 5){
        cout << "please input args: carPath, roadPath, crossPath, answerPath" << endl;
        exit(1);
    }

    string carPath(argv[1]);
    string roadPath(argv[2]);
    string crossPath(argv[3]);
    string answerPath(argv[4]);

    cout << "carPath is " << carPath << endl;
    cout << "roadPath is " << roadPath << endl;
    cout << "crossPath is " << crossPath << endl;
    cout << "answerPath is " << answerPath << endl;


    // buffered sstream mode
    stringstream localstream; /* 创建流*/
    char line_buffer[MAXIMUM_LENGTH_PER_LINE];

    ifstream file(carPath);
    if (file)
    {
        /* 获取文件大小 */
        file.seekg(0, ios::end);
        size_t length = file.tellg();
        file.seekg(0, ios::beg);

        /* 一次性读取全部内容 */
        char *buffer = new char[length];
        file.read(buffer, length);

        localstream.rdbuf()->pubsetbuf(buffer, length);

        /* 逐行读取车辆信息 */
        while (localstream.getline(line_buffer, 50))
        {
            if ('(' == line_buffer[0])
            {
                auto pCar = new CAR;
                sscanf(line_buffer, "(%d, %d, %d, %d, %d)",
                       &(pCar->id), &(pCar->from), &(pCar->to), &(pCar->speed), &(pCar->plan_time));
            }
        }
        delete[] buffer;
        file.close();
    }
    else
        exit(-1);

    getchar();
    return 0;
}