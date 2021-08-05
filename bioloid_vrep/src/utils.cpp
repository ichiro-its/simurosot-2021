#include "../include/utils.h"

string int_to_str(int input){
    string res;         // 返回值
    stringstream ss;    // 字符串流

    ss << input;        // int流入
    ss >> res;          // 流出string
    return res;
}

// 获取第i个机器人的关节前缀字符串
// 例： "joint_0_"
string get_member_str(int no){
    string res_str = "joint_";
    res_str += int_to_str(no) + "_";
    return res_str;
}

// map相加
void map_plus(map<string, float> &para1, map<string, float> para2){
    for(const auto &ele: para2){
        if(para1.count(ele.first) == 1)
            para1[ele.first] += ele.second;
    }
}
