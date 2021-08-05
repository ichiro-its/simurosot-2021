// Standard C++ Library
#include <iostream>
#include <cmath>

// ROS C++ Library
#include <ros/ros.h>

// Custom C++ Library
#include "../include/referee.h"


using namespace std;


void Referee::start_game(){
    if(!record_mark && !started_mark){
        cout << "Time Starts" << endl;
        record_mark = true;
        started_mark = true;
    }
}

void Referee::record_time(float cur_time) {
    if(!record_mark)             // 针对没开始的情况
        return;
    switch (current_phrase) {   // 分阶段考虑
        case 0:                 // 准备开始时记录开始时间
            start_time = cur_time;
            break;
        case 1:                 // 第一段结束的时间记录
            first_finish_time = cur_time;
            break;
        case 2:                 // 第二段结束的时间记录
            second_finish_time = cur_time;
            end_game("finish the game");
            break;
    }
    record_mark = false;
    current_phrase++;           // 阶段切换
}

void Referee::end_game(string reason="") {
    if(first_finish_time == -1){
        cout << "failed to finish the game" << endl;
    }
    else if(second_finish_time == -1){
        cout << "finish the first phrase" << endl;
        cout << "  first phrase time: " << first_finish_time - start_time << endl;
    }
    else {
        cout << "finish the full game" << endl;
        cout << "  first phrase time: " << first_finish_time - start_time << endl;
        cout << "  second phrase time: " << second_finish_time - first_finish_time << endl;
        cout << "  the full time: " << second_finish_time - start_time << endl;
    }
    if(reason != ""){
        cout << "reason: " << reason << endl;
    }
    // 关闭ROS监听，结束主程序
    ros::shutdown();
}

bool Referee::foul_judge(map<string, float> &body) {
    if(start_time == -1)
        return false;
    // 对于赛道，左右为y轴，前后为x轴
    if(abs(body["y"]) > abs(left_border)){
        return true;
    }
    return false;
}

bool Referee::finish_judge(map<string, float> &left_foot, map<string, float> &right_foot) {
    if(start_time == -1)
        return false;
    if(first_finish_time == -1){
        if(left_foot["x"] < goal_border && right_foot["x"] < goal_border){
            record_mark = true;
            return true;
        }
    }
    else if(second_finish_time == -1){
        if(left_foot["x"] > ready_border && right_foot["x"] > ready_border){
            real_finish_mark = true;
            record_mark = true;
            return true;
        }
    }
    return false;
}

bool Referee::time_up_judge(float cur_time) {
    if (start_time == -1)
        return false;
    else if (cur_time - start_time > 180)
        return true;
    else
        return false;
}

bool Referee::full_finish_judge() {
    return real_finish_mark;
}