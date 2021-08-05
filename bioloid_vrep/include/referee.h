#ifndef BIOLOID_VREP_ROS_REFEREE_H
#define BIOLOID_VREP_ROS_REFEREE_H

// Standard C++ Library
#include <map>
#include <string>


using namespace std;
/*
 * 类型说明
 * 提供裁判服务
 * 判断出界、失败等各种情况
 * */
class Referee{
private:    // 比赛进程相关
    bool record_mark;               // 是否可以记录时间的标志
    bool started_mark;              // 是否开始过，不能再次开始
    bool real_finish_mark;          // 是否完全完成比赛
    // 当前阶段(0:准备开始 1:等待第一段完成 2:等待第二段完成)
    int current_phrase;
private:    // 时间相关
    float start_time;               // 第一段开始的时间
    float first_finish_time;        // 第一段完成的时间
    float second_finish_time;       // 第二段完成的时间
private:    // 场地相关
    const float left_border = -0.45;
    const float right_border = 0.45;
    const float ready_border = 1.5;
    const float goal_border = -1.5;
private:
    const float reset_x = 1.55;
    const float reset_y = 0;
    const float reset_z = 0.31349;
private:    // 单例相关
    Referee(){                      // 构造函数
        record_mark = false;
        started_mark = false;
        real_finish_mark = false;

        start_time = -1;
        first_finish_time = -1;
        second_finish_time = -1;

        current_phrase = 0;         // 等待开始的阶段
    }
    Referee(Referee &){}
public:
    static Referee &getInstance(){
        static Referee instance;
        return instance;
    }
    // 处理开始的标记，使得时间消息监听程序开始计时
    void start_game();
    // 记录当前时间（phrase三种状态 0：开始 1:第一段结束 2：第二段结束）
    void record_time(float cur_time);
    // 处理本场结束的所有信息，包含完成的段数、完成每段的时间
    void end_game(string reason);
    // 犯规判定，根据传入的身体的位置判定是否出界
    bool foul_judge(map<string, float>&body);
    // 完成判定，根据传入的双脚位置判定是否完成
    bool finish_judge(map<string, float>&left_foot, map<string, float>&right_foot);
    // 超时判定，根据传入的时间来判定是否超时
    bool time_up_judge(float cur_time);
    // 返回是否完全完成了比赛
    bool full_finish_judge();
};


#endif //BIOLOID_VREP_ROS_REFEREE_H