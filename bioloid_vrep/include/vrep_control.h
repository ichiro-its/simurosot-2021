#ifndef BIOLOID_VREP_VREP_CONTROL_H
#define BIOLOID_VREP_VREP_CONTROL_H

#include <string>

using namespace std;


/*
 * 类型说明
 * 与V-REP通信
 * 与V-REP的连接管理
 * V-REP中节点的handle管理
 * 带有执行时间和动作分解的动作执行方法
 * */
class VrepControl{
private:
    string server_ip;
    int server_port;
    int client_id;
    map<string, int> joints_handle;
private:
    VrepControl(){
        client_id = -1;
    }
    VrepControl(VrepControl&){};

public:
    static VrepControl &getInstance(){
        static VrepControl instance;
        return instance;
    }
    int get_client_id();
    bool connect(string _server_ip="127.0.0.1", int _server_port=19999);
    void disconnect();
    void init_handle();
    int found_joint(char* joint_name, int &c);
    int get_joint_handle(string joint_name);
    vector<string> get_all_names(int robotid);
};



#endif //BIOLOID_VREP_VREP_CONTROL_H
