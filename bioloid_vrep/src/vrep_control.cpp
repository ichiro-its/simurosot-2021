// Custom C++ Library
#include <iostream>
#include <map>
#include <string>
#include <unistd.h>
#include <time.h>
#include <cstring>
#include <vector>


// Custom C++ Library
#include "../remoteApi/extApi.h"
#include "../include/vrep_control.h"
#include "../include/utils.h"


using namespace std;


int VrepControl::get_client_id(){
    if(client_id == -1){
        connect();
    }
    return client_id;
}

bool VrepControl::connect(string _server_ip, int _server_port) {
    server_ip = _server_ip;
    server_port = _server_port;
    client_id = simxStart((simxChar*)server_ip.c_str(), server_port, true, true, 2000, 1);
    if(client_id != -1) {
        cout << "connect successful" << endl;
        return true;
    }
    cout << "connect failed" << endl;
    return false;
}

void VrepControl::disconnect() {
    if(client_id != -1)
        simxFinish(client_id);
    cout << "Connection over!" << std::endl;
}

void VrepControl::init_handle() {
    vector<string>joints_name = get_all_names(0);
    for(const auto&joint_name: joints_name){
        char* ch = new char[joint_name.length() + 1];
        strcpy(ch, joint_name.c_str());
        int temp_handle = 0;
        temp_handle = found_joint(ch, temp_handle);
        joints_handle[joint_name] = temp_handle;
        delete[] ch;
    }
}

int VrepControl::found_joint(char *joint_name, int &c) {
    if(!(simxGetObjectHandle(client_id, (const simxChar*) joint_name, (simxInt *) &c, (simxInt) simx_opmode_oneshot_wait))){
        return c;
    }
    return 0;
}

int VrepControl::get_joint_handle(string joint_name){
    if(joints_handle.count(joint_name) == 0)
        return 0;
    return joints_handle[joint_name];
}

vector<string>  VrepControl::get_all_names(int robotid){
    vector<string > res;
    for(int i = 0; i < 20; i++){
        if (i == 7 || i == 8)
            continue;
        res.push_back(get_member_str(robotid) + int_to_str(i));
    }
    return res;
}



