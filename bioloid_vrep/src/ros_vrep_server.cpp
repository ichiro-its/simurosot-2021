// Standard C++ Library
#include <vector>
#include <string>
#include <map>


// Custom C++ Library
#include "../remoteApi/extApi.h"
#include "../include/ros_vrep_server.h"
#include "../include/vrep_control.h"
#include "../include/referee.h"
using namespace std;



void joint_callback_ser(const sensor_msgs::JointState& data)
{
    for (int i = 0; i < data.name.size(); ++i){
        int temp_handle = VrepControl::getInstance().get_joint_handle(data.name.at(i));
        int temp_client_id = VrepControl::getInstance().get_client_id();
        if (temp_handle == 0 || temp_client_id == -1)
            continue;
        simxSetJointTargetPosition(temp_client_id, (simxInt) temp_handle, data.position.at(i), simx_opmode_oneshot);
    }
    Referee::getInstance().start_game();
}

void pos_callback_ser(const tf2_msgs::TFMessage& message){
    map<string, float> left_foot;
    map<string, float> right_foot;
    map<string, float> body;
    int msg_size = message.transforms.size();
    for (int i = 0; i < msg_size; i++){
        if (message.transforms[i].child_frame_id == "bioloidrobot_0"){
            body["x"] = message.transforms[i].transform.translation.x;
            body["y"] = message.transforms[i].transform.translation.y;
            body["z"] = message.transforms[i].transform.translation.z;
        }
        else if (message.transforms[i].child_frame_id == "left_foot"){
            left_foot["x"] = message.transforms[i].transform.translation.x;
            left_foot["y"] = message.transforms[i].transform.translation.y;
            left_foot["z"] = message.transforms[i].transform.translation.z;
        }
        else if (message.transforms[i].child_frame_id == "right_foot"){
            right_foot["x"] = message.transforms[i].transform.translation.x;
            right_foot["y"] = message.transforms[i].transform.translation.y;
            right_foot["z"] = message.transforms[i].transform.translation.z;
        }
    }
    if(Referee::getInstance().foul_judge(body)){
        int temp_client_id = VrepControl::getInstance().get_client_id();
        // int *obj_handle = new int(-1);
        // simxGetObjectHandle(temp_client_id, "bioloidrobot_0", obj_handle, simx_opmode_blocking);
        cout << "Foul detected" << endl;
        // float * temp_pos = new float[3];
        // temp_pos[0] = 1.55;
        // temp_pos[1] = 0;
        // temp_pos[2] = 0.31349;
        
        // simxSetObjectPosition(temp_client_id, *obj_handle, -1, temp_pos, simx_opmode_oneshot);
        // cout << "Need call v-rep control to reset the robot" << endl;
        // simExecuteScriptString();
        // simxCallScriptFunction(simxInt clientID,const simxChar* scriptDescription,simxInt options,const simxChar* functionName,simxInt inIntCnt,const simxInt* inInt,simxInt inFloatCnt,const simxFloat* inFloat,simxInt inStringCnt,const simxChar* inString,simxInt inBufferSize,const simxUChar* inBuffer,simxInt* outIntCnt,simxInt** outInt,simxInt* outFloatCnt,simxFloat** outFloat,simxInt* outStringCnt,simxChar** outString,simxInt* outBufferSize,simxUChar** outBuffer,simxInt operationMode)
        simxCallScriptFunction(temp_client_id, 	"bioloidrobot_0", sim_scripttype_childscript, "resetRobotPosition", 0,  nullptr, 0, nullptr, 0, nullptr,  0, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, simx_opmode_oneshot);
        return;
    }
    if(Referee::getInstance().finish_judge(left_foot, right_foot)){
        cout << "Finished part of the full game" << endl;
    }
    if(Referee::getInstance().full_finish_judge()){
        cout << "Finished the full game" << endl;
        int temp_client_id = VrepControl::getInstance().get_client_id();
        simxStopSimulation(temp_client_id, simx_opmode_oneshot);
    }
}

void time_callback_ser(const std_msgs::Float32& simulation_time){
    if(Referee::getInstance().time_up_judge(simulation_time.data)){
        Referee::getInstance().end_game("Time is up");
    }
    Referee::getInstance().record_time(simulation_time.data);
}