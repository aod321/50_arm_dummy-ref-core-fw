#ifndef REF_STM32F4_FW_DUMMY_ROBOT_H
#define REF_STM32F4_FW_DUMMY_ROBOT_H

#include "algorithms/kinematic/6dof_kinematic.h"
#include "actuators/ctrl_step/ctrl_step.hpp"
#include "string"
#define ALL 0

/*
  |   PARAMS   | `current_limit` | `acceleration` | `dce_kp` | `dce_kv` | `dce_ki` | `dce_kd` |
  | ---------- | --------------- | -------------- | -------- | -------- | -------- | -------- |
  | **Joint1** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint2** | 2               | 30             | 1000     | 80       | 200      | 200      |
  | **Joint3** | 2               | 30             | 1500     | 80       | 200      | 250      |
  | **Joint4** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint5** | 2               | 30             | 1000     | 80       | 200      | 250      |
  | **Joint6** | 2               | 30             | 1000     | 80       | 200      | 250      |
 */


class DummyHand
{
public:
    uint16_t nodeID = 0x601;
    float CurrentLimit = 0.27f;
    float currentAngle = 0.0f;
    // Control modes:
    // 0: Torque mode
    // 1: Velocity mode
    // 2: Position trajectory mode
    // 3: Position filter mode
    // 4: Position direct mode
    uint8_t mode = 0;

    DummyHand(CAN_HandleTypeDef* _hcan, uint16_t _id);


    void SetCurrentLimit(float _val);
    float GetCurrentLimit();
    void SetMode(uint8_t _mode);
    void SetAngle(float _angle);
    void SetRelativePosition(float _angle);
    void UpdateAngle();
    void UpdateMode();
    void UpdateAngleCallback(uint8_t* data);
    float GetAngle();
    uint8_t GetMode();
    void SetTorque(float _current);
    void EnableCloseLoop();
    void Save();


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_ro_property("angle", &currentAngle),
            make_protocol_function("get_mode", *this, &DummyHand::GetMode),
            make_protocol_function("get_current_limit", *this, &DummyHand::GetCurrentLimit),
            make_protocol_function("set_angle", *this, &DummyHand::SetAngle, "angle"),
            make_protocol_function("set_mode", *this, &DummyHand::SetMode, "mode"),
            make_protocol_function("set_current_limit", *this, &DummyHand::SetCurrentLimit, "current"),
            make_protocol_function("set_torque", *this, &DummyHand::SetTorque, "current"),
            make_protocol_function("enable_close_loop", *this, &DummyHand::EnableCloseLoop),
            make_protocol_function("save", *this, &DummyHand::Save)
        );
    }


private:
    CAN_HandleTypeDef* hcan;
    uint8_t canBuf[8];
    CAN_TxHeaderTypeDef txHeader;
    float minAngle = -360;
    float maxAngle = 360;
};


class DummyRobot
{
public:
    explicit DummyRobot(CAN_HandleTypeDef* _hcan);
    ~DummyRobot();


    enum CommandMode
    {
        COMMAND_TARGET_POINT_SEQUENTIAL = 1,
        COMMAND_TARGET_POINT_INTERRUPTABLE,
        COMMAND_CONTINUES_TRAJECTORY,
        COMMAND_MOTOR_TUNING
    };

    class EEFPoseHelper
    {
    public:
        float x=0,y=0,z=0,a=0,b=0,c=0;
        explicit EEFPoseHelper(DummyRobot* _context): context(_context)
        {
        }
        void UpdatePose6D();

        auto MakeProtocolDefinitions() {
            return make_protocol_member_list(
                    make_protocol_ro_property("x", &x),
                    make_protocol_ro_property("y", &y),
                    make_protocol_ro_property("z", &z),
                    make_protocol_ro_property("a", &a),
                    make_protocol_ro_property("b", &b),
                    make_protocol_ro_property("c", &c),
                    make_protocol_function("update_pose_6D", *this,
                                           &EEFPoseHelper::UpdatePose6D)
            );
        }
    private:
        DummyRobot* context;
    };


    class TuningHelper
    {
    public:
        explicit TuningHelper(DummyRobot* _context) : context(_context)
        {
        }

        void SetTuningFlag(uint8_t _flag);
        void Tick(uint32_t _timeMillis);
        void SetFreqAndAmp(float _freq, float _amp);


        // Communication protocol definitions
        auto MakeProtocolDefinitions()
        {
            return make_protocol_member_list(
                make_protocol_function("set_tuning_freq_amp", *this,
                                       &TuningHelper::SetFreqAndAmp, "freq", "amp"),
                make_protocol_function("set_tuning_flag", *this,
                                       &TuningHelper::SetTuningFlag, "flag")
            );
        }


    private:
        DummyRobot* context;
        float time = 0;
        uint8_t tuningFlag = 0;
        float frequency = 1;
        float amplitude = 1;
    };
    TuningHelper tuningHelper = TuningHelper(this);
    EEFPoseHelper eefPoseHelper = EEFPoseHelper(this);


    // This is the pose when power on.
    const DOF6Kinematic::Joint6D_t REST_POSE = {0, -73, 180, 0, 0, 0};
    const float DEFAULT_JOINT_SPEED = 30;  // degree/s
    const DOF6Kinematic::Joint6D_t DEFAULT_JOINT_ACCELERATION_BASES = {150, 100, 200, 200, 200, 200};
    const float DEFAULT_JOINT_ACCELERATION_LOW = 30;    // 0~100
    const float DEFAULT_JOINT_ACCELERATION_HIGH = 100;  // 0~100
    const CommandMode DEFAULT_COMMAND_MODE = COMMAND_TARGET_POINT_INTERRUPTABLE;


    DOF6Kinematic::Joint6D_t currentJointsVelocities = {0, 0, 0, 0, 0, 0};
    DOF6Kinematic::Joint6D_t currentJointsCurrents = {0, 0, 0, 0, 0, 0};
    DOF6Kinematic::Joint6D_t currentJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t targetJoints = REST_POSE;
    DOF6Kinematic::Joint6D_t initPose = REST_POSE;

    DOF6Kinematic::Pose6D_t currentPose6D = {};
    volatile uint8_t jointsStateFlag = 0b00000000;
    CommandMode commandMode = DEFAULT_COMMAND_MODE;
    CtrlStepMotor* motorJ[7] = {nullptr};
    DummyHand* hand = {nullptr};



    void Init();
    bool MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6);
    bool MoveL(float _x, float _y, float _z, float _a, float _b, float _c);
    bool MoveL_Direct(float _x, float _y, float _z, float _r11, float _r21, float _r31, float _r12, float _r22, float _r32);
    void MoveJoints(DOF6Kinematic::Joint6D_t _joints);
    void SetJointSpeed(float _speed);
    void SetJointAcceleration(float _acc);
    void UpdateJointAngles();
    void UpdateJointAnglesCallback();
    void UpdateJointPose6D();

    void UpdateJointCurrents();
    void UpdateJointCurrentsCallback();

    void UpdateJointVelocities();
    void UpdateJointVelocitiesCallback();

    void Reboot();
    void SetEnable(bool _enable);
    void SetRGBEnable(bool _enable);
    bool GetRGBEnabled();
    void SetRGBMode(uint32_t mode);

    uint32_t GetRGBMode();
    void CalibrateHomeOffset();
    void Homing();
    void Resting();
    bool IsMoving();
    bool IsEnabled();
    void SetCommandMode(uint32_t _mode);
    


    // Communication protocol definitions
    auto MakeProtocolDefinitions()
    {
        return make_protocol_member_list(
            make_protocol_function("calibrate_home_offset", *this, &DummyRobot::CalibrateHomeOffset),
            make_protocol_function("homing", *this, &DummyRobot::Homing),
            make_protocol_function("resting", *this, &DummyRobot::Resting),
            make_protocol_object("joint_1", motorJ[1]->MakeProtocolDefinitions()),
            make_protocol_object("joint_2", motorJ[2]->MakeProtocolDefinitions()),
            make_protocol_object("joint_3", motorJ[3]->MakeProtocolDefinitions()),
            make_protocol_object("joint_4", motorJ[4]->MakeProtocolDefinitions()),
            make_protocol_object("joint_5", motorJ[5]->MakeProtocolDefinitions()),
            make_protocol_object("joint_6", motorJ[6]->MakeProtocolDefinitions()),
            make_protocol_object("joint_all", motorJ[ALL]->MakeProtocolDefinitions()),
            make_protocol_object("hand", hand->MakeProtocolDefinitions()),
            make_protocol_function("reboot", *this, &DummyRobot::Reboot),
            make_protocol_function("set_enable", *this, &DummyRobot::SetEnable, "enable"),
            make_protocol_function("set_rgb_enable", *this, &DummyRobot::SetRGBEnable, "enable"),
            make_protocol_function("set_rgb_mode", *this, &DummyRobot::SetRGBMode, "mode"),
            make_protocol_function("move_j", *this, &DummyRobot::MoveJ, "j1", "j2", "j3", "j4", "j5", "j6"),
            make_protocol_function("move_l", *this, &DummyRobot::MoveL, "x", "y", "z", "a", "b", "c"),
            make_protocol_function("move_l_direct", *this, &DummyRobot::MoveL_Direct, 
                                  "x", "y", "z", 
                                  "r11", "r21", "r31", 
                                  "r12", "r22", "r32"),
            make_protocol_function("set_joint_speed", *this, &DummyRobot::SetJointSpeed, "speed"),
            make_protocol_function("set_joint_acc", *this, &DummyRobot::SetJointAcceleration, "acc"),
            make_protocol_function("set_command_mode", *this, &DummyRobot::SetCommandMode, "mode"),
            make_protocol_object("tuning", tuningHelper.MakeProtocolDefinitions()),
            make_protocol_object("eef_pose", eefPoseHelper.MakeProtocolDefinitions())
        );
    }




    class CommandHandler
    {
    public:
        explicit CommandHandler(DummyRobot* _context) : context(_context)
        {
            commandFifo = osMessageQueueNew(16, 64, nullptr);
        }

        uint32_t Push(const std::string &_cmd);
        std::string Pop(uint32_t timeout);
        uint32_t ParseCommand(const std::string &_cmd);
        uint32_t GetSpace();
        void ClearFifo();
        void EmergencyStop();


    private:
        DummyRobot* context;
        osMessageQueueId_t commandFifo;
        char strBuffer[64]{};
    };
    CommandHandler commandHandler = CommandHandler(this);


private:
    CAN_HandleTypeDef* hcan;
    float jointSpeed = DEFAULT_JOINT_SPEED;
    float jointSpeedRatio = 1;
    DOF6Kinematic::Joint6D_t dynamicJointSpeeds = {1, 1, 1, 1, 1, 1};
    DOF6Kinematic* dof6Solver;
    bool isEnabled = false;
    bool isRGBEnabled = false;
    uint32_t rgbMode = 0;

    bool isDragEnabled = false;
};


#endif //REF_STM32F4_FW_DUMMY_ROBOT_H
