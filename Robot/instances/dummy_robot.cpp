#include "communication.hpp"
#include "dummy_robot.h"

inline float AbsMaxOf6(DOF6Kinematic::Joint6D_t _joints, uint8_t &_index)
{
    float max = -1;
    for (uint8_t i = 0; i < 6; i++)
    {
        if (abs(_joints.a[i]) > max)
        {
            max = abs(_joints.a[i]);
            _index = i;
        }
    }

    return max;
}


DummyRobot::DummyRobot(CAN_HandleTypeDef* _hcan) :
    hcan(_hcan)
{
    motorJ[ALL] = new CtrlStepMotor(_hcan, 0, false, 1, -180, 180);
    motorJ[1] = new CtrlStepMotor(_hcan, 1, true, 50, -170, 170);
    motorJ[2] = new CtrlStepMotor(_hcan, 2, false, 50, -75, 90);
    motorJ[3] = new CtrlStepMotor(_hcan, 3, false, 50, 0, 180);
    motorJ[4] = new CtrlStepMotor(_hcan, 4, true, 50, -180, 180);
    motorJ[5] = new CtrlStepMotor(_hcan, 5, true, 50, -100, 120);
    motorJ[6] = new CtrlStepMotor(_hcan, 6, true, 50, -720, 720);
    hand = new DummyHand(_hcan, 0x601);

    // dof6Solver = new DOF6Kinematic(0.109f, 0.035f, 0.146f, 0.115f, 0.052f, 0.072f);
    dof6Solver = new DOF6Kinematic(0.1265f, 0.035f, 0.146f, 0.117f, 0.052f, 0.0755f);
}


DummyRobot::~DummyRobot()
{
    for (int j = 0; j <= 6; j++)
        delete motorJ[j];

    delete hand;
    delete dof6Solver;
}


void DummyRobot::Init()
{
    SetCommandMode(DEFAULT_COMMAND_MODE);
    SetJointSpeed(DEFAULT_JOINT_SPEED);
}


void DummyRobot::Reboot()
{
    motorJ[ALL]->Reboot();
    osDelay(500); // waiting for all joints done
    HAL_NVIC_SystemReset();
}

void DummyRobot::MoveJoints(DOF6Kinematic::Joint6D_t _joints)
{
    for (int j = 1; j <= 6; j++)
    {
        motorJ[j]->SetAngleWithVelocityLimit(_joints.a[j - 1] - initPose.a[j - 1],
                                             dynamicJointSpeeds.a[j - 1]);
    }
}


bool DummyRobot::MoveJ(float _j1, float _j2, float _j3, float _j4, float _j5, float _j6)
{
    DOF6Kinematic::Joint6D_t targetJointsTmp(_j1, _j2, _j3, _j4, _j5, _j6);
    bool valid = true;

    for (int j = 1; j <= 6; j++)
    {
        if (targetJointsTmp.a[j - 1] > motorJ[j]->angleLimitMax ||
            targetJointsTmp.a[j - 1] < motorJ[j]->angleLimitMin)
            valid = false;
    }

    if (valid)
    {
        DOF6Kinematic::Joint6D_t deltaJoints = targetJointsTmp - currentJoints;
        uint8_t index;
        float maxAngle = AbsMaxOf6(deltaJoints, index);
        float time = maxAngle * (float) (motorJ[index + 1]->reduction) / jointSpeed;
        for (int j = 1; j <= 6; j++)
        {
            dynamicJointSpeeds.a[j - 1] =
                abs(deltaJoints.a[j - 1] * (float) (motorJ[j]->reduction) / time * 0.1f); //0~10r/s
        }

        jointsStateFlag = 0;
        targetJoints = targetJointsTmp;

        return true;
    }

    return false;
}


bool DummyRobot::MoveL(float _x, float _y, float _z, float _a, float _b, float _c)
{
    DOF6Kinematic::Pose6D_t pose6D(_x, _y, _z, _a, _b, _c);
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};

    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);

    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

/**
 * @brief Move robot to specified position with rotation matrix
 * @details Uses the first two columns of a rotation matrix (6 values) to specify orientation
 *          Avoids Euler angle conversion by directly using the rotation matrix for IK
 * 
 * @param _x X position in mm
 * @param _y Y position in mm
 * @param _z Z position in mm
 * @param _r11,_r21,_r31 First column of the rotation matrix
 * @param _r12,_r22,_r32 Second column of the rotation matrix
 * @return true if motion is valid and started, false otherwise
 */
bool DummyRobot::MoveL_Direct(float _x, float _y, float _z, 
                             float _r11, float _r21, float _r31,
                             float _r12, float _r22, float _r32)
{
    #include "algorithms/kinematic/rotation_utils.hpp"
    
    // Create a pose
    DOF6Kinematic::Pose6D_t pose6D;
    pose6D.X = _x;
    pose6D.Y = _y;
    pose6D.Z = _z;
    
    // Create complete rotation matrix from two columns
    RotationUtils::CreateFromTwoColumns(
        _r11, _r21, _r31,
        _r12, _r22, _r32,
        pose6D.R
    );
    
    // Mark that we're using rotation matrix directly
    pose6D.hasR = true;
    
    // Solve inverse kinematics
    DOF6Kinematic::IKSolves_t ikSolves{};
    DOF6Kinematic::Joint6D_t lastJoint6D{};
    
    dof6Solver->SolveIK(pose6D, lastJoint6D, ikSolves);
    
    // Check for valid solutions
    bool valid[8];
    int validCnt = 0;

    for (int i = 0; i < 8; i++)
    {
        valid[i] = true;

        for (int j = 1; j <= 6; j++)
        {
            if (ikSolves.config[i].a[j - 1] > motorJ[j]->angleLimitMax ||
                ikSolves.config[i].a[j - 1] < motorJ[j]->angleLimitMin)
            {
                valid[i] = false;
                continue;
            }
        }

        if (valid[i]) validCnt++;
    }

    if (validCnt)
    {
        // Find the best configuration (closest to current position)
        float min = 1000;
        uint8_t indexConfig = 0, indexJoint = 0;
        for (int i = 0; i < 8; i++)
        {
            if (valid[i])
            {
                for (int j = 0; j < 6; j++)
                    lastJoint6D.a[j] = ikSolves.config[i].a[j];
                DOF6Kinematic::Joint6D_t tmp = currentJoints - lastJoint6D;
                float maxAngle = AbsMaxOf6(tmp, indexJoint);
                if (maxAngle < min)
                {
                    min = maxAngle;
                    indexConfig = i;
                }
            }
        }

        // Move to the best configuration
        return MoveJ(ikSolves.config[indexConfig].a[0], ikSolves.config[indexConfig].a[1],
                     ikSolves.config[indexConfig].a[2], ikSolves.config[indexConfig].a[3],
                     ikSolves.config[indexConfig].a[4], ikSolves.config[indexConfig].a[5]);
    }

    return false;
}

void DummyRobot::UpdateJointAngles()
{
    motorJ[ALL]->UpdateAngle();
}

void DummyRobot::UpdateJointVelocities()
{
    motorJ[ALL]->UpdateVelocity();
}

void DummyRobot::UpdateJointVelocitiesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJointsVelocities.a[i - 1] = motorJ[i]->velocity;
    }
}

void DummyRobot::UpdateJointCurrents()
{
    motorJ[ALL]->UpdateCurrent();
}


void DummyRobot::UpdateJointCurrentsCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJointsCurrents.a[i - 1] = motorJ[i]->current;
    }
}

void DummyRobot::UpdateJointAnglesCallback()
{
    for (int i = 1; i <= 6; i++)
    {
        currentJoints.a[i - 1] = motorJ[i]->angle + initPose.a[i - 1];

        if (motorJ[i]->state == CtrlStepMotor::FINISH)
            jointsStateFlag |= (1 << i);
        else
            jointsStateFlag &= ~(1 << i);
    }
}


void DummyRobot::SetJointSpeed(float _speed)
{
    if (_speed < 0)_speed = 0;
    else if (_speed > 100) _speed = 100;

    jointSpeed = _speed * jointSpeedRatio;
}


void DummyRobot::SetJointAcceleration(float _acc)
{
    if (_acc < 0)_acc = 0;
    else if (_acc > 100) _acc = 100;

    for (int i = 1; i <= 6; i++)
        motorJ[i]->SetAcceleration(_acc / 100 * DEFAULT_JOINT_ACCELERATION_BASES.a[i - 1]);
}


void DummyRobot::CalibrateHomeOffset()
{
    // Disable FixUpdate, but not disable motors
    isEnabled = false;
    motorJ[ALL]->SetEnable(true);

    // 1.Manually move joints to L-Pose [precisely]
    // ...
    motorJ[2]->SetCurrentLimit(0.5);
    motorJ[3]->SetCurrentLimit(0.5);
    osDelay(500);

    // 2.Apply Home-Offset the first time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);

    // 3.Go to Resting-Pose
    initPose = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    currentJoints = DOF6Kinematic::Joint6D_t(0, 0, 90, 0, 0, 0);
    Resting();
    osDelay(500);

    // 4.Apply Home-Offset the second time
    motorJ[ALL]->ApplyPositionAsHome();
    osDelay(500);
    motorJ[2]->SetCurrentLimit(1);
    motorJ[3]->SetCurrentLimit(1);
    osDelay(500);

    Reboot();
}


void DummyRobot::Homing()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(0, 0, 90, 0, 0, 0);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::Resting()
{
    float lastSpeed = jointSpeed;
    SetJointSpeed(10);

    MoveJ(REST_POSE.a[0], REST_POSE.a[1], REST_POSE.a[2],
          REST_POSE.a[3], REST_POSE.a[4], REST_POSE.a[5]);
    MoveJoints(targetJoints);
    while (IsMoving())
        osDelay(10);

    SetJointSpeed(lastSpeed);
}


void DummyRobot::SetEnable(bool _enable)
{
    motorJ[ALL]->SetEnable(_enable);
    isEnabled = _enable;
}

void DummyRobot::SetRGBEnable(bool _enable)
{
    isRGBEnabled = _enable;
}

bool DummyRobot::GetRGBEnabled()
{
    return isRGBEnabled;
}

void DummyRobot::SetRGBMode(uint32_t mode)
{
    rgbMode = mode;
}

uint32_t DummyRobot::GetRGBMode()
{
    return rgbMode;
}

void DummyRobot::UpdateJointPose6D()
{
    dof6Solver->SolveFK(currentJoints, currentPose6D);
    currentPose6D.X *= 1000; // m -> mm
    currentPose6D.Y *= 1000; // m -> mm
    currentPose6D.Z *= 1000; // m -> mm
}


bool DummyRobot::IsMoving()
{
    return jointsStateFlag != 0b1111110;
}


bool DummyRobot::IsEnabled()
{
    return isEnabled;
}


void DummyRobot::SetCommandMode(uint32_t _mode)
{
    if (_mode < COMMAND_TARGET_POINT_SEQUENTIAL ||
        _mode > COMMAND_MOTOR_TUNING)
        return;

    commandMode = static_cast<CommandMode>(_mode);

    switch (commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            jointSpeedRatio = 1;
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_LOW);
            break;
        case COMMAND_CONTINUES_TRAJECTORY:
            SetJointAcceleration(DEFAULT_JOINT_ACCELERATION_HIGH);
            jointSpeedRatio = 0.3;
            break;
        case COMMAND_MOTOR_TUNING:
            break;
    }
}


DummyHand::DummyHand(CAN_HandleTypeDef* _hcan, uint16_t _id) :
    nodeID(_id), hcan(_hcan)
{
    txHeader =
        {
            .StdId = 0,
            .ExtId = 0,
            .IDE = CAN_ID_STD,
            .RTR = CAN_RTR_DATA,
            .DLC = 8,
            .TransmitGlobalTime = DISABLE
        };
}

/**
 * @brief Save current parameters to non-volatile memory
 * @return Whether the command was sent successfully
 */
void DummyHand::Save()
{
    // Set CAN frame ID
    txHeader.StdId = nodeID;  // Use node ID (0x601)
    
    // Prepare data packet - Save parameters command
    canBuf[0] = 0x2B;        // Command byte - Write to one register
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0xA4;        // Register low byte - 0x00A4 indicates save parameters
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // High byte
    canBuf[5] = 0x01;        // Low byte - 0x01 means execute save operation
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    // Send CAN message
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void DummyHand::EnableCloseLoop()
{
  // Set CAN frame ID
    txHeader.StdId = nodeID;  // Use node ID (0x601)
    
    // Prepare data packet - Enter closed-loop control command
    canBuf[0] = 0x2B;        // Command byte - Write to one register
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0xA2;        // Register low byte - 0x00A2 indicates closed-loop control
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // High byte
    canBuf[5] = 0x01;        // Low byte - 0x01 means enter closed-loop state
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}



/**
 * @brief Set motor torque (by setting current value)
 * @param _current Current value in Amperes, typically in the range of 0-1A
 */
void DummyHand::SetTorque(float _current)
{
    // Limit current range to prevent motor damage
    if (_current > 2.0f) _current = 2.0f; 
    if (_current < 0.0f) _current = 0.0f;

    // Scale by 100 as required by the CAN communication protocol
    uint16_t scaled_current = static_cast<uint16_t>(_current * 100);
    
    // Set CAN frame ID
    txHeader.StdId = nodeID;  // Use node ID (0x601)
    
    // Prepare data packet
    canBuf[0] = 0x2B;        // Command byte - write to one register
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x20;        // Register low byte - 0x0020 indicates current setting
    canBuf[3] = 0x00;        // null
    
    // Convert 16-bit current value to 2 bytes
    canBuf[4] = (scaled_current >> 8) & 0xFF;  // High byte
    canBuf[5] = scaled_current & 0xFF;         // Low byte
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    // Send CAN message
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

/**
 * Set the absolute position of the gripper
 * @param _angle Absolute position angle value (degrees)
 */
void DummyHand::SetAngle(float _angle)
{
    // Check angle range, limit to reasonable range
    if (_angle > maxAngle) _angle = maxAngle;
    if (_angle < minAngle) _angle = minAngle;

    // Multiply by 100 to amplify, as required by protocol
    int32_t angle_value = (int32_t)(_angle * 100);
    
    // Set CAN frame ID
    txHeader.StdId = nodeID;  // Use node ID
    
    // Prepare CAN data
    canBuf[0] = 0x23;        // Command byte: write 2 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x23;        // Register low byte: absolute position register
    canBuf[3] = 0x00;        // null
    
    // Convert angle value to byte format (4 bytes, high byte first)
    canBuf[4] = (angle_value >> 24) & 0xFF;  // Highest byte
    canBuf[5] = (angle_value >> 16) & 0xFF;  // High byte
    canBuf[6] = (angle_value >> 8) & 0xFF;   // Low byte
    canBuf[7] = angle_value & 0xFF;         // Lowest byte

    // Send CAN message
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

/**
 * Set the relative position of the gripper
 * @param _angle Relative position angle value (degrees)
 */
void DummyHand::SetRelativePosition(float _angle)
{
    // Check angle range, apply appropriate limits
    if (_angle > maxAngle) _angle = maxAngle;
    if (_angle < minAngle) _angle = minAngle;

    // Multiply by 100 to amplify, as required by protocol
    int32_t angle_value = (int32_t)(_angle * 100);
    
    // Set CAN frame ID
    txHeader.StdId = nodeID;  // Use node ID
    
    // Prepare CAN data
    canBuf[0] = 0x23;        // Command byte: write 2 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x25;        // Register low byte: relative position register
    canBuf[3] = 0x00;        // null
    
    // Convert angle value to byte format (4 bytes, high byte first)
    canBuf[4] = (angle_value >> 24) & 0xFF;  // Highest byte
    canBuf[5] = (angle_value >> 16) & 0xFF;  // High byte
    canBuf[6] = (angle_value >> 8) & 0xFF;   // Low byte
    canBuf[7] = angle_value & 0xFF;         // Lowest byte

    // Send CAN message
    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}


void DummyHand::SetCurrentLimit(float _val)
{
    if (_val > 1)_val = 1;
    if (_val < 0)_val = 0;

    // Convert to current value (0-100) as per protocol
    uint16_t current = static_cast<uint16_t>(_val * 100);
    
    // Set up CAN frame according to protocol
    txHeader.StdId = nodeID;  // Standard ID: 0x601 (for node 1)
    
    // Prepare data according to protocol
    canBuf[0] = 0x2B;        // Command byte
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x61;        // Register low byte
    canBuf[3] = 0x00;        // null
    canBuf[4] = (current >> 8) & 0xFF;  // Current high byte
    canBuf[5] = current & 0xFF;         // Current low byte
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void DummyHand::SetMode(uint8_t _mode)
{
    if (_mode < 0) _mode = 0;
    if (_mode > 4) _mode = 4;

    // Set up CAN frame according to protocol
    txHeader.StdId = nodeID;  // Standard ID: 0x601 (for node 1)
    
    // Prepare data according to protocol
    canBuf[0] = 0x2B;        // Command byte
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x60;        // Register low byte
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // Mode high byte
    canBuf[5] = _mode & 0xFF; // Mode low byte
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
}

void DummyHand::UpdateAngle()
{
    txHeader.StdId = nodeID;  // Using node ID (0x601)
    
    // Prepare data packet - Read real-time position command
    canBuf[0] = 0x43;        // Command byte - Read 2 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x08;        // Register low byte - 0x0008 represents real-time position
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // null
    canBuf[5] = 0x00;        // null
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
    
}


uint8_t DummyHand::GetMode()
{

    txHeader.StdId = nodeID;  // Using node ID (0x601)
    
    // Prepare data packet - Read real-time position command
    canBuf[0] = 0x4B;        // Command byte - Read 1 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x60;        // Register low byte - Real-time mode register
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // null
    canBuf[5] = 0x00;        // null
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
    return mode;
}



// Add angle callback handler function
void DummyHand::UpdateAngleCallback(uint8_t* data)
{
    // Extract angle value from CAN data (4 bytes, int32_t format)
    int32_t scaled_angle = (static_cast<int32_t>(data[4]) << 24) |
                           (static_cast<int32_t>(data[5]) << 16) |
                           (static_cast<int32_t>(data[6]) << 8) |
                           static_cast<int32_t>(data[7]);
    
    // Convert scaled angle value back to actual angle (divide by 100)
    currentAngle = static_cast<float>(scaled_angle) / 100.0f;
}


float DummyHand::GetAngle()
{
    txHeader.StdId = nodeID;  // Using node ID (0x601)
    
    // Prepare data packet - Read real-time position command
    canBuf[0] = 0x4B;        // Command byte - Read 1 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x60;        // Register low byte - Real-time mode register
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // null
    canBuf[5] = 0x00;        // null
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
    return currentAngle;
}

float DummyHand::GetCurrentLimit()
{
    txHeader.StdId = nodeID;  // Using node ID (0x601)
    
    // Prepare data packet - Read real-time position command
    canBuf[0] = 0x4B;        // Command byte - Read 1 registers
    canBuf[1] = 0x00;        // Register high byte
    canBuf[2] = 0x61;        // Register low byte - Real-time mode register
    canBuf[3] = 0x00;        // null
    canBuf[4] = 0x00;        // null
    canBuf[5] = 0x00;        // null
    canBuf[6] = 0x00;        // null
    canBuf[7] = 0x00;        // null

    CanSendMessage(get_can_ctx(hcan), canBuf, &txHeader);
    return CurrentLimit;
}



uint32_t DummyRobot::CommandHandler::Push(const std::string &_cmd)
{
    osStatus_t status = osMessageQueuePut(commandFifo, _cmd.c_str(), 0U, 0U);
    if (status == osOK)
        return osMessageQueueGetSpace(commandFifo);

    return 0xFF; // failed
}


void DummyRobot::CommandHandler::EmergencyStop()
{
    context->MoveJ(context->currentJoints.a[0], context->currentJoints.a[1], context->currentJoints.a[2],
                   context->currentJoints.a[3], context->currentJoints.a[4], context->currentJoints.a[5]);
    context->MoveJoints(context->targetJoints);
    context->isEnabled = false;
    ClearFifo();
}


std::string DummyRobot::CommandHandler::Pop(uint32_t timeout)
{
    osStatus_t status = osMessageQueueGet(commandFifo, strBuffer, nullptr, timeout);

    return std::string{strBuffer};
}


uint32_t DummyRobot::CommandHandler::GetSpace()
{
    return osMessageQueueGetSpace(commandFifo);
}


uint32_t DummyRobot::CommandHandler::ParseCommand(const std::string &_cmd)
{
    uint8_t argNum;

    switch (context->commandMode)
    {
        case COMMAND_TARGET_POINT_SEQUENTIAL:
        case COMMAND_CONTINUES_TRAJECTORY:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                // Trigger a transmission immediately, in case IsMoving() returns false
                context->MoveJoints(context->targetJoints);

                while (context->IsMoving() && context->IsEnabled())
                    osDelay(5);
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }

            break;

        case COMMAND_TARGET_POINT_INTERRUPTABLE:
            if (_cmd[0] == '>' || _cmd[0] == '&')
            {
                float joints[6];
                float speed;

                if (_cmd[0] == '>')
                    argNum = sscanf(_cmd.c_str(), ">%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (_cmd[0] == '&')
                    argNum = sscanf(_cmd.c_str(), "&%f,%f,%f,%f,%f,%f,%f", joints, joints + 1, joints + 2,
                                    joints + 3, joints + 4, joints + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveJ(joints[0], joints[1], joints[2],
                                   joints[3], joints[4], joints[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            } else if (_cmd[0] == '@')
            {
                float pose[6];
                float speed;

                argNum = sscanf(_cmd.c_str(), "@%f,%f,%f,%f,%f,%f,%f", pose, pose + 1, pose + 2,
                                pose + 3, pose + 4, pose + 5, &speed);
                if (argNum == 6)
                {
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                } else if (argNum == 7)
                {
                    context->SetJointSpeed(speed);
                    context->MoveL(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                }
                Respond(*usbStreamOutputPtr, "ok");
                Respond(*uart4StreamOutputPtr, "ok");
            }
            break;

        case COMMAND_MOTOR_TUNING:
            break;
    }

    return osMessageQueueGetSpace(commandFifo);
}


void DummyRobot::CommandHandler::ClearFifo()
{
    osMessageQueueReset(commandFifo);
}


void DummyRobot::TuningHelper::SetTuningFlag(uint8_t _flag)
{
    tuningFlag = _flag;
}


void DummyRobot::TuningHelper::Tick(uint32_t _timeMillis)
{
    time += PI * 2 * frequency * (float) _timeMillis / 1000.0f;
    float delta = amplitude * sinf(time);

    for (int i = 1; i <= 6; i++)
        if (tuningFlag & (1 << (i - 1)))
            context->motorJ[i]->SetAngle(delta);
}


void DummyRobot::TuningHelper::SetFreqAndAmp(float _freq, float _amp)
{
    if (_freq > 5)_freq = 5;
    else if (_freq < 0.1) _freq = 0.1;
    if (_amp > 50)_amp = 50;
    else if (_amp < 1) _amp = 1;

    frequency = _freq;
    amplitude = _amp;
}


void DummyRobot::EEFPoseHelper::UpdatePose6D()
{
    context->UpdateJointPose6D();
    x = context->currentPose6D.X;
    y = context->currentPose6D.Y;
    z = context->currentPose6D.Z;
    a = context->currentPose6D.A;
    b = context->currentPose6D.B;
    c = context->currentPose6D.C;
}

