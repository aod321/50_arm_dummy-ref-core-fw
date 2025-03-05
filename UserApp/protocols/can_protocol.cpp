#include "common_inc.h"


// Used for response CAN message.
static CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0,
        .ExtId = 0,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE
    };

extern DummyRobot dummy;

void OnCanMessage(CAN_context* canCtx, CAN_RxHeaderTypeDef* rxHeader, uint8_t* data)
{
    // Common CAN message callback, uses ID 32~0x7FF.
    if (canCtx->handle->Instance == CAN1)
    {
        uint8_t id = rxHeader->StdId >> 7; // 4Bits ID & 7Bits Msg
        uint8_t cmd = rxHeader->StdId & 0x7F; // 4Bits ID & 7Bits Msg



        /*----------------------- ↓ Add Your CAN1 Packet Protocol Here ↓ ------------------------*/
        // Check if the response is from DummyHand
        // Assuming DummyHand node ID's lower 7 bits are 0x01 (lower 7 bits of 0x581)
        if (rxHeader->StdId == 0x581) {
            // Check command byte and register address to confirm it's an angle data response
            if (data[0] == 0x43 && data[1] == 0x00 && data[2] == 0x08) {
                // Call DummyHand's angle callback handler function
                dummy.hand->UpdateAngleCallback(data);
                return; // Processing complete, return
            }
            else if (data[0] == 0x4B && data[1] == 0x00 && data[2] == 0x60) {
                // Control mode response
                memcpy(&dummy.hand->mode, &data[5], sizeof(uint8_t)); // Extract mode from response
                return;
            }
            else if (data[0] == 0x4B && data[1] == 0x00 && data[2] == 0x61) {
                uint16_t current_value = (static_cast<uint16_t>(data[4]) << 8) | data[5];
                dummy.hand->CurrentLimit = static_cast<float>(current_value) / 100.0f;
                return;
            }
        }
        switch (cmd)
        {
            case 0x21:
                dummy.motorJ[id]->UpdateCurrentCallback(*(float*) (data), data[4]);
                break;
            case 0x22:
                dummy.motorJ[id]->UpdateVelocityCallback(*(float*) (data), data[4]);
                break;
            case 0x23:
                dummy.motorJ[id]->UpdateAngleCallback(*(float*) (data), data[4]);
                break;
            case 0x25:
                 memcpy(&dummy.motorJ[id]->temperature, data, sizeof(uint32_t));//(uint32_t) (data);
                break;
            default:
                break;
        }

        dummy.UpdateJointAnglesCallback();

    } else if (canCtx->handle->Instance == CAN2)
    {
        /*----------------------- ↓ Add Your CAN2 Packet Protocol Here ↓ ------------------------*/
    }
    /*----------------------- ↑ Add Your Packet Protocol Here ↑ ------------------------*/
}