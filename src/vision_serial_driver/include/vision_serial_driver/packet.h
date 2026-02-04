#ifndef SERIAL_PACKET_H
#define SERIAL_PACKET_H

#include <cstdint>
#include <algorithm>

#pragma pack(2)

#define visionMsg inf_visionMsg
#define robotMsg inf_robotMsg
#define VelMsg Vel_visionMsg

struct inf_visionMsg
{
    uint16_t head;
    uint8_t fire;     // 开火标志
    uint8_t tracking; // 跟踪标志
    float aimYaw;     // 目标Yaw
    float aimPitch;   // 目标Pitch
    float vx;
    float vy;
    float wz;
};

struct Vel_visionMsg
{
    uint16_t head;
    uint8_t fire;     // 开火标志
    uint8_t tracking; // 跟踪标志
    uint8_t VelControl; // 速度控制标志
    float aimYaw;     // 目标Yaw
    float aimPitch;   // 目标Pitch
    float aimPVel;      // 目标Pitch速度
    float aimYVel;      // 目标Yaw速度
}

struct inf_robotMsg
{
    uint16_t head;
    uint8_t mode;
    uint8_t foeColor;  // 敌方颜色 0-blue 1-red
    float robotYaw;    // 自身Yaw
    float robotPitch;  // 自身Pitch
    float muzzleSpeed; // 弹速
    uint8_t data;   //map state
};

union visionArray
{
    struct visionMsg msg;
    uint8_t array[sizeof(struct visionMsg)];
};

union VelArray
{
    struct VelMsg msg;
    uint8_t array[sizeof(struct VelMsg)];
};

union robotArray
{
    struct robotMsg msg;
    uint8_t array[sizeof(struct robotMsg)];
};

#pragma pack()

#endif // SERIAL_PACKET_H
