#ifndef CAN_FRAMES_H
#define CAN_FRAMES_H

#include <stdint.h> // uint32_t
#include "can2040.h"

#define sendIDCluster   0x100
#define recvIDCluster   0x500
#define SteppermotorID 2
#define BoardDiagnosticsID 2
///////////////////////////////////////////////////////////////////////////////
typedef enum{
    DefaultAddress,
    HomeStepperMotor,
    MoveToPos,
    Velocity,
    Accelaration,
    MAXmotorRPM,
    ReadPosition,
    Emergencyshutdown,
    MAXAccelaration,
    MAXDecelaration,
    Leadscrewpitch,
    gearratio,
    MAXPositionlimit,
    motorErrorFlagReset,
    dxl_position,
    dxl_velocity,            
    dxl_accelaration,
    dxl_currentlimit
} CAN_IDs;

struct CAN_frametype{
            const uint8_t txAddress;
            const uint8_t rxAddress;
            const uint8_t sizeinBytes;
            uint32_t data;
        };
#define CAN_start_data 2

// CAN_frametype
//     stepper_frame = {set_stepper_pos, get_stepper_pos, 2, 0},
//     dxl0_frame = {set_dxl0_pos, get_dxl0_pos, 2, 0},
//     dxl1_frame = {set_dxl1_pos, get_dxl1_pos, 2, 0};

#endif