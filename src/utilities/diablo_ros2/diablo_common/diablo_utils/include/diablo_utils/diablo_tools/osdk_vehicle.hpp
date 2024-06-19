/**
 * @file OSDK_Vehicle.hpp
 * @brief main interface to interact with robot
 */
#pragma once

#include "osdk_hal.hpp"
#include "osdk_vehicle.hpp"
#include "osdk_movement.hpp"
#include "osdk_telemetry.hpp"

namespace DIABLO{
namespace OSDK{

class Vehicle
{
public: 
    Vehicle(HAL* hal): hal(hal),
    movement_ctrl(NULL), 
    telemetry(NULL)
    {}

    ~Vehicle()
    {
        if(movement_ctrl) delete movement_ctrl;
        if(telemetry)     delete telemetry;
    }

    /**
     * @brief   Initialize SDK port to vehicle
     */
    uint8_t init(void);

public:
    HAL*               hal;
    
public:
    Movement_Ctrl*  movement_ctrl;
    Telemetry*      telemetry;

};

}
}
