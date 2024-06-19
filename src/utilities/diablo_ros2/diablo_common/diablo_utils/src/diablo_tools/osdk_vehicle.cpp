#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"

using namespace DIABLO::OSDK;

/**
 * @brief   Initialize SDK port to vehicle
 */
uint8_t Vehicle::init(void)
{

    movement_ctrl = new Movement_Ctrl(this);
    if(!movement_ctrl)
    {
        std::cerr<<"Failed to allocate memory for Movement_Ctrl!\n"<<std::endl;
        return 1;
    }


    telemetry = new Telemetry(this);
    if(!telemetry)
    {
        std::cerr<<"Failed to allocate memory for Telemetry!\n"<<std::endl;
        return 1;
    }

    return 0;
}
