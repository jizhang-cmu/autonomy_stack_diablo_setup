/**
 * @file OSDK_HAL.hpp
 * @brief SDK HAL Library, handle all communication issue
 */
#pragma once

#include <list>
#include <mutex>
#include <thread>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <pthread.h>
#include <functional>
#include <condition_variable>

#include "onboard_sdk_uart_protocol.h"
#include "diablo_utils/VulcanSerial/SerialPort.hpp"


namespace DIABLO{
namespace OSDK{

/**
 * @brief Abstract HAL class
 */
class HAL
{
public:
    /** 
     * @brief get transmission packed sequence number
     * @note  NON-API FUNCTION
     */
    uint32_t serial_getSeq(void)
    {
        return serial_seq;
    }

    /** 
     * @brief get data pointer to received pack 
     * @note  NON-API FUNCTION
     */
    void* getRXData(void)
    {
        return rx_data;
    }

    /** 
     * @brief get cmd set to received pack 
     * @note  NON-API FUNCTION
     */
    uint8_t getRXCmdSet(void)
    {
        return *((uint8_t*)rx_data - 2);
    }

    /** 
     * @brief get cmd id to received pack 
     * @note  NON-API FUNCTION
     */
    uint8_t getRXCmdID(void)
    {
        return *((uint8_t*)rx_data - 1);
    }

    /** 
     * @brief get ACK value
     * @note  NON-API FUNCTION
     */
    uint16_t getACK(void)
    {
        return *((uint16_t*)rx_data);
    }

    /**
     * @brief get baudrate
     * @note NON-API FUNCTION
     */
    uint32_t getSerialBr(void)
    {
        return serial_br;
    }

    /**
     * @brief format serial data
     * @note NON-API FUNCTION
     */
    void serialPackData(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /** 
     * @brief process serial transmission of generic SDK message in non-blocking mode
     * @note  NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     */
    virtual uint8_t serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len) = 0;
    
    /** 
     * @brief process serial transmission of generic SDK message, and wait for ack packet
     * @note  NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     *         2: ack packet wait timeout \n
     */
    uint8_t serialSend_ack(const OSDK_Uart_Header_t& header, uint16_t& ack,
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /**
     * @brief       serial wait receive data 
     * @param[in]   cmd_set cmd set value of packet to wait
     * @param[in]   cmd_id  cmd id value of packet to wait
     * @note        NON-API FUNCTION
     * @return      pointer to data packet, NULL if no packet received in 100ms
     */
    void* serialWaitRXDataS(std::unique_lock<std::mutex>& lock, 
        const uint8_t cmd_set, const uint8_t cmd_id);

    /**
     * @brief       get SDK time stamp
     * @return      a double-precision number indicates seconds elasped from the start of SDK 
     */
    double getTimeStamp(void)
    {
        std::chrono::duration<double> sec = std::chrono::system_clock::now() - start_tp;
        return sec.count();
    }

protected:
    /** 
     * @brief serial receive thread function
     * @note  NON-API FUNCTION
     */
    virtual void RXMonitorProcess(void) = 0;

    /**
     * @brief check rx type is data or ack
     * @note  NON-API FUNCTION
     */
    bool verifyRXType(const uint8_t cmd_set, const uint8_t cmd_id)
    {
        //printf("%u\t%u\n%u\t%u\n",cmd_id,cmd_set,this->getRXCmdID(),this->getRXCmdSet());
        return cmd_set == this->getRXCmdSet() && cmd_id == this->getRXCmdID();
    }

public:
    std::mutex                     serial_rx_mtx;
    std::mutex                     serial_tx_mtx;
    std::mutex                  serial_prerx_mtx;

protected:
    HAL():
        serial_tx_thd(NULL), serial_rx_thd(NULL),
        start_tp(std::chrono::system_clock::now()), VRC_Data_packet_num(0), VRC_REQ_packet_num(0)
        {}

    ~HAL()
    {
        if(serial_tx_thd)
        {
            pthread_cancel(serial_tx_thd->native_handle());
        }
        if(serial_rx_thd)
        {
            pthread_cancel(serial_rx_thd->native_handle());

        }
    }

    uint32_t                            serial_seq;     // number of packets sent by serial port
    uint32_t                             serial_br;     // baud
    uint8_t                      serial_txbuf[256];
    uint8_t                      serial_rxbuf[256];
    void*                                  rx_data;
    uint8_t                   serial_prerxbuf[256];
    uint8_t                               prebytes;
    bool                             first_rx_flag;

protected:
//transmission handling
    bool                            serial_tx_idle;
    std::condition_variable         serial_tx_cond;
    double                      serial_tx_duration;
    std::thread*                     serial_tx_thd;

//receive handling
    std::thread*                   serial_rx_thd;
    
    std::condition_variable   serial_rx_ack_cond;
    std::condition_variable  serial_rx_data_cond;

    /** 
     * @brief serial transmission multiplexing thread function
     * @note  NON-API FUNCTION
     */
    void TXMonitorProcess(void);

protected:
    std::chrono::system_clock::time_point start_tp;

protected:
    uint16_t VRC_Data_packet_num;
    uint16_t VRC_REQ_packet_num;
};


/**
 * @brief HAL class for Pi
 */
class HAL_Pi: public HAL
{
public:
    HAL_Pi(){}

    /**
     * @brief initialize serial port for pi
     * @param dev name of the port
     * @param baud serial baudrate
     * @return 0 initialization ok \n
     *         1 start wiringPi error \n
     *         2 start heartbeat error \n
     *         3 initialization port fail \n
     */
    uint8_t init(const std::string dev = "/dev/ttyAMA0", const int baud = 460800);

    ~HAL_Pi()
    {
        mySerial.Close();
    }

    /**
     * @brief process serial transmission of generic SDK message in non-blocking mode
     * @note NON-API FUNCTION
     * @return 0: successfully send \n
     *         1: multiplexer wait timeout(most possible reason is bandwidth full) \n
     */
    uint8_t serialSend(const OSDK_Uart_Header_t& header, 
        const uint8_t cmd_set, const uint8_t cmd_id, 
        const void* data, const uint32_t data_len);

    /** 
     * @brief serial receive thread function
     * @note  NON-API FUNCTION
     */
    void RXMonitorProcess(void);

private:
    VulcanSerial::SerialPort      mySerial;

};

}
}
