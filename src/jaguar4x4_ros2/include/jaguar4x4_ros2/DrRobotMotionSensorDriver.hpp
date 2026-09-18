/*!
 * DrRobotMotionSensorDriver.hpp
 * Migrated to C++11/ROS 2 standards (removed Boost)
 * Copyright (C) 2013-2014  Dr Robot Inc
 */

#ifndef DRROBOTMOTIONSENSORDRIVER_H_
#define DRROBOTMOTIONSENSORDRIVER_H_

#include <stdexcept>
#include <termios.h>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <fcntl.h>
#include <poll.h>

// ROS 2 / C++11 replacements for Boost
#include <thread>
#include <mutex>
#include <memory>
#include <atomic>
#include <string> // Added for debugCommMessage

#define COMM_LOST_TH 200
#define KNNOT2MS 0.5144444
#define FULLAD 4095

//! A namespace containing the DrRobot Motion/Sensor driver
namespace DrRobot_MotionSensorDriver
{
    typedef unsigned char BYTE;

    /*! This definition limits the motor number
     */
    const int MOTORSENSOR_NUM = 4;

    /*! This definition limits the motor driver board number
     */
    const int MOTORBOARD_NUM = 2;

    /*! This definition limits all the string variables length < 255
     * such as robotID, robotIP and serial port name
     */
    const int CHAR_BUF_LEN = 255;

    /*! This definition limits the communication receiving buffer length
     */
    const int MAXBUFLEN = 4096;

    /*! \enum CommMethod
     * Normally for standard robot, driver will use Network to communicate with robot
     * If you use RS232 module connect robot with your PC, you need set as Serial
     */
    enum CommMethod { Network, Serial };
    /*! \enum CommState
     * Driver communication status
     */
    enum CommState { Disconnected, Connected };

    /*! \enum robot type
     * specify the robot system on the robot
     */
    enum RobotType { Jaguar, Puma };
    /*! \enum CtrlMethod
     * specify control method of the motor control command
     */
    enum CtrlMethod { OpenLoop, Velocity, Position };

    /*! \struct  DrRobotMotionConfig
     * to configure the driver
     */
    struct DrRobotMotionConfig
    {
        char robotID[CHAR_BUF_LEN];      //!< robotID, you could specify your own name for your robot
        char robotIP[CHAR_BUF_LEN];      //!< robot main WiFi module IP address, you could get it by manual
        int portNum;                     //!< robot main WiFi module port number, default is power system on 10001 port, motion system on 10002
        CommMethod commMethod;           //!< communication method enum CommMethod
        char serialPortName[CHAR_BUF_LEN]; //!< serial port name if you use serial communication
        RobotType robotType;             //!< specify the control system on the robot, enum BoardType
    };

    /*! \struct  MotorSensorData
     * for motor sensor data
     */
    struct MotorSensorData
    {
        int motorSensorEncoderPos[MOTORSENSOR_NUM];      //!< encoder count reading
        int motorSensorEncoderVel[MOTORSENSOR_NUM];      //!< encoder velocity reading
        int motorSensorCurrent[MOTORSENSOR_NUM];         //!< motor current AD value reading,
        int motorSensorTemperature[MOTORSENSOR_NUM];     //!< motor temperature sensor reading
        int motorSensorPWM[MOTORSENSOR_NUM];             //!< motor driver board output PWM value,
        int motorSensorEncoderPosDiff[MOTORSENSOR_NUM];  //!< encoder count reading difference related with last reading
    };

    /*! \struct  MotorDriverBoardData
     * for motor driver board information
     */
    struct MotorBoardData
    {
        int status[MOTORBOARD_NUM];           //!< motor board status, read back from query "FF"
        int temp1[MOTORBOARD_NUM];            //!< motor board internal temperature 1
        int temp2[MOTORBOARD_NUM];            //!< motor board internal temperature 2
        int temp3[MOTORBOARD_NUM];            //!< motor board internal temperature 3

        double volMain[MOTORBOARD_NUM];       //!< motor board main power voltage, default is battery voltage
        double vol12V[MOTORBOARD_NUM];        //!< motor board 12V power voltage
        double vol5V[MOTORBOARD_NUM];         //!< motor board 5V power voltage

        int dinput[MOTORBOARD_NUM];           //!< digital input, not used now
        int doutput[MOTORBOARD_NUM];          //!< digital output, not used now
        int ack[MOTORBOARD_NUM];              //!< 0- right command received("+"), -1 wrong command("-")
    };

    /*! \struct  GPSInfoData
     * this structure will decoded $GPRSMC message from GPS
     */
    struct GPSSensorData
    {
        long timeStamp;          //!< GPS Message time stamp, format: hhmmss
        long dateStamp;          //!< GPS date stamp, format:ddmmyy

        int gpsStatus;           //!< GPS status, 0-fixed,1-differential, -1 -- invalid
        double latitude;         //!< GPS latitude, - ==south, + == north
        double longitude;        //!< GPS longitude, - ==west, + == east
        double vog;              //!< GPS velocity over ground   m/s
        double cog;              //!< GPS course over ground,,radian
    };

    /*! \struct  IMUSensorData
     * this structure will decoded IMU data
     */
    struct IMUSensorData
    {
        int seq;                 //!< IMU sensor package sequence number, 0~ 255
        double yaw;              //!< yaw estimate from robot, unit:radian
        double pitch;            //!< pitch estimate from robot, unit:radian, not used now
        double roll;             //!< roll estimate from robot, unit:radian, not used now

        int gyro_x;              //!< raw gyro x axis data
        int gyro_y;              //!< raw gyro y axis data
        int gyro_z;              //!< raw gyro z axis data

        int accel_x;             //!< raw accel x axis data
        int accel_y;             //!< raw accel y axis data
        int accel_z;             //!< raw accel z axis data

        int comp_x;              //!< raw magnetic sensor x axis data
        int comp_y;              //!< raw magnetic sensor y axis data
        int comp_z;              //!< raw magnetic sensor z axis data
    };

    /*! \class DrRobotMotionSensorDriver
     * This is the main class declare
     */
    class DrRobotMotionSensorDriver
    {
    public:
        /*! @brief
         * Constructor function
         */
        DrRobotMotionSensorDriver();

        /*! @brief
         * Destructor function
         */
        ~DrRobotMotionSensorDriver();

        /*! @brief
         * This function is used for detecting the communication status,
         */
        bool portOpen();

        /*! @brief
         * This function is used for closing the communication
         */
        void close();

        /*! @brief
         * If the driver is configured as using serial communication, this function could open serial port and starting communication
         */
        int openSerial(const char *serialPort, const long BAUD);

        /*! @brief
         * If the driver is configured as using network communication, this function could open UDP port to connect with robot
         */
        int openNetwork(const char *robotIP, const int portNum);

        /*! @brief
         * This function will use struct DrRobotMotionConfig to configure the driver
         */
        void setDrRobotMotionDriverConfig(DrRobotMotionConfig *driverConfig);

        /*! @brief
         * This function will return the configuration of the driver
         */
        void getDrRobotMotionDriverConfig(DrRobotMotionConfig *driverConfig);

        /*! @brief
         * This function is used for reading motor sensor back from motion controller
         */
        int readMotorSensorData(MotorSensorData *motorSensorData);

        /*! @brief
         * This function is used for reading IMU sensor back from robot
         */
        int readIMUSensorData(IMUSensorData *imuSensorData);

        /*! @brief
         * This function is used for reading GPS sensor back from robot
         */
        int readGPSSensorData(GPSSensorData *gpsSensorData);

        /*! @brief
         * This function is used for reading all the motor board information back from robot
         */
        int readMotorBoardData(MotorBoardData *motorBoardData);

        /*! @brief
         * This function is used for sending all the command to robot
         */
        int sendCommand(const char *msg, const int nLen);

    private:
        char _recBuf[MAXBUFLEN];
        char _dataBuf[MAXBUFLEN];
        int _nMsgLen;
        int _sockfd;
        int _serialfd;
        struct sockaddr_in _addr;
        socklen_t _addr_len;
        char _sAddr[INET6_ADDRSTRLEN];
        int _numbytes;
        struct timeval _tv;
        fd_set _readfds;
        int _comCnt;
        
        // C++11 replacements
        std::mutex _mutex_Data_Buf;             // Replaced pthread_mutex_t
        std::thread _commThread;                // Replaced boost::thread
        std::atomic<bool> _stopComm;            // Replaced bool _stopComm for thread safety
        
        DrRobotMotionConfig *_robotConfig;
        
        //private functions here
        void debug_ouput(const char *errorstr);
        int vali_ip(const char *ip_str);
        int sendAck();
        void commWorkingThread();
        void DealWithPacket(const char *lpComData, const int nLen);
        void handleComData(const char *data, const int nLen);
        void processIMUMessage(char *pData, const int nLen);
        void processGPSMessage(char *pData, const int nLen);
        void processMotorMessage(char *pData, const int nLen);
        double trans2Degree(double angle);
        double trans2Temperature(double adValue);
        
        CommState _eCommState;
        void debugCommMessage(std::string msg);

        //sensor data here
        struct GPSSensorData _gpsSensorData;
        struct IMUSensorData _imuSensorData;
        struct MotorSensorData _motorSensorData;
        struct MotorBoardData _motorBoardData;
        int longhem;
        int lathem;
        static double resTable[25];
        static double tempTable[25];
    };

} // namespace DrRobot_MotionSensorDriver

#endif /* DRROBOTMOTIONSENSORDRIVER_H_ */
