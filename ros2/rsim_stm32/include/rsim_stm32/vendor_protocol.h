/**
 * @author HuZheHua/WangYuan
 * @version v1.0.0
 * @attention Copyright@Robint
 * @brief
 * ROS ---Downlink---> MCU
 * ROS <---Uplink--- MCU
 *
 * ReleaseNote:
 * v1.0.0: 基于消杀协议进行的较大调整
 */

#ifndef __PROTOCOL_ROS_STM32_H
#define __PROTOCOL_ROS_STM32_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>


/*
  可以通过__linux__宏的定义与否来自动识别代码应用的平台
*/


/* 指定当前协议结构体的对齐方式 */
#pragma pack(push) 
#pragma pack(4)  //协议结构体指定为单字节对齐

/*  %c:%d,0x%X
    @:64,0x40
    #:35,0x23
    %:37,0x25
    !:33,0x21
 */
#define PACKET_HEAD1  0x40
#define PACKET_HEAD2  0x40
#define PACKET_TAIL1  0x23
#define PACKET_TAIL2  0x23

#define ROS_RX_EOL      "\n\n"

/* 机器人协议类型定义 */
typedef enum __attribute__((packed)) {
  UNDEFINE_ROBOT_PROTOCOL = 0,
  HOTEL_ROBOT_PROTOCOL,      //酒店机器人通信协议
  HOME_ROBOT_PROTOCOL,       //家庭机器人通信协议
  DISINFECT_ROBOT_PROTOCOL,  //消杀机器人通信协议（送餐机器人通信协议）
  WHEELCHAIR_ROBOT_PROTOCOL, //辅行机器人通信协议
  DELIVERY_ROBOT_PROTOCOL    //简易配送机器人通信协议
}RobotProtocolType_e;

/* 电机驱动器类型定义 */
typedef enum __attribute__((packed)) {
  UNDEFINE_MOTOR_DRIVER = 0,
  DS_MOTOR_DRIVER,      //和利时电机驱动器
  L2DB_MOTOR_DRIVER,    //时代方舟二合一驱动器
  LDS_MOTOR_DRIVER,     //八达威驱动器
  ROBINT_MOTOR_DRIVER   //洛必德自研驱动器
}MotorDriverType_e;

/* 电池类型定义 */
typedef enum __attribute__((packed)) {
  UNDEFINE_BATTERY = 0,
  BESTWAY_BATTERY, //百威BMS板的电池
}BatteryType_e;


/* 数据包解包状态 */
typedef enum __attribute__((packed)) {
	PACKET_OK = 0,
	PACKET_LENGTH_ERROR,
	PACKET_HEAD_OR_TAIL_ERROR,
	PACKET_CHECKSUM_ERROR,
}UnpackStatus_e;

/* 电池错误码 */
typedef enum __attribute__((packed)) {
  UBAT_ERR_NULL = 0,
  UBAT_ERR_CHARGER_OVRE_CURRENT,
  UBAT_ERR_DISCHARGE_OVRE_CURRENT,
  UBAT_ERR_SHORT_CIRCUIT,
  UBAT_ERR_CELL_OPEN_CIRCUIT,
  UBAT_ERR_TEMP_NTC_OPEN_CIRCUIT,
  UBAT_ERR_CELL_OVER_VOLTAGEM,
  UBAT_ERR_CELL_UNDER_VOLTAGE,
  UBAT_ERR_ALL_OVER_VOLTAGE,
  UBAT_ERR_ALL_UNDER_VOLTAGE,
  UBAT_ERR_CELL_TEMP_OVER_CHARGE_TEMP_UPPER_LIMIT,
  UBAT_ERR_CELL_TEMP_OVER_DISCHARGE_TEMP_UPPER_LIMIT,
  UBAT_ERR_CELL_TEMP_UNDER_CHARGE_TEMP_LOWER_LIMIT,
  UBAT_ERR_CELL_TEMP_UNDER_DISCHARGE_TEMP_lOWER_LIMIT,
  UBAT_ERR_CELL_TEMP_DIFF_OVER_CHARGE_TEMP_UPPER_LIMIT,
  UBAT_ERR_CELL_TEMP_DIFF_OVER_DISCHARGE_TEMP_UPPER_LIMIT,
}BatteryErrorCode_e;

typedef enum __attribute__((packed)) {
  NUM1 = 0,
  NUM2,
  NUM3,
  NUM4,
  NUM5,
  NUM6
}RangeSensorNumber_e;


/*--------------------------------common struct-------------------------------*/
typedef struct {
	uint8_t head1;
	uint8_t head2;
	uint16_t payload_length;
} PacketHead_t;

typedef struct {
	uint16_t checksum;
	uint8_t tail1;
	uint8_t tail2;
}PacketTail_t;

typedef struct {
  uint8_t uplink_freq;
  uint8_t reserved1;
  uint16_t wheel_space;
  uint16_t wheel_diameter;
  uint16_t reserved2;
}SystemConfig_t; //8bytes

typedef struct {
	bool mcu_reset;             // MCU复位,断总电重启
	bool odom_reset;            // MCU上机器人位置的全局变量清零
	bool hubmotor_reset;        // 给轮毂电机发reset命令
	bool hubmotor_disable;      // 给轮毂电机发disable命令
	bool hubmotor_enable;       // 给轮毂电机发 enable 命令
  bool imu_reset;             // IMU复位
  bool upgrade_flag;          // 升级标志位,用于进入升级模式
  bool emergency_brake;
  bool reserved2;
  bool reserved3;
}ChassisCommand_t; //10 bytes

/*---------------------------------uplink struct-------------------------------*/
typedef struct {
  RobotProtocolType_e protocol_type;  //机器人协议类型
  uint8_t reserved1;
  uint16_t reserved2;
  uint32_t software_version;          //主控板软件版本
  uint32_t hardware_version;          //主控板硬件版本
}SystemInfo_t; //12 bytes

typedef struct {
  uint8_t type;                //电池类型
  uint8_t error_code;          //电池状态信息
  uint8_t temperature;         //温度，单位：1℃，电池内所有温度传感器测得温度的最高值
  uint8_t reserved1;           //预留位1
  uint16_t voltage;            //电池电压，单位：0.01V
  int16_t current;             //电池电流, 单位：0.01A
  uint16_t remain_capacity;    //剩余容量，单位：0.1AH
  uint16_t total_capacity;     //总容量，单位：0.1AH
  uint16_t cell_voltage_max;   //电芯电压的最大值, 单位0.01V
  uint16_t cell_voltage_min;   //电芯电压的最小值，单位0.01V
  uint16_t cycles;             //充放电循环次数
  uint16_t reserved2;          //预留位2
}BatteryData_t; //20 bytes

/* IMU数据实际未用到 */
typedef struct {
  int16_t acc[3];
  int16_t gyro[3];
  int16_t euler[3];
  int16_t mag[3];
  float q[4];
}ImuData_t; //40bytes

typedef struct {
  int16_t ultrasonic[6];  //超声波数据，单位：mm
  int16_t tof[6];         //TOF模块数据，单位：mm
}RangeSensorData_t; //24bytes

typedef struct {
	int16_t wheel_speed[2];  //轮子速度，单位rpm
	float robot_speed[3];    //机器人的速度
	float robot_position[3]; //机器人当前位置信息
} MotionData_t; //28bytes

typedef struct {
	uint8_t estop_status;        //硬件急停开关状态：1急停按钮触发，0急停按钮释放
 	uint8_t soft_estop_status;   //软急停状态：1软急停触发，0软急停释放

	bool charge_voltage_detect;  //充电电压检测标志位
	bool charge_current_detect;  //充电电流检测标志位
  uint16_t charge_voltage;     //单位：0.01V //用于检测充电桩的电压
	uint16_t charge_current;     //单位：0.01A //用于检测充电桩输入的电流

	uint8_t mcu_temperature;     //单位：℃
  uint8_t reserved1;           // 碰撞信号
  uint16_t reserved2;
} MonitorData_t; //12bytes

typedef struct {
  MotorDriverType_e type; //指定当前使用的电机驱动器的类型
  uint8_t bus_voltage;    //总线电压
  uint16_t bus_current;   //总线电流
  uint8_t driver_temp;    //驱动器温度，单位：℃
  uint8_t reserved1;
  uint8_t wheel_temp[2];  //轮子温度
  uint8_t error_code[2];  //错误码
  uint8_t reserved3[2];
  uint16_t reserved4[2]; 
}MotorDriverData_t; //16bytes

typedef struct {
  uint16_t counter;                // 计数变量用于计算误包率
  uint16_t reserved;
  SystemInfo_t sys_info;           // 系统信息
  SystemConfig_t sys_config;       // 系统配置参数，跟随下发的系统设置更新 
  ChassisCommand_t cmd_callback;   // 底盘指令的回调，用于指示指令是否执行成功
  MonitorData_t monitor;           // 系统监控参数
  RangeSensorData_t range_sensor;  // 测距传感器数据
  MotionData_t motion;             // 底盘运动信息
  BatteryData_t battery;           // 电池数据
  MotorDriverData_t motor_driver;  // 驱动器上报信息
}_PacketUplinkPayload_t;

typedef union {
  _PacketUplinkPayload_t data;
  uint8_t raw_data[sizeof(_PacketUplinkPayload_t)];
}PacketUplinkPayload_t;

typedef struct {
  PacketHead_t head;
  PacketUplinkPayload_t payload;
  PacketTail_t tail;
}_UplinkPacket_t;

typedef union {
  _UplinkPacket_t data;
  uint8_t raw_data[sizeof(_UplinkPacket_t)];
}UplinkPacket_t;


/*---------------------------------downlink struct-------------------------------*/
typedef struct {
  float linear_x;
  float linear_y;
  float linear_z;
  float angular_x;
  float angular_y;
  float angular_z;
}RosTwist_t;

/*
 * # LED控制
 * ## b 亮度 [0-255], 不允许设置 10 ('\n')!!!
 * ## s 速度 [11,16959], 越小越快，初始化默认1000
 * ## c 颜色 [0x000000,0xFFFFFF], 初始化白色0xFFFFFF
 * ## m 模式 [0-55]
 * ### 0: 静态单色
 * ### 12: 彩虹全彩转圈 Rainbow Cycle
 * ### 3: 转圈 Color wipe
 * ### 44: 彗星尾巴 Comet
 * ### 30: BlueInWhite
 * ### 32: WholeColorCicle
 */
typedef struct {
	uint32_t color;
	uint8_t mode;
  uint8_t brightness;
	uint16_t speed;
} Ws2812Led_t;

typedef struct{
  uint16_t counter;          // 计数变量用于计算误包率
  uint8_t twist_enable;      // 用于标记下发速度是否有效
  uint8_t ws2812_update;     // 用于标记是否更新炫彩灯 
  SystemConfig_t sys_config; // 设置系统参数
  ChassisCommand_t cmd;      // 下发底盘特定控制指令
  RosTwist_t ros_twist;      // 下发速度信息
  Ws2812Led_t ws2812_led;    // 下发炫彩灯控制参数
  uint8_t reserved1[4];      // 预留位
  uint16_t reserved2[2];     // 预留位
}_PacketDownlinkPayload_t;

typedef union {
  _PacketDownlinkPayload_t data;
  uint8_t raw_data[sizeof(_PacketDownlinkPayload_t)];
}PacketDownlinkPayload_t;

typedef struct {
  PacketHead_t head;
  PacketDownlinkPayload_t payload;
  PacketTail_t tail;
}_DownlinkPacket_t;

typedef union {
  _DownlinkPacket_t data;
  uint8_t raw_data[sizeof(_DownlinkPacket_t)];
}DownlinkPacket_t;

extern DownlinkPacket_t g_downlink;
extern UplinkPacket_t g_uplink;

#ifdef __linux__
void downlinkPacketInit(DownlinkPacket_t *downlink);
void downlinkPacketPack(DownlinkPacket_t *packet);
UnpackStatus_e uplinkPacketUnpack(uint8_t *arr, uint16_t len, UplinkPacket_t *uplink);

uint16_t uint16Checksum(uint8_t *arr, uint32_t length);
const char* convertBatteryErrorToString(BatteryErrorCode_e err_enum);
#else
void uplinkPacketInit(UplinkPacket_t *uplink);
void uplinkPacketPack(UplinkPacket_t *packet);
UnpackStatus_e downlinkPacketUnpack(uint8_t *arr, uint16_t len, DownlinkPacket_t *packet);
#endif

char* convertVersionToString(uint32_t version_num);
const char *convertUnpackStatusToString(UnpackStatus_e status);
const char *convertBatteryErrorToString(BatteryErrorCode_e err_enum);

#pragma pack(pop)

#ifdef __cplusplus
 }
#endif

#endif /* __PROTOCOL_ROS_STM32_H */

