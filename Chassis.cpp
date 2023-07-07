#include "Chassis.hpp"
#include "Sensor.hpp"
#include "Conditions.hpp"
extern cus::M2006_Moter_Group Chassis_MoterGroup; //底盘控制类
extern cus::Chassis_SensorsClass Chassis_Sensors; //底盘传感器类
extern __IO int num_pass;

int carVx = 0.0, carVy = 0.0, carVz = 0.0;

cus::Chassis_Class Chassis; //底盘实例

namespace cus
{
    //默认构造函数
    Chassis_Class::Chassis_Class()
    {

        //初始化电机1(左上角)电流控制PID类
        MoterCurrent_PIDs[0] = float_PID(CHASSIS_MOTER1_KP,
                                         CHASSIS_MOTER1_KI,
                                         CHASSIS_MOTER1_KD,
                                         M2006_MOTER_CURRENT_MAX,
                                         M2006_MOTER_CURRENT_MIN);
        //初始化电机2(右上角)电流控制PID类
        MoterCurrent_PIDs[1] = float_PID(CHASSIS_MOTER2_KP,
                                         CHASSIS_MOTER2_KI,
                                         CHASSIS_MOTER2_KD,
                                         M2006_MOTER_CURRENT_MAX,
                                         M2006_MOTER_CURRENT_MIN);
        //初始化电机3(右下角)电流控制PID类
        MoterCurrent_PIDs[2] = float_PID(CHASSIS_MOTER3_KP,
                                         CHASSIS_MOTER3_KI,
                                         CHASSIS_MOTER3_KD,
                                         M2006_MOTER_CURRENT_MAX,
                                         M2006_MOTER_CURRENT_MIN);
        //初始化电机4(左下角)电流控制PID类
        MoterCurrent_PIDs[3] = float_PID(CHASSIS_MOTER4_KP,
                                         CHASSIS_MOTER4_KI,
                                         CHASSIS_MOTER4_KD,
                                         M2006_MOTER_CURRENT_MAX,
                                         M2006_MOTER_CURRENT_MIN);

        GyroPIDs = float_PID(GYRO_KP,GYRO_KI,GYRO_KD,10,-10);
        
        //初始化方向
        pre_Direction = DIRECTION_UNKNOWN;
        this->MoterGroup = &Chassis_MoterGroup;
    }

    //初始化函数,如果没有正确初始化会进入死循环直到初始化成功
    void Chassis_Class::init_MoterInfo_Receive()
    {
        MoterGroup->init_MoterInfo_Receive();
    }

    /**
     * 内嵌PID的电机匀速前进（需要均匀控制周期）
     * @param speed 前进速度 r/m 圈/分
     * @return 无
     */
    void Chassis_Class::send_Speed_Config(int16_t speed)
    {

        //经由独立PID计算的电流环控制输出
        MoterGroup->send_MoterCurrent(1, (int16_t)MoterCurrent_PIDs[0].calculate(speed, MoterGroup->getMoterRotor_Speed(1)));
        MoterGroup->send_MoterCurrent(2, (int16_t)MoterCurrent_PIDs[1].calculate(-speed, MoterGroup->getMoterRotor_Speed(2)));
        MoterGroup->send_MoterCurrent(3, (int16_t)MoterCurrent_PIDs[2].calculate(-speed, MoterGroup->getMoterRotor_Speed(3)));
        MoterGroup->send_MoterCurrent(4, (int16_t)MoterCurrent_PIDs[3].calculate(speed, MoterGroup->getMoterRotor_Speed(4)));
    }

    /**
     * 内嵌PID的电机差速前进，用于转向（需要均匀控制周期）
     * @param leftSpeed 左侧前进速度 r/m 圈/分
     * @param rightSpeed 右侧前进速度 r/m 圈/分
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::send_Speed_Config_Till(int16_t leftSpeed,
                                               int16_t rightSpeed,
                                               bool (*lpStop_Func)(void *lpParam),
                                               void *lpParam, bool BarkeEnd)
    {

        while (lpStop_Func(lpParam) != true)
        {
            send_Speed_Config(leftSpeed, rightSpeed);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        if (BarkeEnd)
            barke();
        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID的电机配速移动（需要均匀控制周期）
     * @param UpLeft_Speed 左前电机前进速度 r/m 圈/分
     * @param UpRightSpeed 右前电机前进速度 r/m 圈/分
     * @param DownLeft_Speed 左后电机侧前进速度 r/m 圈/分
     * @param DownRightSpeed 右后电机前进速度 r/m 圈/分
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @param num 经过直线
     * @return 无
     */
    void Chassis_Class::send_Speed_Config_Till_Num(int16_t leftSpeed,
                                                   int16_t rightSpeed,
                                                   bool (*lpStop_Func)(void *lpParam),
                                                   void *lpParam,
                                                   int num)
    {
        int temp = 0;
        // float Yaw_temp = 5, Yaw_get;
        // static float Yaw_begin;
        // Yaw_begin = IMU.get_Yaw();
        while (temp < num)
        {
            if (lpStop_Func(lpParam) != false)
            {
                //检测到一点补偿一段距离直到达到要求经过白线的数量
                send_Speed_Config(leftSpeed, rightSpeed);
                HAL_Delay(300);
                temp++;
            }
            send_Speed_Config(leftSpeed, rightSpeed);
            HAL_Delay(PID_DUTY_CYCLE);
        }

        //刹车
        barke();
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_Left(
        bool (*isYaw_Passed_Left)(void *lpParam),
        void *lpParam)
    {

        while (isYaw_Passed_Left(lpParam) != true)
        {
            send_Speed_Config(-80 * 36, 80 * 36, 0 * 36, 80 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        barke(); // 0.25s停止
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_Right(
        bool (*isYaw_Passed_Right)(void *lpParam),
        void *lpParam)
    {
        while (isYaw_Passed_Right(lpParam) != true)
        {
            send_Speed_Config(80 * 36, -80 * 36, 80 * 36, -40 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        barke(); // 0.25s停止
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_Till(int16_t UpLeft_Speed,
                                               int16_t UpRightSpeed,
                                               int16_t DownLeft_Speed,
                                               int16_t DownRightSpeed,
                                               bool (*lpStop_Func)(void *lpParam),
                                               void *lpParam,
                                               bool BarkeEnd)
    {

        while (lpStop_Func(lpParam) != true)
        {
            send_Speed_Config(UpLeft_Speed, UpRightSpeed, DownLeft_Speed, DownRightSpeed);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        if (BarkeEnd)
            barke();
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_Right_90(
        bool (*isYaw_Passed_Right)(void *lpParam),
        bool (*lpStop_Func)(void *lpParam))
    {
        while (isYaw_Passed_Right((void *)25) != true)
        {
            send_Speed_Config(80 * 36, -80 * 36, 80 * 36, -40 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        while (lpStop_Func((void *)0b10111) != true)
        {
            send_Speed_Config(60 * 36, 60 * 36, 60 * 36, 60 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        while (isYaw_Passed_Right((void *)50) != true)
        {
            send_Speed_Config(80 * 36, -80 * 36, 80 * 36, -40 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        barke(); // 0.25s停止
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_Left_90(
        bool (*isYaw_Passed_Left)(void *lpParam),
        bool (*lpStop_Func)(void *lpParam))
    {
        while (isYaw_Passed_Left((void *)-25) != true)
        {
            send_Speed_Config(-80 * 36, 80 * 36, -40 * 36, 80 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        while (lpStop_Func((void *)0b01111) != true)
        {
            send_Speed_Config(60 * 36, 60 * 36, 60 * 36, 60 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        while (isYaw_Passed_Left((void *)-50) != true)
        {
            send_Speed_Config(-80 * 36, 80 * 36, -40 * 36, 80 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        barke(); // 0.25s停止
        //清除误差积分
        clear_err_integral();
    }
    void Chassis_Class::send_Speed_Config_inLeft(
        bool (*isYaw_Passed_Left)(void *lpParam))
    {
        //后调位置调整角度
        for (int i = 0; i < 60; i++)
        {
            Chassis.send_Speed_Config(-100 * 36, -100 * 36, -100 * 36, -100 * 36);
        }
        while (isYaw_Passed_Left((void *)85) != true)
        {
            send_Speed_Config(-120 * 36, 120 * 36, 0 * 36, 120 * 36);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //刹车
        barke(); // 0.25s停止
        //清除误差积分
        clear_err_integral();
    }
    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机匀速前进,带有路径修正
     * @param speed 前进速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::forward_Till_withFix(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {

        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }

            while (lpStop_Func(lpParam) != true)
            {
                switch (Chassis_Sensors.getFrontSensorStatus())
                {

                case 0b11001: //轻微左偏
                case 0b11101:
                case 0b11100:
                    pre_Direction = DIRECTION_RIGHT; //记录位置
                    send_Speed_Config(speed * 1.5, speed * 0.5);
                    break;
                case 0b11110:                        //极端左偏
                    pre_Direction = DIRECTION_RIGHT; //记录位置
                    send_Speed_Config(200 * 36, -200 * 36, 200 * 36, -200 * 36);
                    break;
                case 0b10011: //轻微右偏
                case 0b10111:
                case 0b00111:
                    pre_Direction = DIRECTION_LEFT; //记录位置
                    send_Speed_Config(speed * 0.5, speed * 1.5);
                    break;
                case 0b01111:                       //极端右偏
                    pre_Direction = DIRECTION_LEFT; //记录位置
                    send_Speed_Config(-200 * 36, 200 * 36, -200 * 36, 200 * 36);
                    break;
                case 0b11111: //丢失路线
                    if (pre_Direction == DIRECTION_LEFT)
                    {
                        send_Speed_Config(-200 * 36, 200 * 36, -200 * 36, 200 * 36);
                        break;
                    }
                    if (pre_Direction == DIRECTION_RIGHT)
                    {
                        send_Speed_Config(200 * 36, -200 * 36, 200 * 36, -200 * 36);
                        break;
                    }

                default:
                    send_Speed_Config(speed); //发送配速
                    break;
                }
                HAL_Delay(PID_DUTY_CYCLE);
            }
            //刹车
            if (BarkeEnd)
                barke();
                //num_pass++;
        }
        //清除误差积分
        clear_err_integral();
    }

    

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机匀速前进,不带有路径修正
     * @param speed 前进速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::forward_Till(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {

        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }

            while (lpStop_Func(lpParam) != true)
            {
                send_Speed_Config(speed); //发送配速
                HAL_Delay(PID_DUTY_CYCLE);
            }
            //刹车
            if (BarkeEnd)
                barke();
        }
        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机匀速后退,不带有路径修正
     * @param speed 前进速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::backward_Till(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {

        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(-speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            while (lpStop_Func(lpParam) != true)
            {
                send_Speed_Config(-speed); //发送配速
                HAL_Delay(PID_DUTY_CYCLE);
            }
        }
        //刹车
        if (BarkeEnd)
            barke();

        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机匀速后退,带有路径修正
     * @param speed 前进速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::backward_Till_withFix(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {

        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(-speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            while (lpStop_Func(lpParam) != true)
            {
                switch (Chassis_Sensors.getBackSensorStatus())
                {

                case 0b11001: //左偏
                case 0b11101:
                case 0b11100:
                    pre_Direction = DIRECTION_RIGHT; //记录位置
                    send_Speed_Config(-speed * 0.5, -speed * 1.5);
                    break;
                case 0b11110:                        //极端左偏
                    pre_Direction = DIRECTION_RIGHT; //记录位置
                    send_Speed_Config(-200 * 36, 200 * 36, -200 * 36, 200 * 36);
                    break;
                case 0b10011: //右偏
                case 0b10111:
                case 0b00111:
                    pre_Direction = DIRECTION_LEFT; //记录位置
                    send_Speed_Config(-speed * 1.5, -speed * 0.5);
                    break;
                case 0b01111:                       //极端右偏
                    pre_Direction = DIRECTION_LEFT; //记录位置
                    send_Speed_Config(200 * 36, -200 * 36, 200 * 36, -200 * 36);
                    break;
                case 0b11111: //丢失路线
                    if (pre_Direction == DIRECTION_LEFT)
                    {
                        send_Speed_Config(200 * 36, -200 * 36, 200 * 36, -200 * 36);
                        break;
                    }
                    if (pre_Direction == DIRECTION_RIGHT)
                    {
                        send_Speed_Config(-200 * 36, 200 * 36, -200 * 36, 200 * 36);
                        break;
                    }

                default:
                    send_Speed_Config(-speed); //发送配速
                    break;
                }
                HAL_Delay(PID_DUTY_CYCLE);
            }
            //刹车
            if (BarkeEnd)
                barke();
                 //num_pass++;
        }

        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机左转向
     * @param speed 转向速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::turnLeft_Till(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {
        //方向记录
        pre_Direction = DIRECTION_LEFT;
        if (speed > 0)
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(-speed, speed, -speed, speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }

            //循环动作
            while (lpStop_Func(lpParam) != true)
            {
                send_Speed_Config(-speed, speed, -speed, speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            //刹车
            if (BarkeEnd)
                barke();
        }
        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机右转向
     * @param speed 转向速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param preMotionTimes 初动补偿时间，默认为DEF_FIRST_MOTION_COMPENSATION
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::turnRight_Till(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, int preMotionTimes, bool BarkeEnd)
    {
        pre_Direction = DIRECTION_RIGHT;
        if (speed > 0)
        {
            //初动补偿
            for (int count = 0; count < preMotionTimes; count++)
            {
                send_Speed_Config(speed, -speed, speed, -speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }

            while (lpStop_Func(lpParam) != true)
            {
                send_Speed_Config(speed, -speed, speed, -speed);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            //刹车
            if (BarkeEnd)
                barke();
        }
        //清除误差积分
        clear_err_integral();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机定轴左转向
     * @param speed 转向速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::fixed_axis_Left_rotation_Till(bool (*lpStop_Func)(void *lpParam), void *lpParam, bool BarkeEnd)
    {
        pre_Direction = DIRECTION_LEFT;
        send_Speed_Config_Till(-120 * 36, 120 * 36, -40 * 36, 120 * 36, lpStop_Func, lpParam, BarkeEnd);
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机定轴右转向
     * @param speed 转向速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::fixed_axis_Right_rotation_Till(bool (*lpStop_Func)(void *lpParam), void *lpParam, bool BarkeEnd)
    {
        pre_Direction = DIRECTION_RIGHT;
        send_Speed_Config_Till(120 * 36, -120 * 36, 120 * 36, -40 * 36, lpStop_Func, lpParam, BarkeEnd);
    }

    /**
     * 内嵌PID的电机差速前进，用于转向（需要均匀控制周期）
     * @param leftSpeed 左侧前进速度
     * @param rightSpeed 右侧前进速度
     * @return 无
     */
    void Chassis_Class::send_Speed_Config(int16_t leftSpeed, int16_t rightSpeed)
    {

        //经由独立PID计算的电流环控制输出
        MoterGroup->send_MoterCurrent(1, (int16_t)MoterCurrent_PIDs[0].calculate(leftSpeed, MoterGroup->getMoterRotor_Speed(1)));
        MoterGroup->send_MoterCurrent(2, (int16_t)MoterCurrent_PIDs[1].calculate(-rightSpeed, MoterGroup->getMoterRotor_Speed(2)));
        MoterGroup->send_MoterCurrent(3, (int16_t)MoterCurrent_PIDs[2].calculate(-rightSpeed, MoterGroup->getMoterRotor_Speed(3)));
        MoterGroup->send_MoterCurrent(4, (int16_t)MoterCurrent_PIDs[3].calculate(leftSpeed, MoterGroup->getMoterRotor_Speed(4)));
    }

    /**
     * 内嵌PID的电机配速移动，用于转向（需要均匀控制周期）
     * @param UpLeft_Speed 左前电机前进速度 r/m 圈/分
     * @param UpRightSpeed 右前电机前进速度 r/m 圈/分
     * @param DownLeft_Speed 左后电机前进速度 r/m 圈/分
     * @param DownRightSpeed 右后前进速度 r/m 圈/分
     * @return 无
     */
    void Chassis_Class::send_Speed_Config(int16_t UpLeft_Speed, int16_t UpRightSpeed, int16_t DownLeft_Speed, int16_t DownRightSpeed)
    {
        //经由独立PID计算的电流环控制输出
        MoterGroup->send_MoterCurrent(1, (int16_t)MoterCurrent_PIDs[0].calculate(UpLeft_Speed, MoterGroup->getMoterRotor_Speed(1)));
        MoterGroup->send_MoterCurrent(2, (int16_t)MoterCurrent_PIDs[1].calculate(-UpRightSpeed, MoterGroup->getMoterRotor_Speed(2)));
        MoterGroup->send_MoterCurrent(3, (int16_t)MoterCurrent_PIDs[2].calculate(-DownRightSpeed, MoterGroup->getMoterRotor_Speed(3)));
        MoterGroup->send_MoterCurrent(4, (int16_t)MoterCurrent_PIDs[3].calculate(DownLeft_Speed, MoterGroup->getMoterRotor_Speed(4)));
    }

    /**
     * 获取整车平均速度
     * @param 无
     * @return 底盘电机平均速度
     */
    int16_t Chassis_Class::get_AverageSpeed()
    {

        int64_t speed = 0;

        speed += MoterGroup->getMoterRotor_Speed(1);
        speed -= MoterGroup->getMoterRotor_Speed(2);
        speed -= MoterGroup->getMoterRotor_Speed(3);
        speed += MoterGroup->getMoterRotor_Speed(4);

        return speed / 4;
    }

    /**
     * 清除电机PID误差累积量
     * @param 无
     * @return 无
     */
    void Chassis_Class::clear_err_integral()
    {
        for (int count = 0; count < 4; count++)
            MoterCurrent_PIDs[count].clear_err_integral();
    }

    //刹车
    void Chassis_Class::barke()
    {
        for (int count = 0; count < 50; count++)
        {
            send_Speed_Config(0);
            HAL_Delay(PID_DUTY_CYCLE);
        }
        //清除误差积分
        clear_err_integral();
    }

    /**
     * 麦轮合成速度
     * @param vx x轴运动速度 横向 正数向右平移 负数向左平移
     * @param vy y轴运动速度 纵向 正数向前平移 负数向后平移
     * @param vz yaw轴运动速度 垂直Z轴旋转 正数为顺 负数为逆
     * @note 麦轮顺序，从左上角逆时针数起 0-1-3-2
     */
    void Chassis_Class::set_MW_speed(float vx, float vy, float vz)
    {
        float temp1 = 0, temp2 = 0, temp3 = 0, temp4 = 0;

        carVx = vx;
        carVy = vy;
        carVz = vz;

        temp1 = vx + vy + vz;
        temp2 = -vx + vy - vz;
        temp3 = -vx + vy + vz;
        temp4 = vx + vy - vz;

        send_Speed_Config(temp1 * 36, temp2 * 36, temp3 * 36, temp4 * 36);

    }

    /**
     * 麦轮角度设置
     * @param angle 选定方向角度
     */
    void Chassis_Class::set_MW_angle(float angle)
    { // 计算yaw转动速度
        float vz = GyroPIDs.calculate_gyro(angle);
        // 设置yaw转动速度，继承上一次x,y轴速度
        set_MW_speed(carVx, carVy, vz);
        HAL_Delay(PID_DUTY_CYCLE);
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机定轴右转向
     * @param speed 转向速度(可以为负数)
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::Transverse_move_Till_Right(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, bool BarkeEnd)
    {
        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < 50; count++)
            {
                set_MW_speed(speed,carVy,carVz);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            while (lpStop_Func(lpParam) != true)
            {
                switch (Chassis_Sensors.getRightSensorStatus())
                {
                case 0b11001: //偏上
                case 0b11101:
                case 0b11100:
                    pre_Direction = DIRECTION_FRONT; //记录位置
                    set_MW_speed(speed, -speed * 0.5, carVz);
                    break;
                case 0b11110:                        //极端左偏
                    pre_Direction = DIRECTION_FRONT; //记录位置
                    set_MW_speed(speed, -speed * 0.7, carVz);
                    break;
                case 0b10011: //偏下
                case 0b10111:
                case 0b00111:
                    pre_Direction = DIRECTION_BACK; //记录位置
                    set_MW_speed(speed, speed * 0.5, carVz);
                    break;
                case 0b01111:                       //极端右偏
                    pre_Direction = DIRECTION_BACK; //记录位置
                    set_MW_speed(speed, speed * 0.7, carVz);
                    break;
                case 0b11111: //丢失路线
                    if (pre_Direction == DIRECTION_FRONT)
                    {
                        set_MW_speed(0, -speed * 0.8, carVz);
                        break;
                    }
                    if (pre_Direction == DIRECTION_BACK)
                    {
                        set_MW_speed(0, speed * 0.8, carVz);
                        break;
                    }

                default:
                    set_MW_speed(speed, 0,carVz ); //发送配速
                    break;
                }
                HAL_Delay(PID_DUTY_CYCLE);
            }
        }
        //刹车
        if (BarkeEnd)
            barke();
    }

    /**
     * 由用户传入lpStop_Func函数控制结束条件的内嵌PID电机定轴左转向
     * @param speed 转向速度
     * @param lpStop_Func 控制结束前进的用户自定义条件函数
     * @param lpParam 传给条件函数的参数列表
     * @param BarkeEnd 结束后刹车，默认为true
     * @return 无
     */
    void Chassis_Class::Transverse_move_Till_Left(int16_t speed, bool (*lpStop_Func)(void *lpParam), void *lpParam, bool BarkeEnd)
    {
        if (speed > 0) //防止反向
        {
            //初动补偿
            for (int count = 0; count < 50; count++)
            {
                set_MW_speed(-speed,carVy,carVz);
                HAL_Delay(PID_DUTY_CYCLE);
            }
            while (lpStop_Func(lpParam) != true)
            {
                switch (Chassis_Sensors.getLeftSensorStatus())
                {

                case 0b11001: //偏上
                case 0b11101:
                case 0b11100:
                    pre_Direction = DIRECTION_FRONT; //记录位置
                    set_MW_speed(speed, speed * 0.5, carVz);
                    break;
                case 0b11110:                        //极端左偏
                    pre_Direction = DIRECTION_FRONT; //记录位置
                    set_MW_speed(-speed, carVy, speed);
                    set_MW_speed(speed, speed * 0.7, carVz);

                    break;
                case 0b10011: //偏下
                case 0b10111:
                case 0b00111:
                    pre_Direction = DIRECTION_BACK; //记录位置
                    set_MW_speed(speed, -speed * 0.5, carVz);
                    break;
                case 0b01111:                       //极端右偏
                    pre_Direction = DIRECTION_BACK; //记录位置
                    set_MW_speed(speed, -speed * 0.7, carVz);
                    break;
                case 0b11111: //丢失路线
                    if (pre_Direction == DIRECTION_FRONT)
                    {
                        set_MW_speed(speed, -speed * 0.8, carVz);
                        break;
                    }
                    if (pre_Direction == DIRECTION_BACK)
                    {
                        set_MW_speed(speed, speed * 0.8, carVz);
                        break;
                    }

                default:
                    set_MW_speed(-speed, 0, carVz); //发送配速
                    break;
                }
                HAL_Delay(PID_DUTY_CYCLE);
            }
        }
        //刹车
        if (BarkeEnd)
            barke();

        //清除误差积分
        clear_err_integral();

    }


}