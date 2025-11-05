## 自测M3508参数结构体
motor_power_init_t M3508_power_init_data(0.65213,(-0.15659),0.00041660,0.00235415,0.20022,1.08e-7,1000);

## 自测GM6020参数结构体
motor_power_init_t GM6020_power_init_data(0.7507578,(-0.0759636),(-0.00153397),0.01225624,0.19101805,0.0000066450,1000);

## rotate_speed_allocation函数常用参数
alpha = 0.6;

## rotate_theta_forwardfeed函数常用参数
kp = 0.0050;

## 常见的底盘四3508电机限制功率使用示例:

```c++
//init
motor_power_init_t motor_3508_power_data(0.65213,(-0.15659),0.00041660,0.00235415,0.20022,1.08e-7,1000);
MotorPower m3508_1_power(motor_3508_power_data);
MotorPower m3508_2_power(motor_3508_power_data);
MotorPower m3508_3_power(motor_3508_power_data);
MotorPower m3508_4_power(motor_3508_power_data);
ChassisPowerManager chassis(&m3508_1_power, &m3508_2_power, &m3508_3_power, &m3508_4_power);
//update(in while)
        chassis.updateMotorError(0, m1_target - static_cast<float>(m3508_1.feedback_.speed));
        chassis.updateMotorError(1, m2_target - static_cast<float>(m3508_2.feedback_.speed));
        chassis.updateMotorError(2, m3_target - static_cast<float>(m3508_3.feedback_.speed));
        chassis.updateMotorError(3, m4_target - static_cast<float>(m3508_4.feedback_.speed));
        chassis.allocatePower(target);
        m3508_1_power.limiter(&m1_output,m3508_1.feedback_.speed,m3508_1_power.power_limit);
        m3508_2_power.limiter(&m2_output,m3508_2.feedback_.speed,m3508_2_power.power_limit);
        m3508_3_power.limiter(&m3_output,m3508_3.feedback_.speed,m3508_3_power.power_limit);
        m3508_4_power.limiter(&m4_output,m3508_4.feedback_.speed,m3508_4_power.power_limit);
```