//
// Created by 3545 on 25-11-1.
//

#ifndef POWERCTRL_FORFRAMEWORK_H
#define POWERCTRL_FORFRAMEWORK_H

#include <cmath>
#include <vector>

#define M_Too_Small_AllErrors 500.0
// #define M_Enable_PowerCompensation
#define M_SmallGyro_Power_Compensation_Alpha 0.05
#define M_Motor_ReservedPower_Border 54.0
#define M_PerMotor_ReservedPower 8.0

class MotorPowerCtrl{
protected:
    const double K0,K1,K2,K3,K4,K5,Current_Conversion;
    bool predict_flag = false;
public:
    enum E_CalMotorPower_Negative_Status_Type{E_disabled_negative,E_enable_negative};
    enum E_Predict_Status_Type{E_disabled_predict,E_enable_predict,E_enable_not_limit_predict};
    double feedback_power = 0;
    double predict_not_limit_power = 0;
    double predict_power = 0;
    double power_limit = 0;
    MotorPowerCtrl(double k0, double k1, double k2, double k3, double k4, double k5,double current_conversion);
    [[nodiscard]] double getMotorRealCurrent(double current) const;
    double update(double current, double speed,E_Predict_Status_Type Predict_status = E_disabled_predict, E_CalMotorPower_Negative_Status_Type Negative_Status = E_disabled_negative);
    double limiter(double *desired_current, double current_speed, double motor_power_limit);
};

#endif //POWERCTRL_FORFRAMEWORK_H
