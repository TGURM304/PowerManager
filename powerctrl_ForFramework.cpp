//
// Created by 3545 on 25-11-1.
//

#include "powerctrl_ForFramework.h"


MotorPowerCtrl::MotorPowerCtrl(const double k0,const double k1,const double k2,const double k3,const double k4,const double k5 ,const double current_conversion )
    :K0(k0),K1(k1),K2(k2),K3(k3),K4(k4),K5(k5),Current_Conversion(current_conversion) {}

double MotorPowerCtrl::getMotorRealCurrent(const double current) const {
    const double real_current = current/Current_Conversion;
    return real_current;
}

double MotorPowerCtrl::update(double current ,double speed,const E_Predict_Status_Type Predict_status,const E_CalMotorPower_Negative_Status_Type Negative_Status) {
    const double product = current*speed;
    double power_sign = 1;

    if (Negative_Status == E_enable_negative) {
        if(product < 0 ) {
            power_sign = -1;
        }
    }

    current = std::abs(getMotorRealCurrent(current));
    speed = std::abs(speed);

    const double power =  (K0 +
                   K1 * current +
                   K2 * speed +
                   K3 * current * speed +
                   K4 * current * current +
                   K5 * speed * speed)*power_sign;

    if(Predict_status == E_enable_predict) {
        predict_power = power;
    }else if(Predict_status == E_disabled_predict) {
        feedback_power = power;
    }else if(Predict_status == E_enable_not_limit_predict) {
        predict_not_limit_power = power;
    }

    return power;
}

double MotorPowerCtrl::limiter(double *desired_current,const double current_speed, const double motor_power_limit ) {

    if (desired_current == nullptr) {
        return 0.0;
    }

    power_limit = motor_power_limit;

    if(motor_power_limit < 0) {
        *desired_current = 0;
        return 0.0;
    }

    const double desired_current_ = *desired_current;

    const double real_desired_current = std::abs(getMotorRealCurrent(desired_current_));
    const double real_current_speed = std::abs(current_speed);

    if (const double predicted_power = update(desired_current_, current_speed,E_enable_not_limit_predict);
       predicted_power <= motor_power_limit) {
        return 1.0;
       }

    const double a = K4 * real_desired_current * real_desired_current;
    const double b = (K1 + K3 * real_current_speed) * real_desired_current;
    const double c = K0 + K2 * real_current_speed + K5 * real_current_speed * real_current_speed - motor_power_limit;
    const double discriminant = b * b - 4 * a * c;

    if (std::abs(a) < 1e-9) {
        if (std::abs(b) < 1e-9) {
            if(c <= 1e-9) {
                return 1.0;
            }else {
                *desired_current = 0;
                return 0.0;
            }
        } else {
            double k = -c / b;
            if (k < 0.0) {
                *desired_current = 0;
                return 0.0;
            }
            if (k > 1.0) {
                return 1.0;
            }
            return k;
        }
    }

    if (discriminant < 0) {
        *desired_current = 0;
        return 0.0;
    }

    if(std::abs(discriminant) < 1e-9) {
        double k = -1.0 * b / (2 * a);
        if(k < 0.0) {
            *desired_current = 0;
            return 0.0;
        }
        if(k > 1.0) {
            return 1.0;
        }
        *desired_current *= k;
        return k;
    }

    const double k1 = (-b - std::sqrt(discriminant)) / (2 * a);
    const double k2 = (-b + std::sqrt(discriminant)) / (2 * a);
    if ( (k1 > 0.0 and k1 < 1.0 ) and (k2 > 0.0 and k2 < 1.0) ) {
        *desired_current *=  std::max(k1, k2);
        return std::max(k1, k2);
    }else if ( (k1 > 0.0 and k1 < 1.0) and(k2 > 1.0 or k2 < 0.0) ) {
        *desired_current *= k1;
        return k1;
    }else if ( (k1 < 0.0 or k1 > 1.0 ) and(k2 > 0.0 and k2 < 1.0) ) {
        *desired_current *= k2;
        return k2;
    }else {
        *desired_current = 0;
        return 0.0;
    }

}

std::vector<double> power_allocation_by_error(std::vector<double>& motor_errors_vector, double total_power_limit) {

#ifdef M_Enable_PowerCompensation
    total_power_limit *= (1-M_SmallGyro_Power_Compensation_Alpha);
#endif

    if (motor_errors_vector.size() != 4) {
        return {0.0, 0.0, 0.0, 0.0} ;
    }

    for (double& error : motor_errors_vector) {
        error = std::abs(error);
    }

    if (total_power_limit <= 1e-9) {
        return {0.0, 0.0, 0.0, 0.0};
    }

    const double total_error = motor_errors_vector[0]+motor_errors_vector[1]+motor_errors_vector[2]+motor_errors_vector[3];

    if (total_error <= M_Too_Small_AllErrors) {
        double equal_share = total_power_limit / motor_errors_vector.size();
        return {equal_share, equal_share, equal_share, equal_share};
    }

    std::vector<double> motor_power_limits_vector(4);
    if(total_power_limit < M_Motor_ReservedPower_Border) {
        for (int i = 0; i < 4; ++i) {
            const double ratio = motor_errors_vector[i] / total_error;
            motor_power_limits_vector[i] = ratio * total_power_limit;
        }
    }else {
        for (int j = 0; j < 4; ++j) {
            const double ratio = motor_errors_vector[j] / (total_error - 4*M_PerMotor_ReservedPower);
            motor_power_limits_vector[j] = ratio * (total_power_limit-4 * M_PerMotor_ReservedPower) + M_PerMotor_ReservedPower;
        }
    }

    return motor_power_limits_vector;
}