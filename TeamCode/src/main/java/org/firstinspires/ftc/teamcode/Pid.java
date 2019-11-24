package org.firstinspires.ftc.teamcode;

class Pid {
    double Kp;
    double Ki;
    double Kd;

    double previous_error;
    double integral;

    Pid(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;

        previous_error = 0;
        integral = 0;
    }

    double getPower(double error) {
        integral += error;
        double output = Kp * error + Ki * integral + Kd * (error - previous_error);
        previous_error = error;
        return output;
    }
}
