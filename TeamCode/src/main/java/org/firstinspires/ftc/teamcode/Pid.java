package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

class Pid {
    double Kp;
    double Ki;
    double Kd;
    double previous_error;
    double previous_time;
    double output;
    double integral;
    double derivative;
    ElapsedTime runtime;

    Pid(double Kp, double Ki, double Kd){
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        previous_error = 0;
        previous_time = 0;
        output = 0;
        integral = 0;
        derivative = 0;
        runtime = new ElapsedTime();
    }

    double getPower(double error) {
        double dt = runtime.time() - previous_time;
        previous_time += dt;
        integral += error * dt;
        derivative = (error - previous_error) / dt;
        output = Kp * error + Ki * integral + Kd * derivative;
        previous_error = error;
        return output;
    }

    void update(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}