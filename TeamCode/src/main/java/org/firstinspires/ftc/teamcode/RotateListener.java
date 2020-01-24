package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RotateListener extends Thread {

    boolean pressed;

    Telemetry telemetry;
    Robot robot;

    RotateListener(Telemetry telemetry, Robot robot) {
        this.telemetry = telemetry;
    }

    void setRunnable(boolean input) {
        this.pressed = input;
    }

    @Override
    public void run() {
        /*
        while(true) {
            if(pressed == true)
                t.addData("thread", "test success");
        }
        */
            robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //GYRO = 0, might potentially mess something up
            robot.resetGyro();

            telemetry.addData("test","successful");
            /*
            while(true) {
                if(pressed) {

                    telemetry.addData("Spinning","true");

                    while(robot.getGyroAngle() <= 180 && robot.getGyroAngle() >= -5) {
                        robot.leftPower = 5;
                        robot.rightPower = -5;
                    }
                    robot.leftPower  = 0;
                    robot.rightPower = 0;
                    robot.resetGyro();

                    pressed = false;
                } else {
                    telemetry.addData("Spinning","false");
                }
            }
        */
        }
    }