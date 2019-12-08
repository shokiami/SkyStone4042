package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

class RobotAuto {
    double leftPower = 0;
    double rightPower = 0;
    double strafePower = 0;
    double liftPower = 0;
    double intakePower = 0;
    double intakeAngle = 0;
    double hookAngle = 0;
    double valveAngle = 0;
    double speed = 1;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor strafeDrive;
    DcMotor liftMotor1;
    DcMotor liftMotor2;
    DcMotor intakeMotor;
    Servo intakeServo;
    Servo hookServo1;
    Servo hookServo2;
    Servo valveServo;

    RobotAuto(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class, "strafe_drive");
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift_motor_2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        hookServo1 = hardwareMap.get(Servo.class, "hook_servo_1");
        hookServo2 = hardwareMap.get(Servo.class, "hook_servo_2");
        valveServo = hardwareMap.get(Servo.class, "valve_servo");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        strafeDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);
        hookServo1.setDirection(Servo.Direction.REVERSE);
        hookServo2.setDirection(Servo.Direction.FORWARD);
        valveServo.setDirection(Servo.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void adjustSpeed(double value) {
        speed = value;
    }

    void toggleIntake() {
        if (intakePower == 0) {
            intakePower = 1;
        } else {
            intakePower = 0;
        }
    }

    void toggleValve() {
        if (valveAngle == 0) {
            valveAngle = 1;
        } else {
            valveAngle = 0;
        }
    }

    void toggleHook() {
        if (hookAngle == 0) {
            hookAngle = 0.6;
        } else {
            hookAngle = 0;
        }
    }

    void update() {
        leftPower *= speed;
        rightPower *= speed;
        strafePower *= speed;
        liftPower *= speed;

        leftDrive.setPower(Range.clip(leftPower,-1.0, 1.0));
        rightDrive.setPower(Range.clip(rightPower,-1.0, 1.0));
        strafeDrive.setPower(Range.clip(strafePower,-1.0, 1.0));
        liftMotor1.setPower(0.95 * liftPower);
        liftMotor2.setPower(liftPower);
        intakeMotor.setPower(intakePower);
        intakeServo.setPosition(intakeAngle);
        hookServo1.setPosition(hookAngle);
        hookServo2.setPosition(hookAngle);
        valveServo.setPosition(valveAngle);
    }
/*
    void updatePIDCoefficients(double Kp, double Ki, double Kd) {
        leftDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
        rightDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
        strafeDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
    }
*/
    int leftPosition() {
        return leftDrive.getCurrentPosition();
    }
    int rightPosition() {
        return rightDrive.getCurrentPosition();
    }
    int strafePosition() {
        return strafeDrive.getCurrentPosition();
    }

    void move(int z_inches, int x_inches) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + 55 * z_inches);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 55 * z_inches);
        strafeDrive.setTargetPosition(strafeDrive.getCurrentPosition() + 500);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        strafeDrive.setPower(1);
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html