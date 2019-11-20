package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class Robot {
    double leftPower = 0;
    double rightPower = 0;
    double strafePower = 0;
    double liftPower = 0;
    double intakePower = 0;
    double intakeAngle = 0;
    double hookAngle = 0;
    double valveAngle = 0;

    double speed = 1;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor strafeDrive;
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;
    private DcMotor intakeMotor;
    private Servo intakeServo;
    private Servo hookServo1;
    private Servo hookServo2;
    private Servo valveServo;

     Robot(HardwareMap hardwareMap){
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
        strafeDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);
        hookServo1.setDirection(Servo.Direction.REVERSE);
        hookServo2.setDirection(Servo.Direction.FORWARD);
        valveServo.setDirection(Servo.Direction.FORWARD);
    }

    void toggleSpeed() {
        if (speed == 1) {
            speed = 0.5;
        } else {
            speed = 1;
        }
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

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        strafeDrive.setPower(strafePower);
        liftMotor1.setPower(liftPower);
        liftMotor2.setPower(liftPower);
        intakeMotor.setPower(intakePower);
        intakeServo.setPosition(intakeAngle);
        hookServo1.setPosition(hookAngle);
        hookServo2.setPosition(hookAngle);
        valveServo.setPosition(valveAngle);
    }
}
