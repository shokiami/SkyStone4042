package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

class Robot {
    double leftPower = 0;
    double rightPower = 0;
    double strafePower = 0;
    double liftPower = 0;
    double intakePower = 0;
    double intakeAngle = 0.6;
    double hookAngle = 0;
    double valveAngle = 0;
    double speed = 1;

    static final double Z_TICKS_PER_INCH = 49.606;
    static final double X_TICKS_PER_INCH = 58.504;
    static final double TURN_RADIUS = 8.4925;

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
    Gyro gyro;

    Robot(HardwareMap hardwareMap) {
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

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void toggleSpeed() {
        if (speed == 1) {
            speed = 0.3;
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
            hookAngle = 0.5;
        } else {
            hookAngle = 0;
        }
    }

    void toggleIntakeAngle() {
        if (intakeAngle == 0) {
            intakeAngle = 0.6;
        } else {
            intakeAngle = 0;
        }
    }

    void update() {
        //leftPower *= speed;
        //rightPower *= speed;
        //strafePower *= speed;
        //liftPower *= speed;

        leftDrive.setPower(Range.clip(leftPower,-1.0, 1.0));
        rightDrive.setPower(Range.clip(rightPower,-1.0, 1.0));
        strafeDrive.setPower(Range.clip(strafePower,-1.0, 1.0));
        liftMotor1.setPower(0.65 * liftPower);
        liftMotor2.setPower(liftPower);
        intakeMotor.setPower(intakePower);
        intakeServo.setPosition(intakeAngle);
        hookServo1.setPosition(hookAngle);
        hookServo2.setPosition(hookAngle);
        valveServo.setPosition(valveAngle);
    }

    void move(int z_inches, int x_inches) {
        int left_target_z = leftDrive.getCurrentPosition() + (int)(Z_TICKS_PER_INCH * z_inches);
        int right_target_z = rightDrive.getCurrentPosition() + (int)(Z_TICKS_PER_INCH * z_inches);
        int strafe_target_x = strafeDrive.getCurrentPosition() + (int)(X_TICKS_PER_INCH * x_inches);
        leftDrive.setTargetPosition(left_target_z);
        rightDrive.setTargetPosition(right_target_z);
        strafeDrive.setTargetPosition(strafe_target_x);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        strafeDrive.setPower(speed);
        while (Math.abs(leftDrive.getCurrentPosition() - left_target_z) > 10 || Math.abs(rightDrive.getCurrentPosition() - right_target_z) > 10 || Math.abs(strafeDrive.getCurrentPosition() - strafe_target_x) > 10) {
            //Wait
        }
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());
        strafeDrive.setTargetPosition(strafeDrive.getCurrentPosition());
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    void rotate(double angle) { // clockwise
        angle *= Math.PI / 180.0;
        int left_target = (int) (TURN_RADIUS * angle * Z_TICKS_PER_INCH);
        int right_target = (int) (- TURN_RADIUS * angle * Z_TICKS_PER_INCH);
        leftDrive.setTargetPosition(left_target);
        rightDrive.setTargetPosition(right_target);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (Math.abs(leftDrive.getCurrentPosition() - left_target) > 10 || Math.abs(rightDrive.getCurrentPosition() - right_target) > 10) {
            //Wait
        }
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition());
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html