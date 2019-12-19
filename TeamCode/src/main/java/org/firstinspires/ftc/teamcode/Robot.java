package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

class Robot {
    // This is a test message.
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

    DcMotorEx leftDrive;
    DcMotorEx rightDrive;
    DcMotorEx strafeDrive;
    DcMotor liftMotor1;
    DcMotor liftMotor2;
    DcMotor intakeMotor;
    Servo intakeServo;
    Servo hookServo1;
    Servo hookServo2;
    Servo valveServo;

    ElapsedTime elapsedTime;
    Vuforia vuforia;
    Gyro gyro;

    Robot(HardwareMap hardwareMap) {
        leftDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = (DcMotorEx)hardwareMap.get(DcMotor.class, "strafe_drive");
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

        resetEncoders();

        leftDrive.setVelocityPIDFCoefficients(10,0,0,0);
        rightDrive.setVelocityPIDFCoefficients(10,0,0,0);
        strafeDrive.setVelocityPIDFCoefficients(10,0,0,0);

        vuforia = new Vuforia(hardwareMap);
        gyro = new Gyro(hardwareMap);
        elapsedTime = new ElapsedTime();
    }

    void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void updatePIDCoefficients(double p ,double i ,double d ) {
        leftDrive.setVelocityPIDFCoefficients(p,i,d,0);
        rightDrive.setVelocityPIDFCoefficients(p,i,d,0);
        strafeDrive.setVelocityPIDFCoefficients(p,i,d,0);
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

    void resetElapsedTime() {
        elapsedTime.reset();
    }

    double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    void turnOnFlashlight() {
        vuforia.flashlight(true);
    }

    boolean isTargetVisible() {
        vuforia.update();
        return  vuforia.isTargetVisible();
    }

    double getVuforiaZ() {
        vuforia.update();
        return -vuforia.getX();
    }

    double getVuforiaX() {
        vuforia.update();
        return vuforia.getY();
    }

    double getVuforiaHeading() {
        vuforia.update();
        return vuforia.getHeading();
    }

    void resetGyro() {
        gyro.resetAngle();
    }

    double getGyroAngle() {
        return gyro.getAngle();
    }

    void update() {
        leftPower *= speed;
        rightPower *= speed;
        strafePower *= speed;
        liftPower *= speed;

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

    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }

    void move(double z_inches, double x_inches) {
        int left_target_z = (int)(Z_TICKS_PER_INCH * z_inches);
        int right_target_z = (int)(Z_TICKS_PER_INCH * z_inches);
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
        resetEncoders();
    }

    void rotate(double angle) {
        int left_target_z = (int)(Z_TICKS_PER_INCH * angle);
        int right_target_z = (int)(Z_TICKS_PER_INCH * angle);
        leftDrive.setTargetPosition(left_target_z);
        rightDrive.setTargetPosition(right_target_z);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);
        while (Math.abs(leftDrive.getCurrentPosition() - left_target_z) > 10 || Math.abs(rightDrive.getCurrentPosition() - right_target_z) > 10) {
            //Wait
        }
        resetEncoders();
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html