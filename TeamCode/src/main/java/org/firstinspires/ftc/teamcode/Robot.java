package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
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
    double targetAngle = 0;

    static final double Z_TICKS_PER_INCH = 55;
    static final double X_TICKS_PER_INCH = 61.73;
    static final double TURN_RADIUS = 8.493;
    static final double Y_TICKS_PER_INCH = 415.0;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor strafeDrive;
    DcMotor liftMotor;
    DcMotor intakeMotor;
    Servo intakeServo;
    Servo hookServo1;
    Servo hookServo2;
    Servo valveServo;
    TouchSensor touchSensor;

    ElapsedTime elapsedTime;
    Vuforia vuforia;
    Gyro gyro;

    Robot(HardwareMap hardwareMap, boolean vuforia) {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class, "strafe_drive");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor_2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");
        hookServo1 = hardwareMap.get(Servo.class, "hook_servo_1");
        hookServo2 = hardwareMap.get(Servo.class, "hook_servo_2");
        valveServo = hardwareMap.get(Servo.class, "valve_servo");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        strafeDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);
        hookServo1.setDirection(Servo.Direction.REVERSE);
        hookServo2.setDirection(Servo.Direction.FORWARD);
        valveServo.setDirection(Servo.Direction.FORWARD);

        gyro = new Gyro(hardwareMap);
        elapsedTime = new ElapsedTime();
        if (vuforia) {
            this.vuforia = new Vuforia(hardwareMap);
            this.vuforia.flashlight(true);
        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strafeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetBallDrive();

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetLift();

        updateBallDrive(false);
        resetLift();
    }

    void resetBallDrive() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void updateBallDrive(boolean targetAngle) {
        double dp = 0;
        if (targetAngle) {
            dp = 0.05 * (this.targetAngle - getGyroAngle());
        }
        leftDrive.setPower(Range.clip(speed * (leftPower - dp),-1.0, 1.0));
        rightDrive.setPower(Range.clip(speed * (rightPower + dp),-1.0, 1.0));
        strafeDrive.setPower(Range.clip(speed * strafePower,-1.0, 1.0));
        if (!targetAngle) {
            this.targetAngle = getGyroAngle();
        }
    }

    void move(double z_inches, double x_inches) {
        int z_ticks = (int)(z_inches * Z_TICKS_PER_INCH);
        int x_ticks = (int)(x_inches * X_TICKS_PER_INCH);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + z_ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + z_ticks);
        strafeDrive.setTargetPosition(strafeDrive.getCurrentPosition() + x_ticks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        strafeDrive.setPower(1);
        updateBallDrive(false);
        while (Math.abs(leftDrive.getCurrentPosition() - z_ticks) > 5 || Math.abs(strafeDrive.getCurrentPosition() - x_ticks) > 5) {
            //Wait
        }
        resetBallDrive();
    }

//    void move(double z_inches, double x_inches) {
//        double targetAngle= getGyroAngle();
//        int z_ticks = (int)(z_inches * Z_TICKS_PER_INCH);
//        int x_ticks = (int)(x_inches * X_TICKS_PER_INCH);
//        while (Math.abs(z_ticks - leftDrive.getCurrentPosition()) > 5 || Math.abs(x_ticks - strafeDrive.getCurrentPosition()) > 5) {
//            double dz = 0.02 * (z_ticks - leftDrive.getCurrentPosition());
//            double dx = 0.02 * (x_ticks - strafeDrive.getCurrentPosition());
//            leftPower = dz;
//            rightPower = dz;
//            strafePower = dx;
//            updateBallDrive();
//            targetAngle(targetAngle);
//        }
//        leftPower = 0;
//        rightPower = 0;
//        strafePower = 0;
//        updateBallDrive();
//    }

    void rotate(double targetAngle) {
        while (Math.abs(targetAngle - getGyroAngle()) > 1) {
            updateBallDrive(true);
        }
        leftPower = 0;
        rightPower = 0;
        updateBallDrive(false);
    }

    void targetAngle(double targetAngle) {
        double dp = 0.05 * (targetAngle - getGyroAngle());
        leftDrive.setPower(Range.clip(leftPower - dp,-1.0, 1.0));
        rightDrive.setPower(Range.clip(rightPower + dp,-1.0, 1.0));
    }

    boolean liftAtBottom() {
        return touchSensor.isPressed();
    }

    void resetLift() {
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void updateLift() {
        liftMotor.setPower(liftPower);
    }

//    void tuneLift(double height) {
//        liftHeight += height;
//        updateLift();
//    }

//    void updateLift() {
//        int y_ticks;
//        if (liftHeight == 0) {
//            y_ticks = 0;
//        } else {
//            y_ticks = (int)((4 * liftHeight - 2) * Y_TICKS_PER_INCH);
//        }
//        liftMotor.setTargetPosition(y_ticks);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower(1);
//    }

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
        intakeMotor.setPower(intakePower);
    }

    void toggleValve() {
        if (valveAngle == 0) {
            valveAngle = 1;
        } else {
            valveAngle = 0;
        }
        valveServo.setPosition(valveAngle);
    }

    void toggleHook() {
        if (hookAngle == 0) {
            hookAngle = 0.5;
        } else {
            hookAngle = 0;
        }
        hookServo1.setPosition(hookAngle);
        hookServo2.setPosition(hookAngle);
    }

    void toggleIntakeAngle() {
        if (intakeAngle == 0) {
            intakeAngle = 0.6;
        } else {
            intakeAngle = 0;
        }
        intakeServo.setPosition(intakeAngle);
    }

    void resetElapsedTime() {
        elapsedTime.reset();
    }

    double getElapsedTimeSeconds() {
        return elapsedTime.seconds();
    }

    void wait(double seconds) {
        double start = getElapsedTimeSeconds();
        while (getElapsedTimeSeconds() - start < seconds) {}
    }

    void alignVuforia() {
        while (Math.abs(getVuforiaZ() - 10) < 1 && Math.abs(getVuforiaX()) < 1) {
            double dz = 0.05 * (getVuforiaZ() - 10);
            double dx = 0.05 * (getVuforiaX());
            leftDrive.setPower(Range.clip(leftPower + dz,-1.0, 1.0));
            rightDrive.setPower(Range.clip(rightPower + dz,-1.0, 1.0));
            strafeDrive.setPower(Range.clip(strafePower + dx,-1.0, 1.0));
        }
        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        updateBallDrive(false);
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
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html