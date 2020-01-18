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
    double intakePower = 0;
    double intakeAngle = 0.53; //down
    double hookAngle = 0;
    double valveAngle = 0;
    double speed = 1;
    double liftHeight = 0;
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

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strafeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        toggleIntakeAngle();

        gyro = new Gyro(hardwareMap);
        resetGyro();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        if (vuforia) {
            this.vuforia = new Vuforia(hardwareMap);
            this.vuforia.flashlight(true);
        }

        resetBallDrive();
        resetLift();
    }

    void resetBallDrive() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        updateBallDrive(false);
    }

    void updateBallDrive(boolean targetAngle) {
        if (targetAngle) {
            double dp = 0.05 * (this.targetAngle - getGyroAngle());
            leftDrive.setPower(Range.clip(speed * (leftPower - dp),-1.0, 1.0));
            rightDrive.setPower(Range.clip(speed * (rightPower + dp),-1.0, 1.0));
        } else {
            leftDrive.setPower(Range.clip(speed * leftPower,-1.0, 1.0));
            rightDrive.setPower(Range.clip(speed * rightPower,-1.0, 1.0));
        }
        strafeDrive.setPower(Range.clip(speed * strafePower,-1.0, 1.0));
    }

    void move(double z_inches, double x_inches, double wait) {
        targetAngle = getGyroAngle();
        int z_ticks = (int)(z_inches * Z_TICKS_PER_INCH);
        int x_ticks = (int)(x_inches * X_TICKS_PER_INCH);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + z_ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + z_ticks);
        strafeDrive.setTargetPosition(strafeDrive.getCurrentPosition() + x_ticks);
        leftPower = 1;
        rightPower = 1;
        strafePower = 1;
        updateBallDrive(true);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Math.abs(leftDrive.getCurrentPosition() - z_ticks) > 5 || Math.abs(strafeDrive.getCurrentPosition() - x_ticks) > 5) {
            //updateBallDrive(true);
        }
        resetBallDrive();
        wait(0.0 + wait);
    }

//    void move(double z_inches, double x_inches, double wait) {
//        targetAngle = getGyroAngle();
//        int z_ticks = (int)(z_inches * Z_TICKS_PER_INCH);
//        int x_ticks = (int)(x_inches * X_TICKS_PER_INCH);
//        while (Math.abs(leftDrive.getCurrentPosition() - z_ticks) > 20 || Math.abs(rightDrive.getCurrentPosition() - z_ticks) > 20 || Math.abs(strafeDrive.getCurrentPosition() - x_ticks) > 20) {
//            double dz = 0.005 * (z_ticks - leftDrive.getCurrentPosition() + z_ticks - rightDrive.getCurrentPosition()) / 2;
//            double dx = 0.005 * (x_ticks - strafeDrive.getCurrentPosition());
//            leftPower = dz;
//            rightPower = dz;
//            strafePower = dx;
//            updateBallDrive(true);
//        }
//        leftPower = 0;
//        rightPower = 0;
//        strafePower = 0;
//        updateBallDrive(false);
//        wait(0.0 + wait);
//    }

    void rotate(double angle) {
        targetAngle = angle;
        while (Math.abs(angle - getGyroAngle()) > 1) {
            updateBallDrive(true);
        }
        leftPower = 0;
        rightPower = 0;
        updateBallDrive(false);
    }

    boolean liftAtBottom() {
        return touchSensor.isPressed();
    }

    void resetLift() {
//        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        while (!liftAtBottom()) {
//            tuneLift(-0.003);
//        }
//        while (liftAtBottom()) {
//            tuneLift(0.003);
//        }
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftHeight = 0;
    }

    void tuneLift(double height) {
        resetLift();


        liftHeight = 0;
        updateLift();
        resetLift();
    }

    void updateLift() {
        int y_ticks;
        if (liftHeight == 0) {
            y_ticks = 0;
        } else {
            y_ticks = (int)((4 * liftHeight - 2) * Y_TICKS_PER_INCH);
        }
        liftMotor.setTargetPosition(y_ticks);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
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
        if (intakeAngle == 1.5) {
            intakeAngle = 0.53;
        } else {
            intakeAngle = 1.5;
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
        while (Math.abs(getVuforiaZ() - 10) > 1 && Math.abs(getVuforiaX()) > 1) {
            double dz = 0.05 * (getVuforiaZ() - 10);
            double dx = 0.05 * (getVuforiaX());
            leftPower = dz;
            rightPower = dz;
            strafePower = dx;
            updateBallDrive(true);
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