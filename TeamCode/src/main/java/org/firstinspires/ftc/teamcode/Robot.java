package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Robot {
    double leftPower = 0;
    double rightPower = 0;
    double strafePower = 0;
    double intakePower = 0;
    double intakeAngle = 0.53;
    double hookAngle = 0;
    double valveAngle = 0;
    double speed = 1;
//    double liftHeight = 0;
    double targetAngle = 0;
//    int liftZeroPos = 0;
    boolean liftUp = false;

    static final double Z_TICKS_PER_INCH = 54.000;
    static final double X_TICKS_PER_INCH = 59.529; //
    static final double TURN_RADIUS = 8.493;
    static final double Y_TICKS_PER_INCH = 50; //415.0

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
    Telemetry telemetry;

    ElapsedTime delta = new ElapsedTime();
    double lastError;

    Robot(HardwareMap hardwareMap, Telemetry telemetry, boolean vuforia) {
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
        this.telemetry = telemetry;

        resetBallDrive();
        //resetLift();
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
        double error = this.targetAngle - getGyroAngle();
        double pCoeff = 0.03; //0.03
        double dCoeff = 0.003; //0.003
        if (strafeDrive.getPower() > 0.6){
            pCoeff = 0.01;
            dCoeff = 0.002; // 0.002
        }
        double p = pCoeff * error;
        double d = dCoeff * (error - lastError) / delta.seconds();
        telemetry.addData("pd","p = " + p + ", d = " + d);
        telemetry.addData("angles", "Gyro = " + getGyroAngle() + ", tAngle = " + this.targetAngle);
        telemetry.addData("powers", "leftPow = " + leftPower + ", rightPow = " + rightPower);
        telemetry.addData("rightTicks", "" + rightDrive.getCurrentPosition());
        telemetry.addData("liftMotor", "" + liftMotor.getCurrentPosition());
        telemetry.addData("strafePos", "" + strafeDrive.getCurrentPosition());
        telemetry.update();
        double tuning = Range.clip(p + d, -0.75, 0.75);
        double slowTuning = Range.clip(tuning, (-1) * (0.3 + 0.45 * speed) , 0.3 + 0.45 * speed);
        leftDrive.setPower(speed * leftPower - (targetAngle ? slowTuning : 0));
        rightDrive.setPower(speed * rightPower + (targetAngle ? slowTuning : 0));
        strafeDrive.setPower(speed * strafePower);
        lastError = error;
        delta.reset();
    }

    void move(double zInches, double xInches, double wait) {
        resetBallDrive();
        double targetZ = zInches * Z_TICKS_PER_INCH;
        double targetX = xInches * X_TICKS_PER_INCH;
        while (true) {
            double dz = targetZ - rightDrive.getCurrentPosition();
            double dx = targetX - strafeDrive.getCurrentPosition();
            leftPower = 0.0013 * dz;
            rightPower = 0.0013 * dz;
            strafePower = 0.001 * dx;
            updateBallDrive(true);
            if (Math.sqrt(dz * dz + dx * dx) < 20) {
                break;
            }
        }
        resetBallDrive();
        wait(wait);
    }

    void move(double zInches, double xInches, double angle, double wait) {
        rotate(angle);
        move(zInches, xInches, wait);
    }

    void rotate(double angle) {
        targetAngle = angle;
        resetBallDrive();
        while (Math.abs(angle - getGyroAngle()) > 5) {
            updateBallDrive(true);
        }
        resetBallDrive();
    }

//    void updateLift() {
//        int y_ticks;
//        if (liftHeight == 0) {
//            y_ticks = liftZeroPos;
//        } else {
//            y_ticks = liftZeroPos - 150;
//        }
//        liftMotor.setTargetPosition(y_ticks);
//        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftMotor.setPower(0.7);
//    }

    void toggleLift() {
        if (liftUp) {
            liftMotor.setPower(-0.3);
        } else {
            liftMotor.setPower(0.8);
        }
        wait(0.6);
        liftMotor.setPower(0);
        liftUp = !liftUp;
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
//        rotate(0);
        while (isTargetVisible()) {
            double dz = (getVuforiaZ() - 10);
            double dx = (getVuforiaX() - 2);
            leftPower = 0.07 * dz;
            rightPower = 0.07 * dz;
            strafePower = 0.05 * dx;
            updateBallDrive(true);
            telemetry.addData("vuforiaX", "" + dx);
            telemetry.addData("vuforiaZ", "" + dz);
            telemetry.update();
            if (Math.sqrt(dz * dz + dx * dx) < 1) {
                break;
            }
        }
        resetBallDrive();
        wait(0.1);
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