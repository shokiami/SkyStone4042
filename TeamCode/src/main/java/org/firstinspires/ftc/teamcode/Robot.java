package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    double hookAngle = 0.25;
    double valveAngle = 0;
    double speed = 1;
    double targetAngle = 0;
    boolean liftUp = false;

    static final double Z_TICKS_PER_INCH = 54.000;
    static final double X_TICKS_PER_INCH = 59.529; //
    static final double TURN_RADIUS = 8.493;
    static final double Y_TICKS_PER_INCH = 50; //415.0

    double minPowerZ = 0;
    double minPowerX = 0;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor strafeDrive;
    DcMotor liftMotor;
    DcMotor intakeMotor;
    DcMotor leftDriveEncoder;
    DcMotor rightDriveEncoder;
    
    
    Servo intakeServo;
    Servo hookServo1;
    Servo hookServo2;
    Servo valveServo;

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
        leftDriveEncoder = hardwareMap.get(DcMotor.class, "left_encoder");
        rightDriveEncoder = hardwareMap.get(DcMotor.class, "right_encoder");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        strafeDrive.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);
        hookServo1.setDirection(Servo.Direction.REVERSE);
        hookServo2.setDirection(Servo.Direction.FORWARD);
        valveServo.setDirection(Servo.Direction.FORWARD);
        leftDriveEncoder.setDirection(DcMotor.Direction.REVERSE);
        rightDriveEncoder.setDirection(DcMotor.Direction.FORWARD);

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
        calibrateMinPower();
    }

    void calibrateMinPower() {
        resetBallDrive();
        while(rightDriveEncoder.getCurrentPosition() < 10) {
            minPowerZ += 0.002;
            leftPower = minPowerZ;
            rightPower = minPowerZ;
            updateBallDrive(false);
        }
        resetBallDrive();
        while(strafeDrive.getCurrentPosition() < 10) {
            minPowerX += 0.002;
            strafePower = minPowerX;
            updateBallDrive(false);
        }
        resetBallDrive();
    }

    void resetBallDrive() {
        leftDriveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        telemetry.addData("minPowerZ", "" + minPowerZ);
        telemetry.addData("minPowerX", "" + minPowerX);
        telemetry.addData("angles", "Gyro = " + getGyroAngle() + ", targetAngle = " + this.targetAngle);
        telemetry.addData("leftTicks", "" + leftDriveEncoder.getCurrentPosition());
        telemetry.addData("rightTicks", "" + rightDriveEncoder.getCurrentPosition());
        telemetry.addData("strafeTicks", "" + strafeDrive.getCurrentPosition());
        telemetry.addData("liftTicks", "" + liftMotor.getCurrentPosition());
        telemetry.addData( "leftPower", "" + leftDrive.getPower());
        telemetry.addData("rightPower", "" + rightDrive.getPower());
        telemetry.addData("strafePower", "" + strafeDrive.getPower());
        telemetry.update();
        double tuning = Range.clip(p + d, (-1) * (0.3 + 0.45 * speed) , 0.3 + 0.45 * speed);
        leftDrive.setPower(speed * leftPower - (targetAngle ? tuning : 0) + Math.signum(leftPower) * minPowerZ);
        rightDrive.setPower(speed * rightPower + (targetAngle ? tuning : 0) + Math.signum(rightPower) * minPowerZ);
        strafeDrive.setPower(speed * strafePower + Math.signum(strafePower) * minPowerX);
        lastError = error;
        delta.reset();
    }

    void move(double zInches, double xInches) {
        resetBallDrive();
        double targetZ = zInches * Z_TICKS_PER_INCH;
        double targetX = xInches * X_TICKS_PER_INCH;
        while (true) {
            double dz = targetZ - rightDriveEncoder.getCurrentPosition();
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
            liftMotor.setPower(1);
        }
        wait(0.4);
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
        if (hookAngle == 0.25) {
            hookAngle = 0.75;
        } else {
            hookAngle = 0.25;
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

    void waitUntilTargetVisible() {
        while (!isTargetVisible()) {}
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