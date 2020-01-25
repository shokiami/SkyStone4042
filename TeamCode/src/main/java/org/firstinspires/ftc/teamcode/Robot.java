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
    double liftHeight = 0;
    double targetAngle = 0;

    static final double Z_TICKS_PER_INCH = 54.000;
    static final double X_TICKS_PER_INCH = 59.529;
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

    ElapsedTime delta = new ElapsedTime();
    double lastError;
    Telemetry telemetry;

    Robot(HardwareMap hardwareMap, boolean vuforia) {
        this(hardwareMap, null, vuforia);
    }

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

        this.telemetry = telemetry;

        gyro = new Gyro(hardwareMap);
        resetGyro();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        if (vuforia) {
            this.vuforia = new Vuforia(hardwareMap);
            this.vuforia.flashlight(true);
        }

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
        double pCoeff = 0.02; //0.02
        double dCoeff = 0.003; //0.001
        if (strafeDrive.getPower() > 0.6){
            pCoeff = 0.01;
            dCoeff = 0.002; // 0.002
        }
        double p = pCoeff * error;
        double d = dCoeff * (error - lastError) / delta.seconds();
        telemetry.addData("pd","p = " + p + ", d = " + d);
        telemetry.addData("angles", "Gyro = " + getGyroAngle() + ", tAngle = " + targetAngle);
        telemetry.addData("powers", "leftPow = " + leftPower + ", rightPow = " + rightPower);
        double tuning = Range.clip(p + d, -0.75, 0.75);
        double slowTuning = Range.clip(tuning, (-1) * (0.3 + 0.45 * speed) , 0.3 + 0.45 * speed);
        leftDrive.setPower(speed * leftPower - (targetAngle ? slowTuning : 0));
        rightDrive.setPower(speed * rightPower + (targetAngle ? slowTuning : 0));
        strafeDrive.setPower(speed * strafePower);
        lastError = error;
        delta.reset();
    }

    void moveOld(double zInches, double xInches, double wait) {
        resetBallDrive();
        double targetZ = zInches * Z_TICKS_PER_INCH;
        double targetX = xInches * X_TICKS_PER_INCH;
        leftDrive.setTargetPosition((int)targetZ);
        rightDrive.setTargetPosition((int)targetZ);
        strafeDrive.setTargetPosition((int)targetX);
        leftPower = 1;
        rightPower = 1;
        strafePower = 1;
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (true) {
            updateBallDrive(true);
            double dz = targetZ - (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition()) / 2;
            double dx = targetX - strafeDrive.getCurrentPosition();
            if (Math.sqrt(dz * dz + dx * dx) < 10) {
                break;
            }
        }
        resetBallDrive();
        wait(wait);
    }

    void move(double zInches, double xInches, double wait) {
        double targetZ = zInches * Z_TICKS_PER_INCH + (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition()) / 2;
        double targetX = xInches * X_TICKS_PER_INCH + strafeDrive.getCurrentPosition();
        while (true) {
            double dz = targetZ - (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition()) / 2;
            double dx = targetX - strafeDrive.getCurrentPosition();
            leftPower = 0.002 * dz;
            rightPower = 0.002 * dz;
            strafePower = 0.002 * dx;
            updateBallDrive(true);
            if (Math.sqrt(dz * dz + dx * dx) < 10) {
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
        while (Math.abs(angle - getGyroAngle()) > 1) {
            updateBallDrive(true);
        }
        resetBallDrive();
    }

    void resetLift() {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!touchSensor.isPressed()) {
            liftMotor.setPower(-0.2);
        }
        while (touchSensor.isPressed()) {
            liftMotor.setPower(0.2);
        }
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftHeight = 0;
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
        while (true) {
            double dz = (getVuforiaZ() - 10);
            double dx = (getVuforiaX());
            leftPower = 0.05 * dz;
            rightPower = 0.05 * dz;
            strafePower = 0.02 * dx;
            updateBallDrive(true);
            telemetry.addData("vuforiaX", "" + dx);
            telemetry.addData("vuforiaZ", "" + dz);
            if (Math.sqrt(dz * dz + dx * dx) < 1) {
                break;
            }
        }
        resetBallDrive();
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