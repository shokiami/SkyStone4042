package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
    double liftHeight = 0;

    //Virtual coordinate plane
    double x = 0; // perpendicular to bridge, 0 to 144 in (or 144 - 18 inches due to robot's length)
    double z = 0; // parallel axis to bridge, 0 to 144 in
    int prevPosLeft = 0;
    int prevPosRight = 0;
    int prevPosStrafe = 0;

    static final double Z_TICKS_PER_INCH = 49.606;
    static final double X_TICKS_PER_INCH = 58.504;
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

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        strafeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        updateBallDrive();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void updateBallDrive() {
        leftDrive.setPower(Range.clip(speed * leftPower,-1.0, 1.0));
        rightDrive.setPower(Range.clip(speed * rightPower,-1.0, 1.0));
        strafeDrive.setPower(Range.clip(speed * strafePower,-1.0, 1.0));
        updateXZ();
    }

    void move(double z_inches, double x_inches) {
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strafeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        while (Math.abs(leftDrive.getCurrentPosition() - z_ticks) > 5 || Math.abs(rightDrive.getCurrentPosition() - z_ticks) > 5 || Math.abs(strafeDrive.getCurrentPosition() - x_ticks) > 5) {
            //Wait
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        strafeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void rotate(double angle) {
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int left_ticks = (int)(angle / 180 * Math.PI * TURN_RADIUS * Z_TICKS_PER_INCH);
        int right_ticks = -(int)(angle / 180 * Math.PI * TURN_RADIUS * Z_TICKS_PER_INCH);
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + left_ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + right_ticks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(1);
        rightDrive.setPower(1);
        while (Math.abs(leftDrive.getCurrentPosition() - left_ticks) > 5 || Math.abs(rightDrive.getCurrentPosition() - right_ticks) > 5) {
            //Wait
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    void tuneLift(double height) {
        liftHeight += height;
        updateLift();
        liftHeight = 0;
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    void updateXZ() {
        double distForward = ((leftDrive.getCurrentPosition() - prevPosLeft) + (rightDrive.getCurrentPosition() - prevPosRight)) / 2 / Z_TICKS_PER_INCH;
//        double distLeftFoward = 0;
//        double distRightFoward = 0;
        double distSideways = (strafeDrive.getCurrentPosition() - prevPosStrafe) / X_TICKS_PER_INCH;
        x += (-1) * Math.sin(getGyroAngle()) * distForward + Math.cos(getGyroAngle()) * distSideways;
        z += Math.cos(getGyroAngle()) * distForward + Math.sin(getGyroAngle()) * distSideways;
        prevPosLeft = leftDrive.getCurrentPosition();
        prevPosRight = rightDrive.getCurrentPosition();
        prevPosStrafe = strafeDrive.getCurrentPosition();
    }

    void moveToXZ(double tX, double tZ){
        double iX = x; //initial x
        double iZ = z; //initial z
        double xTT = tX - x; //x to target
        double zTT = tZ - z; //z to target
        double iD = Math.sqrt(xTT * xTT + zTT * zTT); //initial distance to target
        double iTheta = 90 - getGyroAngle() - Math.atan2(zTT, xTT); //initial angle to target
        double iRX = Math.sin(iTheta) * iD; //initial rotated x
        double iRZ = Math.cos(iTheta) * iD; //initial rotated z
        double dTT = iD; //distance to target (changeable)
        double prevDTT = iD; //previous distance to target (for comparison to previous iteration)
        double thetaTT = iTheta; //angle to target (changeable)
        double tuning = 0.01; //value for automatic fine-tuning

        //z = mx + b
        double m = (tZ - iZ) / (tX - iX);
        //double b = iZ - m * iX;
        double currentSlopeFromInit = m;

        if (thetaTT > 0) {
            leftPower = Math.cos(thetaTT) * Math.cos(thetaTT) * 0.5;
            rightPower = Math.cos(thetaTT) * Math.cos(thetaTT) * 0.5;
            strafePower = Math.sin(thetaTT) * Math.sin(thetaTT) * 0.5;
        } else {
            leftPower = Math.cos(thetaTT) * Math.cos(thetaTT) * 0.5;
            rightPower = Math.cos(thetaTT) * Math.cos(thetaTT) * 0.5;
            strafePower = (-1) * Math.sin(thetaTT) * Math.sin(thetaTT) * 0.5;
        }

        while (dTT > 3 && dTT <= prevDTT) { //consider: dTT > 3 && dTT <= prevDTT
            xTT = tX - x;
            zTT = tZ - z;
            prevDTT = dTT;
            dTT = Math.sqrt(xTT * xTT + zTT * zTT);
            thetaTT = 90 - getGyroAngle() - Math.atan2(zTT, xTT);
            currentSlopeFromInit = (z - iZ) / (x - iX);

            if (0 <= thetaTT && thetaTT <= 90) {
                if (currentSlopeFromInit > m) {
                    leftPower -= tuning;
                    rightPower -= tuning;
                    strafePower += tuning;
                } else {
                    leftPower += tuning;
                    rightPower += tuning;
                    strafePower -= tuning;
                }
            } else if (-90 <= thetaTT && thetaTT <= 0) {
                if (currentSlopeFromInit > m) {
                    leftPower += tuning;
                    rightPower += tuning;
                    strafePower -= tuning;
                } else {
                    leftPower -= tuning;
                    rightPower -= tuning;
                    strafePower += tuning;
                }
            }
            updateBallDrive();
            updateXZ();
        }

        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        updateBallDrive();
    }
}

//https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html