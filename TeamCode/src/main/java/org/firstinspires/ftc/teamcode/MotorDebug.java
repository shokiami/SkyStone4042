package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Motor Debug", group="Iterative Opmode")
public class MotorDebug extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private int lastLeftTicks = 0, lastRightTicks = 0;
    private ElapsedTime delta = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        leftPower  = -gamepad1.left_stick_y ;
        rightPower = -gamepad1.right_stick_y ;

        if (gamepad1.a)
            leftPower = rightPower = .1;
        if (gamepad1.b)
            leftPower = rightPower = .2;
        if (gamepad1.x)
            leftPower = rightPower = .5;
        if (gamepad1.y)
            leftPower = rightPower = 1.0;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Encoders", "left (%d), right (%d)", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Speeds", "left (%.2f), right (%.2f)",
                (leftDrive.getCurrentPosition() - lastLeftTicks) * delta.seconds(),
                (rightDrive.getCurrentPosition() - lastRightTicks) * delta.seconds());
        while (delta.seconds() < .01) {
            try {
                Thread.sleep(1);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        delta.reset();
        lastLeftTicks = leftDrive.getCurrentPosition();
        lastRightTicks = rightDrive.getCurrentPosition();
    }

    @Override
    public void stop() {
    }

}
