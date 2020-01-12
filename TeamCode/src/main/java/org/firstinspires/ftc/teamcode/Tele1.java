package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Tele1", group="Iterative Opmode")
public class Tele1 extends OpMode {
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    Controller controller2;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        controller1.update();
        controller2.update();

        //Speed
        if (controller1.x.equals("pressing")) {
            robot.toggleSpeed();
        }

        //Ball Drive
        robot.leftPower = controller1.left_stick_y;
        robot.rightPower = controller1.right_stick_y;
        robot.strafePower = 0.5 * controller1.right_stick_x + 0.5 * controller1.left_stick_x;
        robot.updateBallDrive();

        //Lift
        if (controller2.dpad_right.equals("pressed")) {
            robot.tuneLift(0.002);
        }
        if (controller2.dpad_left.equals("pressed")) {
            robot.tuneLift(-0.002);
        }
        if (controller2.dpad_up.equals("pressing") && robot.liftHeight < 6) {
            robot.liftHeight += 1;
            robot.updateLift();
        } else if (controller2.dpad_down.equals("pressing") && robot.liftHeight > 0) {
            robot.liftHeight -= 1;
            robot.updateLift();
        }

        //Intake
        if (controller2.right_bumper.equals("pressing")) {
            robot.toggleIntake();
        }

        //Hook
        if (controller1.a.equals("pressing")) {
            robot.toggleHook();
        }

        //Valve (not in use)
        if (controller1.left_bumper.equals("pressing")) {
            robot.toggleValve();
        }

        //Spin 180
        if (controller1.left_bumper.equals("pressing")) {
            robot.rotate(180);
        }

        telemetry.addData("liftMotor", "" + robot.liftMotor.getCurrentPosition());
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop () {
    }
}