/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Tele1", group="Iterative Opmode")
public class Tele1 extends OpMode
{
    // Declare OpMode members
    private Robot robot = new Robot(hardwareMap);
    private ElapsedTime runtime = new ElapsedTime();
    private float rightTriggerCheck;
    private boolean leftBumperCheck;
    private boolean xCheck;
    private boolean aCheck;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        rightTriggerCheck = 0;
        xCheck = false;
        aCheck = false;
        leftBumperCheck = false;
        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        //Ball Drive
        robot.leftPower = gamepad1.left_stick_y;
        robot.rightPower = gamepad1.right_stick_y;
        robot.strafePower = 0.5 * gamepad1.right_stick_x + 0.5 * gamepad1.left_stick_x;

        //Speed
        if (gamepad1.x && !xCheck) {
            robot.toggleSpeed();
        }
        xCheck = gamepad1.x;

        //Lift
        if (gamepad1.dpad_up) {
            robot.liftPower = 1;
        } else if (gamepad1.dpad_down) {
            robot.liftPower = -1;
        } else {
            robot.liftPower = 0;
        }

        //Intake
        if (gamepad1.right_trigger != 0 && rightTriggerCheck == 0) {
            robot.toggleIntake();
        }
        rightTriggerCheck = gamepad1.right_trigger;

        if (gamepad1.y) {
            robot.intakeAngle += 0.01;
        }
        if (gamepad1.b) {
            robot.intakeAngle -= 0.01;
        }

        //Hook Servos
        if (gamepad1.a && !aCheck) {
            robot.toggleHook();
        }
        aCheck = gamepad1.a;

        //Valve Servos
        if (gamepad1.left_bumper && !leftBumperCheck) {
            robot.toggleValve();
        }
        leftBumperCheck = gamepad1.left_bumper;

        robot.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
