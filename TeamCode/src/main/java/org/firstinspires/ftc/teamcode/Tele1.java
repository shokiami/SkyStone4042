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

@TeleOp(name="Tele1", group="Iterative Opmode")
public class Tele1 extends OpMode {
    //Declare OpMode members
    Robot robot;
    Controller controller1;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, false);
        controller1 = new Controller(gamepad1);
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

        //Speed
        if (controller1.x.equals("pressing")) {
            robot.toggleSpeed();
        }

        //Ball Drive
        robot.leftPower = controller1.left_stick_y;
        robot.rightPower = controller1.right_stick_y;
        robot.strafePower = 0.5 * controller1.right_stick_x + 0.5 * controller1.left_stick_x;

        //Lift
        if (controller1.dpad_up.equals("pressing")) {
            robot.lift(4);
        } else if (controller1.dpad_down.equals("pressing")) {
            robot.lift(-4);
        }

        //Intake
        if (controller1.right_bumper.equals("pressing")) {
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

        robot.update();
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop () {
    }
}
