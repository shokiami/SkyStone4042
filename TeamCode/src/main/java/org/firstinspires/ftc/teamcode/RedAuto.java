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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedAuto", group="Linear Opmode")
public class RedAuto extends LinearOpMode {
    //Declare OpMode members
    Robot robot;
    ElapsedTime runtime;
    Vuforia vuforia;
    boolean run;

    void move(int z_inches, int x_inches) {
        robot.move(z_inches, x_inches);
    }

    void wait(double waitTime) {
        double start = runtime.seconds();
        while (runtime.seconds() - start < waitTime) {}
    }

    @Override
    public void runOpMode() {
        //Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap);
        robot.update();
        runtime = new ElapsedTime();
        vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        run = true;

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
        vuforia.flashlight(true);
        robot.toggleIntakeAngle();
        robot.update();

        while (opModeIsActive()) {
            if (run) {
                wait(0.5);
                move(20, 0);
                wait(0.5);
                int disp = 0;
                while (disp > -16) {
                    vuforia.update();
                    if (vuforia.isTargetVisible()) {
                        break;
                    }
                    disp -= 8;
                    move(0, -8);
                    wait(0.5);
                }
                robot.toggleIntake();
                robot.update();
                wait(0.5);
                move(8, 0);
                wait(0.5);
                robot.liftPower = 1;
                robot.update();
                wait(0.5);
                robot.liftPower = 0;
                robot.update();
                wait(0.5);
                move(-12, 0);
                wait(0.5);
                move(0, 77 - disp);
                wait(0.5);
                robot.toggleSpeed();
                robot.update();
                wait(0.5);
                move(20, 0);
                wait(0.4);
                robot.toggleIntake();
                robot.toggleHook();
                robot.toggleSpeed();
                robot.update();
                wait(5.0);
                move(-30, 0);
                wait(0.5);
                robot.toggleHook();
                robot.update();
                wait(0.5);
                move(0, -60);
                wait(0.5);
                robot.liftPower = -1;
                robot.update();
                wait(0.3);
                robot.liftPower = 0;
                run = false;
            }
        }
    }
}

