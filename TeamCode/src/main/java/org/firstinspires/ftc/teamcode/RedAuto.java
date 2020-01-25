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

@Autonomous(name="RedAuto", group="Linear Opmode")
public class RedAuto extends LinearOpMode {
    //Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        //Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap, telemetry, true);
        double wait = 0.4;
        int stoneConfig; // SkyStone rightmost (001 001) = 1; 2nd rightmost (010 010) = 2; 3rd rightmost (100 100) = 3

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();

        robot.wait(0.5);
        robot.move(20, 0, 0, wait);
        if (robot.isTargetVisible() && Math.abs(robot.getVuforiaX()) < 4) { // if SkyStone straight ahead
            stoneConfig = 3;
        } else { // SkyStone is to the right
            robot.move(0, 8, 0, wait);
            if (robot.isTargetVisible() && Math.abs(robot.getVuforiaX()) < 4) { // if SkyStone straight ahead
                stoneConfig = 2;
            } else { // SkyStone is to the right
                stoneConfig = 1;
                robot.move(0, 8, 0, wait);
            }
        }
//        robot.alignVuforia();
        robot.toggleIntake();
        robot.wait(wait);
        robot.move(10, 0, 0, wait); // runs into skystone (since the intake protrudes)
//        robot.liftHeight = 1;
//        robot.updateLift();

        robot.move(-5, 0, 0, wait);
        robot.rotate(-90); // rotates to face the bridge
        robot.wait(wait);
        robot.move(64 /*+ 8 * stoneConfig */, 0, -90, wait); // goes next to foundation
        robot.rotate(0); // faces foundation
        robot.speed = 0.15;
        robot.move(6, 0, 0, wait); // one inch over shoot into foundation
//        robot.toggleHook();
//        robot.toggleIntake();
        robot.speed = 0.5;
        robot.move(-33, 0, 0, wait); // backs into wall
        robot.toggleHook();
        robot.speed = 1;
        robot.wait(0.1);
        robot.move(1, -30, 0, 0);
//        robot.liftHeight = 0;
//        robot.updateLift();
        robot.move(22, 0, 0, wait);
        if (stoneConfig != 3) { // code for second stone
            robot.move(0, -(58 + 8 * stoneConfig), 0, wait); // supposedly 0 inches short of second skystone
            robot.move(-3, 0, 0, 0);
            if (robot.isTargetVisible()) {
                robot.alignVuforia();
            }
            robot.toggleIntake();
            robot.wait(wait);
            robot.move(10, 0, 0, wait); // runs into skystone (since the intake protrudes)
//        robot.liftHeight = 1;
//        robot.updateLift();

            robot.move(-5, 0, 0, wait);
            robot.rotate(-90); // rotates to face the bridge
            robot.wait(wait);
            robot.move(30, 0, -90, 0);
            robot.toggleIntake();
        }
    }
}

