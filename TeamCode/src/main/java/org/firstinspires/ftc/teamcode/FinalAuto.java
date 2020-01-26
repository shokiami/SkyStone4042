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

@Autonomous(name="FinalAuto", group="Linear Opmode")
public class FinalAuto extends LinearOpMode {
    //Declare OpMode members
    Robot robot;

    @Override
    public void runOpMode() {
        //Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap, telemetry, true);
        double wait = 0.05;
        int stoneConfig = 0; // SkyStone rightmost (001 001) = 1; 2nd rightmost (010 010) = 2; 3rd rightmost (100 100) = 3

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();

        robot.toggleLift();
        robot.toggleLift();
        robot.move(15, 0, 0, 1.);
        int count = 0;
        while(!robot.isTargetVisible() && count < 20) {
            robot.move(0, 1, 0, .5);
            count++;
        }
//        robot.alignVuforia();
        robot.toggleIntake();
        robot.wait(wait);
        robot.move(4, 0, 0, 1.); // runs into skystone (since the intake protrudes)

        robot.move(-6, 0, 0, wait);
        robot.rotate(-95); // rotates to face the bridge
        robot.wait(wait);
        robot.move(40, 0, -90, 0);
        robot.toggleLift();
        robot.move(8 * stoneConfig, 0, -90, wait); // goes next to foundation
        robot.rotate(0); // faces foundation
        robot.speed = 0.15;
        robot.move(8, 0, 0, wait); // one inch over shoot into foundation
        robot.toggleHook();
        robot.toggleIntake();
        robot.speed = 0.5;
        robot.move(-26, 0, 0, wait); // backs into wall
        robot.toggleHook();
        robot.speed = 1;
        robot.wait(0.1);
        robot.move(1, -15, 0, 0);
        robot.toggleLift();
        robot.move(20, 0, 0, wait);
        robot.move(0, -13, 0, wait);

//        robot.move(4, 0, 0, wait);
//        if (stoneConfig != 3) { // code for second stone
//            robot.move(0, -(26 + 8 * stoneConfig), 0, wait); // supposedly 0 inches short of second skystone
//            robot.move(-3, 0, 0, 0);
//            if (robot.isTargetVisible()) {
//                robot.alignVuforia();
//            }
//            robot.toggleIntake();
//            robot.wait(wait);
//            robot.move(10, 0, 0, wait); // runs into skystone (since the intake protrudes)
//            robot.toggleLift();
//
//            robot.move(-5, 0, 0, wait);
//            robot.rotate(-90); // rotates to face the bridge
//            robot.wait(wait);
//            robot.move(30, 0, -90, 0);
//            robot.toggleIntake();
//        }
    }
}

