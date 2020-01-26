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
        int stoneConfig; // SkyStone rightmost (001 001) = 1; 2nd rightmost (010 010) = 2; 3rd rightmost (100 100) = 3

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();
        robot.toggleLift();
        robot.toggleLift();

        //Ok, here we go
        robot.move(20, 0);
        if (robot.isTargetVisible()) { // if SkyStone straight ahead
            stoneConfig = 3;
        } else { // SkyStone is to the right
            robot.move(0, 8);
            if (robot.isTargetVisible()) { // if SkyStone straight ahead
                stoneConfig = 2;
            } else { // SkyStone is to the right
                robot.move(0, 8);
                stoneConfig = 1;
            }
        }
        robot.toggleIntake();
        robot.move(7, 0); // runs into skystone (since the intake protrudes)
        robot.move(-3, 0);
        robot.rotate(-90); // rotates to face the bridge
        robot.move(70, 0);
        robot.toggleLift();
        robot.move(8 * stoneConfig, 0); // goes next to foundation
        robot.rotate(0); // faces foundation
        robot.speed = 0.2;
        robot.move(8, 0); // one inch over shoot into foundation
        robot.toggleHook();
        robot.toggleIntake();
        robot.speed = 1;
        robot.move(-30, 0); // backs into wall
        robot.toggleHook();
        robot.move(1, -20);
        robot.toggleLift();
        robot.move(20, -10);
        robot.move(0, -23);

    }
}

