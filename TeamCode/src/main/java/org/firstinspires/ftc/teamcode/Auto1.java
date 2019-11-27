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
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Auto1", group="Linear Opmode")
public class Auto1 extends LinearOpMode {
    //Declare OpMode members
    Robot robot;
    Vuforia vuforia;
    ElapsedTime runtime;
    Pid pidX;
    Pid pidY;
    Pid pidHeading;

    @Override
    public void runOpMode() {
        //Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap);
        vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        runtime = new ElapsedTime();
        pidX = new Pid(0.1,0,0);
        pidY = new Pid(0.1,0,0);
        pidHeading = new Pid(0.1,0,0);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        runtime.reset();
        vuforia.flashlight(true);

        //Code to run REPEATEDLY until time runs out
        while (opModeIsActive()) {

            if (vuforia.isTargetVisible()) {
                double leftPower = pidX.getPower(10 + vuforia.getX())
                        + 0.5 * pidHeading.getPower(vuforia.getHeading());
                double rightPower = pidX.getPower(10 + vuforia.getX())
                        - 0.5 * pidHeading.getPower(vuforia.getHeading());
                double strafePower = pidY.getPower(2 - vuforia.getY());

                robot.leftPower = Range.clip(leftPower,-1.0, 1.0);
                robot.rightPower = Range.clip(rightPower,-1.0, 1.0);
                robot.strafePower = Range.clip(strafePower,-1.0, 1.0);
            } else {
                robot.leftPower = 0;
                robot.rightPower = 0;
                robot.strafePower = 0;
            }

            vuforia.update();
            robot.update();
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.update();
        }
        vuforia.close();
    }
/*
X: depth displacement (further = more negative)
Y: horizontal displacement (right = positive)
Heading: horizontal rotation (right = positive)

goal x=-10 y=2 heading = 0
 */

