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
        telemetry.addData("Status", "Initialized");

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();
        robot.toggleHook();
        robot.wait(0.5);
        robot.toggleLift();
        robot.toggleLift();

        //Aight, here we go!

        //Locating SkyStone configuration
        int stoneConfig; //(001 001) = 0; (010 010) = 1; (100 100) = 2
        robot.move(18, 0);
        robot.wait(0.5);
        if (robot.isTargetVisible()) {
            stoneConfig = 1;
        } else {
            stoneConfig = 0;
            robot.move(0, 8);
        }

        //Sucking SkyStone 1
        robot.toggleIntake();
        robot.move(10, 0);
        robot.move(-12, 0);

        //Traversing Map
        robot.move(0, 104 + 8 * stoneConfig);

        //Dropping off SkyStone 1
        robot.toggleLift(); // lift up
        robot.speed = 0.7;
        robot.move(13, 0);
        robot.toggleIntake();
        robot.toggleHook(); // hook down
        robot.wait(0.4);

        //Pulling foundation back into build site
        robot.speed = 1.0;
        robot.move(-32, 0);
        robot.toggleHook(); //hook up

        //Parking under the bridge
        robot.move(1, -30);
        robot.toggleLift();
        robot.move(1, -22);

        //Nice!

        /*

        //Traversing Map
        robot.move(0, -64 - 8 * stoneConfig);

        //Sucking SkyStone 2
        robot.toggleIntake();
        robot.move(11, 0);
        robot.move(-11, 0);

        //Traversing Map
        robot.move(0, 64 + 8 * stoneConfig);

        //Dropping off SkyStone 2
        robot.speed = 0.2;
        robot.toggleLift();
        robot.toggleIntake();
        robot.move(8, 0);

        */
    }
}

