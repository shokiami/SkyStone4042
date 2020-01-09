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

@Autonomous(name="XYAutoTest", group="Linear Opmode")
public class XYAutoTest extends LinearOpMode {
    //Declare OpMode members
    Robot robot;
    double x; // perpendicular to bridge, 0 to 144 in (or 144 - 18 inches due to robot's length)
    double z; // parallel axis to bridge, 0 to 144 in
    int prevPosLeft = 0;
    int prevPosRight = 0;
    int prevPosStrafe = 0;

    @Override
    public void runOpMode() {
        //Code to run ONCE when the driver hits INIT
        robot = new Robot(hardwareMap, true);
        telemetry.addData("Status", "Initialized");
        x = 0;
        z = 0;

        waitForStart();

        //Code to run ONCE when the driver hits PLAY
        robot.resetElapsedTime();
        robot.toggleIntakeAngle();

        //Let's go!
        moveToXZ(18, 20);
    }

    public void updateXZ() {
        x += Math.sin(robot.getGyroAngle()) * (((robot.leftDrive.getCurrentPosition() - prevPosLeft) + (robot.rightDrive.getCurrentPosition() - prevPosRight))/2) / robot.X_TICKS_PER_INCH + Math.sin(robot.getGyroAngle() + 90) * (robot.strafeDrive.getCurrentPosition() - prevPosStrafe) / robot.Z_TICKS_PER_INCH;
        z += Math.cos(robot.getGyroAngle()) * (((robot.leftDrive.getCurrentPosition() - prevPosLeft) + (robot.rightDrive.getCurrentPosition() - prevPosRight))/2) / robot.X_TICKS_PER_INCH + Math.cos(robot.getGyroAngle() + 90) * (robot.strafeDrive.getCurrentPosition() - prevPosStrafe) / robot.Z_TICKS_PER_INCH;
        prevPosLeft = robot.leftDrive.getCurrentPosition();
        prevPosRight = robot.rightDrive.getCurrentPosition();
        prevPosStrafe = robot.strafeDrive.getCurrentPosition();
    }

    public void moveToXZ(double target_x, double target_z) {
        double initialX = x;
        double initialZ = z;
        double xToTarget = target_x - x;
        double zToTarget = target_z - z;
        double initialAngleToTarget = 90 - robot.getGyroAngle() - Math.atan2(zToTarget, xToTarget);
        robot.leftPower = 0.5;
        robot.rightPower = 0.5;
        robot.strafePower = 0.5;

        while (Math.abs(target_x - x) > 0.1 || Math.abs(target_z - z) > 0.1) {
            xToTarget = target_x - x;
            zToTarget = target_z - z;
            if (true) {}
            updateXZ();
        }
    }
}

