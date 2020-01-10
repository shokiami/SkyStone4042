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

    public void updateXZ(double dTT) {
        double distForward = ((robot.leftDrive.getCurrentPosition() - prevPosLeft) + (robot.rightDrive.getCurrentPosition() - prevPosRight)) / 2 / robot.Z_TICKS_PER_INCH;
        double distSideways = (robot.strafeDrive.getCurrentPosition() - prevPosStrafe) / robot.X_TICKS_PER_INCH;
        x += (-1) * Math.sin(robot.getGyroAngle()) * distForward + Math.cos(robot.getGyroAngle()) * distSideways;
        z += Math.cos(robot.getGyroAngle()) * distForward + Math.sin(robot.getGyroAngle()) * distSideways;
        prevPosLeft = robot.leftDrive.getCurrentPosition();
        prevPosRight = robot.rightDrive.getCurrentPosition();
        prevPosStrafe = robot.strafeDrive.getCurrentPosition();
        telemetry.addData("Coordinates: ", "x = " + x + ", z = " + z);
        telemetry.addData("Dist to Target: ", "dTT = " + dTT);
        telemetry.addData("Gyro: ", "angle = " + robot.getGyroAngle());
        telemetry.addData("Increment Distances: ", "dForward = " + distForward + ", dSideways = " + distSideways);
        telemetry.addData("Powers: ", "left = " + robot.leftPower + ", right = " + robot.rightPower + ", strafe = " + robot.strafePower);
        telemetry.addData("Positions: ", "left = " + robot.leftDrive.getCurrentPosition() + ", right = " + robot.rightDrive.getCurrentPosition() + ", strafe = " + robot.strafeDrive.getCurrentPosition());
        telemetry.update();
    }

    public void moveToXZ(double target_x, double target_z) {
        double iX = x; //initial x
        double iZ = z; //initial z
        double xTT = target_x - x; //x to target
        double zTT = target_z - z; //z to target
        double iD = Math.sqrt(xTT * xTT + zTT * zTT); //initial distance to target
        double iTheta = 90 - robot.getGyroAngle() - Math.atan2(zTT, xTT); //initial angle to target
        double iRX = Math.sin(iTheta) * iD; //initial rotated x
        double iRZ = Math.cos(iTheta) * iD; //initial rotated z
        double dTT = iD; //distance to target (changeable)
        double thetaTT = iTheta; //angle to target (changeable)
        double tuning = 0.01; //value for automatic fine-tuning

        //z = mx + b
        double m = (target_z - iZ) / (target_x - iX);
        //double b = iZ - m * iX;
        double currentSlopeFromInit = m;

        if (thetaTT > 0) {
            robot.leftPower = 0.5;
            robot.rightPower = 0.5;
            robot.strafePower = 0.5;
        } else {
            robot.leftPower = 0.5;
            robot.rightPower = 0.5;
            robot.strafePower = -0.5;
        }

        while (dTT > 0.1) {
            xTT = target_x - x;
            zTT = target_z - z;
            dTT = Math.sqrt(xTT * xTT + zTT * zTT);
            thetaTT = 90 - robot.getGyroAngle() - Math.atan2(zTT, xTT);
            currentSlopeFromInit = (z - iZ) / (x - iX);

            if (0 <= thetaTT && thetaTT <= 90) {
                if (currentSlopeFromInit > m) {
                    robot.leftPower -= tuning;
                    robot.rightPower -= tuning;
                    robot.strafePower += tuning;
                } else {
                    robot.leftPower += tuning;
                    robot.rightPower += tuning;
                    robot.strafePower -= tuning;
                }
            } else if (-90 <= thetaTT && thetaTT <= 0) {
                if (currentSlopeFromInit > m) {
                    robot.leftPower += tuning;
                    robot.rightPower += tuning;
                    robot.strafePower -= tuning;
                } else {
                    robot.leftPower -= tuning;
                    robot.rightPower -= tuning;
                    robot.strafePower += tuning;
                }
            }
            robot.updateBallDrive();
            updateXZ(dTT);
        }
    }
}