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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TestTele", group="Iterative Opmode")
public class TestTele extends OpMode {
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    double zMove = 0.0013;
    double xMove = 0.001;
    double zRot = 0.03;
    double xRot = 0.01;

    void updateBallDrive(boolean targetAngle) {
        double error = robot.targetAngle - robot.getGyroAngle();
        double pCoeff = zRot; //0.03
        double dCoeff = 0.003; //0.003
        if (robot.strafeDrive.getPower() > 0.6){
            pCoeff = xRot;
            dCoeff = 0.002; // 0.002
        }
        double p = pCoeff * error;
        double d = dCoeff * (error - robot.lastError) / robot.delta.seconds();
        telemetry.update();
        double tuning = Range.clip(p + d, -0.75, 0.75);
        double slowTuning = Range.clip(tuning, -0.75 , 0.75);
        robot.leftDrive.setPower(robot.leftPower - (targetAngle ? slowTuning : 0));
        robot.rightDrive.setPower(robot.rightPower + (targetAngle ? slowTuning : 0));
        robot.strafeDrive.setPower(robot.strafePower);
        robot.lastError = error;
        robot.delta.reset();
    }

    void move(double zInches, double xInches, double wait) {
        robot.resetBallDrive();
        double targetZ = zInches * robot.Z_TICKS_PER_INCH;
        double targetX = xInches * robot.X_TICKS_PER_INCH;
        while (true) {
            double dz = targetZ - robot.rightDrive.getCurrentPosition();
            double dx = targetX - robot.strafeDrive.getCurrentPosition();
            robot.leftPower = zMove * dz;
            robot.rightPower = zMove * dz;
            robot.strafePower = xMove * dx;
            updateBallDrive(true);
            if (Math.sqrt(dz * dz + dx * dx) < 20) {
                break;
            }
        }
        robot.resetBallDrive();
        robot.wait(wait);
    }

    void rotate(double angle) {
        robot.targetAngle = angle;
        robot.resetBallDrive();
        while (Math.abs(angle - robot.getGyroAngle()) > 5) {
            updateBallDrive(true);
        }
        robot.resetBallDrive();
    }

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, false);
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

        //Adjust coefficients
        if (controller1.y.equals("pressing") && controller1.left_stick_y > 0) {
            zMove += 0.0001;
        }
        if (controller1.y.equals("pressing") && controller1.left_stick_y < 0) {
            zMove -= 0.0001;
        }
        if (controller1.x.equals("pressing") && controller1.left_stick_y > 0) {
            xMove += 0.0001;
        }
        if (controller1.x.equals("pressing") && controller1.left_stick_y < 0) {
            xMove -= 0.0001;
        }
        if (controller1.b.equals("pressing") && controller1.left_stick_y > 0) {
            zRot += 0.0025;
        }
        if (controller1.b.equals("pressing") && controller1.left_stick_y < 0) {
            zRot -= 0.0025;
        }
        if (controller1.a.equals("pressing") && controller1.left_stick_y > 0) {
            xRot += 0.0025;
        }
        if (controller1.a.equals("pressing") && controller1.left_stick_y < 0) {
            xRot -= 0.0025;
        }

        //Move
        if (controller1.dpad_up.equals("pressing")) {
            move(10, 0, 0);
        }
        if (controller1.dpad_down.equals("pressing")) {
            move(-10, 0, 0);
        }
        if (controller1.dpad_right.equals("pressing")) {
            move(0, 10, 0);
        }
        if (controller1.dpad_left.equals("pressing")) {
            move(0, -10, 0);
        }

        //Spin 180
        if (controller1.left_bumper.equals("pressing")) {
            if (robot.getGyroAngle() > 90) {
                rotate(0);
            } else {
                rotate(180);
            }
        }

        telemetry.addData("zMove", "" + zMove);
        telemetry.addData("xMove", "" + xMove);
        telemetry.addData("zRot", "" + zRot);
        telemetry.addData("xRot", "" + xRot);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop () {
    }
}
