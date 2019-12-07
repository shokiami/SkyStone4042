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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleTest", group="Iterative Opmode")
public class TeleTest extends OpMode
{
    //Declare OpMode members
    Robot robot;
    Controller controller1;
    Vuforia vuforia;
    ElapsedTime runtime;
    double Kp;
    double Ki;
    double Kd;
    final double TICKS_PER_REVOLUTION = 3892;
    final double WHEEL_DIAMETER = 4;
    final double ROBOT_DIAMETER = 15;


    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        controller1 = new Controller(gamepad1);
        vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        runtime = new ElapsedTime();
        Kp = 10;
        Ki = 0;
        Kd = 0;
        telemetry.addData("Status", "Initialized");
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        controller1.update();
        vuforia.update();

        if (controller1.y.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Kp += 1;
        }
        if (controller1.y.equals("pressed") && controller1.dpad_down.equals("pressing") && (Kp > 0)) {
            Kp -= 1;
        }
        if (controller1.b.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Ki += 1;
        }
        if (controller1.b.equals("pressed") && controller1.dpad_down.equals("pressing") && (Ki > 0)) {
            Ki -= 1;
        }
        if (controller1.a.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Kd += 1;
        }
        if (controller1.a.equals("pressed") && controller1.dpad_down.equals("pressing") && (Kd > 0)) {
            Kd -= 1;
        }

        robot.leftDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
        robot.rightDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);
        robot.strafeDrive.setVelocityPIDFCoefficients(Kp, Ki, Kd, 0);

        if (vuforia.isTargetVisible() && ((vuforia.getX() != 5 || vuforia.getY() != 0) || vuforia.getHeading() != 0)) {
            int left_ticks = robot.leftDrive.getCurrentPosition();
            int right_ticks = robot.rightDrive.getCurrentPosition();
            int strafe_ticks = robot.strafeDrive.getCurrentPosition();
            if (vuforia.getHeading() != 0) {
                left_ticks += Math.PI * ROBOT_DIAMETER * vuforia.getHeading();
                right_ticks -= Math.PI * ROBOT_DIAMETER * vuforia.getHeading();
            } else {
                left_ticks += vuforia.getX() / (Math.PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
                right_ticks += vuforia.getX() / (Math.PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
                strafe_ticks += vuforia.getY() / (Math.PI * WHEEL_DIAMETER) * TICKS_PER_REVOLUTION;
            }
            robot.leftDrive.setTargetPosition(left_ticks);
            robot.rightDrive.setTargetPosition(right_ticks);
            robot.rightDrive.setTargetPosition(strafe_ticks);
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.strafeDrive.setPower(0);
        }

        telemetry.addData("Kp:", "" + Kp);
        telemetry.addData("Ki:", "" + Ki);
        telemetry.addData("Kd:", "" + Kd);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
