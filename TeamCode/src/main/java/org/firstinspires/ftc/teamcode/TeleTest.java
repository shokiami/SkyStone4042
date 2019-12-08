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
    Pid pidX;
    Pid pidY;
    Pid pidHeading;
    double Kp;
    double Ki;
    double Kd;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        controller1 = new Controller(gamepad1);
        vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        runtime = new ElapsedTime();
        pidX = new Pid(0.1, 0, 0);
        pidY = new Pid(0.1, 0, 0);
        pidHeading = new Pid(0.1, 0, 0);
        Kp = 0;
        Ki = 0;
        Kd = 0;
        runtime = new ElapsedTime();

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
        vuforia.flashlight(true);
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        controller1.update();

        if (controller1.y.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Kp += 0.01;
        }
        if (controller1.y.equals("pressed") && controller1.dpad_down.equals("pressing")) {
            Kp -= 0.01;
        }
        if (controller1.b.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Ki += 0.01;
        }
        if (controller1.b.equals("pressed") && controller1.dpad_down.equals("pressing")) {
            Ki -= 0.01;
        }
        if (controller1.a.equals("pressed") && controller1.dpad_up.equals("pressing")) {
            Kd += 0.01;
        }
        if (controller1.a.equals("pressed") && controller1.dpad_down.equals("pressing")) {
            Kd -= 0.01;
        }

        pidX.update(Kp, Ki, Kd);
        pidY.update(Kp, Ki, Kd);
        pidHeading.update(Kp, Ki, Kd);

        if (vuforia.isTargetVisible()) {
            double leftPower = pidX.getPower(-5 - vuforia.getX()) + 0.5 * pidHeading.getPower(vuforia.getHeading());
            double rightPower = pidX.getPower(-5 - vuforia.getX()) - 0.5 * pidHeading.getPower(vuforia.getHeading());
            double strafePower = pidY.getPower(vuforia.getY() - 2);

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
        telemetry.addData("Kp:", "" + Kp);
        telemetry.addData("Ki:", "" + Ki);
        telemetry.addData("Kf:", "" + Kd);
        telemetry.addData("left_power", "" + robot.leftPower);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}