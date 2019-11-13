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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Auto1", group="Linear Opmode")
public class Auto1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        double leftPower = 0;
        double rightPower = 0;
        double strafePower = 0;
        double liftPower = 0;
        double intakeAngle = 0;
        boolean intake = false;
        double hookAngle = 0;
        Vuforia vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());
        Config config = new Config(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();
        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();
        vuforia.flashlight(true);

        while (opModeIsActive()) {
            if (!vuforia.isTargetStone()) {
                leftPower = 1;
                rightPower = -1;
            } else {
                leftPower = 1;
                rightPower = 1;
                if (vuforia.getY() < 0) {
                    rightPower -= 0.5;
                }
                if (vuforia.getY() > 0) {
                    leftPower -= 0.5;
                }
                if (vuforia.getX() > -10) {
                    leftPower = 0;
                    rightPower = 0;
                }
            }
            vuforia.update();
            config.update(leftPower, rightPower, strafePower, liftPower, intake, intakeAngle, hookAngle);
            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.update();
        }
        vuforia.close();
    }
}
