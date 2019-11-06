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

@Autonomous(name="AutoV", group="Linear Opmode")
public class AutoV extends LinearOpMode {
    private Vuforia vuforia;

    private double leftPower;
    private double rightPower;
    private double strafePower;
    private double liftPower;
    private double hookAngle;
    private double intakeAngle;
    private boolean intake;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor strafeDrive;
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;
    private DcMotor intakeMotor;
    private Servo hookServo1;
    private Servo hookServo2;
    private Servo intakeServo;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        vuforia = new Vuforia(hardwareMap, telemetry, PhoneInfoPackage.getPhoneInfoPackage());

        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        liftPower = 0;
        hookAngle = 0;
        intakeAngle = 0;
        intake = false;

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        strafeDrive = hardwareMap.get(DcMotor.class, "strafe_drive");
        liftMotor1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
        liftMotor2 = hardwareMap.get(DcMotor.class, "lift_motor_2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        hookServo1 = hardwareMap.get(Servo.class, "hook_servo_1");
        hookServo2 = hardwareMap.get(Servo.class, "hook_servo_2");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        strafeDrive.setDirection(DcMotor.Direction.FORWARD);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        hookServo1.setDirection(Servo.Direction.REVERSE);
        hookServo2.setDirection(Servo.Direction.FORWARD);
        intakeServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");

        waitForStart();
        runtime.reset();
        vuforia.flashlight(true);

        while (opModeIsActive()) {
            vuforia.update();

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            strafeDrive.setPower(strafePower);
            liftMotor1.setPower(liftPower);
            liftMotor2.setPower(liftPower);
            if (intake) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }
            hookServo1.setPosition(hookAngle);
            hookServo2.setPosition(hookAngle);
            intakeServo.setPosition(intakeAngle);

            telemetry.addData("Run Time:", "" + runtime.toString());
            telemetry.update();
        }
        vuforia.close();
    }
}
