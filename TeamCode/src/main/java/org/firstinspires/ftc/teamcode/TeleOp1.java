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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp1", group="Iterative Opmode")
public class TeleOp1 extends OpMode
{
    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor strafeDrive;
    private DcMotor liftMotor1;
    private DcMotor liftMotor2;
    private DcMotor intakeMotor;
    private Servo hookServo1;
    private Servo hookServo2;
    private Servo intakeServo;

    private double leftPower;
    private double rightPower;
    private double strafePower;
    private double liftPower;
    private double hookAngle;
    private double intakeAngle;
    private boolean intake;
    private double speed;
    private float rightTriggerCheck;
    private boolean xCheck;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        leftPower = 0;
        rightPower = 0;
        strafePower = 0;
        liftPower = 0;
        hookAngle = 0;
        intakeAngle = 0;
        intake = false;
        speed = 1;
        rightTriggerCheck = 0;
        xCheck = false;

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
        //Ball Drive
        leftPower = gamepad1.left_stick_y;
        rightPower = gamepad1.right_stick_y;
        strafePower = gamepad1.right_stick_x;

        //Speed Adjustments
        if (gamepad1.x && !xCheck) {
            if (speed == 0.5) {
                speed = 1;
            } else {
                speed = 0.5;
            }
        }
        /*
        if (0.01 < gamepad1.left_trigger) {
            if (gamepad1.left_trigger == 1 && leftTriggerCheck != 1) {
                if (speed == 0.5) {
                    speed = 1;
                } else {
                    speed = 0.5;
                }
            } else {
                speed = 1 - gamepad1.left_trigger;
            }
        } */
        xCheck = gamepad1.x;


        //Lift
        if (gamepad1.dpad_up) {
            liftPower = 1;
        } else if (gamepad1.dpad_down) {
            liftPower = -1;
        } else {
            liftPower = 0;
        }

        //Hook Servos
        if (gamepad1.y) {
            if (hookAngle == 0) {
                hookAngle = 90;
            } else {
                hookAngle = 0;
            }
        }

        //Intake
        if (gamepad1.right_trigger != 0 && rightTriggerCheck == 0) {
            intake = !intake;
        }
        rightTriggerCheck = gamepad1.right_trigger;

        leftPower *= speed;
        rightPower *= speed;
        strafePower *= speed;
        liftPower *= speed;

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

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower, strafePower);
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }
}
