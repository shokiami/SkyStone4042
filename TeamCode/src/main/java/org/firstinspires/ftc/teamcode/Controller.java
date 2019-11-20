package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

class Controller {
    public Gamepad gamepad;

    public String a, b, x, y;
    public String dpad_right, dpad_up, dpad_left, dpad_down;
    public String back, guide, start;
    public String left_stick_button, right_stick_button;
    public String left_bumper, right_bumper;

    public double left_stick_x, left_stick_y;
    public double right_stick_x, right_stick_y;
    public double left_trigger, right_trigger;

    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        left_stick_x = gamepad.left_stick_x;
        left_stick_y = -gamepad.left_stick_y;

        right_stick_x = gamepad.right_stick_x;
        right_stick_y = -gamepad.right_stick_y;

        left_trigger  = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
    }

    String check(String input) {
        private check = gamepad.input;
        return output;
    }
}
