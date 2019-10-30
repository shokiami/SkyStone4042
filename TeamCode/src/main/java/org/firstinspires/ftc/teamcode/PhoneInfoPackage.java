package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class PhoneInfoPackage {
    private static final float mmPerInch = 25.4f;
    VuforiaLocalizer.CameraDirection CAMERA_CHOICE;
    boolean PHONE_IS_PORTRAIT;
    // looking out from red alliance side
    // the robot is facing to the right (along +X)
    final float CAMERA_FORWARD_DISPLACEMENT;  // X axis: left to right (centered)
    final float CAMERA_VERTICAL_DISPLACEMENT; // Z axis: up from ground
    final float CAMERA_LEFT_DISPLACEMENT;     // Y axis: near to far (centered)

    private PhoneInfoPackage(boolean useBackCamera, boolean phoneIsPortrait, double forwardDisplacement, double upwardDisplacement, double leftwardDisplacement) {
        if (useBackCamera)
            CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.BACK;
        else
            CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
        PHONE_IS_PORTRAIT = phoneIsPortrait;
        CAMERA_FORWARD_DISPLACEMENT = (float) forwardDisplacement * mmPerInch;
        CAMERA_VERTICAL_DISPLACEMENT = (float) upwardDisplacement * mmPerInch;
        CAMERA_LEFT_DISPLACEMENT = (float) leftwardDisplacement * mmPerInch;
    }

    public static PhoneInfoPackage getPhoneInfoPackage() {
        return new PhoneInfoPackage(
                true,
                false,
                0,
                0,
                0);
    }
}
