package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Vuforia {
    private final VuforiaLocalizer.CameraDirection CAMERA_CHOICE; // or BACK
    private final boolean PHONE_IS_PORTRAIT;

    private static final String VUFORIA_KEY = VuforiaKey.VUKEY;

    private static final float mmPerInch      = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;  // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;  // degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private double xvel, yvel, zvel;
    private VuforiaLocalizer vuforia;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate;
    private float phoneZRotate = 0;

    private List<VuforiaTrackable> allTrackables;
    private VuforiaTrackables targetsSkyStone;
    private Telemetry telemetry;

    // looking out from red alliance side
    // the robot is facing to the right (along +X)
    private final float CAMERA_FORWARD_DISPLACEMENT;  // X axis: left to right (centered)
    private final float CAMERA_VERTICAL_DISPLACEMENT; // Z axis: up from ground
    private final float CAMERA_LEFT_DISPLACEMENT;     // Y axis: near to far (centered)

    private boolean targetStone = false;
    private boolean noNewInfo = false;

    private ElapsedTime elapsedTime;
    private int whoopsCount = 0;

    // if the target is a stone:
    // robot is -X away from target, -Y to the right of target, +Z above target

    Vuforia(HardwareMap hardwareMap, Telemetry telemetry, PhoneInfoPackage infoPackage) {
        CAMERA_CHOICE = infoPackage.CAMERA_CHOICE;
        PHONE_IS_PORTRAIT = infoPackage.PHONE_IS_PORTRAIT;
        CAMERA_FORWARD_DISPLACEMENT = infoPackage.CAMERA_FORWARD_DISPLACEMENT;
        CAMERA_VERTICAL_DISPLACEMENT = infoPackage.CAMERA_VERTICAL_DISPLACEMENT;
        CAMERA_LEFT_DISPLACEMENT = infoPackage.CAMERA_LEFT_DISPLACEMENT;

        elapsedTime = new ElapsedTime();

        this.telemetry = telemetry;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // no camera monitor
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection   = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);

        // perspective from red alliance:
        // X axis: left to right (centered)
        // Y axis: near to far (centered)
        // Z axis: up from ground

        // can move, but assumed to be at field origin
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // auto-rotate must be disabled

        // robot coordinates life field (right side of robot facing/parallel to red alliance wall)
        // flat on back, top of phone facing +Y (away from wall)
        // The two examples below assume that the camera is facing forward out the front of the robot.

        if (CAMERA_CHOICE == BACK)
            phoneYRotate = -90;
        else
            phoneYRotate = 90;

        if (PHONE_IS_PORTRAIT)
            phoneXRotate = 90 ;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables)
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

        targetsSkyStone.activate();
    }

    void update() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;

        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                //telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                targetStone = trackable.getName().equals("Stone Target");

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                    noNewInfo = false;
                    elapsedTime.reset();
                } else {
                    noNewInfo = true;
                    whoopsCount++;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            telemetrizeTranslation(getTranslation());
            telemetrizeOrientation(getOrientation());
        }
        else {
            telemetry.addData("Visible Target", "none");
        }
        //telemetry.update();
    }

    private void telemetrizeTranslation(VectorF translation) {
        telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", this.getX(), this.getY(), this.getZ());
    }

    private void telemetrizeOrientation(Orientation rotation) {
        telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", this.getRoll(), this.getPitch(), this.getHeading());
    }

    private VectorF getTranslation() {
        return lastLocation.getTranslation();
    }

    double getX() {
        return getTranslation().get(0) / mmPerInch;
    }

    double getY() {
        return getTranslation().get(1) / mmPerInch;
    }

    double getZ() {
        return getTranslation().get(2) / mmPerInch;
    }

    private Orientation getOrientation() {
        return Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
    }

    double getRoll() {
        return getOrientation().firstAngle;
    }

    double getPitch() {
        return getOrientation().secondAngle;
    }

    double getHeading() {
        return getOrientation().thirdAngle;
    }

    boolean isTargetStone() {
        return targetStone;
    }

    boolean isTargetVisible() {
        return targetVisible;
    }

    boolean hasNewInfo() {
        return !noNewInfo;
    }

    void flashlight(boolean flashlight) {
        CameraDevice.getInstance().setFlashTorchMode(flashlight);
    }

    void close() {
        CameraDevice.getInstance().setFlashTorchMode(false);
        targetsSkyStone.deactivate();
    }
}
