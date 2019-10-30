package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



public class Vuforia{
    OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia;
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    public Vuforia(HardwareMap hardwareMap){
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AYQeJpT/////AAAAGTYIMyXJhERVtNOZjU3Ij86J7nhWU3V9lqwfv7A2iNUi52+j5tjLmc2XIa89AQXNmcyeExAdhhPy6zwU6R6T2fCLS2fr8YeYzG64ljMLW9VNwArYgr1CPSG+79VWsMrC67AfiWtTiQVx2X0b96dIrrACtijHut4BDc/JI9Lth1vHWkR4f5BtclEtnMz/dnRmP8M1QFnxPLCRm8jQVQUAwmedpLAYfeasec2ZcWSb5JRDJwuBmQMDBD6sAHcyfDy9EZXsFHfoscAv/t6U37x3/VoRQncONjYr8/Et7Km4inIzs/N3Dc4h63vPkSu61eZGlJojCwfbHlwshGzq4fti5PCfqVnJv6NI6LijknZ3vSfy";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }

    public double[] view(){
        double[] data = new double[6];
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { //if it sees a vumark
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            VectorF trans = pose.getTranslation();
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            data[0] = trans.get(0);
            data[1] = trans.get(1);
            data[2] = trans.get(2);
            data[3] = rot.firstAngle;
            data[4] = rot.secondAngle;
            data[5] = rot.thirdAngle;
        }
        return data;
    }
}