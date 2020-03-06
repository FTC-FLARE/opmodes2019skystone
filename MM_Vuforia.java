package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class MM_Vuforia {
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private boolean targetVisible = false;
    private LinearOpMode opMode = null;
    private static final double mmPerInch = 25.4;
    private VuforiaTrackables targetsSkyStone = null;
    private VuforiaTrackable stoneTarget = null;
    private ElapsedTime runtime = new ElapsedTime();


    public MM_Vuforia(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AZ5woGn/////AAABmSDumo9pA0BDovmvaV5gG7wLT6ES1QrKcI14JsHiEtQ7Gb6e+KM8ILBQGt8hjfHFNwKixlUDQ6vuz0AdKiYelGz5KcfJ9UV4xCMuDxDGvzOqYIS46QLHeFtsx4c4EP5o5a+H4ZM4crit1cva6avYORJXAH4EYCNluvawI+qm7qOru223kxOmNw83qfl17h9ASLtxxZuZ6OiAnQEq0OsSJf5n43QzVRFI55ZYdVAq+7bSeBEMptf1ZbrzvAZWnq8diTq+ojaADlkeZloub6tSLn4OqqbVtnjk65dNVejK2nTY1y7j7v0BQAkqc0w6oMkg30ynxOoyGid1xjSDDEaS1DvbVjQO0ODZZ4O9v6C30dtQ";
        parameters.cameraDirection = CAMERA_CHOICE;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, (float) (2 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));


        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation((float) (-2 * mmPerInch), (float) (-6.5 * mmPerInch), (float) (5 * mmPerInch))
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, -90, 0, 180));

        ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        CameraDevice.getInstance().setField("zoom", "35"); // used to be 50
    }

    public void activateTarget(){
        targetsSkyStone.activate();
    }

    public int detectSkystone() {
        int targetLocation = 0;
        targetVisible = false;
        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            targetLocation = 1;
        }
        return targetLocation;
    }

    public int getSkystone(boolean alliance) {
        runtime.reset();
        int skystonePosition = 0;  //0 is the left side 1 is center 2 is right
        boolean targetFound = false;
        while (opMode.opModeIsActive() && !targetFound && runtime.seconds() < 1.5) {
            skystonePosition = detectSkystone();
            if (skystonePosition != 0) {
                targetFound = true;
            }
        }
        return skystonePosition;
    }

    public OpenGLMatrix getDistanceToSkyStone() {
        return ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getRobotLocation();
    }
}