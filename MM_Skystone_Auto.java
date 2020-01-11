package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

@Autonomous (name = "MM_Skystone_Auto", group = "Mechanical Meltdown")
public class MM_Skystone_Auto extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);
    OpenGLMatrix position = null;
    boolean alliance = true;

    public void runOpMode() {
        telemetry.addLine("initializing...");
        telemetry.update();
        robot.init();
        robot.vuforia.init();
        boolean alliance = true;
        String ally = "";
        while (!opModeIsActive()) { //x is blue which is false, b is red which is true
            if (gamepad1.x) {
                alliance = false;
                ally = "Blue";
            } else if (gamepad1.b) {
                alliance = true;
                ally = "Red";
            }
            telemetry.addLine("Press button that corresponds to the alliance color");
            telemetry.addData("Current alliance color" , ally);
            telemetry.addLine("Press Play to Start");
            telemetry.update();
        }

        waitForStart();

        robot.vuforia.activateTarget();

        CameraDevice.getInstance().setFlashTorchMode(true);
        robot.drivetrain.resetEncoder();
        robot.driveToTarget(alliance);
        resetStartTime();
        while (opModeIsActive() && getRuntime() < .25) {
            position = robot.vuforia.getDistanceToSkyStone();
        }
        robot.driveToSkystone();
        CameraDevice.getInstance().setFlashTorchMode(false);
        robot.collectSkystone();
        robot.deliverSkystone(alliance);
    }
}

//        while(opModeIsActive()){
//            position = robot.vuforia.getDistanceToSkyStone();
//            if (position != null) {
//                xDistance = position.getTranslation().get(0);
//                yDistance = position.getTranslation().get(1);
//                double errorX = xDistance - (targetOffset * 25.4);
//                telemetry.addData("x", xDistance / 25.4);
//                telemetry.addData("y", yDistance / 25.4);
//                telemetry.addData("error x", errorX / 25.4);
//            }else{
//                telemetry.addLine("Lost Target :(");
//            }
//            telemetry.update();
//
//        }
