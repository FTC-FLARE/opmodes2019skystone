package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import static android.os.SystemClock.sleep;

public class MM_Robot {
    private LinearOpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Arm arm;
    public MM_Vuforia vuforia;

    OpenGLMatrix position = null;
    boolean turn = false;
    public static final double TARGET_OFFSET = toMM(-8);

    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        arm = new MM_Arm(opMode);
        vuforia = new MM_Vuforia(opMode);
    }
    public void driveToTarget(boolean alliance){
        boolean targetSeen = false;
        drivetrain.setMotorPowersSame(-.125);
        while(opMode.opModeIsActive() && !targetSeen && drivetrain.currentPosition() > -16){
            position = vuforia.getDistanceToSkyStone();
            if(position != null){
                targetSeen = true;
            }
        }
        drivetrain.setMotorPowersSame(0);
        if (!targetSeen) {
            if(!turn) {
                turn = true;
                if(alliance) {
                    drivetrain.gyroTurn(.25, -45);
                }else{
                    drivetrain.gyroTurn(.25, 45);
                }
                drivetrain.driveWithInches(12, .25);
                drivetrain.gyroTurn(.25, 0);
                sleep(50);
                driveToTarget(alliance);
            }
        } else {
            drivetrain.setMotorPowersSame(.25);
            while (opMode.opModeIsActive() && position.getTranslation().get(1) > toMM(-18)) {
                position = vuforia.getDistanceToSkyStone();
                opMode.telemetry.addData("x", position.getTranslation().get(0));
                opMode.telemetry.addData("y", position.getTranslation().get(1));
                opMode.telemetry.update();
            }
        }
        drivetrain.setMotorPowersSame(0);
    }

    public void driveToSkystone(){
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && opMode.getRuntime() < .25) {
            position = vuforia.getDistanceToSkyStone();
        }
        double xError = position.getTranslation().get(0) - TARGET_OFFSET;
        double yDistance = toMM(8); //currently this is just a random hard coded value

        double driveDistance = Math.sqrt(Math.pow(yDistance,2) + Math.pow(xError,2));
        double angle = (Math.toDegrees(Math.atan2(xError,yDistance)));

        drivetrain.gyroTurn(.25,angle);
        drivetrain.driveWithInches(toInches(-driveDistance),.125);
        drivetrain.gyroTurn(.25,0);

    }

    public void collectSkystone() {
        drivetrain.driveWithInches(-7.5,.25);
        collector.redSkystickDown();
        sleep(500);
        drivetrain.driveWithInches(12.5,.25);
    }

    public void deliverSkystone(boolean alliance) {
        if (alliance) {
            if(turn) {
                drivetrain.gyroTurn(.25, 90);
                drivetrain.driveWithInches(68, 1);
                drivetrain.gyroTurn(.25, -90);
                collector.blueSkystickUp();
                sleep(500);
                drivetrain.driveWithInches(12, 1);
            }else{
                drivetrain.gyroTurn(.25, 90);
                drivetrain.driveWithInches(52, .75);
                drivetrain.gyroTurn(.25, -90);
                collector.blueSkystickUp();
                sleep(500);
                drivetrain.driveWithInches(12, .75);
            }
        }else{
            if (turn) {
                drivetrain.gyroTurn(.25, -90);
                drivetrain.driveWithInches(62, 1);
                drivetrain.gyroTurn(.25, 90);
                collector.blueSkystickUp();
                sleep(500);
                drivetrain.driveWithInches(12, 1);
            }else{
                drivetrain.gyroTurn(.25, -90);
                drivetrain.driveWithInches(48, .75);
                drivetrain.gyroTurn(.25, 90);
                collector.blueSkystickUp();
                sleep(500);
                drivetrain.driveWithInches(12, .75);
            }
        }
    }

    public static double toMM(double inches){
        return inches * 25.4;
    }

    public static double toInches(double MM){
        return MM / 25.4;
    }
}
//            opMode.telemetry.addData("x",position.getTranslation().get(0));
//            opMode.telemetry.addData("y",position.getTranslation().get(1));
//            opMode.telemetry.addData("xError",xError);
//            opMode.telemetry.addData("angle",angle);
//            opMode.telemetry.addData("drive distance",toInches(driveDistance));
//            opMode.telemetry.update();

//    public void driveCloseToSkystone(){
//        double initialVelocity = -.05;
//        double leftPower = 0;
//        double rightPower = 0;
//        double PCoeff = .00025;
//        position = vuforia.getDistanceToSkyStone();
//        double xError = position.getTranslation().get(0) - TARGET_OFFSET;
//
//        while (opMode.opModeIsActive() && position.getTranslation().get(1) > toMM(11)) {
//            position = vuforia.getDistanceToSkyStone();
//            if(position != null) {
//                xError = position.getTranslation().get(0) - (TARGET_OFFSET);
//
//                leftPower = (initialVelocity - (xError * PCoeff));
//                rightPower = (initialVelocity + (xError * PCoeff));
//
//                drivetrain.setMotorPowers(leftPower, rightPower);
//
//                opMode.telemetry.addData("xError", toInches(xError));
//                opMode.telemetry.addData("left power", leftPower);
//                opMode.telemetry.addData("right power", rightPower);
//            }else{
//                opMode.telemetry.addLine("lost target :(");
//            }
//            opMode.telemetry.update();
//        }
//        drivetrain.setMotorPowersSame(0);
//    }

//    public void driveToSkystone(){
//        double initialVelocity = -.05;
//        double leftPower = 0;
//        double rightPower = 0;
//        double PCoeff = .00025;
//        double xError = position.getTranslation().get(0) - TARGET_OFFSET;
//
//        while (opMode.opModeIsActive() && (xError > toMM(1) || xError < toMM(-1))) {
//            position = vuforia.getDistanceToSkyStone();
//            if(position != null) {
//                xError = position.getTranslation().get(0) - (TARGET_OFFSET);
//
//                leftPower = (initialVelocity - (xError * PCoeff));
//                rightPower = (initialVelocity + (xError * PCoeff));
//
//                drivetrain.setMotorPowers(leftPower, rightPower);
//
//                opMode.telemetry.addData("xError", toInches(xError));
//                opMode.telemetry.addData("left power", leftPower);
//                opMode.telemetry.addData("right power", rightPower);
//            }else{
//                opMode.telemetry.addLine("lost target :(");
//            }
//            opMode.telemetry.update();
//        }
//        drivetrain.setMotorPowersSame(0);
//    }

//    public boolean determineStick(boolean alliance){
//        if (!turn) {
//            alliance = !alliance;
//        }
//        return alliance;
//    }
//    public void driveToSkystone(float xDistance, float yDistance, boolean alliance){
//        double driveDistance = 0;
//        if(alliance){
//            drivetrain.gyroTurn(.5, Math.toDegrees(Math.atan((xDistance + 9) / yDistance)));
//            driveDistance = Math.sqrt((Math.pow(xDistance + 9,2) + Math.pow(yDistance,2)));
//            drivetrain.driveWithInches(driveDistance,.25);
//        } else {
//            drivetrain.gyroTurn(.5, Math.toDegrees(Math.atan((xDistance - 9)/yDistance)));
//            driveDistance = Math.sqrt((Math.pow(xDistance - 9,2) + Math.pow(yDistance,2)));
//            drivetrain.driveWithInches(driveDistance,.25);
//
//        }
//        drivetrain.gyroTurn(.5, Math.toDegrees(Math.atan((xDistance) / yDistance)));
//    }
