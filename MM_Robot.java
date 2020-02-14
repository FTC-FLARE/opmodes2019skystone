package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import static android.os.SystemClock.sleep;

public class MM_Robot {
    private LinearOpMode opMode;
    public MM_Drivetrain drivetrain;
    public MM_Collector collector;
    public MM_Arm arm;
    public MM_Vuforia vuforia;

    OpenGLMatrix position = null;
    boolean targetSeen = false;
    boolean turn = false;
    public static final double TARGET_OFFSET = toMM(-8);
    public static final double LOOK_TIME = 1;

    public MM_Robot(LinearOpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        drivetrain = new MM_Drivetrain(opMode);
        collector = new MM_Collector(opMode);
        arm = new MM_Arm(opMode);
        vuforia = new MM_Vuforia(opMode);
    }

    public void driveToTarget(boolean alliance) {
        drivetrain.resetEncoder();
        boolean targetSeen = false;
        CameraDevice.getInstance().setFlashTorchMode(true);
        drivetrain.setMotorPowersSame(.05);
        while (opMode.opModeIsActive() && !targetSeen && drivetrain.currentPosition() < 26) {
            position = vuforia.getDistanceToSkyStone();
            if (position != null) {
                targetSeen = true;
            }
        }
        drivetrain.setMotorPowersSame(0);
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    //use the gyro and look for target
    public void turnToSee(double angle, double speed){
        drivetrain.runWithEncoder();
        double error = angle - drivetrain.getAngle();
        double power;
        while (opMode.opModeIsActive() && Math.abs(error) > MM_Drivetrain.HEADING_THRESHOLD) {
            error = drivetrain.correctError(angle - drivetrain.getAngle());
            power = drivetrain.exponentialControl(error,speed,.0075,2);
            if (error < 0) {
                power = -power;
            }
            drivetrain.setMotorPowers(-power, power);
            position = vuforia.getDistanceToSkyStone();
            if (position != null){
                drivetrain.setMotorPowersSame(0);
                return;
            }
        }
        while (opMode.opModeIsActive()) {
            drivetrain.setMotorPowersSame(0);
        }
    }


    public void findTarget(){
        boolean targetSeen = false;

        drivetrain.gyroTurn(1, -72.5);
        turnToSee(-77.5,.5);
        targetSeen = isTargetSeen(targetSeen);

        if (!targetSeen){
            drivetrain.gyroTurn(1,-85);
            turnToSee(-90,.5);
            targetSeen = isTargetSeen(targetSeen);
        }

        if(!targetSeen){
            drivetrain.gyroTurn(1,-97.5);
            turnToSee(-105,.5);
            targetSeen = isTargetSeen(targetSeen);
        }

        if(!targetSeen){
            while(opMode.opModeIsActive()){
                opMode.telemetry.addLine("didn't see target");
                opMode.telemetry.update();
            }
        }
    }

    public void alignToSkystone(){
        position = vuforia.getDistanceToSkyStone();
        drivetrain.gyroTurn(1,-90);
        double xError = position.getTranslation().get(0);
        drivetrain.driveWithInches(toInches(xError),.25);
    }

    private boolean isTargetSeen(boolean targetSeen) {
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && opMode.getRuntime() < LOOK_TIME && !targetSeen){
            position = vuforia.getDistanceToSkyStone();
            if (position != null){
                targetSeen = true;
            }
        }
        return targetSeen;
    }

    public boolean isTargetSeen(double time) {
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && opMode.getRuntime() < time && !targetSeen){
            position = vuforia.getDistanceToSkyStone();
            if (position != null){
                targetSeen = true;
            }
        }
        return targetSeen;
    }

    public void wasTargetSeen(){
        while(opMode.opModeIsActive()){
            opMode.telemetry.addData("target seen", targetSeen);
            opMode.telemetry.update();
        }
    }


    public void driveToTarget(double inches, double speed) {
        drivetrain.driveWithInches(inches,speed,vuforia);
    }

//    if the phone is on the back
//    public void driveToSkystone(){
//        opMode.resetStartTime();
//        while (opMode.opModeIsActive() && opMode.getRuntime() < 1) {
//            position = vuforia.getDistanceToSkyStone();
//        }
//        double xError = position.getTranslation().get(0) - TARGET_OFFSET;
//        double yDistance = toMM(8); //currently this is just a random hard coded value
//
//        double driveDistance = Math.sqrt(Math.pow(yDistance,2) + Math.pow(xError,2));
//        double angle = (Math.toDegrees(Math.atan2(xError,yDistance)));
//
//        drivetrain.gyroTurn(.25,angle);
//        drivetrain.driveWithInches(toInches(-driveDistance),.125);
//        drivetrain.gyroTurn(.25,0);
//    }

    // drive so that skystick is in line with skystone
//    public void driveToSkystoneRange(boolean blue){
//        opMode.resetStartTime();
//        while (opMode.opModeIsActive() && opMode.getRuntime() < .25) {
//            position = vuforia.getDistanceToSkyStone();
//        }
//        double xError = toMM(11) - position.getTranslation().get(0);
//        double yError = position.getTranslation().get(1) + toMM(7.5);
//
//        double driveDistance = Math.hypot(yError,xError);
//        double angle = (Math.toDegrees(Math.atan2(xError,yError)));
//        if(blue){
//            angle = -angle;
//        }
//
//        drivetrain.gyroTurn(1,angle);
//        drivetrain.driveWithInches(toInches(driveDistance)+6,.125);
//        drivetrain.driveWithInches(-6,.125);
//        drivetrain.gyroTurn(1,90);
//    }

    public void driveToSkystone(){
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && opMode.getRuntime() < .25) {
            position = vuforia.getDistanceToSkyStone();
        }
        double xError = toMM(-10) - position.getTranslation().get(0);
        double yError = position.getTranslation().get(1) + toMM(8);

        double driveDistance = (Math.hypot(yError,xError));
        double angle = (Math.toDegrees(Math.atan2(xError,yError)));

        drivetrain.driveWithInches(-18,.5);
        drivetrain.gyroTurn(1,165);
        collector.powerFlywheels(-1);
        arm.autoArm(-750);
        drivetrain.driveWithInches(toInches(driveDistance),.125);
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
