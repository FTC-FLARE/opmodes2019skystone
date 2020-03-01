package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MM_Collector {

    private LinearOpMode opMode = null;

    private DcMotor flywheelLeft = null;
    private DcMotor flywheelRight = null;

    private Servo alignerServo = null;
    private Servo redSkystick = null;
    private Servo blueSkystick = null;
    private Servo pokerServo = null;

    private DistanceSensor collectorRange;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        flywheelLeft = opMode.hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = opMode.hardwareMap.get(DcMotor.class, "flywheelRight");
        alignerServo = opMode.hardwareMap.get(Servo.class, "alignerServo");
        pokerServo = opMode.hardwareMap.get(Servo.class, "Lance");
        redSkystick = opMode.hardwareMap.get(Servo.class, "redSkystick");
        blueSkystick = opMode.hardwareMap.get(Servo.class, "blueSkystick");
        collectorRange = opMode.hardwareMap.get(DistanceSensor.class, "collectorRange");


        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        //skystick servos are opposite each other
        pokerServo.setPosition(1);
        redSkystick.setPosition(1);
        blueSkystick.setPosition(1);
        alignerServo.setPosition(.5);
    }
    public void rangeTest(){
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("collector range",collectorRange.getDistance(DistanceUnit.INCH));
            opMode.telemetry.update();
        }
    }

    public double getCollectorDistance(){
        double distance = 0;
        if (!(collectorRange.getDistance(DistanceUnit.INCH) < Double.MAX_VALUE)){
            distance = 100;
        }
        else{
            distance = collectorRange.getDistance(DistanceUnit.INCH);
        }
        opMode.telemetry.addData("distance",distance);
        opMode.telemetry.update();
        return distance;
    }


    public void controlFlywheels() {
        if (opMode.gamepad1.left_bumper) {
            powerFlywheels(1);
        } else if (opMode.gamepad1.right_bumper) {
            powerFlywheels(-1);
        } else {
            powerFlywheels(0);
        }
    }

    public void pokerServoDown(){
        pokerServo.setPosition(0.1);
    }

    public void pokerServoUp(){
        pokerServo.setPosition(1);
    }

    public void powerFlywheels(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);

    }

    public void alignStone() {
        if (opMode.gamepad1.y) {
            alignerServo.setPosition(1);
        } else {
            alignerServo.setPosition(.5);
        }
    }

    public void autoAlignStone(){
        alignerServo.setPosition(1);
        opMode.sleep(500);
        alignerServo.setPosition(.5);
        opMode.sleep(250);
    }

    public void moveRedSkystick() {
        if (opMode.gamepad2.y) {
            redSkystick.setPosition(0);
        } else {
            redSkystick.setPosition(1);
        }
    }

    public void moveBlueSkystick() {
        if (opMode.gamepad2.y) {
            blueSkystick.setPosition(1);
        } else {
            blueSkystick.setPosition(0);
        }
    }

    public void redSkystickDown() {
        redSkystick.setPosition(0);
    }

    public void redSkystickUp() {
        redSkystick.setPosition(1);
    }

    public void blueSkystickDown() {
        blueSkystick.setPosition(1);
    }

    public void blueSkystickUp() {
        blueSkystick.setPosition(0);
    }

    public void skystickUp(boolean alliance, int stonePosition) {
        if (alliance){
            if (stonePosition == 0 || stonePosition == 1){
                blueSkystickUp();
            }else{
                redSkystickUp();
            }
        }else{
            if (stonePosition == 2 || stonePosition == 1){
                redSkystickUp();
            }else{
                blueSkystickUp();
            }
        }
    }

    public void skystickDown(boolean alliance,int stonePosition) {
        if (alliance){
            if (stonePosition == 0 || stonePosition == 1){
                blueSkystickDown();
            }else{
                redSkystickDown();
            }
        }else{
            if (stonePosition == 2 || stonePosition == 1){
                redSkystickDown();
            }else{
                blueSkystickDown();
            }
        }
    }
}
