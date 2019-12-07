package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector {

    private LinearOpMode opMode = null;

    private DcMotor flywheelLeft = null;
    private DcMotor flywheelRight = null;

    private Servo alignerServo = null;
    private Servo blueSkystick = null;
    private Servo redSkystick = null;

    public MM_Collector(LinearOpMode opMode) {
        this.opMode = opMode;

        flywheelLeft = opMode.hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = opMode.hardwareMap.get(DcMotor.class, "flywheelRight");
        alignerServo = opMode.hardwareMap.get(Servo.class, "alignerServo");
        blueSkystick = opMode.hardwareMap.get(Servo.class, "blueSkystick");
        redSkystick = opMode.hardwareMap.get(Servo.class, "redSkystick");


        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        //skystick servos are opposite each other
        blueSkystick.setPosition(1);
        redSkystick.setPosition(0);
        alignerServo.setPosition(0);
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

    private void powerFlywheels(double power) {
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

    public void moveBlueSkystick() {
        if (opMode.gamepad2.y) {
            blueSkystick.setPosition(0);
        } else {
            blueSkystick.setPosition(1);
        }
    }

    public void moveRedSkystick() {
        if (opMode.gamepad2.y) {
            redSkystick.setPosition(1);
        } else {
            redSkystick.setPosition(0);
        }
    }

    public void blueSkystickDown() {
        blueSkystick.setPosition(0);
    }

    public void blueSkystickUp() {
        blueSkystick.setPosition(1);
    }

    public void redSkystickDown() {
        redSkystick.setPosition(1);
    }

    public void redSkystickUp() {
        redSkystick.setPosition(0);
    }

    public void skystickUp(boolean alliance, int stonePosition) {
        if (alliance){
            if (stonePosition == 0 || stonePosition == 1){
                blueSkystickUp();
            }else{
                redSkystickUp();
            }
        }else{
            if (stonePosition == 0 || stonePosition == 1){
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
            if (stonePosition == 0 || stonePosition == 1){
                redSkystickDown();
            }else{
                blueSkystickDown();
            }
        }
    }
}
