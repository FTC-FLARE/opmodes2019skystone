package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Collector {

    private LinearOpMode opMode = null;

    private DcMotor flywheelLeft = null;
    private DcMotor flywheelRight = null;

    private Servo alignerServo = null;

    public MM_Collector(LinearOpMode opMode){
        this.opMode = opMode;

        flywheelLeft = opMode.hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = opMode.hardwareMap.get(DcMotor.class, "flywheelRight");
        alignerServo = opMode.hardwareMap.get(Servo.class, "alignerServo");

        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);
    }

    public void controlFlywheels() {
        if (opMode.gamepad1.left_bumper){
            powerFlywheels(1);
        }else if(opMode.gamepad1.right_bumper){
            powerFlywheels(-1);
        }else{
            powerFlywheels(0);
        }
    }

    private void powerFlywheels(double power) {
        flywheelLeft.setPower(power);
        flywheelRight.setPower(power);
    }

    public void alignStone(){
        if(opMode.gamepad1.y){
            alignerServo.setPosition(1);
        } else {
            alignerServo.setPosition(.5);
        }
    }
}
