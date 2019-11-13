package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Arm {

    private DcMotor armMotor = null;
    private Servo gripperServo = null;
    private Servo armServo = null;

    private boolean isHandled = false;
    private boolean isOpen = true;

    private final int ARM_SPEED = 125;

    private LinearOpMode opMode = null;

    public MM_Arm(LinearOpMode opMode){
        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripServo");
        armServo = opMode.hardwareMap.get(Servo.class, "armServo");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

//    public void armUp(double speed) {
//        int armPosition = armMotor.getCurrentPosition() + (int)(speed * 100);
//        armMotor.setTargetPosition(armPosition);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(speed);
//    }
//    public void armDown(double speed) {
//        int armPosition = armMotor.getCurrentPosition() - (int)(speed * 100);
//        armMotor.setTargetPosition(armPosition);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(speed);
//    }
//    public void armHold () {
//        armMotor.setTargetPosition(armMotor.getCurrentPosition());
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        armMotor.setPower(.5);
//    }

    public void armMove(){
        //TODO add sensors or encoder counts to protect hardware
        int targetArm = armMotor.getCurrentPosition() + (int)((opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger) * ARM_SPEED);
        armMotor.setTargetPosition(targetArm);
        armMotor.setPower(1);
    }
    public void toggleGripper() {
        if (opMode.gamepad2.x && !isHandled) {
            if(isOpen){
                gripperServo.setPosition(1);
                isOpen = false;
            }else{
                gripperServo.setPosition(0);
                isOpen = true;
            }
            isHandled = true;
        }
        else if(!opMode.gamepad2.x){
            isHandled = false;
        }
    }
    public void rotateGripperWrist(){
        if(opMode.gamepad2.b){
            armServo.setPosition(1);
        }else{
            armServo.setPosition(0);
        }
    }
}
