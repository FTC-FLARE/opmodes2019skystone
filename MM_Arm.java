package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Arm {

    private final int ARM_SPEED = 75;

    private DcMotor armMotor = null;
    private Servo gripperServo = null;
    private Servo wristServo = null;
    private Servo capstoneServo = null;
    private DigitalChannel lowerBoundArm = null;
    private DigitalChannel upperBoundArm = null;
    private DigitalChannel haveBlockArm = null;

    private int targetArm = 0;

    private boolean isHandled = false;
    private boolean isOpen = true;

    private LinearOpMode opMode = null;

    public MM_Arm(LinearOpMode opMode){

        this.opMode = opMode;


        armMotor = opMode.hardwareMap.get(DcMotor.class, "armMotor");
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripServo");
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo");
        capstoneServo = opMode.hardwareMap.get(Servo.class,"capstoneServo");
        lowerBoundArm = opMode.hardwareMap.get(DigitalChannel.class, "lowerArm");
        upperBoundArm = opMode.hardwareMap.get(DigitalChannel.class, "upperArm");
        haveBlockArm = opMode.hardwareMap.get(DigitalChannel.class, "haveBlockArm");

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initialize servos
        gripperServo.setPosition(0);
        wristServo.setPosition(0);
        capstoneServo.setPosition(1);
    }

    public void armMove() {
        targetArm = targetArm + (int)((opMode.gamepad2.left_trigger - opMode.gamepad2.right_trigger) * ARM_SPEED);
        armMotor.setTargetPosition(targetArm);
        if (!upperBoundArm.getState() && opMode.gamepad2.left_trigger == 0){
            double maxArm = armMotor.getCurrentPosition();
            targetArm = (int)maxArm;
        }else if(!lowerBoundArm.getState() && opMode.gamepad2.right_trigger == 0){
            double minArm = armMotor.getCurrentPosition();
            targetArm = (int)minArm;
        }
        armMotor.setPower(1);

        opMode.telemetry.addData("Left trigger", opMode.gamepad2.left_trigger);
        opMode.telemetry.addData("Bottom Sensor", !lowerBoundArm.getState());
        opMode.telemetry.addData("Arm Position", armMotor.getCurrentPosition());
    }

    public void autoArm(int target){
        armMotor.setTargetPosition(target);
        armMotor.setPower(1);

        opMode.telemetry.addData("Left trigger", opMode.gamepad2.left_trigger);
        opMode.telemetry.addData("Bottom Sensor", !lowerBoundArm.getState());
        opMode.telemetry.addData("Arm Position", armMotor.getCurrentPosition());
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

    public void deployCapstone(){
        if(opMode.gamepad2.dpad_up){
            capstoneServo.setPosition(1);
        }else if(opMode.gamepad2.dpad_down){
            capstoneServo.setPosition(0);
        }
    }

    public void gripBlock(){
        wristServo.setPosition(1);
    }

    public void releaseBlock(){
        wristServo.setPosition(0);
    }

    public void rotateGripperWrist(){
        if(opMode.gamepad2.b){
            wristServo.setPosition(1);
        }else{
            wristServo.setPosition(0);
        }
    }
}