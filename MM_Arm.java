package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class MM_Arm {

    private final int ARM_SPEED = 125;

    private DcMotor armMotor = null;
    private Servo gripperServo = null;
    private Servo wristServo = null;

    private boolean isHandled = false;
    private boolean isOpen = true;

    private LinearOpMode opMode = null;

    public MM_Arm(LinearOpMode opMode){

        this.opMode = opMode;

        armMotor = opMode.hardwareMap.get(DcMotor.class, "armMotor");
        gripperServo = opMode.hardwareMap.get(Servo.class, "gripServo");
        wristServo = opMode.hardwareMap.get(Servo.class, "wristServo");

        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gripperServo.setPosition(0);
        wristServo.setPosition(0);
    }

    public void armMove(){
        //TODO add sensors or encoder counts to protect hardware
        int targetArm = armMotor.getCurrentPosition() + (int)((opMode.gamepad2.right_trigger - opMode.gamepad2.left_trigger) * ARM_SPEED);
        armMotor.setTargetPosition(targetArm);
        armMotor.setPower(1);
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

    public void rotateGripperWrist(){
        if(opMode.gamepad2.b){
            wristServo.setPosition(1);
        }else{
            wristServo.setPosition(0);
        }
    }
}
