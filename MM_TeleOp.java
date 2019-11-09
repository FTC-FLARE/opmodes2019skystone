package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

 @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MM_TeleOp", group = "test")
public class MM_TeleOp extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    private DcMotor flyWheel1 = null;
    private DcMotor flyWheel2 = null;
    private DcMotor armMotor = null;
    private Servo gripperServo;
    private Servo armServo;
    private Servo servo;
    private CRServo testServo;
    private boolean isHandled = false;
    private boolean isOpen = true;

    public void runOpMode() {
        robot.init();
        initial();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveWithSticks();
            runFlyWheels();
            moveFoundationServo();
            toggleGripper();
            moveServo();
            rotateGripperWrist(gamepad2.b);
            armMove(-gamepad2.left_stick_y);
        }
    }

    public void runFlyWheels() {
        if (gamepad1.left_bumper){
            flyWheel1.setPower(1);
            flyWheel2.setPower(1);
        }else if(gamepad1.right_bumper){
            flyWheel1.setPower(-.75);
            flyWheel2.setPower(-.75);
        }else{
            flyWheel1.setPower(0);
            flyWheel2.setPower(0);
        }
    }

    public void moveFoundationServo(){
        if (gamepad2.a) {
            servo.setPosition(.45);
        } else {
            servo.setPosition(1);
        }
    }

    public void toggleGripper() {

        if (gamepad2.x && !isHandled) {
            if(isOpen){
                gripperServo.setPosition(1);
                isOpen = false;
            }else{
                gripperServo.setPosition(0);
                isOpen = true;
            }
            isHandled = true;
        }
        else if(!gamepad2.x){
            isHandled = false;
        }
    }
    public void moveServo(){
        if(gamepad1.y){
            testServo.setPower(1);
        } else if (gamepad1.x){
            testServo.setPower(-1);
        } else {
            testServo.setPower(0);
        }
    }

    public void rotateGripperWrist(boolean turn){
        if(turn){
            armServo.setPosition(1);
        }else{
            armServo.setPosition(0);
        }
    }

    public void armUp(double speed) {
        int armPosition = armMotor.getCurrentPosition() + (int)(speed * 100);
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(speed);
    }
    public void armDown(double speed) {
        int armPosition = armMotor.getCurrentPosition() + (int)(speed * 100);
        armMotor.setTargetPosition(armPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(speed);
    }
    public void armHold () {
        armMotor.setTargetPosition(armMotor.getCurrentPosition());
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(.5);
    }
    public void armMove(double speed){
        if (speed > 0) {
            armUp(speed);
        }
        else if (speed < 0) {
            armDown(speed);
        }
        else {
            armHold();
        }
    }


    public void initial() {
        flyWheel1 = hardwareMap.get(DcMotor.class, "flyWheel1");
        flyWheel2 = hardwareMap.get(DcMotor.class, "flyWheel2");
        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        gripperServo = hardwareMap.get(Servo.class, "gripServo");
        armServo = hardwareMap.get(Servo.class, "armServo");
        servo = hardwareMap.get(Servo.class, "servo");
        testServo = hardwareMap.get(CRServo.class, "testServo");
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}