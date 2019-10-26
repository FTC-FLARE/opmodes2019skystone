package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

 @com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "MM_TeleOp", group = "test")
public class MM_TeleOp extends LinearOpMode {
    private MM_Robot robot = new MM_Robot(this);

    private CRServo flyWheelServo1;
    private CRServo flyWheelServo2;
    private DcMotor flyWheel1 = null;
    private DcMotor flyWheel2 = null;
    private Servo armServo;
    private Servo servo;

    public void runOpMode(){
        robot.init();
        initial();

        waitForStart();

        while (opModeIsActive()) {
            robot.driveWithSticks();
            runFlyWheels();
            openServo(gamepad1.a);
            moveServoArm(gamepad1.b);
        }
    }

    public void runFlyWheels() {
        if (gamepad1.left_bumper){
            flyWheel1.setPower(1);
            flyWheel2.setPower(1);
            flyWheelServo1.setPower(.75);
            flyWheelServo2.setPower(.75);
        }else if(gamepad1.right_bumper){
            flyWheel1.setPower(-1);
            flyWheel2.setPower(-1);
            flyWheelServo1.setPower(.25);
            flyWheelServo2.setPower(.25);
        }else{
            flyWheel1.setPower(0);
            flyWheel2.setPower(0);
            flyWheelServo1.setPower(.5);
            flyWheelServo2.setPower(.5);
        }
    }

    public void openServo(boolean open){
        if (open) {
            servo.setPosition(.5);
        }else{
            servo.setPosition(.125);
        }
    }

    public void moveServoArm(boolean turn){
        if(turn){
            armServo.setPosition(1);
        }else{
            armServo.setPosition(0);
        }
    }


    public void initial() {
        flyWheelServo1 = hardwareMap.crservo.get("servo1");
        flyWheelServo2 = hardwareMap.crservo.get("servo2");
        flyWheel1 = hardwareMap.get(DcMotor.class, "flyWheel1");
        flyWheel2 = hardwareMap.get(DcMotor.class, "flyWheel2");
        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);
        armServo = hardwareMap.get(Servo.class, "armServo");
        servo = hardwareMap.get(Servo.class, "servo");
    }
}