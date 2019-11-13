package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MM_Collector {

    private LinearOpMode opMode = null;

    private DcMotor flyWheel1 = null;
    private DcMotor flyWheel2 = null;

    private CRServo alignerServo = null;

    public MM_Collector(LinearOpMode opMode){
        this.opMode = opMode;

        flyWheel1 = opMode.hardwareMap.get(DcMotor.class, "flyWheel1");
        flyWheel2 = opMode.hardwareMap.get(DcMotor.class, "flyWheel2");
        flyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        flyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        alignerServo = opMode.hardwareMap.get(CRServo.class, "alignerServo");
    }

    public void controlFlywheels() {
        if (opMode.gamepad1.left_bumper){
            flyWheel1.setPower(1);
            flyWheel2.setPower(1);
        }else if(opMode.gamepad1.right_bumper){
            flyWheel1.setPower(-1);
            flyWheel2.setPower(-1);
        }else{
            flyWheel1.setPower(0);
            flyWheel2.setPower(0);
        }
    }
    public void alignStone(){
        if(opMode.gamepad1.y){
            alignerServo.setPower(1);
        } else if (opMode.gamepad1.x){
            alignerServo.setPower(-1);
        } else {
            alignerServo.setPower(0);
        }
    }


}
