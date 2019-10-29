package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MM_Drivetrain {
    private LinearOpMode opMode = null;

    private DcMotor LMotor = null;
    private DcMotor RMotor = null;

    private final double wheelDiameter = 4.3;
    private final double ticksPerRotation = 723.24;
    private final double ticksPerInch = ticksPerRotation / (wheelDiameter * 3.14);

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");

        LMotor.setDirection(DcMotor.Direction.REVERSE);
        RMotor.setDirection(DcMotor.Direction.FORWARD);
    }
    public void init() {
        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveWithInches(double inches, double speed) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * ticksPerInch);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * ticksPerInch);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(Math.abs(speed));
        RMotor.setPower(Math.abs(speed));

        while(RMotor.isBusy() || LMotor.isBusy() && opMode.opModeIsActive()){

        }
    }

        public void driveWithSticks(double leftPower, double rightPower) {
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }
}