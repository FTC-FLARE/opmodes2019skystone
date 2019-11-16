package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;


public class MM_Drivetrain {
    private BNO055IMU imu;
    private DcMotor LMotor = null;
    private DcMotor RMotor = null;
    private Servo foundationServo = null;

    private final double PROPORTIONAL_CONSTANT = 0.02;
    private final double HEADING_THRESHOLD = 1;
    private final double WHEEL_DIAMETER = 4.3;
    private final double TICKS_PER_ROTATION = 723.24;
    private final double TICKS_PER_INCH = TICKS_PER_ROTATION / (WHEEL_DIAMETER * 3.14);

    private LinearOpMode opMode = null;
    private Orientation angles;
    private boolean isHandled = false;
    private boolean isFast = true;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");
        LMotor.setDirection(DcMotor.Direction.REVERSE);
        RMotor.setDirection(DcMotor.Direction.FORWARD);
        resetEncoder();

        //init gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //init servo
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundationServo");
        foundationServo.setPosition(1);
    }

    private void resetEncoder() {
        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void gyroTurn(double speed, double angle) {
        while (opMode.opModeIsActive() && !onHeading(speed, angle)) {
        }
    }

    boolean onHeading(double speed, double angle) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        LMotor.setPower(leftSpeed);
        RMotor.setPower(rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {

        double robotError;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error) {
        return Range.clip(error * PROPORTIONAL_CONSTANT, -1, 1);
    }

    public void driveWithInches(double inches, double speed) {
        int newLeftTarget;
        int newRightTarget;

        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(Math.abs(speed));
        RMotor.setPower(Math.abs(speed));

        while (RMotor.isBusy() || LMotor.isBusy() && opMode.opModeIsActive()) {

        }
    }

    public void driveWithSticks() {
        double leftPower = -opMode.gamepad1.left_stick_y;
        double rightPower = -opMode.gamepad1.right_stick_y;
        if (opMode.gamepad1.a && !isHandled) {
            isFast = !isFast;
            isHandled = true;
        } else if (!opMode.gamepad1.a && isHandled) {
            isHandled = false;
        }
        if(!isFast){
            leftPower = leftPower / 4;
            rightPower = rightPower / 4;
        }

        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }

    public void controlFoundation(){
        if (opMode.gamepad2.a) {
            foundationServo.setPosition(.45);
        } else {
            foundationServo.setPosition(1);
        }
    }

    public void runToPosition() {
        LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}