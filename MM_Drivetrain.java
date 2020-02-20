package org.firstinspires.ftc.teamcode.opmodes2019skystone;

import android.view.Display;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;


public class MM_Drivetrain {
    private BNO055IMU imu;
    private DcMotor LMotor = null;
    private DcMotor RMotor = null;
    private Servo foundationServo = null;
    private Servo rightFoundationGrabber = null;
    private Servo leftFoundationGrabber = null;

    private final double P_COEFF = 0.025;  //first attempt .02, PI method .025
    private final double D_COEFF = 0.0025;  //first attempt .0015, PI method .0025
    private final double I_COEFF = 0.00125;  //first attempt .001, PI method .0025
    public static final double HEADING_THRESHOLD = .75;
    private final double RANGE_THRESHOLD = .25;
    private final double MINIMUM_POWER = .075;
    private final double WHEEL_DIAMETER = 4.3;
    private final double TICKS_PER_ROTATION = 537.6;
    private final double TICKS_PER_INCH = TICKS_PER_ROTATION / (WHEEL_DIAMETER * 3.14);

    double previousError = 0;
    double derivative;
    double integral = 0;
    double deltaT = 0;

    private LinearOpMode opMode = null;
    private boolean isHandled = false;
    private boolean isFast = true;
    OpenGLMatrix position = null;

    private DistanceSensor leftRange;
    private DistanceSensor rightRange;
    private ModernRoboticsI2cRangeSensor MRleftRange;
    private ModernRoboticsI2cRangeSensor MRrightRange;

    public MM_Drivetrain(LinearOpMode opMode) {
        this.opMode = opMode;

        //init motors
        LMotor = opMode.hardwareMap.get(DcMotor.class, "LMotor");
        RMotor = opMode.hardwareMap.get(DcMotor.class, "RMotor");
        LMotor.setDirection(DcMotor.Direction.FORWARD);
        RMotor.setDirection(DcMotor.Direction.REVERSE);
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //don't forget that the brakes are on
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoder();
        setEncoderTargets(0, 0);

        //init sensors
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        leftRange = opMode.hardwareMap.get(DistanceSensor.class,"leftRange");
        rightRange = opMode.hardwareMap.get(DistanceSensor.class,"rightRange");
        MRleftRange = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class,"MRleftRange");
        MRrightRange = opMode.hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "MRrightRange");

        //init servo
        foundationServo = opMode.hardwareMap.get(Servo.class, "foundationServo");
        foundationServo.setPosition(0);

        rightFoundationGrabber = opMode.hardwareMap.get(Servo.class, "rightFoundGrab");
        rightFoundationGrabber.setPosition(0);
        leftFoundationGrabber = opMode.hardwareMap.get(Servo.class, "leftFoundGrab");
        leftFoundationGrabber.setPosition(1);
    }

    public void rangeTest(){
        while (opMode.opModeIsActive()) {
            opMode.telemetry.addData("Left Range", MRleftRange.getDistance(DistanceUnit.INCH));
            opMode.telemetry.addData("Right Range", MRrightRange.getDistance(DistanceUnit.INCH));
            opMode.telemetry.update();
        }
    }
    public void unbrake(){
        LMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void gyroTurnPID(double speed, double angle) {
        double power;
        double error = angle - getAngle();
        opMode.resetStartTime();
        while (opMode.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD)){
            error = correctError(error);
            power = PIDcontrol(speed,error);
            if((power < MINIMUM_POWER && power > 0) || (power > -MINIMUM_POWER && power < 0)) {
                if (power > 0){
                    power = MINIMUM_POWER;
                }else{
                    power = -MINIMUM_POWER;
                }
            }
            if (error < 0) power = -power;
            LMotor.setPower(-power);
            RMotor.setPower(power);
            opMode.telemetry.addData("Actual Angle", getAngle());
            opMode.telemetry.addData("Target Angle",angle);
            opMode.telemetry.addData("Error",error);
            opMode.telemetry.addData("Left Power",LMotor.getPower());
            opMode.telemetry.addData("Right Power",RMotor.getPower());
            opMode.telemetry.update();
            error = angle - getAngle();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    //find the power to drive using PID control, don't forget to manipulate the output for the case you are using it in
    public double PIDcontrol(double speed, double error){
        double power;

        deltaT = opMode.getRuntime();
        integral = integral + (error * deltaT);
        derivative = (error - previousError) / deltaT;
        opMode.resetStartTime();
        power = speed * (Math.abs(error) * P_COEFF) + (derivative * D_COEFF) + (integral * I_COEFF);
        previousError = error;
        return power;
    }



    public void gyroTurn(double speed, double angle){
        runWithEncoder();
        double error = angle - getAngle();
        double power;
        while (opMode.opModeIsActive() && (Math.abs(error) > HEADING_THRESHOLD)) {
            error = correctError(angle - getAngle());
            power = exponentialControl(error,speed,.0075,2);
            if (error < 0) {
                power = -power;
            }
            LMotor.setPower(-power);
            RMotor.setPower(power);

        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public double exponentialControl(double error, double speed, double coeff, int exponent){
        return speed * (Math.pow((Math.abs(error)*coeff),exponent) + MINIMUM_POWER);
    }

    public double correctError(double error) {
        if(error > 180){
            error = error - 360;
        }else if(error < -180) {
            error = error + 360;
        }
        return error;
    }

    public double getAngle(){
        double angle = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle;
        return angle;
    }

    public void driveToRange(double speed, double inches){
        double leftError = leftRange.getDistance(DistanceUnit.INCH) - inches;
        double rightError = rightRange.getDistance(DistanceUnit.INCH) - inches;
        double leftPower;
        double rightPower;
        while (opMode.opModeIsActive() && (Math.abs(leftError) > RANGE_THRESHOLD || Math.abs(rightError) > RANGE_THRESHOLD)){
            leftError = leftRange.getDistance(DistanceUnit.INCH) - inches;
            rightError = rightRange.getDistance(DistanceUnit.INCH) - inches;

            if(Math.abs(leftError) > RANGE_THRESHOLD){
                leftPower = exponentialControl(leftError, speed, .04, 2);
            }else{
                leftPower = 0;
            }
            if(Math.abs(rightError) > RANGE_THRESHOLD) {
                rightPower = exponentialControl(rightError, speed, .04, 2);
            }else{
                rightPower = 0;
            }

            if (leftError < 0) leftPower = -leftPower;
            if (rightError < 0) rightPower = -rightPower;

            RMotor.setPower(-leftPower);// the reason the power is negative is because the range sensors are on the back
            LMotor.setPower(-rightPower);

            opMode.telemetry.addData("left power",leftPower);
            opMode.telemetry.addData("left error",leftError);
            opMode.telemetry.addData("right power", rightPower);
            opMode.telemetry.addData("right error", rightError);
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void driveWithInches(double inches, double speed) {
        int newLeftTarget;
        int newRightTarget;

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(speed);
        RMotor.setPower(speed);

        while (opMode.opModeIsActive() && (RMotor.isBusy() || LMotor.isBusy())) {
            opMode.telemetry.addData("left encoder:", LMotor.getCurrentPosition());
            opMode.telemetry.addData("right encoder:", RMotor.getCurrentPosition());
            opMode.telemetry.addData("target Position:", LMotor.getTargetPosition());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void driveWithInches(double inches, double speed, MM_Vuforia vuforia) {
        int newLeftTarget;
        int newRightTarget;
        boolean targetSeen = false;

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        LMotor.setPower(speed);
        RMotor.setPower(speed);

        while (opMode.opModeIsActive() && (RMotor.isBusy() || LMotor.isBusy()) && !targetSeen) {
            position = vuforia.getDistanceToSkyStone();
            if (position != null) {
                targetSeen = true;
            }
            opMode.telemetry.addData("left encoder:", LMotor.getCurrentPosition());
            opMode.telemetry.addData("right encoder:", RMotor.getCurrentPosition());
            opMode.telemetry.addData("target Position:", LMotor.getTargetPosition());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }


    public void gyroDrive(double inches, double angle, double speed){
        int newLeftTarget;
        int newRightTarget;
        double error = angle - getAngle();

        runWithEncoder();
        runToPosition();

        newLeftTarget = LMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        newRightTarget = RMotor.getCurrentPosition() + (int) (inches * TICKS_PER_INCH);
        LMotor.setTargetPosition(newLeftTarget);
        RMotor.setTargetPosition(newRightTarget);

        while (opMode.opModeIsActive() && (RMotor.isBusy() || LMotor.isBusy())) {
            error = correctError(angle - getAngle());
            double power = exponentialControl(error,speed,.0125,2);
            if (error < 0) {
                power = -power;
            }
            LMotor.setPower(speed-power);
            RMotor.setPower(speed+power);
            opMode.telemetry.addData("Angle:", getAngle());
            opMode.telemetry.addData("Error:", error);
            opMode.telemetry.addData("left power", LMotor.getPower());
            opMode.telemetry.addData("right power", RMotor.getPower());
            opMode.telemetry.addData("left encoder:", LMotor.getCurrentPosition());
            opMode.telemetry.addData("right encoder:", RMotor.getCurrentPosition());
            opMode.telemetry.addData("target Position:", LMotor.getTargetPosition());
            opMode.telemetry.update();
        }
        LMotor.setPower(0);
        RMotor.setPower(0);
    }

    public void setMotorPowersSame(double power){
        LMotor.setPower(power);
        RMotor.setPower(power);
        opMode.telemetry.addData("left encoder",LMotor.getCurrentPosition());
        opMode.telemetry.addData("right encoder", RMotor.getCurrentPosition());
        opMode.telemetry.update();
    }

    public double currentPosition(){
        opMode.telemetry.addData("current position", LMotor.getCurrentPosition());
        opMode.telemetry.addData("current position (calc)",LMotor.getCurrentPosition()/TICKS_PER_INCH);
        return LMotor.getCurrentPosition()/TICKS_PER_INCH;
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
            foundationServo.setPosition(1);
            rightFoundationGrabber.setPosition(1);
            leftFoundationGrabber.setPosition(0);
        } else {
            foundationServo.setPosition(0);
            rightFoundationGrabber.setPosition(0);
            leftFoundationGrabber.setPosition(1);
        }
    }

    public void runToPosition() {
        LMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void resetEncoder() {
        LMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runWithEncoder();
    }

    public void runWithEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runWithoutEncoder() {
        LMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setEncoderTargets(int left, int right){
        LMotor.setTargetPosition(left);
        RMotor.setTargetPosition(right);
    }

    public void foundationUp() {
        foundationServo.setPosition(0);
    }

    public void foundationDown() {
        foundationServo.setPosition(1);
    }

    //robot is backwards so all distances are negative
    //blue alliance is false, red is true (same for all uses)
    public void driveLeftConfig(boolean alliance) {
        if (alliance){
            driveWithInches(-8.5,.25);
        }
        else {
            driveWithInches(-8.5,.25);
        }
    }

    public void driveCenterConfig(boolean alliance) {
        if (alliance) {
            gyroTurn(.25,-20);
            driveWithInches(-8.5,.25);
        }
        else {
            gyroTurn(.25,20);
            driveWithInches(-8.5,.25);
        }
    }

    public void driveRightConfig(boolean alliance) {
        if (alliance) {
            driveWithInches(-8.5,.25);
        }
        else {
            driveWithInches(-8.5,.25);
        }
    }

    public void driveToSkystone(int stonePosition, boolean alliance) {
        if (stonePosition == 0) {
            driveLeftConfig(alliance);
        }
        else if(stonePosition == 1) {
            driveCenterConfig(alliance);
        }
        else{
            driveRightConfig(alliance);
        }
    }

    public void setMotorPowers(double leftPower, double rightPower){
        LMotor.setPower(leftPower);
        RMotor.setPower(rightPower);
    }
}