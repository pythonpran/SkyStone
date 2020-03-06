package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class SkyStoneOpMode extends LinearOpMode {

    protected static final double COUNTS_PER_INCH = 23.9;
    public static final double ODOMETER_COUNTS_PER_INCH = 2724 / 1.49606;

    protected DcMotor leftFront, leftRear, rightFront, rightRear; // wheels
    protected DcMotor lifterLeft, lifterRight, actuator; // arm
    protected Servo reach, lift, spin, clamp, cap, foundationGrabberLeft, foundationGrabberRight;

    protected BNO055IMU imu;
    protected ColorSensor colorSensor1, colorSensor2;
    protected DistanceSensor distanceSensor1, distanceSensor2;
    protected DigitalChannel downStop;

    protected void initializeHardwareDevices(boolean calibrateIMU) {
        telemetry.addData("Status", "Initializing hardware devices");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        if (calibrateIMU) calibrateIMU();
        telemetry.addData("Status", "Initializing hardware devices");
        telemetry.update();

        colorSensor1 = hardwareMap.get(ColorSensor.class, "colorSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "colorSensor2");
//        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        downStop = hardwareMap.get(DigitalChannel.class, "downStop");


        // Define and Initialize Motors
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        lifterLeft = hardwareMap.get(DcMotor.class, "lifterLeft");
        lifterRight = hardwareMap.get(DcMotor.class, "lifterRight");
        actuator = hardwareMap.get(DcMotor.class, "actuator");

        foundationGrabberLeft = hardwareMap.get(Servo.class, "foundationGrabberLeft");
        foundationGrabberRight = hardwareMap.get(Servo.class, "foundationGrabberRight");
        reach = hardwareMap.get(Servo.class, "reach");
        lift = hardwareMap.get(Servo.class, "lift");
        clamp = hardwareMap.get(Servo.class, "clamp");
        cap = hardwareMap.get(Servo.class, "cap");
        spin = hardwareMap.get(Servo.class, "spin");
    }

    protected void initialize(boolean calibrateIMU) {
        if (FtcDashboard.getInstance() != null && FtcDashboard.getInstance().getTelemetry() != null) {
//            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        }
        initializeHardwareDevices(calibrateIMU);

        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        lifterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        lifterRight.setDirection(DcMotorSimple.Direction.FORWARD);

        lifterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lifterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        downStop.setMode(DigitalChannel.Mode.INPUT);

        resetEncoders();


        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        actuator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Setting motors power");
        telemetry.update();

        // Set all motors to zero power
        brake();

        lifterLeft.setPower(0);
        lifterRight.setPower(0);

        actuator.setPower(0);

        initServoPositions();
        telemetry.addData("Status", "Done initializing");
        telemetry.update();
    }

    protected void initServoPositions() {
        foundationGrabberLeft.setPosition(0.0);
        foundationGrabberRight.setPosition(1.0);
        lift.setPosition(0.58);
        clamp.setPosition(0.5);
        cap.setPosition(1.0);
        spin.setPosition(0.52);
    }

    protected void calibrateIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Status", "Calibrating imu...");
            telemetry.update();
            sleep(50);
            idle();
        }
    }

    protected void brake() {
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }


    protected void resetEncoders() {
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
