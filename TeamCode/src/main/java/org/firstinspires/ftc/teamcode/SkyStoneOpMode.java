package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class SkyStoneOpMode extends LinearOpMode {

    protected ElapsedTime runtime = new ElapsedTime();

    protected Orientation angles;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    protected DcMotor leftFront, leftRear, rightFront, rightRear; // wheels
    protected DcMotor lifterTop, lifterBottom, actuatorHorizontal, actuatorVertical; // arm
    protected Servo foundationGrabberLeft, foundationGrabberRight, clamp, spin;

    protected BNO055IMU imu;
    protected ColorSensor colorSensor1, colorSensor2;
    protected DigitalChannel downStop;

    protected void initialize(boolean calibrateIMU) {
        // The IMU sensor object
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (calibrateIMU && !isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("Mode", "Calibrating imu...");
            telemetry.update();
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "Initializing hardware devices");
        telemetry.update();


        colorSensor1 = hardwareMap.get(ColorSensor.class, "sensor_color_distance1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance2");
        downStop = hardwareMap.get(DigitalChannel.class, "downStop");


        // Define and Initialize Motors
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        lifterTop = hardwareMap.get(DcMotor.class, "lifterTop");
        lifterBottom = hardwareMap.get(DcMotor.class, "lifterBottom");

        actuatorHorizontal = hardwareMap.get(DcMotor.class, "actuatorHorizontal");
        actuatorVertical = hardwareMap.get(DcMotor.class, "actuatorVertical");


        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        lifterTop.setDirection(DcMotorSimple.Direction.REVERSE);
        lifterBottom.setDirection(DcMotorSimple.Direction.FORWARD);


        downStop.setMode(DigitalChannel.Mode.INPUT);


        //Set to brake mode
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifterTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifterBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        actuatorVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuatorHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "Setting motors power");
        telemetry.update();

        // Set all motors to zero power
        leftRear.setPower(0);
        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);

        lifterTop.setPower(0);
        lifterBottom.setPower(0);

        actuatorHorizontal.setPower(0);
        actuatorVertical.setPower(0);

        foundationGrabberLeft = hardwareMap.get(Servo.class, "foundationGrabber");
        foundationGrabberRight = hardwareMap.get(Servo.class, "foundationGrabber");
        foundationGrabberLeft.setPosition(0.0);
        foundationGrabberRight.setPosition(0.0);
        clamp = hardwareMap.get(Servo.class, "clamp");
        clamp.setPosition(1.0);
        spin = hardwareMap.get(Servo.class, "spin");
        spin.setPosition(0.45);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        telemetry.addData("Mode", "Done initializing");
        telemetry.update();
    }

}
