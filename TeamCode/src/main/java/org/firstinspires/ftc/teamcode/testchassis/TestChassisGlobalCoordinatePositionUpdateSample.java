package org.firstinspires.ftc.teamcode.testchassis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.Timer;
import java.util.TimerTask;

import org.firstinspires.ftc.teamcode.SkyStoneOpMode;
import org.firstinspires.ftc.teamcode.autonomous.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.testchassis.PurePursuit.TestChassisRobotMovement;


/**
 * Created by Sarthak on 6/1/2019.
 * Example OpMode that runs the GlobalCoordinatePosition thread and accesses the (x, y, theta) coordinate values
 */
@Autonomous
@Config
public class TestChassisGlobalCoordinatePositionUpdateSample extends TestChassisOpMode {

    public static final double ODOMETER_COUNTS_PER_INCH = 1882.0;
    public static double locationX = 1/8;
    public static double locationY = -3/8;
    double targetXPos;
    double targetYPos;

    @Override
    public void runOpMode() {
        initialize();

        //Reset the encoders
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        Reverse the direction of the odometry wheels. THIS WILL CHANGE FOR EACH ROBOT. Adjust the direction (as needed) of each encoder wheel
        such that when the rightRear and leftFront encoders spin forward, they return positive values, and when the
        rightFront encoder travels to the right, it returns positive value
        */
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set the mode of the odometry encoders to RUN_WITHOUT_ENCODER
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Init complete
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();
        /**
         * *****************
         * OpMode Begins Here
         * *****************
         */
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        TestChassisOdometryGlobalCoordinatePosition globalPositionUpdate = new TestChassisOdometryGlobalCoordinatePosition(leftRear, rightFront, rightRear, 75, imu);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
            telemetry.addData("Vertical left encoder position", -leftRear.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", -rightFront.getCurrentPosition());
            telemetry.addData("horizontal encoder position", -rightRear.getCurrentPosition());
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Orientation (Degrees-imu)", imu.getAngularOrientation().firstAngle);
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
//            TestChassisRobotMovement move = new TestChassisRobotMovement();
//            move.goToPosition(5,6,0.5, imu, leftFront, rightFront, leftRear, rightRear);
            targetXPos = 0;
            targetYPos = 12;
            while(opModeIsActive()) {
                if ((Math.abs(targetXPos - globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0) && (Math.abs(targetYPos - globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0)) {

                    break;
                } else {
                    setXYSpeed(targetXPos, targetYPos, 0.5);
                }
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Vertical left encoder position", -leftRear.getCurrentPosition());
                telemetry.addData("Vertical right encoder position", -rightFront.getCurrentPosition());
                telemetry.addData("horizontal encoder position", -rightRear.getCurrentPosition());
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                telemetry.addData("Orientation (Degrees-imu)", imu.getAngularOrientation().firstAngle);
                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
            targetXPos = 12;
            targetYPos = 12;
            while(opModeIsActive()) {
                if ((Math.abs(targetXPos - globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0) && (Math.abs(targetYPos - globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0)) {

                    break;
                } else {
                    setXYSpeed(targetXPos, targetYPos-12, 0.5);
                }
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Vertical left encoder position", -leftRear.getCurrentPosition());
                telemetry.addData("Vertical right encoder position", -rightFront.getCurrentPosition());
                telemetry.addData("horizontal encoder position", -rightRear.getCurrentPosition());
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                telemetry.addData("Orientation (Degrees-imu)", imu.getAngularOrientation().firstAngle);
                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
            targetXPos = 0;
            targetYPos = 0;
            while(opModeIsActive()) {
                if ((Math.abs(targetXPos - globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0) && (Math.abs(targetYPos - globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH) < 2.0)) {

                    break;
                } else {
                    setXYSpeed(targetXPos-12, targetYPos-12, 0.5);
                }
                telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
                telemetry.addData("Vertical left encoder position", -leftRear.getCurrentPosition());
                telemetry.addData("Vertical right encoder position", -rightFront.getCurrentPosition());
                telemetry.addData("horizontal encoder position", -rightRear.getCurrentPosition());
                telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
                telemetry.addData("Orientation (Degrees-imu)", imu.getAngularOrientation().firstAngle);
                telemetry.addData("Thread Active", positionThread.isAlive());
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);



        //Stop the thread
        globalPositionUpdate.stop();
    }
    public void setXYSpeed(double xPos, double yPos, double powerRatio){
        double xPorSpeed = xPos / (Math.abs(xPos) + Math.abs(yPos));
        double yPorSpeed = yPos / (Math.abs(xPos) + Math.abs(yPos));
        double sideForwardPower = 18/31.5;
        leftFront.setPower((xPorSpeed +  sideForwardPower * yPorSpeed)/2 * powerRatio) ;
        rightFront.setPower((-xPorSpeed + sideForwardPower * yPorSpeed)/2 * powerRatio) ;
        leftRear.setPower((-xPorSpeed +  sideForwardPower * yPorSpeed)/2 * powerRatio);
        rightRear.setPower((xPorSpeed +  sideForwardPower * yPorSpeed)/2 * powerRatio);

//        double r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
//        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
//        double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
//        double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
//        double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);
    }
}
