package org.firstinspires.ftc.teamcode.testchassis.PurePursuit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.testchassis.TestChassisOdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.testchassis.TestChassisOpMode;
import org.firstinspires.ftc.teamcode.testchassis.TestChassisOdometryGlobalCoordinatePosition;
public class TestChassisRobotMovement extends TestChassisOpMode {

    public void goToPosition(double x, double y, double movementSpeed, BNO055IMU imu, DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear){
        // Makes some changes in later videos

        TestChassisOdometryGlobalCoordinatePosition globalPositionUpdate = new TestChassisOdometryGlobalCoordinatePosition(leftRear, rightFront, rightRear, 75, imu);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();
        // Absolute angle = fixed point
        //
        double distanceToTarget = Math.hypot(x-  globalPositionUpdate.returnXCoordinate(), y - globalPositionUpdate.returnYCoordinate());
        double absoluteAngleToTarget = Math.atan2(y - globalPositionUpdate.returnYCoordinate(), x - globalPositionUpdate.returnXCoordinate()); //+ imu.getAngularOrientation().firstAngle
        double relativeAngleToPoint = MathFunctions.AngleWrap(absoluteAngleToTarget - (imu.getAngularOrientation().firstAngle - Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;

        // Normalized Vector:

        // Ratio of x / (x+y) allows the speed to be porportionally correct and be <1
        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        setXYSpeed(movementXPower * movementSpeed,movementYPower * movementSpeed, leftFront,  rightFront,  leftRear, rightRear);
    }
    public void setXYSpeed(double xPorSpeed, double yPorSpeed,DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear){
        double sideForwardPower = 14.6/31.5;
        leftFront.setPower((xPorSpeed + sideForwardPower * yPorSpeed)/2);
        rightFront.setPower((-xPorSpeed + sideForwardPower * yPorSpeed)/2);
        leftRear.setPower((-xPorSpeed + sideForwardPower * yPorSpeed)/2);
        rightRear.setPower((xPorSpeed + sideForwardPower * yPorSpeed)/2);
//        double r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
//        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
//        double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
//        double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
//        double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
