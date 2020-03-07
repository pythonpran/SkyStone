package org.firstinspires.ftc.teamcode.autonomous;

import android.speech.tts.TextToSpeech;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

@Autonomous
@Config
public class AutonomousBlue extends AutonomousSkyStoneOpMode {

    public static double forward = 25;
    public static double leftX = -3;
    public static double midX = 6;
    public static double rightX = 8;

    public static double leftXCycle2 = 14;
    public static double midXCycle2 = 23;
    public static double rightXCycle2 = 29;

    public static double pY = 0.3;
    public static double pX = 0.2;
    public static double brakeDistanceY = 15;
    public static double brakeDistanceX = 10;
    public static double brakeVelocity = 0.3;

    public static double buildingZoneX = -70;
    public static double foundationY = 30;
    public static double buildingZoneY = 21;

    @Override
    public void runOpMode() {
        initialize(true);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();
        createGlobalPositionUpdate();

        // synchronously move the actuator
        actuatormove(1, 600, () -> {
            actuator.setPower(0);
            reach.setPosition(0.25);
            spin.setPosition(0.2);
            sleep(1000);
            clamp.setPosition(0);
            System.err.println("actuator move");
        });

        telemetry.addData("Skystone Position", skyStoneLocation);
        telemetry.update();
        phoneCam.stopStreaming();
        double skyStoneXPosition;
        if (skyStoneLocation == SkyStoneLocation.LEFT) {
            skyStoneXPosition = leftX;
            move(leftX, forward - 7, pX, pY, brakeDistanceX, brakeDistanceY, brakeVelocity, 1500);
            sleep(1000); // give servos time to move
            move(leftX, forward, pX, 0.2, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        } else if (skyStoneLocation == SkyStoneLocation.MIDDLE) {
            skyStoneXPosition = midX;
            move(midX, forward - 7, pX, pY, brakeDistanceX, brakeDistanceY, brakeVelocity, 1500);
            sleep(1000); // give servos time to move
            move(midX, forward, pX, 0.2, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        } else if (skyStoneLocation == SkyStoneLocation.RIGHT) {
            skyStoneXPosition = rightX;
            move(rightX, forward - 7, pX, pY, brakeDistanceX, brakeDistanceY, brakeVelocity, 1500);
            sleep(1000); // give servos time to move
            move(rightX, forward, pX, 0.2, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        } else {
            TextToSpeech tts = new TextToSpeech(hardwareMap.appContext, null);
            tts.setLanguage(Locale.US);
            tts.speak("Cannot Detect", TextToSpeech.QUEUE_ADD, null);
            return;
        }

        clamp.setPosition(0.6); // grab block
        sleep(2000);
        lift.setPosition(0);
        sleep(2000);
        actuatormove(-1, 500);
        move(skyStoneXPosition, buildingZoneY, pX, pY, brakeDistanceX, 10, brakeVelocity, 10000); // move backward

        move(buildingZoneX, buildingZoneY, 0.4, pY, 5, brakeDistanceY, brakeVelocity, 5000); // move to building zone
        move(buildingZoneX, foundationY, pX, 0.2, 5, brakeDistanceY, brakeVelocity, 2000); // move to foundation

        clamp.setPosition(0);//drops block
        lift.setPosition(0.58);
        move(buildingZoneX, buildingZoneY, pX, pY, brakeDistanceX, 10, brakeVelocity, 1000); // move backward


        // Second Skystone ------------------------------------------

        if (skyStoneLocation == SkyStoneLocation.LEFT) {
            move(leftXCycle2, buildingZoneY, pX, pY, brakeDistanceX, 10, brakeVelocity, 5000);
            move(leftXCycle2, forward + 8, pX, 0.1, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        } else if (skyStoneLocation == SkyStoneLocation.MIDDLE) {
            move(midXCycle2, buildingZoneY, pX, pY, brakeDistanceX, 10, brakeVelocity, 5000);
            move(midXCycle2, forward + 8, pX, 0.1, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        } else if (skyStoneLocation == SkyStoneLocation.RIGHT) {
            move(rightXCycle2, buildingZoneY, pX, pY, brakeDistanceX, 10, brakeVelocity, 5000);
            move(rightXCycle2, forward + 8, pX, 0.1, brakeDistanceX, brakeDistanceY, brakeVelocity, 2000);
        }

        // grab block two
        clamp.setPosition(0.6);
        sleep(1000);
        lift.setPosition(0);
        sleep(3000);

        move(leftX, buildingZoneY, pX, pY,
                brakeDistanceX, 10, brakeVelocity, 1000); // move backward

        move(buildingZoneX, buildingZoneY, 0.4, pY, 5, brakeDistanceY, brakeVelocity, 5000); // move to building zone
        move(buildingZoneX, 45, pX, 0.2, 5, brakeDistanceY, brakeVelocity, 2500); // move to foundation

        clamp.setPosition(0);//drops block
        lift.setPosition(0.58);
        lift.setPosition(0);
        sleep(1000);

        reach.setPosition(0.45);

        foundationGrabberLeft.setPosition(1.0);
        foundationGrabberRight.setPosition(0.0);

        sleep(1000);

        move(buildingZoneX, -3, pX, 0.6, 5, brakeDistanceY, brakeVelocity, 2000); // back away from foundation
        foundationGrabberLeft.setPosition(0.0);
        foundationGrabberRight.setPosition(1.0);
        sleep(100);
        move(-30, 5, 0.6, 0.6, 5, brakeDistanceY, brakeVelocity, 10000); // side drive away from foundation


//        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
//        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
//        telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
//
//        telemetry.addData("Vertical left encoder position", rightRear.getCurrentPosition());
//        telemetry.addData("Vertical right encoder position", leftFront.getCurrentPosition());
//        telemetry.addData("horizontal encoder position", rightFront.getCurrentPosition());

//        telemetry.update();
//        sleep(1000);
    }
}

