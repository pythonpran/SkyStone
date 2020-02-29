package org.firstinspires.ftc.teamcode.autonomous;

import android.speech.tts.TextToSpeech;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Locale;

@Autonomous
@Config
public class AutonomousBlue extends AutonomousSkyStoneOpMode {

    public static double forward = 23;
    public static double leftX = -10;
    public static double midX = 5;
    public static double rightX = 17;

    public static double pY = 0.3;
    public static double pX = 0.2;
    public static double brakeDistanceY = 15;
    public static double brakeDistanceX = 10;
    public static double brakeVelocity = 0.3;

    @Override
    public void runOpMode() {
        initialize(true);
        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        waitForStart();

        // synchronously move the actuator
        actuatormove(1, 700, () -> {
            actuator.setPower(0);
            reach.setPosition(0.25);
            clamp.setPosition(0.1);
            spin.setPosition(0.2);
        });

        createGlobalPositionUpdate();
        telemetry.addData("Skystone Position", skyStoneLocation);
        telemetry.update();
        phoneCam.stopStreaming();
        if (skyStoneLocation == SkyStoneLocation.LEFT) {
            move(leftX, forward, pX, pY, brakeDistanceY, brakeDistanceX, brakeVelocity);
        } else if (skyStoneLocation == SkyStoneLocation.MIDDLE) {
            move(midX, forward, pX, pY, brakeDistanceY, brakeDistanceX, brakeVelocity);
        } else if (skyStoneLocation == SkyStoneLocation.RIGHT) {
            move(rightX, forward, pX, pY, brakeDistanceY, brakeDistanceX, brakeVelocity);
        } else {
            TextToSpeech tts = new TextToSpeech(hardwareMap.appContext, null);
            tts.setLanguage(Locale.US);
            tts.speak("Cannot Detect", TextToSpeech.QUEUE_ADD, null);
            return;
        }

        clamp.setPosition(0.62);
        sleep(200);
        lift.setPosition(0);
        sleep(300);
        actuatormove(-1, 300);
        move(leftX, 13, pX, pY, 10, brakeDistanceX, brakeVelocity); // move backward

        move(-80, 13, 0.4, pY, brakeDistanceY, 5, brakeVelocity); // move to building zone
        move(-80, forward + 10, pX, pY, brakeDistanceY, 5, brakeVelocity); // move to foundation

        sleep(1000);

        foundationGrabberLeft.setPosition(1.0);
        foundationGrabberRight.setPosition(0.0);
        sleep(2500);

        leftFront.setPower(1);
        leftRear.setPower(1);
        rightFront.setPower(0.3);
        rightRear.setPower(0.3);
        sleep(2000);
        brake();
        foundationGrabberLeft.setPosition(0.0);
        foundationGrabberRight.setPosition(1.0);
        clamp.setPosition(0.1);
        lift.setPosition(0.6);
        sleep(500);
        lift.setPosition(0);

        telemetry.addData("X Position", () -> globalPositionUpdate.returnXCoordinate() / ODOMETER_COUNTS_PER_INCH);
        telemetry.addData("Y Position", () -> globalPositionUpdate.returnYCoordinate() / ODOMETER_COUNTS_PER_INCH);
        telemetry.addData("Orientation (Degrees)", () -> globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical left encoder position", () -> rightRear.getCurrentPosition());
        telemetry.addData("Vertical right encoder position", () -> leftFront.getCurrentPosition());
        telemetry.addData("horizontal encoder position", () -> rightFront.getCurrentPosition());

        telemetry.update();
        sleep(1000);
    }

}

