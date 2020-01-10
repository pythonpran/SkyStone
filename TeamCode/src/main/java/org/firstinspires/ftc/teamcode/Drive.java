package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.math.BigDecimal;
import java.math.RoundingMode;

@TeleOp(name = "Drive")
public class Drive extends SkyStoneOpMode {

    private boolean slowMode = false, accelerating = false;
    private double acceleratePower = 0.0;

    @Override
    public void runOpMode() {
        initialize(false);
        boolean lastDPLeftState = false;
        boolean lastXState = false;
        boolean lastBState2 = false;
        boolean lastAState2 = false;
        composeTelemetry();
        waitForStart();

        while (opModeIsActive()) {
            double lifterPower = -gamepad2.right_stick_y;
            double r = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            double v1 = (r * Math.cos(robotAngle) + rightX) * Math.sqrt(2);
            double v2 = (r * Math.sin(robotAngle) - rightX) * Math.sqrt(2);
            double v3 = (r * Math.sin(robotAngle) + rightX) * Math.sqrt(2);
            double v4 = (r * Math.cos(robotAngle) - rightX) * Math.sqrt(2);


            if (gamepad1.dpad_down) {
                accelerating = false;
                acceleratePower = 0.0;
            }

            if (slowMode && !accelerating) {
                leftFront.setPower(v1 / 2);
                rightFront.setPower(v2 / 2);
                leftRear.setPower(v3 / 2);
                rightRear.setPower(v4 / 2);
            } else if (!accelerating) {
                leftFront.setPower(v1);
                rightFront.setPower(v2);
                leftRear.setPower(v3);
                rightRear.setPower(v4);
            }

            if (gamepad1.dpad_up) {
                accelerating = true;
                if (acceleratePower < 0.5) {
                    acceleratePower += 0.001;
                }
                rightFront.setPower(acceleratePower);
                rightRear.setPower(acceleratePower);
                leftRear.setPower(acceleratePower);
                leftFront.setPower(acceleratePower);
            }

            if (acceleratePower > 0 && !gamepad1.dpad_up) {
                acceleratePower -= 0.001;
                if (acceleratePower <= 0) {
                    acceleratePower = 0;
                    accelerating = false;
                }
                rightFront.setPower(-acceleratePower);
                rightRear.setPower(-acceleratePower);
                leftRear.setPower(-acceleratePower);
                leftFront.setPower(-acceleratePower);
            }

            if (downStop.getState()) { //is not pressed
                if (lifterPower == 0) {
                    lifterPower = 0.15;
                }

                if (lifterPower < 0) {
                    lifterPower /= 5;
                }
                lifterTop.setPower(lifterPower);
                lifterBottom.setPower(lifterPower);
            } else if (!downStop.getState()) { //is pressed
                lifterPower = lifterPower < 0 ? 0 : lifterPower; // if lifterPower < 0 set it to 0


                lifterTop.setPower(lifterPower);
                lifterBottom.setPower(lifterPower);
            }

            actuatorVertical.setPower(-gamepad2.left_stick_y);

            if (gamepad1.x && !lastXState) {
                slowMode = !slowMode;
            }
            lastXState = gamepad1.x;


            if (gamepad1.a && !lastAState2) {
                clamp.setPosition(!(clamp.getPosition() >= 0.07 && clamp.getPosition() <= 0.13) ? 0.1 : 0.48);
            }
            lastAState2 = gamepad2.a;
            if (gamepad1.b && !lastBState2) {
                spin.setPosition(!(spin.getPosition() >= 0.37 && spin.getPosition() <= 0.43) ? 0.4 : 1.0);

            }
            lastBState2 = gamepad2.b;

            if (gamepad1.dpad_left && !lastDPLeftState) {
                // if foundationGrabber != 0.0, set it to 1.0, else set it to 0.0
                foundationGrabberLeft.setPosition(foundationGrabberLeft.getPosition() != 0.0 ? 0.0 : 1.0);
                foundationGrabberRight.setPosition(foundationGrabberRight.getPosition() != 0.0 ? 0.0 : 1.0);
            }
            lastDPLeftState = gamepad1.dpad_left;

            telemetry.update();
        }
    }


    private void composeTelemetry() {
        telemetry.addLine().addData("leftFront", () -> leftFront.getPower());
        telemetry.addLine().addData("leftRear", () -> leftRear.getPower());
        telemetry.addLine().addData("rightFront", () -> rightFront.getPower());
        telemetry.addLine().addData("rightRear", () -> rightRear.getPower());
        telemetry.addLine().addData("lifter Top", () -> lifterTop.getPower());
        telemetry.addLine().addData("lifterBottom", () -> lifterBottom.getPower());
        telemetry.addLine().addData("actuatorVertical" , () -> actuatorVertical.getPower());
        telemetry.addLine().addData("foundationGrabberLeft", () -> foundationGrabberLeft.getPosition());
        telemetry.addLine().addData("foundationGrabberRight", () -> foundationGrabberRight.getPosition());
        telemetry.addLine().addData("clamp", () -> clamp.getPosition());
        telemetry.addLine().addData("spin", () -> spin.getPosition());
        telemetry.addLine().addData("Down Touch Sensor pressed: ", () -> !downStop.getState());
    }

    private static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();

        BigDecimal bd = new BigDecimal(Double.toString(value));
        bd = bd.setScale(places, RoundingMode.HALF_UP);
        return bd.doubleValue();
    }

}
