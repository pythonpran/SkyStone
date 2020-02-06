package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.SkyStoneOpMode;

public abstract class AutonomousSkyStoneOpMode extends SkyStoneOpMode {
    @Override
    protected void initialize(boolean calibrateIMU) {
        super.initialize(true);
    }
}
