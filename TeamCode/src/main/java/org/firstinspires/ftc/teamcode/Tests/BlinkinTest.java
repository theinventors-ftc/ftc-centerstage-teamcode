package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BlinkinTest extends LinearOpMode {
    private RevBlinkinLedDriver driver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
