package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class BlinkinTest extends LinearOpMode {
    private RevBlinkinLedDriver driver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    @Override
    public void runOpMode() throws InterruptedException {
        driver = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        driver.setPattern(pattern);

        waitForStart();

        while (opModeIsActive()) {
            driver.setPattern(pattern);
            sleep(500);
            driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            sleep(500);
        }
    }
}
