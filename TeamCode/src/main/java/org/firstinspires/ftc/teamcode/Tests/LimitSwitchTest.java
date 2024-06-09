package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;


//@Disabled
@TeleOp (name = "LimitSwitchTesting", group = "Tests")
public class LimitSwitchTest extends LinearOpMode {
    DigitalChannel limitSwitch;

    @Override
    public void runOpMode() {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "slider_limit");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Switch State: ", limitSwitch.getState());
            telemetry.update();
        }
    }
}
