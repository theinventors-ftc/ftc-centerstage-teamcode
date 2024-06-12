package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;


//@Disabled
@TeleOp (name = "DistanceSensorTest", group = "Tests")
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensorEx distanceSensor;

    @Override
    public void runOpMode() {
        distanceSensor = new DistanceSensorEx(hardwareMap, "distance_sensor");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
