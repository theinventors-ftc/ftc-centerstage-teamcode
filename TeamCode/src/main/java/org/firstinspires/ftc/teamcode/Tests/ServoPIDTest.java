package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;

@Disabled
@TeleOp
public class ServoPIDTest extends LinearOpMode {
    private CRServoImplEx servo;
    private AnalogInput servoPos;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    private PIDController controller;

    public double analogToDegrees(AnalogInput analogInput) {
        return analogInput.getVoltage() / analogInput.getMaxVoltage() * 360 - 180;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServoImplEx.class, "servo");
        servoPos = hardwareMap.get(AnalogInput.class, "servo_pos");
        controller = new PIDController(0.5, 0.0, 0);
        controller.setSetPoint(0);

        waitForStart();

        while (opModeIsActive()) {
            servo.setPower(-controller.calculate(2 * analogToDegrees(servoPos)/360));
            telemetry.addData("Servo Angle: ", analogToDegrees(servoPos));
//            telemetry.addData("Servo Angle: ", servoPos.getMaxVoltage());
            telemetry.update();
        }
    }
}
