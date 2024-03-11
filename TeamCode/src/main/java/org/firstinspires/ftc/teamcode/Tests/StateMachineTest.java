package org.firstinspires.ftc.teamcode.Tests;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.inventors.ftc.robotbase.controllers.StateMachine;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@TeleOp
public class StateMachineTest extends LinearOpMode {
    private GamepadExEx driveOP;
    private StateMachine stateMachine;

    @Override
    public void runOpMode() throws InterruptedException {
        driveOP = new GamepadExEx(gamepad1);
        stateMachine = new StateMachine(() -> driveOP.getButton(GamepadKeys.Button.A), 2000);

        waitForStart();

        while (opModeIsActive()) {
            stateMachine.update();
            telemetry.addData("Raw Raw: ", driveOP.getButton(GamepadKeys.Button.A));
            telemetry.addData("Raw: ", stateMachine.isActive());
            telemetry.addData("W/ Time: ", stateMachine.isJustActive());
            telemetry.update();
        }
    }
}
