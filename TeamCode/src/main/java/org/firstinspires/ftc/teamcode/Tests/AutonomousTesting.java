package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "AutonomousTesting", group = "Final Autonomous")
public class AutonomousTesting extends LinearOpMode {

    protected IntakeArmSubsystem intakeArmSubsystem;
    protected IntakeSubsystem intakeSubsystem;

    @Override
    public void runOpMode() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
//        schedule(new SequentialCommandGroup(
//                new InstantCommand(intakeSubsystem::run, intakeSubsystem),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(5), intakeArmSubsystem),
//                new WaitCommand(300),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(4), intakeArmSubsystem),
//                new WaitCommand(3000),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(3), intakeArmSubsystem),
//                new WaitCommand(300),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(2), intakeArmSubsystem),
//                new WaitCommand(3000),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(1), intakeArmSubsystem)
//        ));

        waitForStart();

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::run, intakeSubsystem),
                new WaitCommand(150),
                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(5), intakeArmSubsystem),
                new WaitCommand(2000),
                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(4), intakeArmSubsystem),
                new WaitCommand(5000),
                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(3), intakeArmSubsystem),
                new WaitCommand(2000),
                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(2), intakeArmSubsystem),
                new WaitCommand(5000),
                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(1), intakeArmSubsystem),
                new WaitCommand(1000),
                new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem),
                new WaitCommand(2000)
        ));

        while (!isStopRequested() && opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
