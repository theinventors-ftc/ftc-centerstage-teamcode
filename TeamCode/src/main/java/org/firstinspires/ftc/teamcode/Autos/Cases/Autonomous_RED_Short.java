package org.firstinspires.ftc.teamcode.Autos.Cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autos.AutonomousBase;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem_RED;


@Autonomous(name = "RED_Short", group = "Final Autonomous")
public class Autonomous_RED_Short extends AutonomousBase {
    private Pose2d HomePose = new Pose2d(
        RoadRunnerSubsystem.Tile/2, 3 * RoadRunnerSubsystem.TileInverted + 7.93,
        Math.toRadians(90)
    );

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(Alliance.RED);
        RR = new RoadRunnerSubsystem_RED(
            drive, HomePose, RoadRunnerSubsystem.StartingPosition.SHORT,
            RoadRunnerSubsystem.Path.INNER, RoadRunnerSubsystem.PixelStack.INNER,
            RoadRunnerSubsystem.ParkingPosition.INNER, telemetry
        );
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        // BACKDROP - YELLOW
        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem).schedule();
        randomizationPixelElevator().schedule();
        RR.runSpike_RandomizedBackdrop();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        backdropAlignment();

        temp = scoring_randomization();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        // STACK FIRST
        temp = new SequentialCommandGroup(new WaitCommand(600), resetElevator());
        temp.schedule();
        RR.runBackdrop_Station(0);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = stackStationIntake(5);
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = new SequentialCommandGroup(new WaitCommand(1800), elevator_first());

        temp.schedule();

        RR.runStation_Backdrop(0);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        backdropAlignment();

        temp = scoring();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        // STACK Second
        temp = new SequentialCommandGroup(new WaitCommand(600), resetElevator());
        temp.schedule();

        RR.runBackdrop_Station(1);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = stackStationIntake(3);
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = new SequentialCommandGroup(
                new WaitCommand(1800),
                elevator_second()
        );

        temp.schedule();

        RR.runStation_Backdrop(1);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//        backdropAlignment();

        temp = scoring();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = resetElevator();
        temp.schedule();
        RR.runParking();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        save_current_pose();

        reset();
    }
}