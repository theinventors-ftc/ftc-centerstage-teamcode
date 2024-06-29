package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "RED_Long_Purple_Yellow", group = "Final Autonomous")
public class Autonomous_RED_Long_Purple_Yellow extends AutonomousBase {

    private Pose2d HomePose = new Pose2d(
        1.5 * RoadRunnerSubsystem.TileInverted, 3 * RoadRunnerSubsystem.TileInverted + 7.93,
        Math.toRadians(90)
    );

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(Alliance.RED);
        RR = new RoadRunnerSubsystem_RED(
            drive, HomePose, RoadRunnerSubsystem.StartingPosition.LONG,
            RoadRunnerSubsystem.Path.INNER, RoadRunnerSubsystem.PixelStack.INNER,
            RoadRunnerSubsystem.ParkingPosition.MID, telemetry
        );
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem).schedule();
        RR.runSpike_Station();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = new SequentialCommandGroup(
                new WaitCommand(2400),
                randomizationPixelElevator()
        );

        temp.schedule();
        RR.runStation_RandomizedBackdrop();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        temp = scoring();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        temp = elevator_first();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        //Parking
        temp = resetElevator();
        temp.schedule();
        RR.runParking();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        PoseStorage.currentPose = drive.getPoseEstimate();

        reset();
    }
}