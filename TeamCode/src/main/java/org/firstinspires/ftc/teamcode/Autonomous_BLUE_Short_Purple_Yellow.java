package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BLUE_Short_Purple_Yellow", group = "Final Autonomous")
public class Autonomous_BLUE_Short_Purple_Yellow extends AutonomousBase {

    private Pose2d HomePose = new Pose2d(
        RoadRunnerSubsystem.Tile/2, 3 * RoadRunnerSubsystem.Tile - 7.93,
        Math.toRadians(270)
    );

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(Alliance.BLUE);
        RR = new RoadRunnerSubsystem_BLUE(
            drive, HomePose, RoadRunnerSubsystem.StartingPosition.SHORT,
            RoadRunnerSubsystem.Path.INNER, RoadRunnerSubsystem.PixelStack.INNER,
            RoadRunnerSubsystem.ParkingPosition.INNER, telemetry);
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

        temp = scoring_randomization();
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