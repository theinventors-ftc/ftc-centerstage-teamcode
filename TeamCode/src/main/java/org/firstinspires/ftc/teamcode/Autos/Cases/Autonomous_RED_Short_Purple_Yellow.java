package org.firstinspires.ftc.teamcode.Autos.Cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autos.AutonomousBase;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem_RED;

@Disabled
@Autonomous(name = "RED_Short_Purple_Yellow", group = "Final Autonomous")
public class Autonomous_RED_Short_Purple_Yellow extends AutonomousBase {
    private Pose2d HomePose = new Pose2d(
        RoadRunnerSubsystem.Tile/2, 3 * RoadRunnerSubsystem.TileInverted + 6.93,
        Math.toRadians(90)
    );

    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(Alliance.RED);
        RR = new RoadRunnerSubsystem_RED(
            drive, HomePose, RoadRunnerSubsystem.StartingPosition.SHORT,
            RoadRunnerSubsystem.Path.INNER, RoadRunnerSubsystem.PixelStack.INNER,
            RoadRunnerSubsystem.ParkingPosition.OUTER, telemetry
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

        temp = elevator_first();
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }

        // Parking
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