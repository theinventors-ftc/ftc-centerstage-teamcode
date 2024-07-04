package org.firstinspires.ftc.teamcode.Autos.Cases;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autos.AutonomousBase;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.Autos.RoadRunnerSubsystem_RED;

@Disabled
@Autonomous(name = "RED_Long_Purple", group = "Final Autonomous")
public class Autonomous_RED_Long_Purple extends AutonomousBase {
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

        temp = new SequentialCommandGroup(new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem));
        temp.schedule();
        while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
            run();
        }
        RR.runToHome();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            drive.update();
        }

        save_current_pose();

        reset();
    }
}