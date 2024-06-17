package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.inventors.ftc.robotbase.hardware.Camera;
import org.opencv.core.Rect;

@Disabled
@Autonomous(name = "Do not run", group = "Final Autonomous")
public class AutonomousBase extends CommandOpMode {
    protected enum Alliance {BLUE, RED}

    protected OuttakeSusystem outtakeSusystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected IntakeArmSubsystem intakeArmSubsystem;
    protected IntakeSubsystem intakeSubsystem;

    protected SampleMecanumDrive drive;
    protected RoadRunnerSubsystem RR;
    protected RoadRunnerSubsystem.Randomization rand;
    protected RevBlinkinLedDriver ledDriver;

    protected FtcDashboard dashboard;
    protected Camera camera;
    protected final double colorThresh = 30;
    protected final Rect
        leftRect = new Rect(90, 470, 300, 240),
        centerRect = new Rect(600, 450, 150, 160),
        rightRect = new Rect(950, 450, 300, 260);

    protected Pose2d HomePose;

    protected SequentialCommandGroup temp;

    public SequentialCommandGroup randomizationPixelElevator() {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO0),
            new InstantCommand(outtakeSusystem::go_outtake_first),
            new WaitCommand(80),
            new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup elevator_first() {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO1),
            new InstantCommand(outtakeSusystem::go_outtake_first),
            new WaitCommand(80),
            new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup elevator_second() {
        return new SequentialCommandGroup(
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO2),
            new InstantCommand(outtakeSusystem::go_outtake_first),
            new WaitCommand(80),
            new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup scoring_randomization() {
        return new SequentialCommandGroup(
            new InstantCommand(outtakeSusystem::wheel_release),
            new WaitCommand(800),
            new InstantCommand(outtakeSusystem::wheel_stop)
        );
    }

    public SequentialCommandGroup scoring() {
        return new SequentialCommandGroup(
            new InstantCommand(outtakeSusystem::wheel_release),
            new WaitCommand(1300),
            new InstantCommand(outtakeSusystem::wheel_stop)
        );
    }

    public SequentialCommandGroup resetElevator() {
        return new SequentialCommandGroup(
            new WaitCommand(400),
            new InstantCommand(outtakeSusystem::go_intake_second),
            new WaitCommand(80),
            new InstantCommand(outtakeSusystem::go_intake_first),
            new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)
        );
    }

    public SequentialCommandGroup stackStationIntake(int index) {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::run),
            new InstantCommand(outtakeSusystem::wheel_grab),
            new WaitCommand(150),
            new SequentialCommandGroup(
                new InstantCommand(() -> intakeArmSubsystem.auto_pixel(index)),
                new WaitCommand(500),
                new InstantCommand(() -> intakeArmSubsystem.auto_pixel(index - 1)),
                new WaitCommand(800)
            ),
            new ParallelCommandGroup(
                new InstantCommand(intakeSubsystem::stop),
                new InstantCommand(() -> intakeArmSubsystem.auto_pixel(6)),
                new InstantCommand(outtakeSusystem::wheel_stop)
            ),
            new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::reverse),
                new WaitCommand(600),
                new InstantCommand(intakeSubsystem::stop)
            )
        );
    }

    public SequentialCommandGroup stackStationIntakeOnePixel(int index) {
        return new SequentialCommandGroup(
            new InstantCommand(intakeSubsystem::run),
            new InstantCommand(outtakeSusystem::wheel_grab),
            new WaitCommand(150),
            new SequentialCommandGroup(
                new InstantCommand(() -> intakeArmSubsystem.auto_pixel(index)),
                new WaitCommand(500)
            ),
            new ParallelCommandGroup(
                new InstantCommand(intakeSubsystem::stop),
                new InstantCommand(() -> intakeArmSubsystem.auto_pixel(6)),
                new InstantCommand(outtakeSusystem::wheel_stop)
            ),
            new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::reverse),
                new WaitCommand(600),
                new InstantCommand(intakeSubsystem::stop)
            )
        );
    }

    @Override
    public void initialize() {
        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> 0, outtakeSusystem);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        ledDriver = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);

        dashboard = FtcDashboard.getInstance();
    }

    public void initAllianceRelated(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                camera = new Camera(
                    hardwareMap, dashboard, telemetry, TeamPropDetectionPipeline.Alliance.BLUE,
                    colorThresh, leftRect, centerRect, rightRect
                );
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                break;
            default:
                camera = new Camera(
                    hardwareMap, dashboard, telemetry, TeamPropDetectionPipeline.Alliance.RED,
                    colorThresh, leftRect, centerRect, rightRect
                );
                ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                break;
        }
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        if (camera.getTeamPropPos() == 0) {
            rand = RoadRunnerSubsystem_BLUE.Randomization.LEFT;
        } else if (camera.getTeamPropPos() == 1) {
            rand = RoadRunnerSubsystem_BLUE.Randomization.CENTER;
        } else if (camera.getTeamPropPos() == 2) {
            rand = RoadRunnerSubsystem_BLUE.Randomization.RIGHT;
        }

        RR.spikeRandomizationPath(rand);
        RR.TrajectoryInit(rand);

        // SPIKE
        new InstantCommand(intakeArmSubsystem::lockPixel, intakeArmSubsystem).schedule();
        RR.runSpike(rand);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }

    public void poseStoragePeriodic() {
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}