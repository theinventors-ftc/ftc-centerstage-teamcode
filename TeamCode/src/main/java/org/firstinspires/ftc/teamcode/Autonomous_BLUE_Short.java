package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.inventors.ftc.robotbase.hardware.Camera;
import org.opencv.core.Rect;

@Autonomous(name = "BLUE_Short", group = "Final Autonomous")
public class Autonomous_BLUE_Short extends CommandOpMode {

    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private SampleMecanumDrive drive;
    private RoadRunnerSubsystem_BLUE RR_Blue;
    private RoadRunnerSubsystem_BLUE.Randomization rand;
    private RevBlinkinLedDriver ledDriver;

    private FtcDashboard dashboard;
    private Camera camera;
    private final double colorThresh = 30;
    private final Rect leftRect = new Rect(90, 470, 300, 240);
    private final Rect centerRect = new Rect(600, 450, 150, 160);
    private final Rect rightRect = new Rect(950, 450, 300, 260);

    private Pose2d HomePose = new Pose2d(RoadRunnerSubsystem_BLUE.Tile/2, 3 * RoadRunnerSubsystem_BLUE.Tile - 6.93, Math.toRadians(270));

    private SequentialCommandGroup temp;
    public SequentialCommandGroup randomizationPixelElevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO0),
                new InstantCommand(outtakeSusystem::go_outtake_first),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup elevator_first(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO1),
                new InstantCommand(outtakeSusystem::go_outtake_first),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup elevator_second(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO2),
                new InstantCommand(outtakeSusystem::go_outtake_first),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second)
        );
    }

    public SequentialCommandGroup scoring_randomization(){
        return new SequentialCommandGroup(
                new InstantCommand(outtakeSusystem::wheel_release),
                new WaitCommand(800),
                new InstantCommand(outtakeSusystem::wheel_stop)
        );
    }

    public SequentialCommandGroup scoring(){
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
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index)),
                        new WaitCommand(500),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(index - 1)),
                        new WaitCommand(800)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(intakeSubsystem::stop),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(6)),
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
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new SampleMecanumDrive(hardwareMap);
        RR_Blue = new RoadRunnerSubsystem_BLUE(drive, HomePose, RoadRunnerSubsystem_BLUE.StartingPosition.SHORT,
                RoadRunnerSubsystem_BLUE.Path.INNER, RoadRunnerSubsystem_BLUE.PixelStack.INNER,
                RoadRunnerSubsystem_BLUE.ParkingPosition.INNER, telemetry);

        rand = RoadRunnerSubsystem_BLUE.Randomization.LEFT;
        dashboard = FtcDashboard.getInstance();

        camera = new Camera(hardwareMap, dashboard, telemetry, TeamPropDetectionPipeline.Alliance.BLUE,
                colorThresh, leftRect, centerRect, rightRect);
    }

    @Override
    public void runOpMode() {
        initialize();
        waitForStart();

        if (camera.getTeamPropPos() == 0){
            rand = RoadRunnerSubsystem_BLUE.Randomization.LEFT;
        }
        else if (camera.getTeamPropPos() == 1){
            rand = RoadRunnerSubsystem_BLUE.Randomization.CENTER;
        }
        else if (camera.getTeamPropPos() == 2){
            rand = RoadRunnerSubsystem_BLUE.Randomization.RIGHT;
        }

        RR_Blue.spikeRandomizationPath(rand);
        RR_Blue.setCycle();
        RR_Blue.setParking();
        RR_Blue.TrajectoryInit(rand);

        // SPIKE
        new InstantCommand(intakeArmSubsystem::lockPixel, intakeArmSubsystem).schedule();
        RR_Blue.runSpike(rand);
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        // BACKDROP - YELLOW
        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem).schedule();
        randomizationPixelElevator().schedule();
        RR_Blue.runSpike_RandomizedBackdrop();
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
        // STACK
        for (int i = 0; !isStopRequested() && opModeIsActive() && i < 2; ++i ) {

            temp = new SequentialCommandGroup(
                    new WaitCommand(600),
                    resetElevator()
            );
            temp.schedule();
            RR_Blue.runBackdrop_Station(i);
            while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
                run();
                drive.update();
            }
            drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

            temp = stackStationIntake(i == 0 ? 5 : 3);
            temp.schedule();
            while (!isStopRequested() && opModeIsActive() && CommandScheduler.getInstance().isScheduled(temp)) {
                run();
            }

            temp = new SequentialCommandGroup(
                    new WaitCommand(2400),
                    elevator_first()
            );

            temp.schedule();

            RR_Blue.runStation_Backdrop(i);
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
        }

        temp = resetElevator();
        temp.schedule();
        RR_Blue.runParking();
        while (!isStopRequested() && opModeIsActive() && drive.isBusy()) {
            run();
            drive.update();
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        PoseStorage.currentPose = drive.getPoseEstimate();

        reset();
    }
}