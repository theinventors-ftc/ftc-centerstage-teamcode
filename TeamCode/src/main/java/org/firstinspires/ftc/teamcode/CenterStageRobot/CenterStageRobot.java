package org.firstinspires.ftc.teamcode.CenterStageRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.NotifierSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.controllers.ForwardControllerSubsystem;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.DistanceSensorEx;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;


public class CenterStageRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems -----------------------------------//
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSusystem outtakeSusystem;

    private ElevatorSubsystem elevatorSubsystem;

    private DroneSubsystem droneSubsystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;
    private LEDSubsystem ledSubsystem;

    private NotifierSubsystem notifierSubsystem;

    // -------------------------------------- Controllers --------------------------------------- //
    protected DistanceSensorEx distanceSensor;
    protected ForwardControllerSubsystem distanceFollow;

    // ---------------------------------------- Utility ----------------------------------------- //
    private ElapsedTime running_time;

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry,
                            GamepadExEx driverOp, GamepadExEx toolOp, OpModeType opModeType,
                            Alliance alliance, String imuName, boolean camera, Pose2d startingPose,
                            ElapsedTime running_time) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, alliance,
                imuName, camera, startingPose);

        // ----------------------------------- Notifications ------------------------------------ //
        this.running_time = running_time;

//        new Trigger(() -> running_time.seconds() >= 120-10)
//                .whenActive(
//                        new InstantCommand(() -> driverOp.rumble(0.6))
//                );
//
//        new Trigger(() -> running_time.seconds() >= 120-5)
//                .whenActive(
//                        new InstantCommand(() -> driverOp.rumble(5))
//                );

        // TODO TEST This malakia
        notifierSubsystem = new NotifierSubsystem(running_time);

        notifierSubsystem.addNotification(
                120-10, new InstantCommand(() -> driverOp.rumble(0.6))
        );

        notifierSubsystem.addNotification(
                120-5, new InstantCommand(() -> driverOp.rumble(5))
        );
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap, GamepadExEx driverOp,
                                     GamepadExEx toolOp) {
        super.initMechanismsTeleOp(hardwareMap, driverOp, toolOp);

        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> toolOp.getLeftY(), outtakeSusystem);
        droneSubsystem = new DroneSubsystem(hardwareMap);
        pixelColorDetectorSubsystem = new PixelColorDetectorSubsystem(hardwareMap, telemetry,
                driverOp, toolOp);
        ledSubsystem = new LEDSubsystem(hardwareMap, pixelColorDetectorSubsystem, telemetry, running_time);

        // ----------------------------------- Manual Actions ----------------------------------- //
        toolOp.getGamepadButton(GamepadKeys.Button.B) // Outtake Toggle
                .whenPressed(new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.TOOGLE));

        toolOp.getGamepadButton(GamepadKeys.Button.Y) // Outtake SET EXTREME
                .whenPressed(new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.EXTREME));

        toolOp.getGamepadButton(GamepadKeys.Button.A) // Intake Arm
                .toggleWhenPressed(
                        new InstantCommand(intakeArmSubsystem::lowerArm, intakeArmSubsystem),
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem)
                );



        // Releasing Pixels
        new Trigger(() -> driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) // Outtaking
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_release),
                        new InstantCommand(pixelColorDetectorSubsystem::enable),
                        new InstantCommand(elevatorSubsystem::capturePosition, elevatorSubsystem)
                ));
        new Trigger(() -> !driverOp.getButton(GamepadKeys.Button.RIGHT_BUMPER))// Outtaking (Complementary)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_stop),
                        new InstantCommand(pixelColorDetectorSubsystem::disable)
                ));

        // Drone
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8) // Drone Launch
                .whenActive(new InstantCommand(droneSubsystem::release, droneSubsystem));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.8) // Drone Grab
                .whenActive(new InstantCommand(droneSubsystem::grab, droneSubsystem));

        // ---------------------------------- Automated Actions --------------------------------- //

        // Elevator Automated Heights
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP) // Memory
                .whenPressed(new SequentialCommandGroup(
                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.STORAGE),
                        new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.OPEN)
                ));

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.HANGING));
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new InstantCommand(elevatorSubsystem::reset, elevatorSubsystem));

        // Intake Automation
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(pixelColorDetectorSubsystem::enable),
                                new InstantCommand(ledSubsystem::enableIntake),
                                new InstantCommand(intakeArmSubsystem::lowerArm),
                                new InstantCommand(outtakeSusystem::go_intake_second),
                                new WaitCommand(80),
                                new InstantCommand(outtakeSusystem::go_intake_first),
                                new WaitCommand(150),
                                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING),
                                new InstantCommand(outtakeSusystem::wheel_grab),
                                new InstantCommand(intakeSubsystem::run, intakeSubsystem)
//                                new IntakeCommand(intakeSubsystem, pixelColorDetectorSubsystem), // Stops when it sees 2 pixels
//                                new SequentialCommandGroup(
//                                        new InstantCommand(outtakeSusystem::wheel_stop),
//                                        new InstantCommand(intakeArmSubsystem::raiseArm),
//                                        new WaitCommand(150),
//                                        new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
//                                        new WaitCommand(500),
//                                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
//                                        new InstantCommand(pixelColorDetectorSubsystem::disable),
//                                        new WaitCommand(350),
//                                        new InstantCommand(ledSubsystem::disableIntake)
//                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(pixelColorDetectorSubsystem::disable),
                                new InstantCommand(ledSubsystem::disableIntake),
                                new InstantCommand(outtakeSusystem::wheel_stop),
                                new InstantCommand(intakeArmSubsystem::raiseArm),
                                new WaitCommand(80),
                                new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
                                new WaitCommand(500),
                                new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
                        )
                );

        // Aborting Pixels
        new Trigger(() -> toolOp.getRightY() >= 0.5) // Abort Every Pixel Automation
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem),
                        new WaitCommand(200),
                        new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem)
                ));

        new Trigger(() -> toolOp.getRightY() < 0.5) // Abort Every Pixel Automation (Complementary)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)
                ));

        // --------------------------------- Backdrop Alignment --------------------------------- //
        distanceSensor = new DistanceSensorEx(hardwareMap, "distance_sensor");
        distanceFollow = new ForwardControllerSubsystem(() -> distanceSensor.getDistance(DistanceUnit.CM), 3, telemetry);

        // Backdrop Alignment
        driverOp.getGamepadButton(GamepadKeys.Button.A) // Enable Backdrop Alignment
                .whenPressed(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(drive::setRobotCentric),
                                        new InstantCommand(gyroFollow::enable, gyroFollow),
                                        new InstantCommand(() -> gyroFollow.setGyroTarget(this.alliance == Alliance.RED ? 90 : -90), gyroFollow)
                                ),
                                new InstantCommand(distanceFollow::enable, distanceFollow)
                        )
                );

        driverOp.getGamepadButton(GamepadKeys.Button.A) // Disable Backdrop Alignment
                .whenReleased(
                        new ParallelCommandGroup(
                                new InstantCommand(drive::setFieldCentric),
                                new InstantCommand(gyroFollow::disable, gyroFollow),
                                new InstantCommand(distanceFollow::disable, distanceFollow)
                        )
                );
    }

    @Override
    public double drivetrainForward() {
        if(distanceFollow.isEnabled()) return distanceFollow.calculateOutput();

        return super.drivetrainForward();
    }
}