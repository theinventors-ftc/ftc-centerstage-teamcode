package org.firstinspires.ftc.teamcode.CenterStageRobot;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorManualCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.inventors.ftc.robotbase.RobotEx;

public class CenterStageRobot extends RobotEx {
    //----------------------------------- Initialize Subsystems -----------------------------------//
    private IntakeArmSubsystem intakeArmSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSusystem outtakeSusystem;
    private ElevatorSubsystem elevatorSubsystem;

    private DroneSubsystem droneSubsystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;
//    private LEDSubsystem ledSubsystem;

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, "external_imu", false, false, new Pose2d(0, 0, 0));
//        super(hm, RobotConstants, telemetry, driverOp, toolOp, OpModeType.TELEOP, false, false, new Pose2d(0, 0, 0));
    }

    public CenterStageRobot(HardwareMap hm, DriveConstants RobotConstants, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp, OpModeType opModeType, String imuName, boolean camera, boolean distance_sensor,
                            Pose2d startingPose) {
        super(hm, RobotConstants, telemetry, driverOp, toolOp, opModeType, imuName, camera, distance_sensor, startingPose);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        super.initMechanismsAutonomous(hardwareMap);
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> toolOp.getLeftY(), outtakeSusystem);
        droneSubsystem = new DroneSubsystem(hardwareMap);
        pixelColorDetectorSubsystem = new PixelColorDetectorSubsystem(hardwareMap, telemetry);
//        ledSubsystem = new LEDSubsystem(hardwareMap, pixelColorDetectorSubsystem, telemetry);

//        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
//        intakeSubsystem.setDefaultCommand(new IntakeManualCommand(intakeSubsystem, () -> toolOp.getRightY()));

        toolOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.TOOGLE));

        toolOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.EXTREME));

        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(
                        new InstantCommand(intakeArmSubsystem::lowerArm, intakeArmSubsystem),
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOW));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.MID));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.HIGH));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING));
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.HANGING));

        CommandScheduler.getInstance().registerSubsystem(elevatorSubsystem);
//        elevatorSubsystem.setDefaultCommand(new ElevatorManualCommand(elevatorSubsystem, toolOp::getLeftY));

        //Intake
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(pixelColorDetectorSubsystem::enable),
//                                new InstantCommand(ledSubsystem::enableIntake),
                                new InstantCommand(intakeArmSubsystem::lowerArm),
                                new InstantCommand(outtakeSusystem::go_intake_second),
                                new WaitCommand(80),
                                new InstantCommand(outtakeSusystem::go_intake_first),
                                new WaitCommand(150),
                                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING),
                                new InstantCommand(outtakeSusystem::wheel_grab),
//                                new IntakeCommand(intakeSubsystem, pixelColorDetectorSubsystem),
                                new InstantCommand(intakeSubsystem::run, intakeSubsystem)
//                                new WaitCommand(3000),
//                                new InstantCommand(outtakeSusystem::wheel_stop),
//                                new InstantCommand(intakeArmSubsystem::raiseArm),
//                                new WaitCommand(200),
//                                new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
//                                new WaitCommand(600),
//                                new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
//                                new InstantCommand(pixelColorDetectorSubsystem::disable),
//                                new WaitCommand(350),
//                                new InstantCommand(ledSubsystem::disableIntake)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(outtakeSusystem::wheel_stop),
                                new InstantCommand(intakeArmSubsystem::raiseArm),
                                new WaitCommand(150),
                                new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
                                new WaitCommand(500),
                                new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                                new InstantCommand(pixelColorDetectorSubsystem::disable),
                                new WaitCommand(350)
//                                new InstantCommand(ledSubsystem::disableIntake)
                        )
                );

        // Outtake
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.8)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_release),
                        new InstantCommand(pixelColorDetectorSubsystem::enable)
                ));
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.8)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_stop),
                        new InstantCommand(pixelColorDetectorSubsystem::disable)
                ));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.8)
                .whenActive(new InstantCommand(droneSubsystem::release, droneSubsystem));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.8)
                .whenActive(new InstantCommand(droneSubsystem::grab, droneSubsystem));

        new Trigger(() -> toolOp.getRightY() >= 0.8)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem),
                        new WaitCommand(200),
                        new InstantCommand(intakeSubsystem::reverse, intakeSubsystem),
                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem)
                ));

        new Trigger(() -> toolOp.getRightY() < 0.8)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)
                ));
    }
}