package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelColorDetectorSubsystem;
import org.inventors.ftc.robotbase.controllers.StateMachine;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intakeSubsystem;
    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;
    private StateMachine stateMachine;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, PixelColorDetectorSubsystem pixelColorDetectorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.pixelColorDetectorSubsystem = pixelColorDetectorSubsystem;

        stateMachine = new StateMachine(() -> pixelColorDetectorSubsystem.getNumOfPixels() == 2,
                200);

        addRequirements(intakeSubsystem, pixelColorDetectorSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.run();
    }

    @Override
    public void execute() {
        stateMachine.update();
    }

    @Override
    public boolean isFinished() {
        return stateMachine.isJustActive();
//        return pixelColorDetectorSubsystem.getNumOfPixels() == 2;
    }
}
