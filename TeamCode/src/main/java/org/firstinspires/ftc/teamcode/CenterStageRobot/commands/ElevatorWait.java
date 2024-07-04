package org.firstinspires.ftc.teamcode.CenterStageRobot.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;

public class ElevatorWait extends CommandBase {
    private final ElevatorSubsystem elevator;

    public ElevatorWait(ElevatorSubsystem elevator){
        this.elevator = elevator;
    }

    @Override
    public boolean isFinished() {
        return elevator.atTargetLevel();
    }
}
