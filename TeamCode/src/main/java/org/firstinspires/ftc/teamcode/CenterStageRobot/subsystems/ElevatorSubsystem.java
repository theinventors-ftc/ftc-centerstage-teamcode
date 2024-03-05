package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.OuttakeCommand;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ElevatorSubsystem extends SubsystemBase {
    public boolean isAuto = true;
    public final MotorExEx leftMotor;
    public final MotorExEx rightMotor;
    private final MotorGroup motors;

    private boolean isAutoEnabled = true;

    public double MAX_SPEED = 0.9;

    public enum Level {
        LOADING, HANGING, AUTO, LOW, MID, HIGH
    }

    private Level level;

    private int[] levelPositions = { 0, 300, 800, 1050, 1650, 1750 };

    private Telemetry telemetry;
    private DoubleSupplier leftY;

    private OuttakeSusystem outtakeSusystem;

    public ElevatorSubsystem(HardwareMap hm, Telemetry telemetry, DoubleSupplier leftY,
            OuttakeSusystem outtakeSusystem) {

        this.leftY = leftY;
        this.telemetry = telemetry;
        leftMotor = new MotorExEx(hm, "slider_left", 383.6, 435);
        rightMotor = new MotorExEx(hm, "slider_right", 383.6, 435);
        leftMotor.setMaxPower(MAX_SPEED);
        rightMotor.setMaxPower(MAX_SPEED);
        leftMotor.setInverted(true);
        motors = new MotorGroup(leftMotor, rightMotor);

        motors.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motors.setPositionTolerance(30);
        motors.setPositionCoefficient(0.014);

        motors.resetEncoder();

        setAuto();
        level = Level.LOADING;
        setLevel(Level.LOADING);

        this.outtakeSusystem = outtakeSusystem;
    }

    @Override
    public void periodic() {
        if (getHeight() < 600 && leftY.getAsDouble() < -0.05) {
            new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.CLOSE).schedule();
        } else if (getHeight() > 600 && leftY.getAsDouble() > 0.05) {
            if(outtakeSusystem.getState() != OuttakeSusystem.State.EXTREME) {
                new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.OPEN).schedule();
            }
        }
    }

    public void run() {
        motors.set(MAX_SPEED);
    }

    public void stop() {
        motors.stopMotor();
    }

    public void setLevel(Level levelPicked) {
        level = levelPicked;
        int levelIdx = 0;

        if (level == Level.LOADING)
            levelIdx = 0;
        else if (level == Level.HANGING)
            levelIdx = 1;
        else if (level == Level.AUTO)
            levelIdx = 2;
        else if (level == Level.LOW)
            levelIdx = 3;
        else if (level == Level.MID)
            levelIdx = 4;
        else if (level == Level.HIGH)
            levelIdx = 5;

        motors.setTargetPosition(levelPositions[levelIdx]);
    }

    public void setManual() {
        motors.setRunMode(Motor.RunMode.RawPower);
        isAuto = false;
    }

    public void setAuto() {
        motors.setRunMode(Motor.RunMode.PositionControl);
        isAuto = true;
    }

    public void setPower(double power) {
        motors.set(power);
    }

    public Double getHeight() {
        return motors.getPositions().get(0);
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTargetLevel() {
        return motors.atTargetPosition();
    }
}
