package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.common.value.qual.MinLenFieldInvariant;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.OuttakeCommand;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

import java.sql.Time;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    public boolean isAuto = false;
    public final MotorExEx leftMotor;
    public final MotorExEx rightMotor;
    private final MotorGroup motors;

    private final DigitalChannel limitSwitch;

    public double MAX_SPEED = 0.9;

    public enum Level {
        LOADING, HANGING, AUTO0, AUTO1, AUTO2, LOW, MID, HIGH, STORAGE
    }

    private Level level;

    private HashMap<Level, Integer> levels = new HashMap<Level, Integer>() {{
        put(Level.LOADING, 0);
        put(Level.HANGING, 600);
        put(Level.AUTO0, 500);
        put(Level.AUTO1, 800);
        put(Level.AUTO2, 870);
        put(Level.LOW, 1050);
        put(Level.MID, 1650);
        put(Level.HIGH, 1750);
        put(Level.STORAGE, 0); // Remembering Last Position
    }};

    private Telemetry telemetry;
    private DoubleSupplier leftY;

    private OuttakeSusystem outtakeSusystem;

    public final double kS = 230, kG = 250, kV = 1.0, kA= 0.0;

    ElevatorFeedforward feedforward;

    public double feedForwardValue = 0.0;

    public double max_ticks_per_second = 0;
//    private Timing.Timer timer = new Timing.Timer(5, TimeUnit.MILLISECONDS);

    private enum initZeroState {
        INIT,
        SEARCHING,
        FOUND,
    }
    private initZeroState zeroState = initZeroState.INIT;

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

        limitSwitch = hm.get(DigitalChannel.class, "slider_limit");

//        setAuto();
//        level = Level.LOADING;
//        setLevel(Level.LOADING);

        this.outtakeSusystem = outtakeSusystem;

        this.max_ticks_per_second = leftMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );
    }

    @Override
    public void periodic() {
        if (zeroState != initZeroState.FOUND) {
            // TODO: add timer to prevent this running forever
            // for backup (if there are enough buttons) add a button to manually reset the elevator motor
            searchZero();
            telemetry.addData("Zero State: ", zeroState);
            return;
        }

        ////////////////////////////////////////////////////////////////////////

        if (Math.abs(leftY.getAsDouble()) > 0.06) {
            setManual();
        }
        telemetry.addData("Elevator Height: ", levels.get(Level.STORAGE));

        if (!isAuto) {
            feedForwardValue = feedforward.calculate(
                    0.9 * leftY.getAsDouble() * max_ticks_per_second
            );

            if (getHeight() < 100) {
                setPower(leftY.getAsDouble());
            } else {
                setPower((feedForwardValue / max_ticks_per_second) * MAX_SPEED);
            }

            if (getHeight() > 300 && leftY.getAsDouble() > 0.05) { // Open Outtake Automation
                if(outtakeSusystem.getState() != OuttakeSusystem.State.EXTREME) {
                    new OuttakeCommand(outtakeSusystem, OuttakeCommand.Action.OPEN).schedule();
                }
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
        motors.setTargetPosition((int)levels.get(levelPicked));
    }

    public void setManual() {
        motors.setRunMode(Motor.RunMode.RawPower);
        isAuto = false;
    }

    public void setAuto() {
        motors.setRunMode(Motor.RunMode.PositionControl);
        isAuto = true;
    }

    public void capturePosition() {
        levels.put(Level.STORAGE, getHeight().intValue());
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

    public boolean isSliderAtBottom() {
        return limitSwitch.getState();
    }

    public void searchZero() {
        if (!isSliderAtBottom()) {
            motors.set(-0.3);
            zeroState = initZeroState.SEARCHING;
        } else {
            motors.stopMotor();
            motors.resetEncoder();
            zeroState = initZeroState.FOUND;
        }
    }

    public void reset() {
        motors.resetEncoder();
    }
}
