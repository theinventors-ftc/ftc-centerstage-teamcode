package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.OuttakeCommand;
import org.inventors.ftc.robotbase.hardware.MotorExEx;


import java.util.HashMap;
import java.util.function.DoubleSupplier;

public class ElevatorSubsystem extends SubsystemBase {
    public boolean isAuto = false;
    public final MotorExEx leftMotor;
    public final MotorExEx rightMotor;
    private final MotorGroup motors;

//    private final DigitalChannel limitSwitch;

    public double MAX_SPEED = 0.9;

    public enum Level {
        LOADING, HANGING, AUTO0, AUTO1, AUTO2, LOW, MID, HIGH, STORAGE
    }

    private Level level;

    private HashMap<Level, Integer> levels = new HashMap<Level, Integer>() {{
        put(Level.LOADING, 0);
        put(Level.HANGING, 600);
        put(Level.AUTO0, 550);
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

    public final double kS = 230, kG = 300, kV = 1.0, kA= 0.0;

    ElevatorFeedforward feedforward;

    public double joystickPower = 0.0, clippedPower = 0.0;

    public double feedForwardValue = 0.0;

    public double max_ticks_per_second = 0;

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

//        limitSwitch = hm.get(DigitalChannel.class, "slider_limit");

        this.outtakeSusystem = outtakeSusystem;

        this.max_ticks_per_second = leftMotor.ACHIEVABLE_MAX_TICKS_PER_SECOND;
        feedforward = new ElevatorFeedforward(
                kS, kG, kV, kA
        );
    }

    private double calculateFeedForwardPower(double rawPower) {
        feedForwardValue = feedforward.calculate(0.9 * rawPower * max_ticks_per_second);
        return (feedForwardValue / max_ticks_per_second) * MAX_SPEED;
    }

    @Override
    public void periodic() {
//        if (zeroState != initZeroState.FOUND) {
//            // TODO: add timer to prevent this running forever
//            // for backup (if there are enough buttons) add a button to manually reset the elevator motor
//            searchZero();
//            return;
//        }

        ////////////////////////////////////////////////////////////////////////

        joystickPower = leftY.getAsDouble();

        clippedPower = joystickPower;

//        if(isSliderBottom() && joystickPower < 0) clippedPower = 0.0;
        if(isSliderTop() && joystickPower > 0) clippedPower = 0.0;

        if (Math.abs(joystickPower) > 0.06) {
            setManual();
        }

        if (!isAuto) {
            if (getHeight() < 100) setPower(clippedPower);
            else setPower(calculateFeedForwardPower(clippedPower));

            if (getHeight() > 250 && joystickPower > 0.05) { // Open Outtake Automation
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

//    public boolean isSliderBottom() {
//        return limitSwitch.getState();
//    }
    public boolean isSliderTop() {
        return getHeight() >= 1750;
    }

    public void searchZero() {
//        if (!isSliderBottom()) {
//            motors.set(-0.6);
//            zeroState = initZeroState.SEARCHING;
//        } else {
//            motors.stopMotor();
//            motors.resetEncoder();
//            zeroState = initZeroState.FOUND;
//        }
    }

    public void reset() {
        motors.resetEncoder();
    }
}
