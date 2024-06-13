package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.inventors.ftc.robotbase.controllers.StateMachine;
import org.inventors.ftc.robotbase.hardware.ColorSensor;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

public class PixelColorDetectorSubsystem extends SubsystemBase {
    private ColorSensor frontSensor, backSensor;

    public enum PixelColor {
        NONE, // 1, 5, 14
        WHITE, // 31, 64, 71
        YELLOW, // 15, 25, 8
        PURPLE, // 14, 24, 42
        GREEN // 5, 19, 9
    }

    private PixelColor frontPixelColor = PixelColor.NONE;
    private PixelColor backPixelColor = PixelColor.NONE;

    private int frontPixelExistence = 0, backPixelExistence = 0; // Binary

    private int numOfPixels = 0;

    private Telemetry telemetry;

    private boolean isEnabled = false;

    private StateMachine stateMachine;

    private GamepadExEx driverOp, toolOp;

    public PixelColorDetectorSubsystem(HardwareMap hm, Telemetry telemetry, GamepadExEx driverOp,
                                       GamepadExEx toolOp) {
        this.telemetry = telemetry;

        frontSensor = new ColorSensor(hm, "front_color_sensor");
        backSensor = new ColorSensor(hm, "back_color_sensor");

        stateMachine = new StateMachine(() -> numOfPixels == 2, 250);

        this.driverOp = driverOp;
        this.toolOp = toolOp;
    }

    public PixelColor predictColorFront(double redCh, double greenCh, double blueCh) {
        double average = (redCh + greenCh + blueCh) / 3;

        if (average > 20) {
            return PixelColor.WHITE;
        } else if (greenCh > redCh && greenCh > blueCh && redCh > blueCh) {
            return PixelColor.YELLOW;
        } else if (greenCh > redCh && greenCh > blueCh) {
            return PixelColor.GREEN;
        } else if (blueCh > redCh && blueCh > greenCh && greenCh > redCh && average > 10) {
            return PixelColor.PURPLE;
        }

        return PixelColor.NONE;
    }

    public PixelColor predictColorBack(double redCh, double greenCh, double blueCh) {
        double average = (redCh + greenCh + blueCh) / 3;

        if (average > 45) {
            return PixelColor.WHITE;
        } else if (greenCh > redCh && greenCh > blueCh && redCh > blueCh) {
            return PixelColor.YELLOW;
        } else if (greenCh > redCh && greenCh > blueCh) {
            return PixelColor.GREEN;
        } else if (blueCh > redCh && blueCh > greenCh && greenCh > redCh && average > 10) {
            return PixelColor.PURPLE;
        }

        return PixelColor.NONE;
    }

    public void update() {
        double[] frontColors = frontSensor.getNormalizedColors();
        double[] backColors = backSensor.getNormalizedColors();

        frontPixelColor = predictColorFront(frontColors[0], frontColors[1], frontColors[2]);
        backPixelColor = predictColorBack(backColors[0], backColors[1], backColors[2]);

        stateMachine.update();
    }

    @Override
    public void periodic() {
        telemetry.addData("Front Pixel: ", frontPixelColor);
        telemetry.addData("Back Pixel: ", backPixelColor);
        if (!isEnabled) return;

        update();

        if(pocketIsJustFull()) {
            driverOp.rumble();
            toolOp.rumble();
        }

        frontPixelExistence = frontPixelColor != PixelColor.NONE ? 1 : 0;
        backPixelExistence = backPixelColor != PixelColor.NONE ? 1 : 0;

        numOfPixels = frontPixelExistence + backPixelExistence;
    }

    public PixelColor getFrontPixelColor() {
        return frontPixelColor;
    }

    public PixelColor getBackPixelColor() {
        return backPixelColor;
    }

    public boolean isFrontPixel() {
        return frontPixelExistence == 1;
    }

    public boolean isBackPixel() {
        return backPixelExistence == 1;
    }

    public int getNumOfPixels() {
        return numOfPixels;
    }

    public boolean pocketIsJustFull() {
        return stateMachine.isJustActive();
    }

    public void enable() {
        isEnabled = true;
    }

    public void disable() {
        isEnabled = false;
    }

    public boolean isEnabled() {
        return isEnabled;
    }
}
