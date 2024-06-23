package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class LEDSubsystem extends SubsystemBase {

    private RevBlinkinLedDriver driver;

    private RevBlinkinLedDriver.BlinkinPattern BLACK_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private RevBlinkinLedDriver.BlinkinPattern WHITE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    private RevBlinkinLedDriver.BlinkinPattern GREEN_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    private RevBlinkinLedDriver.BlinkinPattern YELLOW_PATTERN = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    private RevBlinkinLedDriver.BlinkinPattern PURPLE_PATTERN = RevBlinkinLedDriver.BlinkinPattern.VIOLET;

    private RevBlinkinLedDriver.BlinkinPattern STOP_PATTERN = RevBlinkinLedDriver.BlinkinPattern.RED;
    private RevBlinkinLedDriver.BlinkinPattern ALMOST_PATTERN = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
    private RevBlinkinLedDriver.BlinkinPattern GOOO_PATTERN = RevBlinkinLedDriver.BlinkinPattern.GREEN;

    private PixelColorDetectorSubsystem pixelColorDetectorSubsystem;
    private Timing.Timer timer;

    enum PixelState {
        PIXEL_ONE,
        DELAY,
        PIXEL_TWO,
        IDLE
    }

    private PixelState state;

    private double initTime = 0;

    private Telemetry telemetry;

    private boolean isIntaking = false;

    private ElapsedTime test;

    public LEDSubsystem(HardwareMap hm, PixelColorDetectorSubsystem pixelColorDetectorSubsystem, Telemetry telemetry, ElapsedTime test) {
        this.pixelColorDetectorSubsystem = pixelColorDetectorSubsystem;

        driver = hm.get(RevBlinkinLedDriver.class, "led");
        driver.setPattern(BLACK_PATTERN);

        timer = new Timing.Timer(0, TimeUnit.MILLISECONDS);
        timer.start();

        this.telemetry = telemetry;

        state = PixelState.PIXEL_ONE;

        this.test = test;
    }

    private RevBlinkinLedDriver.BlinkinPattern getPattern(PixelColorDetectorSubsystem.PixelColor pixelColor) {
        if (pixelColor == PixelColorDetectorSubsystem.PixelColor.WHITE) {
            return WHITE_PATTERN;
        } else if (pixelColor == PixelColorDetectorSubsystem.PixelColor.GREEN) {
            return GREEN_PATTERN;
        } else if (pixelColor == PixelColorDetectorSubsystem.PixelColor.YELLOW) {
            return YELLOW_PATTERN;
        } else if (pixelColor == PixelColorDetectorSubsystem.PixelColor.PURPLE) {
            return PURPLE_PATTERN;
        }

        return BLACK_PATTERN;
    }

    @Override
    public void periodic() {
        if(!isIntaking) {
            if (state == PixelState.PIXEL_ONE) {
                if (timer.elapsedTime()-initTime >= 200) {
                    state = PixelState.DELAY;
                    initTime = timer.elapsedTime();
                }
                driver.setPattern(getPattern(pixelColorDetectorSubsystem.getFrontPixelColor()));
            } else if (state == PixelState.DELAY) {
                if (timer.elapsedTime()-initTime >= 20) {
                    state = PixelState.PIXEL_TWO;
                    initTime = timer.elapsedTime();
                }
                driver.setPattern(BLACK_PATTERN);
            } else if (state == PixelState.PIXEL_TWO) {
                if (timer.elapsedTime()-initTime >= 200) {
                    state = PixelState.IDLE;
                    initTime = timer.elapsedTime();
                }
                driver.setPattern(getPattern(pixelColorDetectorSubsystem.getBackPixelColor()));
            } else if (state == PixelState.IDLE) {
                if (timer.elapsedTime()-initTime >= 400) {
                    state = PixelState.PIXEL_ONE;
                    initTime = timer.elapsedTime();
                }
                driver.setPattern(BLACK_PATTERN);
            }
        } else {
            if (pixelColorDetectorSubsystem.getNumOfPixels() == 0) {
                driver.setPattern(STOP_PATTERN);
            } else if (pixelColorDetectorSubsystem.getNumOfPixels() == 1) {
                driver.setPattern(ALMOST_PATTERN);
            } else if (pixelColorDetectorSubsystem.getNumOfPixels() == 2) {
                driver.setPattern(GOOO_PATTERN);
            }
        }
    }

        public void enableIntake() {
        isIntaking = true;
    }

    public void disableIntake() {
        isIntaking = false;
    }

    public boolean isIntaking() {
        return isIntaking;
    }
}
