package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.drive.TwoWheelTrackingLocalizer;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class LocalizerSubsystem extends SubsystemBase {
    private MyLocalizer localizer;

    private Telemetry telemetry;

    public LocalizerSubsystem(HardwareMap hm, Telemetry telemetry, Pose2d initPose,
                              DoubleSupplier headingSupplier, DoubleSupplier headingVelocitySupplier) {
        this.telemetry = telemetry;
        localizer = new MyLocalizer(hm, headingSupplier, headingVelocitySupplier);
        localizer.setPoseEstimate(initPose);
    }

    @Override
    public void periodic() {
        localizer.update();

        telemetry.addData("Localizer Subsystem X: ", getCurrentPose().getX());
        telemetry.addData("Localizer Subsystem Y: ", getCurrentPose().getY());
        telemetry.addData("Localizer Subsystem Heading: ", Math.toDegrees(getCurrentPose().getHeading()));
    }

    public Pose2d getCurrentPose() {
        return localizer.getPoseEstimate();
    }
}
