package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

public class LocalizerSubsystem extends SubsystemBase {
    private StandardTrackingWheelLocalizer localizer;

    public LocalizerSubsystem(HardwareMap hm, Pose2d initPose) {
        localizer = new StandardTrackingWheelLocalizer(hm, new ArrayList<>(), new ArrayList<>());

        localizer.setPoseEstimate(initPose);
    }

    @Override
    public void periodic() {
        localizer.update();
    }

    public Pose2d getCurrentPose() {
        return localizer.getPoseEstimate();
    }
}
