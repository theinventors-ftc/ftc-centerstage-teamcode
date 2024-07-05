package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CenterStageRobot.CenterStageRobot;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;

@TeleOp(name = "TeleOP RED", group = "Final TeleOPs")
public class CenterStageTeleOpRed extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        initAllianceRelated(RobotEx.Alliance.RED);
    }

    @Override
    public void run() {
        super.run();
    }
}