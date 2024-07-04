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

@TeleOp(name = "MTI TeleOP Blue", group = "Final TeleOPs")
public class CenterStageTeleOpBase extends CommandOpMode {
    private CenterStageRobot robot;

    private DriveConstants RobotConstants;

    public Pose2d pose;

    private ElapsedTime runtime;

    @Override
    public void initialize() {
        runtime = new ElapsedTime();

        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = true;
        RobotConstants.frontRightInverted = true;
        RobotConstants.rearRightInverted = true;
        RobotConstants.rearLeftInverted = true;

        RobotConstants.WHEEL_RADIUS = 1.8898; // inch
        RobotConstants.GEAR_RATIO = 3.25; // output (wheel) speed / input (motor) speed
        RobotConstants.TRACK_WIDTH = 10.433; // in

        RobotConstants.MAX_VEL = 90;
        RobotConstants.MAX_ACCEL = 90;
        RobotConstants.MAX_ANG_VEL = Math.toRadians(360);
        RobotConstants.MAX_ANG_ACCEL = Math.toRadians(360);

        RobotConstants.frontLeftFeedForward[0] = 100;
        RobotConstants.frontLeftFeedForward[1] = 1;
        RobotConstants.frontLeftFeedForward[2] = 0;
        RobotConstants.frontRightFeedForward[0] = 120;
        RobotConstants.frontRightFeedForward[1] = 1;
        RobotConstants.frontRightFeedForward[2] = 0;
        RobotConstants.rearLeftFeedForward[0] = 180;
        RobotConstants.rearLeftFeedForward[1] = 1;
        RobotConstants.rearLeftFeedForward[2] = 0;
        RobotConstants.rearRightFeedForward[0] = 185;
        RobotConstants.rearRightFeedForward[1] = 1;
        RobotConstants.rearRightFeedForward[2] = 0;

        RobotConstants.VELO_KP = 0;
        RobotConstants.VELO_KI = 0;
        RobotConstants.VELO_KD = 0;
        RobotConstants.minIntegralBound = -400;
        RobotConstants.maxIntegralBound = 400;

        RobotConstants.TICKS_PER_REV = 145.6;
        RobotConstants.MAX_RPM = 1150;

        RobotConstants.DEFAULT_SPEED_PERC = 0.75;
        RobotConstants.SLOW_SPEED_PERC = 0.5;
        RobotConstants.FAST_SPEED_PERC = 1;

        pose = PoseStorage.currentPose;

        robot = new CenterStageRobot(hardwareMap, RobotConstants, telemetry, driverOp, toolOp,
                RobotEx.OpModeType.TELEOP,  RobotEx.Alliance.BLUE, "external_imu",
                false, true, pose, runtime);
    }

    @Override
    public void waitForStart() {
        super.waitForStart();
        runtime.reset();
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}