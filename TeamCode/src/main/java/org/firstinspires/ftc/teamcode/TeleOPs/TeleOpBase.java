package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CenterStageRobot.CenterStageRobot;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.inventors.ftc.robotbase.RobotEx;
import org.inventors.ftc.robotbase.drive.DriveConstants;
import org.inventors.ftc.robotbase.hardware.GamepadExEx;
import org.yaml.snakeyaml.Yaml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.HashMap;

@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    GamepadExEx driverOp, toolOp;
    public Pose2d pose;
    private ElapsedTime runtime;
    private CenterStageRobot robot;
    private HashMap constants;

    private HashMap load_constants() {
        Yaml yaml = new Yaml();

        InputStream inputStream = null;

        try {
            inputStream = new FileInputStream("./Constants.yml");
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        return yaml.load(inputStream);
    }

    @Override
    public void initialize() {
        driverOp = new GamepadExEx(gamepad1);
        toolOp = new GamepadExEx(gamepad2);

        constants = load_constants();

        pose = PoseStorage.currentPose;

        runtime = new ElapsedTime();
    }

    public void initAllianceRelated(RobotEx.Alliance alliance) {
        robot = new CenterStageRobot(hardwareMap, telemetry, driverOp, toolOp, constants,
                RobotEx.OpModeType.TELEOP,  alliance, pose, runtime);
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