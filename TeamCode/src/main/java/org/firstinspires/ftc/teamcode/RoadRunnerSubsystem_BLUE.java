package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.concurrent.TimeUnit;

public class RoadRunnerSubsystem_BLUE extends SubsystemBase {
    protected SampleMecanumDrive drive;

    /*-------------------------------------------------------
    -Functions-
    -------------------------------------------------------*/

    /*-------------------------------------------------------
    -Enums-
    -------------------------------------------------------*/
    enum RobotSides{
        FRONT,
        REAR,
        CENTER,
        LEFT,
        RIGHT,
    }

    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }

    enum StartingPosition{
        SHORT,
        LONG
    }

    enum Path{
        INNER,
        OUTER
    }

    enum ParkingPosition{
        INNER,
        MID,
        OUTER
    }

    enum PixelStack{
        INNER,
        MID,
        OUTER
    }

    protected StartingPosition startingPosition;
    protected Path path;
    protected ParkingPosition parkingPosition;
    protected PixelStack pixelStack;


    public Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        return new Pose2d(X,Y,pose.getHeading());
    }

    public Vector2d offsetPoseShifter(Vector2d pose, Pair<Double, Double> xy_offset) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        return new Vector2d(X,Y);
    }

    public Pose2d robotPoseLimitCalculation(Pose2d pose, RoadRunnerSubsystem_BLUE.RobotSides side){
        Double X = pose.getX();
        Double Y = pose.getY();

        if(side == RoadRunnerSubsystem_BLUE.RobotSides.CENTER) return pose;

        if(Math.toDegrees(pose.getHeading()) == 90){
            if(side == RoadRunnerSubsystem_BLUE.RobotSides.LEFT) X = X + (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.RIGHT) X = X - (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.FRONT) Y = Y - (RobotY/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.REAR) Y = Y + (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 270){
            if(side == RoadRunnerSubsystem_BLUE.RobotSides.LEFT) X = X - (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.RIGHT) X = X + (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.FRONT) Y = Y + (RobotY/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.REAR) Y = Y - (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 0){
            if(side == RoadRunnerSubsystem_BLUE.RobotSides.LEFT) Y = Y - (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.RIGHT) Y = Y + (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.FRONT) X = X - (RobotY/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.REAR) X = X + (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 180){
            if(side == RoadRunnerSubsystem_BLUE.RobotSides.LEFT) Y = Y + (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.RIGHT) Y = Y - (RobotX/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.FRONT) X = X + (RobotY/2);
            else if(side == RoadRunnerSubsystem_BLUE.RobotSides.REAR) X = X - (RobotY/2);
        }

        Pose2d finalPose2d = new Pose2d(X, Y, pose.getHeading());

        return finalPose2d;
    }


    /*-------------------------------------------------------
    -Params-
    -------------------------------------------------------*/
    public static double Tile = 24; /*-inches-*/
    public static double TileInverted = -24; /*-inches-*/
    public static double RobotX = 12.6; /*-inches-*/
    public static double RobotY = 18; /*-inches-*/
    public static double AccDefault = 52;
    public static double VelDefault = 52;

    Dictionary<String, Pair<Double, Double>> LEFT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Inner_FirstCycle", new Pair<>(-2.0, 0.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(-2.0, 2.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Left_FirstCycle", new Pair<>(-15.0, 0.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(-15.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
    }};

    Dictionary<String, Pair<Double, Double>> CENTER_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Inner_FirstCycle", new Pair<>(-2.0, 0.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(-2.5, 2.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 3.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
    }};

    Dictionary<String, Pair<Double, Double>> RIGHT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Inner_FirstCycle", new Pair<>(-1.5, 0.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(-1.5, 2.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(2.0, 0.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(2.0, 0.0));
    }};

    Dictionary<String, Pair<Double, Double>> LEFT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Rand", new Pair<>(0.0, 0.0));
        put("Stacks_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Rand", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
    }};

    Dictionary<String, Pair<Double, Double>> CENTER_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Rand", new Pair<>(-1.5, 0.0));
        put("Stacks_Inner_FirstCycle", new Pair<>(-2.0, 0.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Rand", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
    }};

    Dictionary<String, Pair<Double, Double>> RIGHT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
        put("Stacks_Rand", new Pair<>(-1.0, 0.0));
        put("Stacks_Inner_FirstCycle", new Pair<>(-1.0, 1.0));
        put("Stacks_Inner_SecondCycle", new Pair<>(-1.0, 3.0));
        put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
        put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

        put("Backdrop_Rand", new Pair<>(1.0, 0.0));
        put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
        put("Backdrop_Right_FirstCycle", new Pair<>(2.0, 2.0));
        put("Backdrop_Right_SecondCycle", new Pair<>(2.0, 5.0));
    }};

    public Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>> OFFSETS_SHORT = new Hashtable<Randomization, Dictionary<String, Pair<Double, Double>>>() {{
        put(Randomization.LEFT, LEFT_RandomizationOffset_SHORT);
        put(Randomization.CENTER, CENTER_RandomizationOffset_SHORT);
        put(Randomization.RIGHT, RIGHT_RandomizationOffset_SHORT);
    }};

    public Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>> OFFSETS_LONG = new Hashtable<Randomization, Dictionary<String, Pair<Double, Double>>>() {{
        put(Randomization.LEFT, LEFT_RandomizationOffset_LONG);
        put(Randomization.CENTER, CENTER_RandomizationOffset_LONG);
        put(Randomization.RIGHT, RIGHT_RandomizationOffset_LONG);
    }};

    public Dictionary<StartingPosition, Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>>> OFFSETS = new Hashtable<StartingPosition, Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>>>() {{
        put(StartingPosition.SHORT, OFFSETS_SHORT);
        put(StartingPosition.LONG, OFFSETS_LONG);
    }};
    /*-------------------------------------------------------
    -Trajectories-
    -------------------------------------------------------*/
    public Pose2d HomePose;
    protected TrajectorySequenceBuilder test;
    protected TrajectorySequenceBuilder leftSpike;
    protected TrajectorySequenceBuilder centerSpike;
    protected TrajectorySequenceBuilder rightSpike;
    protected TrajectorySequenceBuilder spike_randomizedBackdrop;
    protected TrajectorySequenceBuilder backdrop_station;
    protected TrajectorySequenceBuilder backdrop_station_2;
    protected TrajectorySequenceBuilder station_backdrop;
    protected TrajectorySequenceBuilder station_backdrop_2;
    protected TrajectorySequenceBuilder spike_station;
    protected TrajectorySequenceBuilder station_long_randomizedBackdrop;
    protected TrajectorySequenceBuilder parking;
    /*-------------------------------------------------------
    -Poses-
    -------------------------------------------------------*/

    protected Integer[] rightSpikeStartingTanget = {315, 225}; //For short 45 and long 135 difference
    protected Integer[] rightSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected Integer[] stackStationTanget = {180, 135, 225}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected Integer[] parkingTanget = {180, 135}; // 135 for Inner 225 for Mid and Outer

    protected Integer rightSpikeStartingTangetValue = 0;
    protected Integer rightSpikeFinalTangetValue = 0;
    protected Integer stackStationTangetValue = 0;
    protected Integer parkingTangetValue = 1;

    /*------------------------Spikes------------------------*/

    protected Pose2d leftPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile, 1.8 * Tile, Math.toRadians(270)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.CENTER);

    protected Pose2d centerPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile/2, Tile, Math.toRadians(270)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d rightPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            0, Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d leftPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            TileInverted,1.5 * Tile, Math.toRadians(0)///////PEOS
    ), RobotSides.FRONT);

    protected Pose2d centerPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            1.9 * TileInverted, Tile, Math.toRadians(0)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d rightPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            1.95 * TileInverted, 1.25 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    /*------------------------Randomized Backdrop------------------------*/

    protected Pose2d randomizationBackdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.75 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropRight =  robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.25 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    /*-----------------------------------------------*/

    protected Pose2d randomizationBackdropLeftLong = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.75 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropCenterLong = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropRightLong =  robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.3 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    /*------------------------Backdrops------------------------*/

    protected Pose2d backdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile ,1.75 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d backdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile , 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d backdropRight = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile , 1.4 * Tile, Math.toRadians(180)/////// PEOS
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    /*------------------------Stacks------------------------*/

    protected Pose2d stationInner = robotPoseLimitCalculation(new Pose2d(
            2.9 * TileInverted,Tile/2, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d stationMiddle = robotPoseLimitCalculation(new Pose2d(
            2.9 * TileInverted, Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d stationOuter = robotPoseLimitCalculation(new Pose2d(
            2.9 * TileInverted, 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    /*------------------------Parking------------------------*/

    protected Pose2d parkingInner = new Pose2d(
            2.5 * Tile, Tile/2, Math.toRadians(180));///////

    protected Pose2d parkingMiddle = new Pose2d(
            2 * Tile, 1.5 * Tile, Math.toRadians(180));///////

    protected Pose2d parkingOuter = new Pose2d(
            2.5 * Tile, 2.5 * Tile, Math.toRadians(180));///////

    /*------------------------Mid Points------------------------*/

    protected Vector2d stationClose_Inner = new Vector2d(
            Tile/2, Tile/2);///////

    protected Vector2d stationFar_Inner = new Vector2d(
            TileInverted,Tile/2);///////

    protected Vector2d stationClose_Outer = new Vector2d(
            Tile/2, 2.5 * Tile);///////

    protected Vector2d stationFar_Outer = new Vector2d(
            TileInverted,2.5 * Tile);///////

    /*------------------------------------------------*/

    public Pose2d pixel_cycle_PoseTransfer = rightPixel_SHORT;
    public Pose2d leftPixelSpike = leftPixel_SHORT;
    public Pose2d centerPixelSpike = centerPixel_SHORT;
    public Pose2d rightPixelSpike = rightPixel_SHORT;
    public Pose2d randomizedBackdrop = backdropRight;
    public Vector2d stationClose = stationClose_Inner;
    public Vector2d stationFar = stationFar_Inner;
    public Pose2d backdrop_Unload = backdropLeft;
    public Pose2d stackStation = stationInner;
    public Pose2d parkingPose = parkingMiddle;

    /*-------------------------------------------------------
    -La program-
    -------------------------------------------------------*/

    RoadRunnerSubsystem_BLUE(SampleMecanumDrive sampleDrive, Pose2d HomePose,
                             StartingPosition startingPosition, Path path, PixelStack pixelStack,
                             ParkingPosition parkingPosition){

        this.HomePose = HomePose;
        this.drive = sampleDrive;
        this.startingPosition = startingPosition;
        this.path = path;
        this.pixelStack = pixelStack;
        this.parkingPosition = parkingPosition;

        /*-----------------------------------------------------*/

        drive.setPoseEstimate(HomePose);
    }

    public void TrajectoryInit(Randomization rand){

        /*-----------------------------------------------------*/
        test = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(new Vector2d(0,0));
        /*-----------------------------------------------------*/

        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(leftPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .splineToLinearHeading(centerPixelSpike, Math.toRadians(270));

        /*------------------------------------------------------------------------*/

        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .setTangent(Math.toRadians(rightSpikeStartingTanget[rightSpikeStartingTangetValue])) //tan pair 45/135
                .splineToLinearHeading(rightPixelSpike, Math.toRadians(rightSpikeFinalTanget[rightSpikeFinalTangetValue]));

        /*------------------------------------------------------------------------*/

        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        backdrop_station = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStation.vec(), OFFSETS.get(startingPosition).get(rand).get("Stacks_Inner_FirstCycle")), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        backdrop_station_2 = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStation.vec(), OFFSETS.get(startingPosition).get(rand).get("Stacks_Inner_SecondCycle")), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225


        station_backdrop = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(offsetPoseShifter(backdrop_Unload.vec(), OFFSETS.get(startingPosition).get(rand).get("Backdrop_Right_FirstCycle")), Math.toRadians(0)
                        ,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        station_backdrop_2 = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(offsetPoseShifter(backdrop_Unload.vec(), OFFSETS.get(startingPosition).get(rand).get("Backdrop_Right_SecondCycle")), Math.toRadians(0)
                        ,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /*----------------------------------------------------------------------------------------*/

        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(280))
                .splineToLinearHeading(offsetPoseShifter(stackStation, OFFSETS.get(startingPosition).get(rand).get("Stacks_Rand")), Math.toRadians(180));

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(offsetPoseShifter(randomizedBackdrop.vec(), OFFSETS.get(startingPosition).get(rand).get("Backdrop_Rand")), Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }
}