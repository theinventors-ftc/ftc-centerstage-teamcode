package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

    enum RobotSides{
        FRONT,
        REAR,
        CENTER,
        LEFT,
        RIGHT,
    }

    enum Offsets{
        X,
        Y,
        BOTH
    }

    public Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset, RoadRunnerSubsystem_BLUE.Offsets preference) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        if (preference == RoadRunnerSubsystem_BLUE.Offsets.X){
            Y = pose.getY();
        }
        else if(preference == RoadRunnerSubsystem_BLUE.Offsets.Y){
            X = pose.getX();
        }

        Pose2d finalPose2d = new Pose2d(X,Y,pose.getHeading());

        return finalPose2d;
    }

    public Vector2d offsetPoseShifter(Vector2d pose, Pair<Double, Double> xy_offset, RoadRunnerSubsystem_BLUE.Offsets preference) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        if (preference == RoadRunnerSubsystem_BLUE.Offsets.X){
            Y = pose.getY();
        }
        else if(preference == RoadRunnerSubsystem_BLUE.Offsets.Y){
            X = pose.getX();
        }

        Vector2d finalVector2d = new Vector2d(X,Y);

        return finalVector2d;
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

    Dictionary<String, Pair<Double, Double>> RandomizationOffset_XY = new Hashtable<String, Pair<Double, Double>>() {{
        put("Left", new Pair<>(-5.0, 2.0));
        put("Center", new Pair<>(0.0, 0.0));
        put("Right", new Pair<>(-3.0, 0.0));
        put("Final", new Pair<>(0.0,0.0));
    }};

    /*-------------------------------------------------------
    -Trajectories-
    -------------------------------------------------------*/
    public Pose2d HomePose;
    protected TrajectorySequenceBuilder test;
    protected TrajectorySequenceBuilder leftSpike;
    protected TrajectorySequenceBuilder centerSpike;
    protected TrajectorySequenceBuilder rightSpike;
    protected TrajectorySequenceBuilder rightSpike_LONG;
    protected TrajectorySequenceBuilder spike_randomizedBackdrop;
    protected TrajectorySequenceBuilder backdrop_station_first_cycle;
    protected TrajectorySequenceBuilder backdrop_station_second_cycle;
    protected TrajectorySequenceBuilder station_backdrop_first_cycle;
    protected TrajectorySequenceBuilder station_backdrop_second_cycle;
    protected TrajectorySequenceBuilder spike_station_right;
    protected TrajectorySequenceBuilder spike_station;
    protected TrajectorySequenceBuilder station_long_randomizedBackdrop;
    protected TrajectorySequenceBuilder parking;
    /*-------------------------------------------------------
    -Enums-
    -------------------------------------------------------*/
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

    /*-------------------------------------------------------
    -Poses-
    -------------------------------------------------------*/

    protected Integer[] rightSpikeStartingTanget = {315, 225}; //For short 45 and long 135 difference
    protected Integer[] rightSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected Integer[] stackStationTanget = {180, 135, 225}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected Integer[] parkingTanget = {190, 135}; // 135 for Inner 225 for Mid and Outer

    protected Integer rightSpikeStartingTangetValue = 0;
    protected Integer rightSpikeFinalTangetValue = 0;
    protected Integer stackStationTangetValue = 0;
    protected Integer parkingTangetValue = 1;

    /*------------------------Spikes------------------------*/

    protected Pose2d leftPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile, 1.8 * Tile + 3, Math.toRadians(270)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.CENTER);

    protected Pose2d centerPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile/2, Tile, Math.toRadians(270)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d rightPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            0, Tile + 4, Math.toRadians(180)/////// PEOS
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d leftPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            2 * TileInverted,1.5 * TileInverted, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.CENTER);

    protected Pose2d centerPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            1.5 * TileInverted, TileInverted, Math.toRadians(90)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d rightPixel_LONG = new Pose2d(
            TileInverted - (RobotY/2), TileInverted - (RobotX/2), Math.toRadians(90)///////
    );

    /*------------------------Randomized Backdrop------------------------*/

    protected Pose2d randomizationBackdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile - 1, 1.75 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile - 3, 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d randomizationBackdropRight =  robotPoseLimitCalculation(new Pose2d(
            2.5 * Tile, 1.3 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    /*------------------------Backdrops------------------------*/

    protected Pose2d backdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.6 * Tile + 1 ,1.75 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d backdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.6 * Tile , 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    protected Pose2d backdropRight = robotPoseLimitCalculation(new Pose2d(
            2.6 * Tile , 1.40 * Tile, Math.toRadians(180)/////// PEOS
    ), RoadRunnerSubsystem_BLUE.RobotSides.REAR);

    /*------------------------Stacks Second Cycle------------------------*/

    protected Pose2d stationInnerSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.9 * TileInverted,Tile/2, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d stationMiddleSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.95 * TileInverted,Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    protected Pose2d stationOuterSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.95 * TileInverted, 1.5 * Tile, Math.toRadians(180)///////
    ), RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

    /*------------------------Stacks First Cycle------------------------*/

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
    public Pose2d stackStationSecondCycle = stationInnerSecondCycle;
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

    public void TrajectoryInit(Randomization randomization){

        if(randomization == RoadRunnerSubsystem_BLUE.Randomization.LEFT)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Left"));
        else if(randomization == RoadRunnerSubsystem_BLUE.Randomization.CENTER)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Center"));
        else if(randomization == RoadRunnerSubsystem_BLUE.Randomization.RIGHT)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Right"));

        /*-----------------------------------------------------*/
        test = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(new Vector2d(0,0));
        /*-----------------------------------------------------*/

        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(leftPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        rightSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .lineToLinearHeading(new Pose2d(leftPixelSpike.vec(), Math.toRadians(180)))
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(stackStation, Math.toRadians(135));

        /*------------------------------------------------------------------------*/

        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .setTangent(Math.toRadians(rightSpikeStartingTanget[rightSpikeStartingTangetValue])) //tan pair 45/135
                .splineToLinearHeading(rightPixelSpike, Math.toRadians(rightSpikeFinalTanget[rightSpikeFinalTangetValue]));

        /*----------------------------------------------------------------------------------------*/

        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        backdrop_station_first_cycle = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStation.vec(), RandomizationOffset_XY.get("Final"), Offsets.Y), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        backdrop_station_second_cycle = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStationSecondCycle.vec(), RandomizationOffset_XY.get("Final"), Offsets.Y), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        station_backdrop_first_cycle = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(offsetPoseShifter(backdrop_Unload.vec(), RandomizationOffset_XY.get("Final"), Offsets.Y), Math.toRadians(0)
                        ,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        station_backdrop_second_cycle = drive.trajectorySequenceBuilder(stackStationSecondCycle)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0)
                        ,SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH)
                        ,SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /*----------------------------------------------------------------------------------------*/

        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(stackStation, Math.toRadians(180));

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(randomizedBackdrop.vec(), Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }
}