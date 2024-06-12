package org.firstinspires.ftc.teamcode;

import android.preference.Preference;
import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Dictionary;
import java.util.Hashtable;
import java.util.concurrent.TimeUnit;
public class RoadRunnerSubsystem_RED extends SubsystemBase {
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

    public Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset, Offsets preference) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        if (preference == Offsets.X){
            Y = pose.getY();
        }
        else if(preference == Offsets.Y){
            X = pose.getX();
        }

        Pose2d finalPose2d = new Pose2d(X,Y,pose.getHeading());

        return finalPose2d;
    }

    public Vector2d offsetPoseShifter(Vector2d pose, Pair<Double, Double> xy_offset, Offsets preference) {
        Double X = pose.getX() + xy_offset.first;
        Double Y = pose.getY() + xy_offset.second;

        if (preference == Offsets.X){
            Y = pose.getY();
        }
        else if(preference == Offsets.Y){
            X = pose.getX();
        }

        Vector2d finalVector2d = new Vector2d(X,Y);

        return finalVector2d;
    }

    public Pose2d robotPoseLimitCalculation(Pose2d pose, RobotSides side){
        Double X = pose.getX();
        Double Y = pose.getY();

        if(side == RobotSides.CENTER) return pose;

        if(Math.toDegrees(pose.getHeading()) == 90){
            if(side == RobotSides.LEFT) X = X + (RobotX/2);
            else if(side == RobotSides.RIGHT) X = X - (RobotX/2);
            else if(side == RobotSides.FRONT) Y = Y - (RobotY/2);
            else if(side == RobotSides.REAR) Y = Y + (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 270){
            if(side == RobotSides.LEFT) X = X - (RobotX/2);
            else if(side == RobotSides.RIGHT) X = X + (RobotX/2);
            else if(side == RobotSides.FRONT) Y = Y + (RobotY/2);
            else if(side == RobotSides.REAR) Y = Y - (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 0){
            if(side == RobotSides.LEFT) Y = Y - (RobotX/2);
            else if(side == RobotSides.RIGHT) Y = Y + (RobotX/2);
            else if(side == RobotSides.FRONT) X = X - (RobotY/2);
            else if(side == RobotSides.REAR) X = X + (RobotY/2);
        }
        else if(Math.toDegrees(pose.getHeading()) == 180){
            if(side == RobotSides.LEFT) Y = Y + (RobotX/2);
            else if(side == RobotSides.RIGHT) Y = Y - (RobotX/2);
            else if(side == RobotSides.FRONT) X = X + (RobotY/2);
            else if(side == RobotSides.REAR) X = X - (RobotY/2);
        }

        Pose2d finalPose2d = new Pose2d(X, Y, pose.getHeading());

        return finalPose2d;
    }

    /*-------------------------------------------------------
    -Params-
    -------------------------------------------------------*/
    public static double Tile = 24; /*-inches-*/
    public static double TileInverted = -24; /*-inches-*/
    public static double RobotX = 12.795; /*-inches-*/
    public static double RobotY = 18.11; /*-inches-*/
    public static double AccDefault = 52;
    public static double VelDefault = 52;

    Dictionary<String, Pair<Double, Double>> RandomizationOffset_XY = new Hashtable<String, Pair<Double, Double>>() {{
        put("Left", new Pair<>(0.0, -2.5));
        put("Center", new Pair<>(-0.5, -2.0));
        put("Right", new Pair<>(-2.0, 0.0));
        put("Final", new Pair<>(0.0,0.0));
    }};

    /*-------------------------------------------------------
    -Trajectories-
    -------------------------------------------------------*/
    public Pose2d HomePose;
    protected TrajectorySequence test;
    protected TrajectorySequenceBuilder leftSpike;
    protected TrajectorySequenceBuilder centerSpike;
    protected TrajectorySequenceBuilder rightSpike;
    protected TrajectorySequenceBuilder spike_randomizedBackdrop;
    protected TrajectorySequenceBuilder backdrop_station_first_cycle;
    protected TrajectorySequenceBuilder backdrop_station_second_cycle;
    protected TrajectorySequenceBuilder station_backdrop_first_cycle;
    protected TrajectorySequenceBuilder station_backdrop_second_cycle;
    protected TrajectorySequenceBuilder spike_station;
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

    protected Integer[] leftSpikeStartingTanget = {45, 135}; //For short 45 and long 135 difference
    protected Integer[] leftSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected Integer[] stackStationTanget = {180, 225, 135}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected Integer[] parkingTanget = {135, 225}; // 135 for Inner 225 for Mid and Outer

    protected Integer leftSpikeStartingTangetValue = 0;
    protected Integer leftSpikeFinalTangetValue = 0;
    protected Integer stackStationTangetValue = 0;
    protected Integer parkingTangetValue = 1;

    /*------------------------Spikes------------------------*/

    protected Pose2d leftPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            0, 1.25 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected Pose2d centerPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile/2, TileInverted, Math.toRadians(90)///////
    ), RobotSides.FRONT);

    protected Pose2d rightPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile, 1.5 * TileInverted, Math.toRadians(90)///////
    ), RobotSides.CENTER);

    protected Pose2d leftPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            2 * TileInverted,1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.CENTER);

    protected Pose2d centerPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            1.5 * TileInverted, TileInverted, Math.toRadians(90)///////
    ), RobotSides.FRONT);

    protected Pose2d rightPixel_LONG = new Pose2d(
            TileInverted - (RobotY/2), TileInverted - (RobotX/2), Math.toRadians(90)///////
    );

    /*------------------------Randomized Backdrop------------------------*/

    protected Pose2d randomizationBackdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile, 1.3 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected Pose2d randomizationBackdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected Pose2d randomizationBackdropRight =  robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile, 1.8 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    /*------------------------Backdrops------------------------*/

    protected Pose2d backdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile ,1.3 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected Pose2d backdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile , 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected Pose2d backdropRight = robotPoseLimitCalculation(new Pose2d(
            2.75 * Tile , 1.75 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    /*------------------------Stacks Second Cycle------------------------*/

    protected Pose2d stationInnerSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted,TileInverted/2, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected Pose2d stationMiddleSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted,TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected Pose2d stationOuterSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    /*------------------------Stacks First Cycle------------------------*/

    protected Pose2d stationInner = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted,TileInverted/2, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected Pose2d stationMiddle = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted, TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected Pose2d stationOuter = robotPoseLimitCalculation(new Pose2d(
            2.7 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    /*------------------------Parking------------------------*/

    protected Pose2d parkingInner = new Pose2d(
            2.5 * Tile, TileInverted/2, Math.toRadians(180));///////

    protected Pose2d parkingMiddle = new Pose2d(
            2 * Tile, 1.5 * TileInverted, Math.toRadians(180));///////

    protected Pose2d parkingOuter = new Pose2d(
            2.5 * Tile, 2.5 * TileInverted, Math.toRadians(180));///////

    /*------------------------Mid Points------------------------*/

    protected Vector2d stationClose_Inner = new Vector2d(
            Tile/2, TileInverted/2);///////

    protected Vector2d stationFar_Inner = new Vector2d(
            TileInverted,TileInverted/2);///////

    protected Vector2d stationClose_Outer = new Vector2d(
            Tile, 2.5 * TileInverted);///////

    protected Vector2d stationFar_Outer = new Vector2d(
            1.5 * TileInverted,2.5 * TileInverted);///////

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

    RoadRunnerSubsystem_RED(SampleMecanumDrive sampleDrive, Pose2d HomePose,
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

        if(randomization == Randomization.LEFT)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Left"));
        else if(randomization == Randomization.CENTER)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Center"));
        else if(randomization == Randomization.RIGHT)RandomizationOffset_XY.put("Final", RandomizationOffset_XY.get("Right"));

        /*-----------------------------------------------------*/
        test = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .back(30)
                .build();
        /*-----------------------------------------------------*/

        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(rightPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                .setTangent(Math.toRadians(leftSpikeStartingTanget[leftSpikeStartingTangetValue])) //tan pair 45/135
                .splineToLinearHeading(offsetPoseShifter(leftPixelSpike, new Pair<>(2.0, 0.0), Offsets.BOTH), Math.toRadians(leftSpikeFinalTanget[leftSpikeFinalTangetValue]));

        /*----------------------------------------------------------------------------------------*/

        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(offsetPoseShifter(randomizedBackdrop, new Pair<>(0.0, 0.0),  Offsets.BOTH), Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        backdrop_station_first_cycle = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStation.vec(), RandomizationOffset_XY.get("Final"), Offsets.BOTH), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        backdrop_station_second_cycle = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(offsetPoseShifter(stackStationSecondCycle.vec(), RandomizationOffset_XY.get("Final"), Offsets.BOTH), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225


        station_backdrop_first_cycle = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0)
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
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(stackStation, Math.toRadians(90));

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToLinearHeading(new Pose2d(parkingPose.vec(), Math.toRadians(90)), Math.toRadians(0));

    }
}