package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.TimeUnit;

public class RoadRunnerSubsystem_RED extends SubsystemBase {
    protected SampleMecanumDrive drive;

    /*-------------------------------------------------------
    -Params-
    -------------------------------------------------------*/
    public static double Tile = 24; /*-inches-*/
    public static double TileInverted = -24; /*-inches-*/
    public static double RobotX = 12.6; /*-inches-*/
    public static double RobotY = 18; /*-inches-*/
    public static double BackdropDistance = 1.25; /*-inches-*/
    public static double RandomizationBackdropDistance = 2.25; /*-inches-*/
    public static double StackStationFirstCycleOffset = 1; /*-inches-*/
    public static double StackStationSecondCycleOffset = -2; /*-inches-*/
    public final double StackDistance = 2; /*-inches-*/
    public final double HIGH_VEL_SPEED = 70.0;
    public final double HIGH_ACCEL_SPEED = 60.0;
    public final double LOW_VEL_SPEED = 45.0;
    public final double LOW_ACCEL_SPEED = 45.0;
    /*-------------------------------------------------------
    -Trajectories-
    -------------------------------------------------------*/
    public Pose2d HomePose;
    protected TrajectorySequenceBuilder test;
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

    protected Pose2d leftPixel_SHORT = new Pose2d((RobotY/2), TileInverted - (RobotX/2), Math.toRadians(180));
    protected Pose2d centerPixel_SHORT = new Pose2d(Tile/2, TileInverted - (RobotY/2), Math.toRadians(90));
    protected Pose2d rightPixel_SHORT = new Pose2d(Tile, 1.5 * TileInverted, Math.toRadians(90));

    protected Pose2d leftPixel_LONG = new Pose2d(2 * TileInverted,1.5 * TileInverted, Math.toRadians(180));
    protected Pose2d centerPixel_LONG = new Pose2d(1.5 * TileInverted, TileInverted - (RobotY/2), Math.toRadians(90));
    protected Pose2d rightPixel_LONG = new Pose2d(TileInverted - (RobotY/2), TileInverted - (RobotX/2), Math.toRadians(90));

    protected Pose2d randomizationBackdropLeft = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.2 * TileInverted, Math.toRadians(180)); // Default
    protected Pose2d randomizationBackdropCenter = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.5 * TileInverted, Math.toRadians(180));
    protected Pose2d randomizationBackdropRight = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.75 * TileInverted, Math.toRadians(180)); // Default
    protected Pose2d backdropLeft = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance,1.35 * TileInverted, Math.toRadians(180)); // Default
    protected Pose2d backdropCenter = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance, 1.5 * TileInverted, Math.toRadians(180));
    protected Pose2d backdropRight = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance, 1.75 * TileInverted, Math.toRadians(180)); // Default

    protected Pose2d stationInnerSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2)  + StackDistance,TileInverted/2 - StackStationSecondCycleOffset, Math.toRadians(180)); // Default
    protected Pose2d stationMiddleSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,TileInverted - StackStationSecondCycleOffset, Math.toRadians(180));
    protected Pose2d stationOuterSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance, 1.5 * TileInverted - StackStationSecondCycleOffset, Math.toRadians(180)); // Default

    protected Pose2d stationInner = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,TileInverted/2 - StackStationFirstCycleOffset, Math.toRadians(180)); // Default
    protected Pose2d stationMiddle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,TileInverted - StackStationFirstCycleOffset, Math.toRadians(180));
    protected Pose2d stationOuter = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance, 1.5 * TileInverted - StackStationFirstCycleOffset, Math.toRadians(180)); // Default

    protected Pose2d parkingInner = new Pose2d(2.5 * Tile, TileInverted/2, Math.toRadians(180));
    protected Pose2d parkingMiddle = new Pose2d(2 * Tile, 1.5 * TileInverted, Math.toRadians(180));
    protected Pose2d parkingOuter = new Pose2d(2.5 * Tile, 2.65 * TileInverted, Math.toRadians(180));

    protected Vector2d stationClose_Inner = new Vector2d(Tile, TileInverted/2);
    protected Vector2d stationFar_Inner = new Vector2d(2 * TileInverted,TileInverted/2);

    protected Vector2d stationClose_Outer = new Vector2d(Tile, 2.5 * TileInverted);
    protected Vector2d stationFar_Outer = new Vector2d(1.5 * TileInverted,2.5 * TileInverted);

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

    public void TrajectoryInit(){

        /*-----------------------------------------------------*/
        test = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(new Vector2d(0,0));
        /*-----------------------------------------------------*/

        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(rightPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixelSpike.vec());

        /*------------------------------------------------------------------------*/

        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                .setTangent(Math.toRadians(leftSpikeStartingTanget[leftSpikeStartingTangetValue])) //tan pair 45/135
                .splineToLinearHeading(leftPixelSpike, Math.toRadians(leftSpikeFinalTanget[leftSpikeFinalTangetValue]));

        /*----------------------------------------------------------------------------------------*/

        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        backdrop_station_first_cycle = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(LOW_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineTo(stationFar,
                        SampleMecanumDrive.getVelocityConstraint(HIGH_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(HIGH_ACCEL_SPEED)
                )
                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        backdrop_station_second_cycle = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(LOW_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineTo(stationFar,
                        SampleMecanumDrive.getVelocityConstraint(HIGH_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(HIGH_ACCEL_SPEED)
                )
                .splineToConstantHeading(stackStationSecondCycle.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])); //tan pair 180/225

        station_backdrop_first_cycle = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose,
                        SampleMecanumDrive.getVelocityConstraint(HIGH_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(HIGH_ACCEL_SPEED)
                )
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0));

        station_backdrop_second_cycle = drive.trajectorySequenceBuilder(stackStationSecondCycle)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose,
                        SampleMecanumDrive.getVelocityConstraint(HIGH_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(HIGH_ACCEL_SPEED)
                )
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(stackStation, Math.toRadians(90));

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }
}