package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.concurrent.TimeUnit;

public class RoadRunnerSubsystem_BLUE extends SubsystemBase {
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
    public static double StackStationFirstCycleOffset = 4; /*-inches-*/
    public static double StackStationSecondCycleOffset = 6; /*-inches-*/

    public final double StackDistance = 0; /*-inches-*/
    public final double LOW_VEL_SPEED = 45.0;
    public final double HIGH_VEL_SPEED = 70.0;
    public final double HIGH_ACCEL_SPEED = 60.0;
    public final double LOW_ACCEL_SPEED = 60.0;
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
    protected Integer[] parkingTanget = {225, 135}; // 135 for Inner 225 for Mid and Outer

    protected Integer rightSpikeStartingTangetValue = 0;
    protected Integer rightSpikeFinalTangetValue = 0;
    protected Integer stackStationTangetValue = 0;
    protected Integer parkingTangetValue = 1;

    protected Pose2d leftPixel_SHORT = new Pose2d(Tile, 1.5 * Tile, Math.toRadians(270));
    protected Pose2d centerPixel_SHORT = new Pose2d(Tile/2, Tile + (RobotY/2), Math.toRadians(270));
    protected Pose2d rightPixel_SHORT = new Pose2d((RobotY/2),Tile + (RobotX/2) , Math.toRadians(180));

    protected Pose2d leftPixel_LONG = new Pose2d(TileInverted - (RobotY/2),1.5 * Tile, Math.toRadians(270));
    protected Pose2d centerPixel_LONG = new Pose2d(1.5 * TileInverted, Tile + (RobotY/2), Math.toRadians(270));
    protected Pose2d rightPixel_LONG = new Pose2d(2 * TileInverted + (RobotY/2), Tile + (RobotX/2), Math.toRadians(180));

    protected Pose2d randomizationBackdropLeft = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.75 * Tile, Math.toRadians(180)); // Default
    protected Pose2d randomizationBackdropCenter = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.5 * Tile, Math.toRadians(180));
    protected Pose2d randomizationBackdropRight = new Pose2d(2.5 * Tile - (RobotY/2) + RandomizationBackdropDistance, 1.25 * Tile, Math.toRadians(180)); // Default

    protected Pose2d backdropLeft = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance, 1.75 * Tile, Math.toRadians(180)); // Default
    protected Pose2d backdropCenter = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance, 1.5 * Tile, Math.toRadians(180));
    protected Pose2d backdropRight = new Pose2d(2.5 * Tile - (RobotY/2) + BackdropDistance, 1.2 * Tile, Math.toRadians(180)); // Default

    protected Pose2d stationInnerSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2)  + StackDistance,Tile/2 + StackStationSecondCycleOffset, Math.toRadians(180)); // Default
    protected Pose2d stationMiddleSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,Tile + StackStationSecondCycleOffset, Math.toRadians(180));
    protected Pose2d stationOuterSecondCycle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance, 1.5 * Tile + StackStationSecondCycleOffset, Math.toRadians(180)); // Default

    protected Pose2d stationInner = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,Tile/2 + StackStationFirstCycleOffset, Math.toRadians(180)); // Default
    protected Pose2d stationMiddle = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance,Tile + StackStationFirstCycleOffset, Math.toRadians(180));
    protected Pose2d stationOuter = new Pose2d(3 * TileInverted + (RobotY/2) + StackDistance, 1.5 * Tile + StackStationFirstCycleOffset, Math.toRadians(180)); // Default

    protected Pose2d parkingInner = new Pose2d(2.5 * Tile - 4, Tile/2, Math.toRadians(180));
    protected Pose2d parkingMiddle = new Pose2d(2 * Tile, 1.5 * Tile, Math.toRadians(180));
    protected Pose2d parkingOuter = new Pose2d(2.5 * Tile, 2.5 * Tile, Math.toRadians(180));

    protected Vector2d stationClose_Inner = new Vector2d(Tile, Tile/2);
    protected Vector2d stationFar_Inner = new Vector2d(2 * TileInverted,Tile/2);

    protected Vector2d stationClose_Outer = new Vector2d(Tile, 2.5 * Tile);
    protected Vector2d stationFar_Outer = new Vector2d(2 * TileInverted,2.5 * Tile);

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

    public void TrajectoryInit(){

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
                .splineToConstantHeading(stationClose, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(LOW_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(LOW_ACCEL_SPEED)
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
                        SampleMecanumDrive.getAccelerationConstraint(LOW_ACCEL_SPEED)
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
                .setTangent(Math.toRadians(240))
                .splineToLinearHeading(stackStation, Math.toRadians(180));

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose,
                        SampleMecanumDrive.getVelocityConstraint(HIGH_VEL_SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(HIGH_ACCEL_SPEED)
                )
                .splineToConstantHeading(randomizedBackdrop.vec(), Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }
}