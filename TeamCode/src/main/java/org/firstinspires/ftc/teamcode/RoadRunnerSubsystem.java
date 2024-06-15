package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Dictionary;
import java.util.Hashtable;


public class RoadRunnerSubsystem extends SubsystemBase {
    protected SampleMecanumDrive drive;
    Telemetry telemetry;

    /*----------------------------------------------------------------------------------------------
    -- Enums --
    ----------------------------------------------------------------------------------------------*/
    enum Randomization { LEFT, CENTER, RIGHT }
    enum StartingPosition { SHORT, LONG }
    enum Path { INNER, OUTER }
    enum ParkingPosition { INNER, MID, OUTER }
    enum PixelStack { INNER, MID, OUTER }
    enum RobotSides { FRONT, REAR, CENTER, LEFT, RIGHT }

    protected StartingPosition startingPosition;
    protected Path path;
    protected ParkingPosition parkingPosition;
    protected PixelStack pixelStack;


    /*----------------------------------------------------------------------------------------------
    -- Functions --
    ----------------------------------------------------------------------------------------------*/
    public Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset) {
        return new Pose2d(
                pose.getX() + xy_offset.first,
                pose.getY() + xy_offset.second,
                pose.getHeading()
        );
    }

    public Vector2d offsetPoseShifter(Vector2d pose, Pair<Double, Double> xy_offset) {
        return new Vector2d(
                pose.getX() + xy_offset.first,
                pose.getY() + xy_offset.second
        );
    }

    public Pose2d robotPoseLimitCalculation(Pose2d pose, RobotSides side){
        if(side == RobotSides.CENTER)
            return pose;

        Double X = pose.getX(), Y = pose.getY(), H = pose.getHeading(),
                Afb = (RobotY/2) * Math.sin(H), Bfb = (RobotY/2) * Math.cos(H),
                Arl = (RobotX/2) * Math.sin(H), Brl = (RobotX/2) * Math.cos(H);

        switch (side) {
            case FRONT:
                X -= Bfb;
                Y -= Afb;
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            case RIGHT:
                X -= Arl;
                Y += Brl;
                break;
        }

        return new Pose2d(X, Y, pose.getHeading());
    }

    /*----------------------------------------------------------------------------------------------
    -- Params --
    ----------------------------------------------------------------------------------------------*/
    public static double
            Tile = 24, TileInverted = -Tile, /*-inches-*/
            RobotX = 12.795, RobotY = 18.11, /*-inches-*/
            AccDefault = 52, VelDefault = 52;

    /*----------------------------------------------------------------------------------------------
    -- Trajectories --
    ----------------------------------------------------------------------------------------------*/
    public Pose2d HomePose;
    protected TrajectorySequenceBuilder test,
            leftSpike, centerSpike, rightSpike,
            leftSpike_LONG, centerSpike_LONG, rightSpike_LONG,
            spike_randomizedBackdrop, station_long_randomizedBackdrop, spike_station,
            backdrop_station_1st_cycle, backdrop_station_2nd_cycle,
            station_backdrop_1st_cycle, station_backdrop_2nd_cycle,
            parking;

    /*----------------------------------------------------------------------------------------------
    -- Poses --
    ----------------------------------------------------------------------------------------------*/
    protected Integer[]
            rightSpikeStartingTangent, rightSpikeFinalTangent,
            leftSpikeStartingTangent, leftSpikeFinalTangent,
            stackStationTangent, parkingTangent;

    protected Integer
            leftSpikeStartingTangentValue = 0, leftSpikeFinalTangentValue = 0,
            rightSpikeStartingTangentValue = 0, rightSpikeFinalTangentValue = 0,
            stackStationTangentValue = 0, parkingTangentValue = 1;

    /* Spikes ------------------------------------------------------------------------------------*/
    protected Pose2d
            leftPixel_SHORT, centerPixel_SHORT, rightPixel_SHORT,
            leftPixel_LONG, centerPixel_LONG, rightPixel_LONG;

    /* Randomized Backdrop -----------------------------------------------------------------------*/
    protected Pose2d
            randomizationBackdropLeft, randomizationBackdropCenter, randomizationBackdropRight,
            randomizationBackdropLeftLong, randomizationBackdropCenterLong,
            randomizationBackdropRightLong;

    /* Backdrops ---------------------------------------------------------------------------------*/
    protected Pose2d backdropLeft, backdropCenter, backdropRight;

    /* Stacks ------------------------------------------------------------------------------------*/
    protected Pose2d
            stationInner, stationMiddle, stationOuter,
            stationInnerSecondCycle, stationMiddleSecondCycle, stationOuterSecondCycle;

    /* Parking -----------------------------------------------------------------------------------*/
    protected Pose2d parkingInner, parkingMiddle, parkingOuter;

    /* Mid Points --------------------------------------------------------------------------------*/
    protected Vector2d stationClose_Inner, stationFar_Inner, stationClose_Outer, stationFar_Outer;

    /*------------------------------------------------*/
//    public Pose2d
//            pixel_cycle_PoseTransfer = rightPixel_SHORT,
//            leftPixelSpike = leftPixel_SHORT,
//            centerPixelSpike = centerPixel_SHORT,
//            rightPixelSpike = rightPixel_SHORT,
//            randomizedBackdrop = backdropRight,
//            backdrop_Unload = backdropLeft,
//            stackStation = stationInner,
//            stackStationSecondCycle = stationInnerSecondCycle,
//            parkingPose = parkingMiddle;
//    public Vector2d
//            stationClose = stationClose_Inner,
//            stationFar = stationFar_Inner;

    public Pose2d pixel_cycle_PoseTransfer, leftPixelSpike, centerPixelSpike, rightPixelSpike,
            randomizedBackdrop, backdrop_Unload, stackStation, stackStationSecondCycle, parkingPose;
    public Vector2d stationClose, stationFar;

//    Dictionary<String, Pair<Double, Double>>
//            LEFT_RandomizationOffset_SHORT, LEFT_RandomizationOffset_LONG,
//            CENTER_RandomizationOffset_SHORT, CENTER_RandomizationOffset_LONG,
//            RIGHT_RandomizationOffset_SHORT, RIGHT_RandomizationOffset_LONG;

    /*----------------------------------------------------------------------------------------------
    -- La program --
    ----------------------------------------------------------------------------------------------*/
    RoadRunnerSubsystem(SampleMecanumDrive sampleDrive, Pose2d HomePose,
                        StartingPosition startingPosition, Path path, PixelStack pixelStack,
                        ParkingPosition parkingPosition, Telemetry telemetry) {
        this.HomePose = HomePose;
        this.drive = sampleDrive;
        this.startingPosition = startingPosition;
        this.path = path;
        this.pixelStack = pixelStack;
        this.parkingPosition = parkingPosition;
        this.telemetry = telemetry;

        /*----------------------------------------------------------------------------------------*/
        drive.setPoseEstimate(HomePose);
    }

    public void parking() {
        if (parkingPosition == ParkingPosition.INNER) {
            parkingTangentValue = 0;
            parkingPose = parkingInner;
        } else if (parkingPosition == ParkingPosition.MID) {
            parkingTangentValue = 1;
            parkingPose = parkingMiddle;
        } else if (parkingPosition == ParkingPosition.OUTER) {
            parkingTangentValue = 1;
            parkingPose = parkingOuter;
        }
    }

    public TrajectorySequenceBuilder getSpike(Randomization randomization) {
        if (startingPosition == StartingPosition.SHORT)
            switch (randomization) {
                case RIGHT:
                    return rightSpike;
                case CENTER:
                    return centerSpike;
                default:
                    return leftSpike;
            }

        switch (randomization) {
            case RIGHT:
                return rightSpike_LONG;
            case CENTER:
                return centerSpike_LONG;
            default:
                return leftSpike_LONG;
        }
    }

    public void runSpike(Randomization rand) {
        drive.followTrajectorySequenceAsync(getSpike(rand).build());
    }

    public void runSpike_Station(){
        drive.followTrajectorySequenceAsync(spike_station.build());
    }

    public void runSpike_RandomizedBackdrop() {
        drive.followTrajectorySequenceAsync(spike_randomizedBackdrop.build());
    }

    public void runBackdrop_Station(int idx) {
        if (idx == 0)
            drive.followTrajectorySequenceAsync(backdrop_station_1st_cycle.build());
        else if (idx == 1)
            drive.followTrajectorySequenceAsync(backdrop_station_2nd_cycle.build());
    }

    public void runStation_Backdrop(int idx){
        if (idx == 0)
            drive.followTrajectorySequenceAsync(station_backdrop_1st_cycle.build());
        else if (idx == 1)
            drive.followTrajectorySequenceAsync(station_backdrop_2nd_cycle.build());
    }

    public void runParking(){
        drive.followTrajectorySequenceAsync(parking.build());
    }

    public void runStation_RandomizedBackdrop(){
        drive.followTrajectorySequenceAsync(station_long_randomizedBackdrop.build());
    }

    public void runTest(){
        drive.followTrajectorySequence(test.build());
    }

}