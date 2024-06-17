package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Hashtable;

import kotlin.NotImplementedError;


public class RoadRunnerSubsystem extends SubsystemBase {
    /*----------------------------------------------------------------------------------------------
    -- Params --
    ----------------------------------------------------------------------------------------------*/
    public static double
        Tile = 24, TileInverted = -Tile, /*-inches-*/
        RobotX = 12.795, RobotY = 18.11, IntakeExt = 0.0, /*-inches-*/
        // RobotY = robot_length + 2 * outtake_extension
        // IntakeExt = intake_extension - outtake_extension
//        RobotY = 15.1, RobotX = 13.1, IntakeExt = 3, /*-inches-*/
        AccDefault = 52, VelDefault = 52;

    /*----------------------------------------------------------------------------------------------
    -- Enums --
    ----------------------------------------------------------------------------------------------*/
    enum Randomization {LEFT, CENTER, RIGHT}

    enum StartingPosition {SHORT, LONG}

    enum Path {INNER, OUTER}

    enum ParkingPosition {INNER, MID, OUTER}

    enum PixelStack {INNER, MID, OUTER}

    enum RobotSides {FRONT, REAR, CENTER, LEFT, RIGHT}

    protected Randomization rand;
    protected StartingPosition startingPosition;
    protected Path path;
    protected ParkingPosition parkingPosition;
    protected PixelStack pixelStack;

    protected SampleMecanumDrive drive;
    protected Telemetry telemetry;

    /*----------------------------------------------------------------------------------------------
    -- Offsets --
    ----------------------------------------------------------------------------------------------*/
    protected Pair<Double, Double> ZERO_OFFSET = new Pair<>(0.0, 0.0);

    protected Hashtable<String, Pair<Double, Double>>
        LEFT_RandomizationOffset_SHORT = new Hashtable<>(),
        CENTER_RandomizationOffset_SHORT = new Hashtable<>(),
        RIGHT_RandomizationOffset_SHORT = new Hashtable<>(),
        LEFT_RandomizationOffset_LONG = new Hashtable<>(),
        CENTER_RandomizationOffset_LONG = new Hashtable<>(),
        RIGHT_RandomizationOffset_LONG = new Hashtable<>();

    protected Hashtable<Randomization, Hashtable<String, Pair<Double, Double>>> OFFSETS_SHORT =
        new Hashtable<Randomization, Hashtable<String, Pair<Double, Double>>>() {{
            put(Randomization.LEFT, LEFT_RandomizationOffset_SHORT);
            put(Randomization.CENTER, CENTER_RandomizationOffset_SHORT);
            put(Randomization.RIGHT, RIGHT_RandomizationOffset_SHORT);
        }}, OFFSETS_LONG =
        new Hashtable<Randomization, Hashtable<String, Pair<Double, Double>>>() {{
            put(Randomization.LEFT, LEFT_RandomizationOffset_LONG);
            put(Randomization.CENTER, CENTER_RandomizationOffset_LONG);
            put(Randomization.RIGHT, RIGHT_RandomizationOffset_LONG);
        }};

    protected Hashtable<StartingPosition, Hashtable<Randomization, Hashtable<String, Pair<Double, Double>>>>
        OFFSETS =
        new Hashtable<StartingPosition, Hashtable<Randomization, Hashtable<String, Pair<Double, Double>>>>() {{
            put(StartingPosition.SHORT, OFFSETS_SHORT);
            put(StartingPosition.LONG, OFFSETS_LONG);
        }};

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
    protected Integer[] stackStationTangent, parkingTangent;

    protected Integer stackStationTangentValue = 0, parkingTangentValue = 1;

    /* Spikes ------------------------------------------------------------------------------------*/
    protected Pose2d
        leftPixel_SHORT, centerPixel_SHORT, rightPixel_SHORT,
        leftPixel_LONG, centerPixel_LONG, rightPixel_LONG;

    /* Backdrops ---------------------------------------------------------------------------------*/
    protected Pose2d backdropLeft, backdropCenter, backdropRight;

    /* Stacks ------------------------------------------------------------------------------------*/
    protected Pose2d stationInner, stationMiddle, stationOuter;

    /* Parking -----------------------------------------------------------------------------------*/
    protected Pose2d parkingInner, parkingMiddle, parkingOuter;

    /* Mid Points --------------------------------------------------------------------------------*/
    protected Vector2d stationClose_Inner, stationFar_Inner, stationClose_Outer, stationFar_Outer;

    public Pose2d pixel_cycle_PoseTransfer, leftPixelSpike, centerPixelSpike, rightPixelSpike,
        randomizedBackdrop, backdrop_Unload, stackStation, parkingPose;
    public Vector2d stationClose, stationFar;

    /*----------------------------------------------------------------------------------------------
    -- Util Functions --
    ----------------------------------------------------------------------------------------------*/
    public Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset) {
        return new Pose2d(
            pose.getX() + xy_offset.first, pose.getY() + xy_offset.second, pose.getHeading()
        );
    }

    public Vector2d offsetPoseShifter(Vector2d pose, Pair<Double, Double> xy_offset) {
        return new Vector2d(pose.getX() + xy_offset.first, pose.getY() + xy_offset.second);
    }

    public Pose2d offsetPoseShifter(Pose2d pose, String offset_dict_key) {
        Pair<Double, Double> xy_offset = OFFSETS.get(startingPosition).get(rand).getOrDefault(
            offset_dict_key, ZERO_OFFSET
        );
        return offsetPoseShifter(pose, xy_offset);
    }

    public Vector2d offsetPoseShifter(Vector2d pose, String offset_dict_key) {
        Pair<Double, Double> xy_offset = OFFSETS.get(startingPosition).get(rand).getOrDefault(
            offset_dict_key, ZERO_OFFSET
        );
        return offsetPoseShifter(pose, xy_offset);
    }

    public Pose2d robotPoseLimitCalculation(Pose2d pose, RobotSides side) {
        if (side == RobotSides.CENTER)
            return pose;

        double X = pose.getX(), Y = pose.getY(), H = pose.getHeading(),
            Afb = (RobotY / 2) * Math.sin(H), Bfb = (RobotY / 2) * Math.cos(H),
            Arl = (RobotX / 2) * Math.sin(H), Brl = (RobotX / 2) * Math.cos(H);

        switch (side) {
            case FRONT:
                X -= (Bfb + IntakeExt * Math.cos(H));
                Y -= (Afb + IntakeExt * Math.sin(H));
                break;
            case REAR:
                X += Bfb;
                Y += Afb;
                break;
            case LEFT:
                X += Arl;
                Y -= Brl;
                break;
            default: // RIGHT
                X -= Arl;
                Y += Brl;
                break;
        }

        return new Pose2d(X, Y, H);
    }

    /*----------------------------------------------------------------------------------------------
    -- La program --
    ----------------------------------------------------------------------------------------------*/
    RoadRunnerSubsystem(
        SampleMecanumDrive sampleDrive, Pose2d HomePose,
        StartingPosition startingPosition, Path path, PixelStack pixelStack,
        ParkingPosition parkingPosition, Telemetry telemetry
    ) {
        this.HomePose = HomePose;
        this.drive = sampleDrive;
        this.startingPosition = startingPosition;
        this.path = path;
        this.pixelStack = pixelStack;
        this.parkingPosition = parkingPosition;
        this.telemetry = telemetry;

        /*----------------------------------------------------------------------------------------*/
        drive.setPoseEstimate(HomePose);

        /*----------------------------------------------------------------------------------------*/
        setCycle();
        setParking();
    }

    public void setCycle() {
        if (path == Path.INNER) {
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            stackStation = stationInner;
            stackStationTangentValue = 0;
            return;
        }
        // else if (path == Path.OUTER)
        stationClose = stationClose_Outer;
        stationFar = stationFar_Outer;
        stackStation = stationOuter;
        stackStationTangentValue = 2;
    }

    public void setParking() {
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

    public void TrajectoryInit(Randomization rand) {
        throw new NotImplementedError("Implement Trajectory Init in Child classes");
    }

    /*----------------------------------------------------------------------------------------------
    -- Spike Related --
    ----------------------------------------------------------------------------------------------*/
    public void spikeRandomizationPath(Randomization randomization) {
        if (startingPosition == StartingPosition.LONG) {
            rightPixelSpike = leftPixel_LONG;
            leftPixelSpike = rightPixel_LONG;
            centerPixelSpike = centerPixel_LONG;
        } else {
            rightPixelSpike = rightPixel_SHORT;
            leftPixelSpike = leftPixel_SHORT;
            centerPixelSpike = centerPixel_SHORT;
        }

        if (randomization == Randomization.LEFT) {
            randomizedBackdrop = backdropLeft;
            pixel_cycle_PoseTransfer = leftPixelSpike;
        } else if (randomization == Randomization.CENTER) {
            randomizedBackdrop = backdropCenter;
            pixel_cycle_PoseTransfer = centerPixelSpike;
        } else {
            randomizedBackdrop = backdropRight;
            pixel_cycle_PoseTransfer = rightPixelSpike;
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

    /*----------------------------------------------------------------------------------------------
    -- Run Trajectories --
    ----------------------------------------------------------------------------------------------*/
    public void runSpike(Randomization rand) {
        drive.followTrajectorySequenceAsync(getSpike(rand).build());
    }

    public void runSpike_Station() {
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

    public void runStation_Backdrop(int idx) {
        if (idx == 0)
            drive.followTrajectorySequenceAsync(station_backdrop_1st_cycle.build());
        else if (idx == 1)
            drive.followTrajectorySequenceAsync(station_backdrop_2nd_cycle.build());
    }

    public void runParking() {
        drive.followTrajectorySequenceAsync(parking.build());
    }

    public void runStation_RandomizedBackdrop() {
        drive.followTrajectorySequenceAsync(station_long_randomizedBackdrop.build());
    }

    public void runTest() {
        drive.followTrajectorySequence(test.build());
    }
}