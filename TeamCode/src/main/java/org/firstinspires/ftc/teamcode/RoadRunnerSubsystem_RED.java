package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

import java.util.Dictionary;
import java.util.Hashtable;


public class RoadRunnerSubsystem_RED extends RoadRunnerSubsystem {
    @Override
    public void OffsetsInit() {
        // Available key names:
        // "Stacks_Rand", "Stacks_Inner_FirstCycle","Stacks_Inner_SecondCycle",
        // "Stacks_Mid_FirstCycle", "Stacks_Mid_SecondCycle", "Stacks_Outer_FirstCycle",
        // "Stacks_Outer_SecondCycle", "Corridor_Close_Inner_FirstCycle",
        // "Corridor_Close_Inner_Rand", "Corridor_Far_Inner_Rand",
        // "Corridor_Close_Inner_SecondCycle", "Corridor_Close_Outer_FirstCycle",
        // "Corridor_Close_Outer_SecondCycle", "Corridor_Far_Inner_FirstCycle",
        // "Corridor_Far_Inner_SecondCycle",  "Corridor_Far_Outer_FirstCycle",
        // "Corridor_Far_Outer_SecondCycle", "Backdrop_Rand", "Backdrop_Left_FirstCycle",
        // "Backdrop_Left_SecondCycle", "Backdrop_Center_FirstCycle",
        // "Backdrop_Center_SecondCycle", "Backdrop_Right_FirstCycle", "Backdrop_Right_SecondCycle"

        LEFT_RandomizationOffset_SHORT.put("Backdrop_Rand", new Pair<>(3.5, 0.0));
        LEFT_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-1.0, -3.0));
        LEFT_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-1.5, -3.5));
        LEFT_RandomizationOffset_SHORT.put("Backdrop_Left_FirstCycle", new Pair<>(3.5, 0.0));
        LEFT_RandomizationOffset_SHORT.put("Backdrop_Left_SecondCycle", new Pair<>(3.5, 0.0));

        CENTER_RandomizationOffset_SHORT.put("Backdrop_Rand", new Pair<>(3.0, 0.0));
        CENTER_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-1.5, -4.0));
        CENTER_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-1.5, -4.0));
        CENTER_RandomizationOffset_SHORT.put("Backdrop_Left_FirstCycle", new Pair<>(3.0, 0.0));
        CENTER_RandomizationOffset_SHORT.put("Backdrop_Left_SecondCycle", new Pair<>(3.0, 0.0));

        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Rand", new Pair<>(3.5, 0.0));
        RIGHT_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-2.0, -2.0));
        RIGHT_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-2.0, -2.0));
        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Left_FirstCycle", new Pair<>(3.5, -2.0));
        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Left_SecondCycle", new Pair<>(3.5, -2.0));

        LEFT_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-1.0, -2.0));

        CENTER_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-4.0, -2.0));
        CENTER_RandomizationOffset_LONG.put("Stacks_Inner_FirstCycle", new Pair<>(-5.5, -3.0));
        CENTER_RandomizationOffset_LONG.put("Backdrop_Rand", new Pair<>(-4.0, 0.0));
        CENTER_RandomizationOffset_LONG.put("Backdrop_Left_FirstCycle", new Pair<>(-4.0, 0.0));

        RIGHT_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-3.5, -3.0));
        RIGHT_RandomizationOffset_LONG.put("Stacks_Inner_FirstCycle", new Pair<>(-3.5, -3.0));
        RIGHT_RandomizationOffset_LONG.put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 2.0));
        RIGHT_RandomizationOffset_LONG.put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 2.0));
        RIGHT_RandomizationOffset_LONG.put("Backdrop_Rand", new Pair<>(-1.5, 3.0));
        RIGHT_RandomizationOffset_LONG.put("Backdrop_Left_FirstCycle", new Pair<>(-3.0, -3.0));

        LEFT_RandomizationOffset_LONG.put(
            "Backdrop_Left_SecondCycle", LEFT_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Left_FirstCycle", ZERO_OFFSET
            )
        );
        CENTER_RandomizationOffset_LONG.put(
            "Backdrop_Left_SecondCycle", CENTER_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Left_FirstCycle", ZERO_OFFSET
            )
        );
        RIGHT_RandomizationOffset_LONG.put(
            "Backdrop_Left_SecondCycle", RIGHT_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Left_FirstCycle", ZERO_OFFSET
            )
        );
    }

    /*----------------------------------------------------------------------------------------------
    -- La program --
    ----------------------------------------------------------------------------------------------*/
    RoadRunnerSubsystem_RED(
        SampleMecanumDrive sampleDrive, Pose2d HomePose,
        StartingPosition startingPosition, Path path, PixelStack pixelStack,
        ParkingPosition parkingPosition, Telemetry telemetry
    ) {
        super(
            sampleDrive, HomePose, startingPosition, path, pixelStack, parkingPosition, telemetry
        );
    }

    @Override
    public void PosesInit() {
       /*------------------------------------------------------------------------------------------
        -- Poses --
        ------------------------------------------------------------------------------------------*/
        /* Spikes --------------------------------------------------------------------------------*/
        leftPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(2, 1.6 * TileInverted, Math.toRadians(180)), RobotSides.FRONT
        );
        centerPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile/2, TileInverted, Math.toRadians(90)), RobotSides.FRONT
        );
        rightPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile, 1.6 * TileInverted, Math.toRadians(180)), RobotSides.FRONT
        );
        leftPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(1.9 * TileInverted,1.25 * TileInverted, Math.toRadians(135)),
                RobotSides.FRONT
        );
        centerPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(1.75 * TileInverted, TileInverted, Math.toRadians(359)),
                RobotSides.FRONT
        );
        rightPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(TileInverted, 1.40 * TileInverted, Math.toRadians(0)),
                RobotSides.FRONT
        );

        /* Backdrops -----------------------------------------------------------------------------*/
        backdropLeft = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile ,1.3 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        backdropCenter = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile , 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        backdropRight = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile , 1.75 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
//        randomizationBackdropRight =  robotPoseLimitCalculation(
//                new Pose2d(2.75 * Tile, 1.9 * TileInverted, Math.toRadians(180)),
//                RobotSides.REAR
//        );

        /* Stacks --------------------------------------------------------------------------------*/
        stationInner = robotPoseLimitCalculation(
                new Pose2d(2.8 * TileInverted,TileInverted/2, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationMiddle = robotPoseLimitCalculation(
                new Pose2d(2.8 * TileInverted, TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationOuter = robotPoseLimitCalculation(
                new Pose2d(2.8 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );

        /* Mid Points ----------------------------------------------------------------------------*/
        stationClose_Inner = new Vector2d(Tile/2, TileInverted/2);
        stationFar_Inner = new Vector2d(TileInverted,TileInverted/2);
        stationClose_Outer = new Vector2d(Tile, 2.5 * TileInverted);
        stationFar_Outer = new Vector2d(1.5 * TileInverted,2.5 * TileInverted);

        /* Parking -------------------------------------------------------------------------------*/
        parkingInner = new Pose2d(2.5 * Tile, TileInverted/2, Math.toRadians(180));
        parkingMiddle = new Pose2d(2 * Tile, 1.5 * TileInverted, Math.toRadians(180));
        parkingOuter = new Pose2d(2.5 * Tile, 2.5 * TileInverted, Math.toRadians(180));
    }

    @Override
    public void TrajectoryInit(Randomization rand) {
//        assert stationClose != null : "Is null, stationClose";
//        assert stationFar != null : "Is null, stationFar";
//        assert stackStation != null : "Is null, stackStation";
//        assert parkingPose != null : "Is null, parkingPose";

        /*----------------------------------------------------------------------------------------*/

        this.rand = rand;

        test = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .back(30);

        /*----------------------------------------------------------------------------------------*/
        // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
        stackStationTangent = new Integer[]{180, 225, 135};
        // 135 for Inner 225 for Mid and Outer
        parkingTangent = new Integer[]{135, 225};

        /*----------------------------------------------------------------------------------------*/
        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineToLinearHeading(rightPixelSpike);
        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixelSpike.vec());
        leftSpike = drive.trajectorySequenceBuilder(HomePose)
//                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(leftPixelSpike, Math.toRadians(180));

        /*----------------------------------------------------------------------------------------*/
        rightSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .splineToLinearHeading(rightPixel_LONG, Math.toRadians(0));
        centerSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixel_LONG.vec());
        leftSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .lineToLinearHeading(leftPixel_LONG);

        /*----------------------------------------------------------------------------------------*/
        randomizedBackdrop = offsetPoseShifter(randomizedBackdrop, "Backdrop_Rand");

        /* SHORT ---------------------------------------------------------------------------------*/
        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /* LONG ----------------------------------------------------------------------------------*/
        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(
                offsetPoseShifter(stackStation,"Stacks_Rand"), Math.toRadians(180)
            );

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(
                offsetPoseShifter(stackStation, "Stacks_Rand"))
            .setReversed(true)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(
                offsetPoseShifter(stationFar,"Corridor_Far_Inner_Rand"),
                Math.toRadians(0))
            .lineTo(offsetPoseShifter(stationClose, "Corridor_Close_Inner_Rand"))
            .splineToConstantHeading(
                randomizedBackdrop.vec(),
                Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(
                    35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ),
                SampleMecanumDrive.getAccelerationConstraint(AccDefault)
            );

        /* COMMON --------------------------------------------------------------------------------*/
        backdrop_station_1st_cycle = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(
                    offsetPoseShifter(stationClose, "Corridor_Close_Inner_FirstCycle"),
                    Math.toRadians(180)
                )
                .lineTo(offsetPoseShifter(stationFar, "Corridor_Far_Inner_FirstCycle"))
                .splineToConstantHeading(
                    offsetPoseShifter(stackStation.vec(), "Stacks_Inner_FirstCycle"),
                    Math.toRadians(stackStationTangent[stackStationTangentValue])
                ); //tan pair 180/225

        station_backdrop_1st_cycle = drive.trajectorySequenceBuilder(
                offsetPoseShifter(stackStation, "Stacks_Inner_FirstCycle"))
            .setReversed(true)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(
                offsetPoseShifter(stationFar, "Corridor_Far_Inner_FirstCycle"),
                Math.toRadians(0)
            )
            .lineTo(offsetPoseShifter(stationClose, "Corridor_Close_Inner_FirstCycle"))
            .splineToConstantHeading(
                offsetPoseShifter(backdrop_Unload.vec(), "Backdrop_Left_FirstCycle"),
                Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(
                    45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ),
                SampleMecanumDrive.getAccelerationConstraint(AccDefault)
            );

        /* SHORT (due to time) -------------------------------------------------------------------*/
        backdrop_station_2nd_cycle = drive.trajectorySequenceBuilder(
            offsetPoseShifter(backdrop_Unload, "Backdrop_Left_FirstCycle"))
            .setTangent(Math.toRadians(180))
            .splineToConstantHeading(
                offsetPoseShifter(stationClose, "Corridor_Close_Inner_SecondCycle"),
                Math.toRadians(180)
            )
            .lineTo(offsetPoseShifter(stationFar, "Corridor_Far_Inner_SecondCycle"))
            .splineToConstantHeading(
                offsetPoseShifter(stackStation.vec(), "Stacks_Inner_SecondCycle"),
                Math.toRadians(stackStationTangent[stackStationTangentValue])
            ); //tan pair 180/225


        station_backdrop_2nd_cycle = drive.trajectorySequenceBuilder(
                    offsetPoseShifter(stackStation, "Stacks_Inner_SecondCycle"))
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                    offsetPoseShifter(stationFar, "Corridor_Far_Inner_SecondCycle"),
                    Math.toRadians(0)
                )
                .lineTo(
                    offsetPoseShifter(stationClose, "Corridor_Close_Inner_SecondCycle")
                )
                .splineToConstantHeading(offsetPoseShifter(
                    backdrop_Unload.vec(), "Backdrop_Left_SecondCycle"),
                    Math.toRadians(0),
                    SampleMecanumDrive.getVelocityConstraint(
                        45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                    ),
                    SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /* COMMON --------------------------------------------------------------------------------*/
        // We have made sure that "Backdrop_Left_SecondCycle" is the same as
        // "Backdrop_Left_FirstCycle" for BLUE/RED_LONG
        parking = drive.trajectorySequenceBuilder(
                offsetPoseShifter(backdrop_Unload, "Backdrop_Left_SecondCycle"))
                .setTangent(Math.toRadians(parkingTangent[parkingTangentValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }

    @Override
    public void setCycle() {
        super.setCycle();

        if (path == Path.INNER)
            backdrop_Unload = backdropLeft;
        else // Path.OUTER
            backdrop_Unload = backdropRight;
    }
}