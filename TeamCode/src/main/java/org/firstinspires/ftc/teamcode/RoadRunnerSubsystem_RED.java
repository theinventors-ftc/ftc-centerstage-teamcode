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
    Dictionary<String, Pair<Double, Double>>
        LEFT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(0.0, 0.0));
            put("Stacks_Inner_FirstCycle", new Pair<>(-0.5, -2.5));
            put("Stacks_Inner_SecondCycle", new Pair<>(0.0, -2.5));
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
        }},
        CENTER_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(0.0, 0.0));
            put("Stacks_Inner_FirstCycle", new Pair<>(-1.5, -4.0));
            put("Stacks_Inner_SecondCycle", new Pair<>(-1.5, -4.0));
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
            put("Backdrop_Left_FirstCycle", new Pair<>(0.0, 3.0));
            put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
        }},
        RIGHT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(0.0, 0.0));
            put("Stacks_Inner_FirstCycle", new Pair<>(-2.0, -2.0));
            put("Stacks_Inner_SecondCycle", new Pair<>(-2.0, -2.0));
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
            put("Backdrop_Left_FirstCycle", new Pair<>(0.0, -2.0));
            put("Backdrop_Left_SecondCycle", new Pair<>(0.0, -2.0));
            put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
        }},
        LEFT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(-1.0, -2.0));
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
        }},
        CENTER_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(-4.0, -2.0));
            put("Stacks_Inner_FirstCycle", new Pair<>(-5.5, -3.0));
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

            put("Backdrop_Rand", new Pair<>(-4.0, 0.0));
            put("Backdrop_Left_FirstCycle", new Pair<>(-4.0, 0.0));
            put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
        }},
        RIGHT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
            put("Stacks_Rand", new Pair<>(-3.5, -3.0));
            put("Stacks_Inner_FirstCycle", new Pair<>(-3.5, -3.0));
            put("Stacks_Inner_SecondCycle", new Pair<>(0.0, 0.0));
            put("Stacks_Mid_FirstCycle", new Pair<>(0.0, 0.0));
            put("Stacks_Mid_SecondCycle", new Pair<>(0.0, 0.0));
            put("Stacks_Outer_FirstCycle", new Pair<>(0.0, 0.0));
            put("Stacks_Outer_SecondCycle", new Pair<>(0.0, 0.0));

            put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, 2.0));
            put("Corridor_Close_Inner_SecondCycle", new Pair<>(0.0, 0.0));
            put("Corridor_Close_Outer_FirstCycle", new Pair<>(0.0, 0.0));
            put("Corridor_Close_Outer_SecondCycle", new Pair<>(0.0, 0.0));

            put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, 2.0));
            put("Corridor_Far_Inner_SecondCycle", new Pair<>(0.0, 0.0));
            put("Corridor_Far_Outer_FirstCycle", new Pair<>(0.0, 0.0));
            put("Corridor_Far_Outer_SecondCycle", new Pair<>(0.0, 0.0));

            put("Backdrop_Rand", new Pair<>(-1.5, 3.0));
            put("Backdrop_Left_FirstCycle", new Pair<>(-3.0, -3.0));
            put("Backdrop_Left_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Center_SecondCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_FirstCycle", new Pair<>(0.0, 0.0));
            put("Backdrop_Right_SecondCycle", new Pair<>(0.0, 0.0));
        }};

    public Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>>
            OFFSETS_SHORT = new Hashtable<
            Randomization, Dictionary<String, Pair<Double, Double>>
            >() {{
        put(Randomization.LEFT, LEFT_RandomizationOffset_SHORT);
        put(Randomization.CENTER, CENTER_RandomizationOffset_SHORT);
        put(Randomization.RIGHT, RIGHT_RandomizationOffset_SHORT);
    }},
            OFFSETS_LONG = new Hashtable<
                    Randomization, Dictionary<String, Pair<Double, Double>>
                    >() {{
                put(Randomization.LEFT, LEFT_RandomizationOffset_LONG);
                put(Randomization.CENTER, CENTER_RandomizationOffset_LONG);
                put(Randomization.RIGHT, RIGHT_RandomizationOffset_LONG);
            }};

    public Dictionary<StartingPosition, Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>>> OFFSETS = new Hashtable<StartingPosition, Dictionary<Randomization, Dictionary<String, Pair<Double, Double>>>>() {{
        put(StartingPosition.SHORT, OFFSETS_SHORT);
        put(StartingPosition.LONG, OFFSETS_LONG);
    }};

    /*----------------------------------------------------------------------------------------------
    -- La program --
    ----------------------------------------------------------------------------------------------*/
    RoadRunnerSubsystem_RED(SampleMecanumDrive sampleDrive, Pose2d HomePose,
                            StartingPosition startingPosition, Path path, PixelStack pixelStack,
                            ParkingPosition parkingPosition, Telemetry telemetry) {
        super(sampleDrive, HomePose, startingPosition, path, pixelStack, parkingPosition, telemetry);

        /*------------------------------------------------------------------------------------------
        -- Poses --
        ------------------------------------------------------------------------------------------*/
        //For short 45 and long 135 difference
        leftSpikeStartingTangent = new Integer[]{45, 135};
        //For short 180 and long 0 difference
        leftSpikeFinalTangent = new Integer[]{180, 0};
        // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
        stackStationTangent = new Integer[]{180, 225, 135};
        // 135 for Inner 225 for Mid and Outer
        parkingTangent = new Integer[]{135, 225};

        /* Spikes --------------------------------------------------------------------------------*/
        leftPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(0, 1.40 * TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );
        centerPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile/2, TileInverted, Math.toRadians(90)),
                RobotSides.FRONT
        );
        rightPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile, 1.5 * TileInverted, Math.toRadians(90)),
                RobotSides.FRONT
        );
        leftPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(2 * TileInverted,1.25 * TileInverted, Math.toRadians(135)),
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

        /* Randomized Backdrop -------------------------------------------------------------------*/
        randomizationBackdropLeft = robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile, 1.3 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        randomizationBackdropCenter = robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile, 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        randomizationBackdropRight =  robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile, 1.9 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );

        /* Backdrops -----------------------------------------------------------------------------*/
        backdropLeft = robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile ,1.3 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        backdropCenter = robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile , 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );
        backdropRight = robotPoseLimitCalculation(
                new Pose2d(2.75 * Tile , 1.75 * TileInverted, Math.toRadians(180)),
                RobotSides.REAR
        );

        /* Stacks Second Cycle -------------------------------------------------------------------*/
        stationInnerSecondCycle = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted,TileInverted/2, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationMiddleSecondCycle = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted,TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationOuterSecondCycle = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );

        /* Stacks First Cycle --------------------------------------------------------------------*/
        stationInner = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted,TileInverted/2, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationMiddle = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted, TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );
        stationOuter = robotPoseLimitCalculation(
                new Pose2d(2.7 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)),
                RobotSides.FRONT
        );

        /* Parking -------------------------------------------------------------------------------*/
        parkingInner = new Pose2d(2.5 * Tile, TileInverted/2, Math.toRadians(180));
        parkingMiddle = new Pose2d(2 * Tile, 1.5 * TileInverted, Math.toRadians(180));
        parkingOuter = new Pose2d(2.5 * Tile, 2.5 * TileInverted, Math.toRadians(180));

        /* Mid Points ----------------------------------------------------------------------------*/
        stationClose_Inner = new Vector2d(Tile/2, TileInverted/2);
        stationFar_Inner = new Vector2d(TileInverted,TileInverted/2);
        stationClose_Outer = new Vector2d(Tile, 2.5 * TileInverted);
        stationFar_Outer = new Vector2d(1.5 * TileInverted,2.5 * TileInverted);
    }

    public void TrajectoryInit(Randomization rand){
        test = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .back(30);

        /*----------------------------------------------------------------------------------------*/
        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(rightPixelSpike.vec());
        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixelSpike.vec());
        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                //tan pair 45/135
                .setTangent(Math.toRadians(leftSpikeStartingTangent[leftSpikeStartingTangentValue]))
                .splineToLinearHeading(
                        offsetPoseShifter(leftPixelSpike, new Pair<>(2.0, 0.0)),
                        Math.toRadians(leftSpikeFinalTangent[leftSpikeFinalTangentValue])
                );

        /*----------------------------------------------------------------------------------------*/
        rightSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .splineToLinearHeading(rightPixel_LONG, Math.toRadians(0));
        centerSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .lineTo(centerPixel_LONG.vec());
        leftSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
                .lineToLinearHeading(leftPixel_LONG);

        /*----------------------------------------------------------------------------------------*/
        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(315))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/
        backdrop_station_1st_cycle = drive.trajectorySequenceBuilder(randomizedBackdrop)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(
                        offsetPoseShifter(
                                stackStation.vec(),
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Stacks_Inner_FirstCycle"
                                )
                        ),
                        Math.toRadians(stackStationTangent[stackStationTangentValue])
                ); //tan pair 180/225

        backdrop_station_2nd_cycle = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(
                        offsetPoseShifter(
                                stackStationSecondCycle.vec(),
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Stacks_Inner_SecondCycle"
                                )
                        ),
                        Math.toRadians(stackStationTangent[stackStationTangentValue])
                ); //tan pair 180/225

        /*----------------------------------------------------------------------------------------*/
        station_backdrop_1st_cycle = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(
                        offsetPoseShifter(
                                stationFar,
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Corridor_Far_Inner_FirstCycle"
                                )
                        ),
                        Math.toRadians(0)
                )
                .lineTo(
                        offsetPoseShifter(
                                stationClose,
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Corridor_Close_Inner_FirstCycle"
                                )
                        )
                )
                .splineToConstantHeading(offsetPoseShifter(
                        backdrop_Unload.vec(),
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Backdrop_Left_FirstCycle"
                                )
                        ),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(
                                45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                        ),
                        SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        station_backdrop_2nd_cycle = drive.trajectorySequenceBuilder(stackStationSecondCycle)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(
                        backdrop_Unload.vec(),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(
                                45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                        ),
                        SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /*----------------------------------------------------------------------------------------*/
        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(
                        offsetPoseShifter(
                                stackStation,
                                OFFSETS.get(startingPosition).get(rand).get("Stacks_Rand")
                        ),
                        Math.toRadians(180)
                );

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(
                        offsetPoseShifter(
                                randomizedBackdrop.vec(),
                                OFFSETS.get(startingPosition).get(rand).get("Backdrop_Rand")
                        ),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(
                                35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                        ),
                        SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /*----------------------------------------------------------------------------------------*/
        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTangent[parkingTangentValue])) //tan 135/225
                .splineToLinearHeading(
                        new Pose2d(parkingPose.vec(), Math.toRadians(90)), Math.toRadians(0)
                );
    }

    public void spikeRandomizationPath(Randomization randomization) {
        if (startingPosition == StartingPosition.SHORT) {
            if (randomization == Randomization.LEFT) {
                randomizedBackdrop = randomizationBackdropLeft;
                leftPixelSpike = leftPixel_SHORT;
                pixel_cycle_PoseTransfer = leftPixel_SHORT;
                leftSpikeStartingTangentValue = 0;
                leftSpikeFinalTangentValue = 0;
            } else if (randomization == Randomization.CENTER) {
                randomizedBackdrop = randomizationBackdropCenter;
                centerPixelSpike = centerPixel_SHORT;
                pixel_cycle_PoseTransfer = centerPixel_SHORT;
            } else if (randomization == Randomization.RIGHT) {
                randomizedBackdrop = randomizationBackdropRight;
                rightPixelSpike = rightPixel_SHORT;
                pixel_cycle_PoseTransfer = rightPixel_SHORT;
            }
        } else if (startingPosition == StartingPosition.LONG) {
            if (randomization == Randomization.LEFT) {
                randomizedBackdrop = randomizationBackdropLeft;
                rightPixelSpike = leftPixel_LONG;
                pixel_cycle_PoseTransfer = leftPixel_LONG;

            } else if (randomization == Randomization.CENTER) {
                randomizedBackdrop = randomizationBackdropCenter;
                centerPixelSpike = centerPixel_LONG;
                pixel_cycle_PoseTransfer = centerPixel_LONG;

            } else if (randomization == Randomization.RIGHT) {
                randomizedBackdrop = randomizationBackdropRight;
                leftPixelSpike = rightPixel_LONG;
                pixel_cycle_PoseTransfer = rightPixel_LONG;
                leftSpikeStartingTangentValue = 1;
                leftSpikeFinalTangentValue = 1;
            }
        }
    }

    public void cycle() {
        if (path == Path.INNER) {
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            backdrop_Unload = backdropLeft;
            stackStation = stationInner;
            stackStationSecondCycle = stationInnerSecondCycle;
            stackStationTangentValue = 0;
        } else if (path == Path.OUTER) {
            stationClose = stationClose_Outer;
            stationFar = stationFar_Outer;
            backdrop_Unload = backdropRight;
            stackStation = stationOuter;
            stackStationSecondCycle = stationOuterSecondCycle;
            stackStationTangentValue = 2;
        }
    }
}