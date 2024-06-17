package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

import java.util.Dictionary;
import java.util.Hashtable;


public class RoadRunnerSubsystem_BLUE extends RoadRunnerSubsystem {
    Dictionary<String, Pair<Double, Double>>
        LEFT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
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
        }},
        CENTER_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
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
        }},
        RIGHT_RandomizationOffset_SHORT = new Hashtable<String, Pair<Double, Double>>() {{
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
        }},
        LEFT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
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
        }},
        CENTER_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
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
        }},
        RIGHT_RandomizationOffset_LONG = new Hashtable<String, Pair<Double, Double>>() {{
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
    RoadRunnerSubsystem_BLUE(SampleMecanumDrive sampleDrive, Pose2d HomePose,
                            StartingPosition startingPosition, Path path, PixelStack pixelStack,
                            ParkingPosition parkingPosition, Telemetry telemetry) {
        super(sampleDrive, HomePose, startingPosition, path, pixelStack, parkingPosition, telemetry);

        /*------------------------------------------------------------------------------------------
        -- Poses --
        ------------------------------------------------------------------------------------------*/
        //For short 45 and long 135 difference
        leftSpikeStartingTangent = new Integer[]{315, 225};
        //For short 180 and long 0 difference
        leftSpikeFinalTangent = new Integer[]{180, 0};
        // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
        stackStationTangent = new Integer[]{180, 135, 225};
        // 135 for Inner 225 for Mid and Outer
        parkingTangent = new Integer[]{180, 135};

        /* Spikes --------------------------------------------------------------------------------*/
        leftPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile, 1.8 * Tile, Math.toRadians(270)),
                RoadRunnerSubsystem_BLUE.RobotSides.CENTER
        );
        centerPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(Tile/2, Tile, Math.toRadians(270)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT
        );
        rightPixel_SHORT = robotPoseLimitCalculation(
                new Pose2d(0, Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT
        );
        leftPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(TileInverted,1.5 * Tile, Math.toRadians(0)),
                RobotSides.FRONT);
        centerPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(1.9 * TileInverted, Tile, Math.toRadians(0)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT);
        rightPixel_LONG = robotPoseLimitCalculation(
                new Pose2d(1.95 * TileInverted, 1.25 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT);

        /* Randomized Backdrop -------------------------------------------------------------------*/
        randomizationBackdropLeft = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.75 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        randomizationBackdropCenter = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.5 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        randomizationBackdropRight =  robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.25 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );

        randomizationBackdropLeftLong = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.75 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        randomizationBackdropCenterLong = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.5 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        randomizationBackdropRightLong =  robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile, 1.3 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );

        /* Backdrops -----------------------------------------------------------------------------*/
        backdropLeft = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile ,1.75 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        backdropCenter = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile , 1.5 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );
        backdropRight = robotPoseLimitCalculation(
                new Pose2d(2.5 * Tile , 1.4 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.REAR
        );

        /* Stacks --------------------------------------------------------------------------------*/
        stationInner = robotPoseLimitCalculation(
                new Pose2d(2.9 * TileInverted,Tile/2, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT
        );
        stationMiddle = robotPoseLimitCalculation(
                new Pose2d(2.9 * TileInverted, Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT
        );
        stationOuter = robotPoseLimitCalculation(
                new Pose2d(2.9 * TileInverted, 1.5 * Tile, Math.toRadians(180)),
                RoadRunnerSubsystem_BLUE.RobotSides.FRONT
        );

        /* Parking -------------------------------------------------------------------------------*/
        parkingInner = new Pose2d(2.5 * Tile, TileInverted/2, Math.toRadians(180));
        parkingMiddle = new Pose2d(2 * Tile, 1.5 * TileInverted, Math.toRadians(180));
        parkingOuter = new Pose2d(2.5 * Tile, 2.5 * TileInverted, Math.toRadians(180));

        /* Mid Points ----------------------------------------------------------------------------*/
        stationClose_Inner = new Vector2d(Tile/2, Tile/2);
        stationFar_Inner = new Vector2d(TileInverted,Tile/2);
        stationClose_Outer = new Vector2d(Tile/2, 2.5 * Tile);
        stationFar_Outer = new Vector2d(TileInverted,2.5 * Tile);
    }

    public void TrajectoryInit(Randomization rand){
        test = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .back(30);

        /*----------------------------------------------------------------------------------------*/
        leftSpike = drive.trajectorySequenceBuilder(HomePose)
                .strafeTo(leftPixelSpike.vec());
        centerSpike = drive.trajectorySequenceBuilder(HomePose)
                .splineToLinearHeading(centerPixelSpike, Math.toRadians(270));
        rightSpike = drive.trajectorySequenceBuilder(HomePose)
                //tan pair 45/135
                .setTangent(
                        Math.toRadians(rightSpikeStartingTangent[rightSpikeStartingTangentValue])
                )
                .splineToLinearHeading(
                        rightPixelSpike,
                        Math.toRadians(rightSpikeFinalTangent[rightSpikeFinalTangentValue])
                );

        /*----------------------------------------------------------------------------------------*/
        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(90))
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
                                stackStation.vec(),
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
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(
                        offsetPoseShifter(
                                backdrop_Unload.vec(),
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Backdrop_Right_FirstCycle"
                                )
                        ),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(
                                45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                        ),
                        SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        station_backdrop_2nd_cycle = drive.trajectorySequenceBuilder(stackStation)
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .splineToConstantHeading(offsetPoseShifter(
                        backdrop_Unload.vec(),
                                OFFSETS.get(startingPosition).get(rand).get(
                                        "Backdrop_Right_SecondCycle"
                                )
                        ),
                        Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(
                                45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                        ),
                        SampleMecanumDrive.getAccelerationConstraint(AccDefault)
                );

        /*----------------------------------------------------------------------------------------*/
        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
                .setTangent(Math.toRadians(280))
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
                        Math.toRadians(0)
                );

        /*----------------------------------------------------------------------------------------*/
        parking = drive.trajectorySequenceBuilder(backdrop_Unload)
                .setTangent(Math.toRadians(parkingTangent[parkingTangentValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }

    public void spikeRandomizationPath(Randomization randomization) {
        if (startingPosition == StartingPosition.LONG) {
            rightPixelSpike = rightPixel_LONG;
            leftPixelSpike = leftPixel_LONG;
            centerPixelSpike = centerPixel_LONG;

            if (randomization == Randomization.LEFT) {
                randomizedBackdrop = randomizationBackdropLeftLong;
                leftPixelSpike = leftPixel_LONG;
                pixel_cycle_PoseTransfer = leftPixel_LONG;
                rightSpikeStartingTangentValue = 1;
                rightSpikeFinalTangentValue = 1;
            } else if (randomization == Randomization.CENTER) {
                randomizedBackdrop = randomizationBackdropCenterLong;
                centerPixelSpike = centerPixel_LONG;
                pixel_cycle_PoseTransfer = centerPixel_LONG;
            } else { // Randomization.RIGHT
                randomizedBackdrop = randomizationBackdropRightLong;
                rightPixelSpike = rightPixel_LONG;
                pixel_cycle_PoseTransfer = rightPixel_LONG;
            }
            return;
        }
        // StartingPosition.SHORT
        rightPixelSpike = rightPixel_SHORT;
        leftPixelSpike = leftPixel_SHORT;
        centerPixelSpike = centerPixel_SHORT;

        if (randomization == Randomization.LEFT) {
            randomizedBackdrop = randomizationBackdropLeft;
            pixel_cycle_PoseTransfer = leftPixel_SHORT;
            rightSpikeStartingTangentValue = 0;
            rightSpikeFinalTangentValue = 0;
        } else if (randomization == Randomization.CENTER) {
            randomizedBackdrop = randomizationBackdropCenter;
            pixel_cycle_PoseTransfer = centerPixel_SHORT;
        } else if (randomization == Randomization.RIGHT) {
            randomizedBackdrop = randomizationBackdropRight;
            pixel_cycle_PoseTransfer = rightPixel_SHORT;
        }
    }

    public void cycle() {
        if (path == Path.INNER) {
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            backdrop_Unload = backdropRight;
            stackStation = stationInner;
            stackStationTangentValue = 0;
            return;
        }
        // else if (path == Path.OUTER)
        stationClose = stationClose_Outer;
        stationFar = stationFar_Outer;
        backdrop_Unload = backdropLeft;
        stackStation = stationOuter;
        stackStationTangentValue = 2;
    }
}