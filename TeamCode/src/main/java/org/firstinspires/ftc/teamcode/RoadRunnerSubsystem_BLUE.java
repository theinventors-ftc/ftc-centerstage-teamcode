package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;


public class RoadRunnerSubsystem_BLUE extends RoadRunnerSubsystem {
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

        LEFT_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-7.0, -4.0));
        LEFT_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-7.0, -4.0));
        LEFT_RandomizationOffset_SHORT.put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, -2.0));
        LEFT_RandomizationOffset_SHORT.put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, -2.0));
        LEFT_RandomizationOffset_SHORT.put("Backdrop_Left_FirstCycle", new Pair<>(-15.0, 0.0));
        LEFT_RandomizationOffset_SHORT.put("Backdrop_Left_SecondCycle", new Pair<>(-15.0, 0.0));

        CENTER_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-5.0, -1.5));
        CENTER_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-4.5, 0.0));
        CENTER_RandomizationOffset_SHORT.put("Backdrop_Left_FirstCycle", new Pair<>(0.5, 3.0));
        CENTER_RandomizationOffset_SHORT.put("Backdrop_Left_SecondCycle", new Pair<>(0.5, 6.0));

        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Rand", new Pair<>(-0.5, 0.0));
        RIGHT_RandomizationOffset_SHORT.put("Stacks_Inner_FirstCycle", new Pair<>(-5.0, -1.5));
        RIGHT_RandomizationOffset_SHORT.put("Stacks_Inner_SecondCycle", new Pair<>(-5.0, 0.0));
        RIGHT_RandomizationOffset_SHORT.put("Corridor_Close_Inner_FirstCycle", new Pair<>(0.0, -2.0));
        RIGHT_RandomizationOffset_SHORT.put("Corridor_Far_Inner_FirstCycle", new Pair<>(0.0, -2.0));
        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Right_FirstCycle", new Pair<>(-1.0, 0.0));
        RIGHT_RandomizationOffset_SHORT.put("Backdrop_Right_SecondCycle", new Pair<>(-1.0, 0.0));

        ////////////////////////////////////////////////////////////////////////////////////////////

        LEFT_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-6.0, 0.0));
        LEFT_RandomizationOffset_LONG.put("Backdrop_Rand", new Pair<>(-0.5, 3.0));
        LEFT_RandomizationOffset_LONG.put("Stacks_Inner_FirstCycle", new Pair<>(-6.0, 1.5));
        LEFT_RandomizationOffset_LONG.put("Backdrop_Right_FirstCycle", new Pair<>(-0.5, 4.0));

        CENTER_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-6.0, 0.0));
        CENTER_RandomizationOffset_LONG.put("Stacks_Inner_FirstCycle", new Pair<>(-6.0, 0.0));
        CENTER_RandomizationOffset_LONG.put("Backdrop_Right_FirstCycle", new Pair<>(-0.5, 4.0));

        RIGHT_RandomizationOffset_LONG.put("Stacks_Rand", new Pair<>(-5.0, 0.0)); // PEOS
        RIGHT_RandomizationOffset_LONG.put("Stacks_Inner_FirstCycle", new Pair<>(-6.0, 1.0)); // PEOS
        RIGHT_RandomizationOffset_LONG.put("Stacks_Inner_SecondCycle", new Pair<>(-1.0, 3.0)); // PEOS
        RIGHT_RandomizationOffset_LONG.put("Backdrop_Rand", new Pair<>(-2.0, 0.0)); // PEOS
        RIGHT_RandomizationOffset_LONG.put("Backdrop_Right_FirstCycle", new Pair<>(-1.0, 2.0)); // PEOS

        LEFT_RandomizationOffset_LONG.put(
            "Backdrop_Right_SecondCycle", LEFT_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Right_FirstCycle", ZERO_OFFSET
            )
        ); // PEOS
        CENTER_RandomizationOffset_LONG.put(
            "Backdrop_Right_SecondCycle", CENTER_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Right_FirstCycle", ZERO_OFFSET
            )
        ); // PEOS
        RIGHT_RandomizationOffset_LONG.put(
            "Backdrop_Right_SecondCycle", RIGHT_RandomizationOffset_LONG.getOrDefault(
                "Backdrop_Right_FirstCycle", ZERO_OFFSET
            )
        ); // PEOS
    }

    /*----------------------------------------------------------------------------------------------
    -- La program --
    ----------------------------------------------------------------------------------------------*/
    RoadRunnerSubsystem_BLUE(
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
            new Pose2d(Tile, 1.4 * Tile, Math.toRadians(225)), RobotSides.FRONT
        );
        centerPixel_SHORT = robotPoseLimitCalculation(
            new Pose2d(Tile/2, Tile, Math.toRadians(270)), RobotSides.FRONT
        );
        rightPixel_SHORT = robotPoseLimitCalculation(
            new Pose2d(2, 1.25 * Tile, Math.toRadians(180)), RobotSides.FRONT
        );
        leftPixel_LONG = robotPoseLimitCalculation(
            new Pose2d(TileInverted - 3 , 1.4 * Tile, Math.toRadians(1)), RobotSides.FRONT
        );
        centerPixel_LONG = robotPoseLimitCalculation(
            new Pose2d(1.4 * TileInverted, Tile, Math.toRadians(1)), RobotSides.FRONT
        );
        rightPixel_LONG = robotPoseLimitCalculation(
            new Pose2d(1.9 * TileInverted - 2, 1.3 * Tile, Math.toRadians(180)),
            RobotSides.FRONT
        );

        /* Backdrops -----------------------------------------------------------------------------*/
        backdropLeft = robotPoseLimitCalculation(
            new Pose2d(2.5 * Tile, 1.75 * Tile, Math.toRadians(180)), RobotSides.REAR
        );
        backdropCenter = robotPoseLimitCalculation(
            new Pose2d(2.5 * Tile, 1.5 * Tile, Math.toRadians(180)), RobotSides.REAR
        );
        backdropRight = robotPoseLimitCalculation(
            new Pose2d(2.5 * Tile, 1.3 * Tile, Math.toRadians(180)), RobotSides.REAR
        );

        /* Stacks --------------------------------------------------------------------------------*/
        stationInner = robotPoseLimitCalculation(
            new Pose2d(2.8 * TileInverted, Tile / 2, Math.toRadians(180)), RobotSides.FRONT
        );
        stationMiddle = robotPoseLimitCalculation(
            new Pose2d(2.8 * TileInverted, Tile, Math.toRadians(180)), RobotSides.FRONT
        );
        stationOuter = robotPoseLimitCalculation(
            new Pose2d(2.8 * TileInverted, 1.5 * Tile, Math.toRadians(180)),
            RobotSides.FRONT
        );

        /* Mid Points ----------------------------------------------------------------------------*/
        stationClose_Inner = new Vector2d(Tile / 2, Tile / 2);
        stationFar_Inner = new Vector2d(TileInverted, Tile / 2);
        stationClose_Outer = new Vector2d(Tile / 2, 2.5 * Tile);
        stationFar_Outer = new Vector2d(TileInverted, 2.5 * Tile);

        /* Parking -------------------------------------------------------------------------------*/
        parkingInner = new Pose2d(2.5 * Tile, Tile / 3, Math.toRadians(180));
        parkingMiddle = new Pose2d(2 * Tile, 1.5 * Tile, Math.toRadians(180));
        parkingOuter = new Pose2d(2.5 * Tile, 2.5 * Tile, Math.toRadians(180));
    }

    @Override
    public void TrajectoryInit(Randomization rand) {
        this.rand = rand;

        test = drive.trajectorySequenceBuilder(new Pose2d())
            .forward(30)
            .back(30);

        /*----------------------------------------------------------------------------------------*/
        // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
        stackStationTangent = new Integer[]{180, 135, 225};
        // 135 for Inner 225 for Mid and Outer
        parkingTangent = new Integer[]{180, 135};

        /*----------------------------------------------------------------------------------------*/
        leftSpike = drive.trajectorySequenceBuilder(HomePose)
            .lineToLinearHeading(leftPixelSpike);
        centerSpike = drive.trajectorySequenceBuilder(HomePose)
            .lineTo(centerPixelSpike.vec());
        rightSpike = drive.trajectorySequenceBuilder(HomePose)
//            .setTangent(Math.toRadians(315))
            .splineToLinearHeading(rightPixelSpike, Math.toRadians(180));

        /*----------------------------------------------------------------------------------------*/
        rightSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
            .lineToLinearHeading(rightPixel_LONG);
        centerSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
            .setTangent(Math.toRadians(225))
            .splineToLinearHeading(centerPixel_LONG, Math.toRadians(0));
        leftSpike_LONG = drive.trajectorySequenceBuilder(HomePose)
            .splineToLinearHeading(leftPixel_LONG, Math.toRadians(0));

        /*----------------------------------------------------------------------------------------*/
        randomizedBackdrop = offsetPoseShifter(randomizedBackdrop, "Backdrop_Rand");

        /* SHORT ---------------------------------------------------------------------------------*/
        spike_randomizedBackdrop = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(randomizedBackdrop, Math.toRadians(0));

        /* LONG ---------------------------------------------------------------------------------*/
        spike_station = drive.trajectorySequenceBuilder(pixel_cycle_PoseTransfer)
            .setTangent(Math.toRadians(270))
            .splineToLinearHeading(
                offsetPoseShifter(stackStation, "Stacks_Rand"), Math.toRadians(180)
            );

        station_long_randomizedBackdrop = drive.trajectorySequenceBuilder(
                offsetPoseShifter(stackStation, "Stacks_Rand"))
            .setReversed(true)
            .setTangent(Math.toRadians(0))
            .splineToConstantHeading(
                offsetPoseShifter(stationFar, "Corridor_Far_Inner_Rand"),
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
                offsetPoseShifter(backdrop_Unload.vec(), "Backdrop_Right_FirstCycle"),
                Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(
                    45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ),
                SampleMecanumDrive.getAccelerationConstraint(AccDefault)
            );

        /* SHORT (due to time) -------------------------------------------------------------------*/
        backdrop_station_2nd_cycle = drive.trajectorySequenceBuilder(
                offsetPoseShifter(backdrop_Unload, "Backdrop_Right_FirstCycle"))
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
            .splineToConstantHeading(
                offsetPoseShifter(backdrop_Unload.vec(), "Backdrop_Right_SecondCycle"),
                Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(
                    45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH
                ),
                SampleMecanumDrive.getAccelerationConstraint(AccDefault)
            );

        /* COMMON --------------------------------------------------------------------------------*/
        // We have made sure that "Backdrop_Right_SecondCycle" is the same as
        // "Backdrop_Right_FirstCycle" for BLUE/RED_LONG
        parking = drive.trajectorySequenceBuilder(
                offsetPoseShifter(backdrop_Unload, "Backdrop_Right_SecondCycle"))
            .setTangent(Math.toRadians(parkingTangent[parkingTangentValue])) //tan 135/225
            .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }

    @Override
    public void setCycle() {
        super.setCycle();

        if (path == Path.INNER)
            backdrop_Unload = backdropRight;
        else // Path.OUTER
            backdrop_Unload = backdropLeft;
    }
}