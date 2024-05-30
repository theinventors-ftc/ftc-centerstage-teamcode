package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import jdk.internal.net.http.common.Pair;

public class MeepMeepTesting {

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

//    public static Pose2d offsetPoseShifter(Pose2d pose, Pair<Double, Double> xy_offset, Offsets preference) {
//        Double X = pose.getX() + xy_offset.first;
//        Double Y = pose.getY() + xy_offset.second;
//
//        if (preference == Offsets.X){
//            Y = pose.getY();
//        }
//        else if(preference == Offsets.Y){
//            X = pose.getX();
//        }
//
//        Pose2d finalPose2d = new Pose2d(X,Y,pose.getHeading());
//
//        return finalPose2d;
//    }

    public static Pose2d robotPoseLimitCalculation(Pose2d pose, RobotSides side){
        Double X = pose.getX();
        Double Y = pose.getY();

        if(side == RobotSides.CENTER) return pose;

        if(pose.getHeading() == 90){
            if(side == RobotSides.LEFT) X = X + (RobotX/2);
            else if(side == RobotSides.RIGHT) X = X - (RobotX/2);
            else if(side == RobotSides.FRONT) Y = Y - (RobotY/2);
            else if(side == RobotSides.REAR) Y = Y + (RobotY/2);
        }
        else if(pose.getHeading() == 270){
            if(side == RobotSides.LEFT) X = X - (RobotX/2);
            else if(side == RobotSides.RIGHT) X = X + (RobotX/2);
            else if(side == RobotSides.FRONT) Y = Y + (RobotY/2);
            else if(side == RobotSides.REAR) Y = Y - (RobotY/2);
        }
        else if(pose.getHeading() == 0){
            if(side == RobotSides.LEFT) Y = Y - (RobotX/2);
            else if(side == RobotSides.RIGHT) Y = Y + (RobotX/2);
            else if(side == RobotSides.FRONT) X = X - (RobotY/2);
            else if(side == RobotSides.REAR) X = X + (RobotY/2);
        }
        else if(pose.getHeading() == 180){
            if(side == RobotSides.LEFT) Y = Y + (RobotX/2);
            else if(side == RobotSides.RIGHT) Y = Y - (RobotX/2);
            else if(side == RobotSides.FRONT) X = X + (RobotY/2);
            else if(side == RobotSides.REAR) X = X - (RobotY/2);
        }

        Pose2d finalPose2d = new Pose2d(X, Y, pose.getHeading());

        return finalPose2d;
    }

    public static double Tile = 24; /*-inches-*/
    public static double TileInverted = -24; /*-inches-*/
    public static double RobotX = 12.795; /*-inches-*/
    public static double RobotY = 18.11; /*-inches-*/

//    public static Dictionary<String, Pair<Double, Double>> RandomizationOffset_XY = new Hashtable<String, Pair<Double, Double>>() {{
////        put("Left", new Pair<>(2.5, 6.5));
////        put("Center", new Pair<>(2.5, 6.5));
////        put("Right", new Pair<>(2.5, 6.5));
//        put("Left", new Pair<>(0.0, 0.0));
//        put("Center", new Pair<>(0.0, 0.0));
//        put("Right", new Pair<>(0.0, 0.0));
//        put("Final", new Pair<>(0.0,0.0));
//    }};

    protected static Integer[] leftSpikeStartingTanget = {45, 135}; //For short 45 and long 135 difference
    protected static Integer[] leftSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected static Integer[] stackStationTanget = {180, 225, 135}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected static Integer[] parkingTanget = {135, 225}; // 135 for Inner 225 for Mid and Outer

    protected static Integer leftSpikeStartingTangetValue = 0;
    protected static Integer leftSpikeFinalTangetValue = 0;
    protected static Integer stackStationTangetValue = 0;
    protected static Integer parkingTangetValue = 1;

    /*------------------------Spikes------------------------*/

    protected static Pose2d leftPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            (RobotY/2), TileInverted, Math.toRadians(180)///////
    ), RobotSides.RIGHT);

    protected static Pose2d centerPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile/2, TileInverted, Math.toRadians(90)///////
    ), RobotSides.FRONT);

    protected static Pose2d rightPixel_SHORT = robotPoseLimitCalculation(new Pose2d(
            Tile, 1.5 * TileInverted, Math.toRadians(90)///////
    ), RobotSides.CENTER);

    protected static Pose2d leftPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            2 * TileInverted,1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.CENTER);

    protected static Pose2d centerPixel_LONG = robotPoseLimitCalculation(new Pose2d(
            1.5 * TileInverted, TileInverted, Math.toRadians(90)///////
    ), RobotSides.FRONT);

    protected static Pose2d rightPixel_LONG = new Pose2d(
            TileInverted - (RobotY/2), TileInverted - (RobotX/2), Math.toRadians(90)///////
    );

    /*------------------------Randomized Backdrop------------------------*/

    protected static Pose2d randomizationBackdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.2 * Tile, 1.2 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected static Pose2d randomizationBackdropCenter = robotPoseLimitCalculation( new Pose2d(
            2.2 * Tile, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected static Pose2d randomizationBackdropRight =  robotPoseLimitCalculation(new Pose2d(
            2.2 * Tile, 1.8 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    /*------------------------Backdrops------------------------*/

    protected static Pose2d backdropLeft = robotPoseLimitCalculation(new Pose2d(
            2.2 * Tile ,1.4 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected static Pose2d backdropCenter = robotPoseLimitCalculation(new Pose2d(
            2.2 * Tile , 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    protected static Pose2d backdropRight = robotPoseLimitCalculation(new Pose2d(
            2.2 * Tile , 1.75 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.REAR);

    /*------------------------Stacks Second Cycle------------------------*/

    protected static Pose2d stationInnerSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted,TileInverted/2, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected static Pose2d stationMiddleSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted,TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected static Pose2d stationOuterSecondCycle = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    /*------------------------Stacks First Cycle------------------------*/

    protected static Pose2d stationInner = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted,TileInverted/2, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected static Pose2d stationMiddle = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted, TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    protected static Pose2d stationOuter = robotPoseLimitCalculation(new Pose2d(
            2.85 * TileInverted, 1.5 * TileInverted, Math.toRadians(180)///////
    ), RobotSides.FRONT);

    /*------------------------Parking------------------------*/

    protected static Pose2d parkingInner = new Pose2d(
            2.5 * Tile, TileInverted/2, Math.toRadians(180));///////

    protected static Pose2d parkingMiddle = new Pose2d(
            2 * Tile, 1.5 * TileInverted, Math.toRadians(180));///////

    protected static Pose2d parkingOuter = new Pose2d(
            2.5 * Tile, 2.65 * TileInverted, Math.toRadians(180));///////

    /*------------------------Mid Points------------------------*/

    protected static Vector2d stationClose_Inner = new Vector2d(
            Tile/2, TileInverted/2);///////

    protected static Vector2d stationFar_Inner = new Vector2d(
            TileInverted,TileInverted/2);///////

    protected static Vector2d stationClose_Outer = new Vector2d(
            Tile, 2.5 * TileInverted);///////

    protected static Vector2d stationFar_Outer = new Vector2d(
            1.5 * TileInverted,2.5 * TileInverted);///////

    /*------------------------------------------------*/

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24/2, 3*-24 + 6.93, Math.toRadians(90)))
                                .setTangent(Math.toRadians(leftSpikeStartingTanget[leftSpikeStartingTangetValue])) //tan pair 45/135
                                .splineToLinearHeading(leftPixel_SHORT, Math.toRadians(leftSpikeFinalTanget[leftSpikeFinalTangetValue]))

                                .setTangent(Math.toRadians(270))
                                .splineToLinearHeading(randomizationBackdropLeft, Math.toRadians(0))

                                .setTangent(Math.toRadians(180))
                                .splineToConstantHeading(stationClose_Inner, Math.toRadians(180))
                                .lineTo(stationFar_Inner)
                                .splineToConstantHeading(stationInner.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue]))//tan pair 180/225

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}