package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepStageRed_Center {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d YellowPixelDropOff_CENTER = new Pose2d(45,-37,Math.toRadians(180));
        Pose2d PurplePixelDropOff_CENTER = new Pose2d(22,-25,Math.toRadians(180));
        Pose2d SplinePt1 = new Pose2d(10,-56,Math.toRadians(180));
        Pose2d SplinePt2 = new Pose2d(10,-56,Math.toRadians(180));
        //Pose2d WhiteTravelPart1_CENTER = new Pose2d(14,-36,Math.toRadians(180));
        Pose2d WhiteTravelPart1a_CENTER = new Pose2d(-54,-36, Math.toRadians(180));
        Pose2d WhiteTravelPart2a_CENTER = new Pose2d(10,-35, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_Center = new Pose2d (48,-56, Math.toRadians(180));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -56, Math.toRadians(180)))
                                .lineToLinearHeading(PurplePixelDropOff_CENTER)
                                .lineToLinearHeading(YellowPixelDropOff_CENTER)
                                .lineToLinearHeading(WhiteTravelPart1a_CENTER)
                                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                                .lineToLinearHeading(WhiteTravelPart2b_Center)
                                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                                .lineToLinearHeading(WhiteTravelPart1a_CENTER)
                                .lineToLinearHeading(WhiteTravelPart2a_CENTER)
                                .lineToLinearHeading(WhiteTravelPart2b_Center)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }
}