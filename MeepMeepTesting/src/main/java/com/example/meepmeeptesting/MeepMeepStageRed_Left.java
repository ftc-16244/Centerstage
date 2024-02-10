package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;


public class MeepMeepStageRed_Left {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d YellowPixelDropOff_LEFT = new Pose2d(45,-29,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1a = new Pose2d(10,-35,Math.toRadians(180));
        Pose2d PurplePixelDropOff_LEFT_1b = new Pose2d(4,-35,Math.toRadians(180));
        Pose2d WhiteTravelPart1a_LEFT = new Pose2d(-0,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1b_LEFT = new Pose2d(-20,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1c_LEFT = new Pose2d(-42,-60, Math.toRadians(180));
        Pose2d WhiteTravelPart1d_LEFT = new Pose2d(-40,-50, Math.toRadians(180));
        Pose2d WhiteTravelPart1e_LEFT = new Pose2d(-56,-36, Math.toRadians(180));
        Pose2d WhiteTravelPart2c_LEFT = new Pose2d (48,-56, Math.toRadians(180));
        Pose2d WhiteTravelPart2b_LEFT = new Pose2d(-14,-56, Math.toRadians(180));
        Pose2d WhiteTravelPart2a_LEFT = new Pose2d(-40,-52, Math.toRadians(180));




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10, -56, Math.toRadians(180)))
                                .lineToLinearHeading(PurplePixelDropOff_LEFT_1a)
                                .lineToLinearHeading(PurplePixelDropOff_LEFT_1b)
                                .lineToLinearHeading(YellowPixelDropOff_LEFT)
                                .strafeLeft(8)
                                .splineToLinearHeading(WhiteTravelPart1a_LEFT, Math.toRadians(180))
                                .splineToLinearHeading(WhiteTravelPart1b_LEFT,Math.toRadians(180))
                                .splineToLinearHeading(WhiteTravelPart1c_LEFT,Math.toRadians(180))
                                .splineToLinearHeading(WhiteTravelPart1e_LEFT, Math.toRadians(180))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }
}