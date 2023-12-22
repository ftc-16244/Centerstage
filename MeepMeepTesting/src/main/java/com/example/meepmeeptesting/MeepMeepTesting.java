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


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d RedPark = new Pose2d(45, -13,Math.toRadians(0));
        Pose2d RedRallyPoint1_LEFT = new Pose2d(10.5,-50,Math.toRadians(0));//left and a bit forward from the left pixel drop
        Pose2d RedRallyPoint1_CENTER = new Pose2d(-40, -10.5,Math.toRadians(0));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint1_RIGHT = new Pose2d(10.5,-40,Math.toRadians(0));//left and a bit forward from the center pixel drop
        Pose2d RedRallyPoint2 = new Pose2d(0, -10.5,Math.toRadians(0));// near md field on red side


        //Center Prop
        Pose2d AudRedCenter = new Pose2d(-45, -28.5, Math.toRadians(0));
        //backstage drop
        Pose2d AudRedCenterDropoff = new Pose2d(51, -39, Math.toRadians(0));
        Pose2d AudRedCenterDropoff2 = new Pose2d(35, -39, Math.toRadians(180));

        // Left Prop Poses
        Pose2d AudRedLeft = new Pose2d(33.5,-57.5, Math.toRadians(90));
        Pose2d AudRedLeftDropoff = new Pose2d(29,51, Math.toRadians(90));

        // Right Prop Poses
        Pose2d AudRedRightDropoff = new Pose2d(44, 51, Math.toRadians(90));//backstage
        Pose2d AudRedRightPre = new Pose2d(35.5,-41, Math.toRadians(90));
        Pose2d AudRedRight = new Pose2d(35.5,-34,Math.toRadians(90));//spike mark

        Pose2d leftwhitespikemark1 = new Pose2d(0, -33.5, Math.toRadians(180));
        Pose2d leftwhitespikemark2 = new Pose2d(-57.5, -33.5, Math.toRadians(180));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64.5, 0))
                                .splineToLinearHeading(AudRedCenter,Math.toRadians(0))
                                .splineToLinearHeading(RedRallyPoint1_CENTER, Math.toRadians(0))
                                .splineToLinearHeading(RedRallyPoint2, Math.toRadians(0))
                                .splineToLinearHeading(AudRedCenterDropoff,Math.toRadians(0))
                                .lineToLinearHeading(leftwhitespikemark1)
                                .lineToLinearHeading(leftwhitespikemark2)
                                .lineToLinearHeading(AudRedCenterDropoff2)
                                .lineToLinearHeading(AudRedCenterDropoff)
                                .lineToLinearHeading(RedPark)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_LIGHT)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();



    }
}