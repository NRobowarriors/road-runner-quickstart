package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.ui.MainCanvas;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 75, Math.toRadians(180), Math.toRadians(180), 14)
                .setDimensions(15,15)
                .setStartPose(new Pose2d(8,-65, Math.toRadians(180)))
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(9, -35, Math.toRadians(-90)))
                .splineToSplineHeading(new Pose2d(25,-45, Math.toRadians(90)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44, -14),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(44,-53),Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(52,-14),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(55, -53), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(60,-14),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(66, -55), Math.toRadians(-180))

                //.splineTo(new Vector2d(-23, 16), Math.toRadians(90))
                //.splineTo(new Vector2d(-40,26),Math.toRadians(180))
                //.splineToConstantHeading(new Vector2d(-50, 38), Math.toRadians(180))
                //.splineToConstantHeading(new Vector2d(-16,38),Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}