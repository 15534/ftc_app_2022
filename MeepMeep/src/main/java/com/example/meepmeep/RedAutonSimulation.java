package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.text.ComposedCharIter;

public class RedAutonSimulation {
    static Trajectory goToCarouselFromStarting, goToShippingHubFromCarousel, goToFreightFromShippingHub, goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos;

    static Pose2d startingPosition = new Pose2d(-33, -63, Math.toRadians(0));
    static Pose2d shippingHubPose = new Pose2d(-24,-36.5, ((Math.PI) + Math.atan(11.5/13.5)));
    static Vector2d storageUnitPose = new Vector2d(-48, -36);
    static Vector2d carouselPos = new Vector2d(60, -56);
    static Vector2d wareHousePos = new Vector2d(36, -60);
    static Vector2d switchingPos = new Vector2d(0, -48);

    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 25, Math.toRadians(180), Math.toRadians(184.02607784577722), 5.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startingPosition)
                                .lineTo(carouselPos)
                                .splineToSplineHeading(shippingHubPose, Math.toRadians(0))
//                                .lineTo(wareHousePos)
//                                .turn(Math.toRadians(20))
//                                .lineTo(new Vector2d(wareHousePos.getX() + 12, wareHousePos.getY()))
//                                .lineTo(new Vector2d(switchingPos.getX(), switchingPos.getY()))
//                                .lineTo(new Vector2d(storageUnitPose.getX(), storageUnitPose.getY()))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}