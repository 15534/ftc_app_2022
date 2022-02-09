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

    static Pose2d startingPosition = new Pose2d(-59, -56, ((Math.PI/2) - Math.atan(7 / 17))); //67.62
    static Pose2d shippingHubPose = new Pose2d(-24,-36.5, ((Math.PI) + Math.atan(11.5/13.5))); //220.43
    static Pose2d allianceFreightPose = new Pose2d(-12,-53, -Math.atan(8 / 16)); // -26 account for drift!!
    static Pose2d scoreAllianceFreight = new Pose2d(-22, -42.5, Math.toRadians(-90));
    static Pose2d teammateItem1 = new Pose2d(-28,-49, Math.toRadians(30));
    static Pose2d checkpt000 = new Pose2d(0 ,-46, Math.toRadians(66));
    static Pose2d checkpt1 = new Pose2d(2,-41, Math.toRadians(55));
    static Vector2d carouselPos = new Vector2d(-60, -56);
    static Vector2d checkpt0 = new Vector2d(-18,-50);
    static Vector2d checkpt00 = new Vector2d(-9,-50);


    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 5.5, 0.8*Math.toRadians(180), 0.6*Math.toRadians(184.02607784577722), 5.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startingPosition)
                                        .lineTo(carouselPos)
                                        .splineToSplineHeading(shippingHubPose, Math.toRadians(0))
                                        .splineToSplineHeading(allianceFreightPose, Math.toRadians(220))
                                        .splineToSplineHeading(scoreAllianceFreight, Math.toRadians(90))
                                        .splineToSplineHeading(teammateItem1, Math.toRadians(-50))
                                        .splineToConstantHeading(checkpt0, Math.toRadians(0))
                                        .splineToConstantHeading(checkpt00, Math.toRadians(0))
                                        .splineToSplineHeading(checkpt000, Math.toRadians(0))
                                        //.splineToSplineHeading(checkpt1, Math.toRadians(0))
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