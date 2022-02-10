package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

import sun.text.ComposedCharIter;

public class RedAutonSimulationV2 {
    static Trajectory goToCarouselFromStarting, goToShippingHubFromCarousel, goToFreightFromShippingHub, goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos;

    static Pose2d startingPosition = new Pose2d(-59, -56, ((Math.PI/2) - Math.atan(7 / 17))); //67.62
    static Vector2d shippingHubPose = new Vector2d(-24,-36.5); //220.43
    static Vector2d allianceFreightPose = new Vector2d(-12,-53); // -26 account for drift!!
    static Vector2d scoreAllianceFreight = new Vector2d(-22, -42.5);
    static Vector2d teammateItem1 = new Vector2d(0 ,-46);
    static Pose2d checkpt000 = new Pose2d(0 ,-46);
    //    static Pose2d checkpt1 = new Pose2d(2,-41, Math.toRadians(55));
    static Vector2d carouselPos = new Vector2d(-60, -56);
    static Vector2d checkpt0 = new Vector2d(-18,-50);
    static Vector2d checkpt00 = new Vector2d(-9,-50);


    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 9, 25*1.2, 25*1.2, 5.5)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startingPosition)
                                        .lineTo(carouselPos)
                                        .turn((Math.PI/2)+1.1801) //((Math.PI/2) - Math.atan(7 / 17))
                                        .lineTo(shippingHubPose)
                                        .turn(((Math.PI) + Math.atan(11.5/13.5)))
                                        .lineTo(allianceFreightPose)
                                        .turn(-Math.atan(8.0 / 16.0))
                                        .lineTo(scoreAllianceFreight)
                                        .turn(Math.toRadians(-90))
                                        .lineTo(teammateItem1)
                                        .turn(Math.toRadians(66))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}