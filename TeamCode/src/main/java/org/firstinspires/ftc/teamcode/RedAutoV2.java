package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

import java.util.Vector;

@Config
@Autonomous(name = "RedAutoV2")
public class RedAutoV2 extends LinearOpMode {
    DcMotorEx intakeExtension, carousel, fleft, fright, bleft, bright, outtake;

    public SampleMecanumDrive mecanumDrive;
    public SampleTankDrive tankDrive;

    double time = 0.0;
    double intakeUp = 0.15;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Pose2d startingPosition = new Pose2d(-59, -56, ((Math.PI/2) - Math.atan(7 / 17)));
    Vector2d carouselPos = new Vector2d(-60, -56);
    //Pose2d shippingHubPose = new Pose2d(-39,-25, Math.toRadians(226));

    Vector2d shippingHubPose = new Vector2d(-24,-36.5); //220.43

    //Pose2d allianceFreightPose = new Pose2d(-12,-53, -Math.atan(8 / 16)); // account for drift!!
    Vector2d allianceFreightPose = new Vector2d(-12,-53); // account for drift!!
    //    Pose2d checkpt1 = new Pose2d(2,-41+20, Math.toRadians(55));
    Vector2d scoreAllianceFreight = new Vector2d(-22, -42.5);
    Pose2d teammateItem1 = new Pose2d(-28,-49, Math.toRadians(30));
    Vector2d checkpt0 = new Vector2d(-18,-50);
    Vector2d checkpt00 = new Vector2d(-9,-50);
    Pose2d checkpt000 = new Pose2d(-3 ,-41, Math.toRadians(65));

    Vector2d storageUnitPose = new Vector2d(-48, -36);
    Pose2d wareHousePos = new Pose2d(48, -48, Math.toRadians(0));
    Vector2d switchingPos = new Vector2d(0, -48);

    Trajectory goToTeammateItemFromShippingHub, goToShippingHubFromAlliance, goToCarouselFromStarting, goToShippingHubFromCarousel, goToAllianceFreightFromShippingHub, goToTeammateItemFromShippingHubPart1, goToTeammateItemFromShippingHubPart2, goToTeammateItemFromShippingHubPart3, goToTeammateItemFromShippingHubPart4, goToFreightFromShippingHub, goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos;

    Servo fr, br, fl, bl, outtakeServo, intakePosition;
    public static int outtakeFirstLevelPosition = -120;
    public static double outtakePower = 0.5;
    public static double outtakeServoClosePosition = 0.9;
    public static double outtakeServoOpenPosition = 0.4;
    public static int outtakeDownPosition = 0;

    enum State {
        GO_TO_CAROUSEL,
        KNOCK_OFF_DUCK,
        GO_TO_SHIPPING_HUB,
        GO_TO_SHIPPING_HUB_TURN,
        GO_TO_ALLIANCE_FREIGHT,
        GO_TO_ALLIANCE_FREIGHT_TURN,
        SCORE_ALLIANCE_FREIGHT,
        SCORE_ALLIANCE_FREIGHT_TURN,
        PICK_UP_TEAMMATE_ITEM_PART_1,
        PICK_UP_TEAMMATE_ITEM_PART_2,
        PICK_UP_TEAMMATE_ITEM_PART_3,
        PICK_UP_TEAMMATE_ITEM_PART_4,
        PICK_UP_TEAMMATE_ITEM,
        TRANSITION_SHIPPING_HUB,
        SCORE_FREIGHT_IN_SHIPPING_HUB,
        TURN_AT_SHIPPING_HUB,
        SWITCH_TO_TANK,
        DRIVE_TO_FREIGHT,
        PICK_UP_FREIGHT,
        TURN_AT_FREIGHT,
        GO_TO_SWITCHING_POS,
        GO_TO_STORAGE_UNIT,
        OUTTAKE_FREIGHT,
        GO_TO_SWITCHING_POS_2,
        DRIVE_TO_FREIGHT_2,
        PARK,
        IDLE,
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    int turnsLeft = 3;

    public void buildTrajectories() {
        goToCarouselFromStarting = mecanumDrive.trajectoryBuilder(startingPosition)
                .lineTo(carouselPos)
                .build();

        goToShippingHubFromCarousel = mecanumDrive.trajectoryBuilder(goToCarouselFromStarting.end())
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                .lineTo(shippingHubPose)
                .build();

        goToAllianceFreightFromShippingHub = mecanumDrive.trajectoryBuilder(goToShippingHubFromCarousel.end())
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                .lineTo(allianceFreightPose)
                .build();

        goToShippingHubFromAlliance = mecanumDrive.trajectoryBuilder(goToAllianceFreightFromShippingHub.end(), Math.toRadians(90))
                .lineTo(scoreAllianceFreight)
                .build();

        goToTeammateItemFromShippingHubPart1 = mecanumDrive.trajectoryBuilder(goToShippingHubFromAlliance.end())
                .splineToLinearHeading(teammateItem1, Math.toRadians(-30))
                .build();

        goToTeammateItemFromShippingHubPart2 = mecanumDrive.trajectoryBuilder(goToTeammateItemFromShippingHubPart1.end())
                .splineToConstantHeading(checkpt0, Math.toRadians(0))
                .build();

        goToTeammateItemFromShippingHubPart3 = mecanumDrive.trajectoryBuilder(goToTeammateItemFromShippingHubPart2.end())
                .splineToConstantHeading(checkpt00, Math.toRadians(0))
                .build();

        goToTeammateItemFromShippingHubPart4 = mecanumDrive.trajectoryBuilder(goToTeammateItemFromShippingHubPart3.end())
                .splineToSplineHeading(checkpt000, Math.toRadians(30))
                .build();

//        goToFreightFromShippingHub = tankDrive.trajectoryBuilder(goToShippingHubFromCarousel.end())
//                .splineToSplineHeading(wareHousePos, Math.toRadians(0))
//                .build();
//
//        goToSwitchingPosFromFreight = tankDrive.trajectoryBuilder(goToFreightFromShippingHub.end())
//                .lineTo(new Vector2d(switchingPos.getX(), switchingPos.getY()))
//                .build();
//
//        goToStorageUnitFromSwitchingPos = mecanumDrive.trajectoryBuilder(goToSwitchingPosFromFreight.end())
//                .lineTo(new Vector2d(storageUnitPose.getX(), storageUnitPose.getY()))
//                .build();
    }

    public void switchFromMecToTank() {
        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);
    }

    public void switchFromTankToMec() {
        fl.setPosition(0.98);
        bl.setPosition(0.034);
        fr.setPosition(0.0175);
        br.setPosition(0.98);
    }

    public void runOpMode() throws InterruptedException {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive  = new SampleTankDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotorEx.class,"caro");
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");
        intakePosition = hardwareMap.get(Servo.class, "intakeLift");
        fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 &
        bl = hardwareMap.get(Servo.class, "backleft");
        // limits: 0.034 & 1
        // tank is whole numbers
        //PoseStorage.currentPose = startingPosition;
        fl.setPosition(0.984);
        fr.setPosition(0.1);
        br.setPosition(0.955);
        bl.setPosition(0.02);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        buildTrajectories();

        runtime.reset();
        intakePosition.setPosition(intakeUp);
        waitForStart();
        mecanumDrive.setPoseEstimate(startingPosition);
        mecanumDrive.turnAsync(Math.toRadians(10));
        next(State.KNOCK_OFF_DUCK);

        while(opModeIsActive()) {
            intakeExtension.setPower(0.2);
            telemetry.addLine("running");
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
                case KNOCK_OFF_DUCK:
                    if (elapsed < 4) {
                        // should spin for 4 seconds
                        carousel.setPower(-0.6);
                        fleft.setPower(-0.1);
                        fright.setPower(-0.1);
                        bleft.setPower(-0.1);
                        bright.setPower(-0.1);
                    } else {
                        carousel.setPower(0.0);
                        fleft.setPower(0.0);
                        fright.setPower(0.0);
                        bleft.setPower(0.0);
                        bright.setPower(0.0);
                    }
                    if (!mecanumDrive.isBusy() && elapsed >= 4) {
                        next(State.GO_TO_SHIPPING_HUB);
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    // path to go to shipping hub
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToShippingHubFromCarousel);
                        next(State.GO_TO_SHIPPING_HUB_TURN);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_TURN:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.turnAsync(((Math.PI) + Math.atan(11.5/13.5)));
                        next(State.GO_TO_ALLIANCE_FREIGHT);
                    }
                    break;
//                case TURN_CORRECT_POS:
//                    if (!mecanumDrive.isBusy()) {
//
//                    }
//                    break;
//                case TRANSITION_SHIPPING_HUB:
//                    if (!mecanumDrive.isBusy()) {
//                        next(State.SCORE_FREIGHT_IN_SHIPPING_HUB);
//                    }
//                    break;
//                case SCORE_FREIGHT_IN_SHIPPING_HUB:
//                    outtakeServo.setPosition(outtakeServoOpenPosition);
//                    telemetry.addData("outtake servo position: ", outtakeServo.getPosition());
//
//                    outtake.setTargetPosition(outtakeFirstLevelPosition);
//                    outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    outtake.setPower(outtakePower);
//                    telemetry.addData("outtake position: ", outtake.getCurrentPosition());
//
//                    outtakeServo.setPosition(outtakeServoClosePosition);
//                    outtake.setTargetPosition(outtakeDownPosition);
//                    next(State.SCORE_FREIGHT_IN_SHIPPING_HUB);
                case GO_TO_ALLIANCE_FREIGHT:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToAllianceFreightFromShippingHub);
                        next(State.GO_TO_ALLIANCE_FREIGHT_TURN);
                    }
                    break;
                case GO_TO_ALLIANCE_FREIGHT_TURN:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.turnAsync(-Math.atan(8.0/16.0));
                        next(State.SCORE_ALLIANCE_FREIGHT);
                    }
                    break;
                case SCORE_ALLIANCE_FREIGHT:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToShippingHubFromAlliance);
                        next(State.SCORE_ALLIANCE_FREIGHT_TURN);
                    }
                    break;
                case SCORE_ALLIANCE_FREIGHT_TURN:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.turnAsync(Math.toRadians(-90));
                        next(State.IDLE);
                    }
                    break;
//                case PICK_UP_TEAMMATE_ITEM_PART_1:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.followTrajectoryAsync(goToTeammateItemFromShippingHubPart1);
//                        next(State.PICK_UP_TEAMMATE_ITEM_PART_2);
//                    }
//                    break;
//                case PICK_UP_TEAMMATE_ITEM_PART_2:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.followTrajectoryAsync(goToTeammateItemFromShippingHubPart2);
//                        next(State.PICK_UP_TEAMMATE_ITEM_PART_3);
//                    }
//                    break;
                case PICK_UP_TEAMMATE_ITEM_PART_3:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToTeammateItemFromShippingHubPart3);
                        next(State.PICK_UP_TEAMMATE_ITEM_PART_4);
                    }
                    break;
                case PICK_UP_TEAMMATE_ITEM_PART_4:
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToTeammateItemFromShippingHubPart4);
                        next(State.IDLE);
                    }
                    break;
//                case TURN_AT_ALLIANCE_FREIGHT:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.turnAsync();
//                    }
                //                case TURN_AT_SHIPPING_HUB:
//                    if (!mecanumDrive.isBusy()) {
////                        double dX = Math.abs(wareHousePos.getX() - shippingHubPose.getX());
////                        double dY = Math.abs(wareHousePos.getY() - shippingHubPose.getY());
////                        double heading = Math.atan(dY / -dX);
//                        mecanumDrive.turnAsync(shippingHubPose.getHeading());
//                        next(State.IDLE);
//                    }
//                    break;
//                case SWITCH_TO_TANK:
//                    if (!mecanumDrive.isBusy()) {
//                       switchFromMecToTank();
//                       next(State.DRIVE_TO_FREIGHT);
//                    }
//                    break;
//                case DRIVE_TO_FREIGHT:
//                    if (!tankDrive.isBusy()) {
//                        tankDrive.followTrajectoryAsync(goToFreightFromShippingHub);
//                        next(State.PICK_UP_FREIGHT);
//                    }
//                    break;
//                case PICK_UP_FREIGHT:
//                    if (!tankDrive.isBusy()) {
//                        //TODO has code to pick up the freight
//                        next(State.TURN_AT_FREIGHT);
//                    }
//                    break;
//                case TURN_AT_FREIGHT:
//                    if (!tankDrive.isBusy()) {
//                        double dX = Math.abs(storageUnitPose.getX() - wareHousePos.getX());
//                        double dY = Math.abs(storageUnitPose.getY() - wareHousePos.getY());
//                        double heading = Math.atan(dY / -dX);
//                        mecanumDrive.turnAsync(heading);
//                        next(State.GO_TO_SWITCHING_POS);
//                    }
//                    break;
//                case GO_TO_SWITCHING_POS:
//                    if (!tankDrive.isBusy()) {
//                        tankDrive.followTrajectoryAsync(goToSwitchingPosFromFreight);
//                        switchFromTankToMec();
//                        next(State.GO_TO_STORAGE_UNIT);
//                    }
//                    break;
//                case GO_TO_STORAGE_UNIT:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.followTrajectoryAsync(goToStorageUnitFromSwitchingPos);
//                        next(State.OUTTAKE_FREIGHT);
//                    }
//                    break;
//                case OUTTAKE_FREIGHT:
//                    if (!mecanumDrive.isBusy()) {
//                       //TODO Outtake the freight
//                        next(State.GO_TO_SWITCHING_POS_2);
//                    }
//                    break;
//                case GO_TO_SWITCHING_POS_2:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.followTrajectoryAsync(goToSwitchingPosFromFreight);
//                        switchFromMecToTank();
//                        next(State.DRIVE_TO_FREIGHT_2);
//                    }
//                    break;
//                case DRIVE_TO_FREIGHT_2:
//                    if (!mecanumDrive.isBusy()) {
//                        mecanumDrive.followTrajectoryAsync(goToFreightFromShippingHub);
//                        switchFromMecToTank();
//                        turnsLeft--;
//                        if (turnsLeft > 0) {
//                            next(State.GO_TO_SWITCHING_POS);
//                        }
//                        else {
//                            next(State.IDLE);
//                        }
//                    }
//                    break;
            }
            // Read pose
            Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;
            mecanumDrive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            //telemetry.addData("elapsed", elapsed);
//            telemetry.addData("push", push);
            telemetry.update();
        }
    }
}
