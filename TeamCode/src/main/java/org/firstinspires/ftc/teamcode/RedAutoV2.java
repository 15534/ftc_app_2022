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

    Trajectory goToShippingHubFromCarousel2, goToTeammateItemFromShippingHub, goToShippingHubFromAlliance, goToCarouselFromStarting, goToShippingHubFromCarousel, goToAllianceFreightFromShippingHub, goToTeammateItemFromShippingHubPart1, goToTeammateItemFromShippingHubPart2, goToTeammateItemFromShippingHubPart3, goToTeammateItemFromShippingHubPart4, goToFreightFromShippingHub, goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos;

    Servo fr, br, fl, bl, outtakeServo, intakePosition;
    public static int outtakeFirstLevelPosition = -120;
    public static int outtakeThirdLevelPosition = -360;

    public static double outtakePower = 0.5;
    public static double outtakeServoClosePosition = 0.9;
    public static double outtakeServoOpenPosition = 0.4;
    public static int outtakeDownPosition = 0;

    public static boolean isMec = false;

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
        IDLE, GO_TO_SHIPPING_HUB_2, GO_TO_SHIPPING_HUB_TURN_2, TRANSITION_SCORE_FREIGHT_IN_SHIPPING_HUB,
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
                .forward(27.586228)
                .build();

        goToShippingHubFromCarousel2 = mecanumDrive.trajectoryBuilder(goToShippingHubFromCarousel.end())
                //.lineToSplineHeading(shippingHubPose)
                //.lineTo(new Vector2d(shippingHubPose.getX(),shippingHubPose.getY()))
                //.lineTo(new Vector2d(shippingHubPose.getX(), shippingHubPose.getY()))
                .back(21.5)
                .build();
    }

    public void switchFromMecToTank() {
        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);
    }

    public void switchFromTankToMec() {
        fl.setPosition(0.984);
        fr.setPosition(0.1);
        br.setPosition(0.955);
        bl.setPosition(0.02);
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
//        fl.setPosition(0.984);
//        fr.setPosition(0.1);
//        br.setPosition(0.955);
//        bl.setPosition(0.02);

        fl.setPosition(0);
        bl.setPosition(1);
        fr.setPosition(1);
        br.setPosition(0);

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
                    if (elapsed < 3.5) {
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
                    if (!mecanumDrive.isBusy() && elapsed >= 3.5) {
                        next(State.GO_TO_SHIPPING_HUB);
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    // path to go to shipping hub
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(goToShippingHubFromCarousel);
                        next(State.GO_TO_SHIPPING_HUB_TURN);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_TURN:
                    if (!tankDrive.isBusy()) {
                        switchFromTankToMec();
                        isMec = true;
                        mecanumDrive.turn(Math.toRadians(248 - 67));
                        switchFromMecToTank();
                        isMec = false;
                        next(State.GO_TO_SHIPPING_HUB_2);
                    }
                    break;
                case GO_TO_SHIPPING_HUB_2:
                    if (!tankDrive.isBusy()) {
                        tankDrive.followTrajectoryAsync(goToShippingHubFromCarousel2);
                        next(State.TRANSITION_SCORE_FREIGHT_IN_SHIPPING_HUB);
                    }
                    break;
                case TRANSITION_SCORE_FREIGHT_IN_SHIPPING_HUB:
                    if (!tankDrive.isBusy()) {
                        next(State.SCORE_FREIGHT_IN_SHIPPING_HUB);
                    }
                    break;
                case SCORE_FREIGHT_IN_SHIPPING_HUB:
                    if (!tankDrive.isBusy()) {
                        outtake.setTargetPosition(outtakeThirdLevelPosition);
                        outtake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        outtake.setPower(outtakePower);
                        telemetry.addData("outtake position: ", outtake.getCurrentPosition());

                        if (elapsed > 2) {
                            outtakeServo.setPosition(outtakeServoOpenPosition);
                            telemetry.addData("outtake servo position: ", outtakeServo.getPosition());
                        }

                        if (elapsed > 3) {
                            outtakeServo.setPosition(outtakeServoClosePosition);
                            outtake.setTargetPosition(outtakeDownPosition);
                        }
                        next(State.IDLE);
                    }
                    break;
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
            }
            // Read pose

            Pose2d poseEstimate;
            if (isMec) {
                poseEstimate = mecanumDrive.getPoseEstimate();
            } else {
                poseEstimate = tankDrive.getPoseEstimate();
            }

            PoseStorage.currentPose = poseEstimate;
            tankDrive.update();
            mecanumDrive.update();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("state", currentState);
            telemetry.addData("MODE", isMec);
            //telemetry.addData("elapsed", elapsed);
//            telemetry.addData("push", push);
            telemetry.update();
        }
    }
}
