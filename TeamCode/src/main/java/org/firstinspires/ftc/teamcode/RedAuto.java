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
@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {
    DcMotorEx intakeExtension, carousel, fleft, fright, bleft, bright;

    public SampleMecanumDrive mecanumDrive;
    public SampleTankDrive tankDrive;

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Pose2d startingPosition = new Pose2d(-59, -56, Math.toRadians(90 - Math.atan(7 / 17)));
    Vector2d carouselPos = new Vector2d(60, -56);

    Pose2d shippingHubPose = new Pose2d(-36,-39, (Math.atan(4.5 / 16) - (Math.PI / 2)));
    Vector2d test = new Vector2d(-36, -39);
    Vector2d storageUnitPose = new Vector2d(-48, -36);
    Pose2d wareHousePos = new Pose2d(48, -48, Math.toRadians(0));
    Vector2d switchingPos = new Vector2d(0, -48);

    Trajectory goToCarouselFromStarting, goToShippingHubFromCarousel, goToFreightFromShippingHub, goToSwitchingPosFromFreight, goToStorageUnitFromSwitchingPos;

    Servo fr, br, fl, bl;

    enum State {
        GO_TO_CAROUSEL,
        KNOCK_OFF_DUCK,
        GO_TO_SHIPPING_HUB,
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

    public void initialize() {
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive  = new SampleTankDrive(hardwareMap);

        carousel = hardwareMap.get(DcMotorEx.class,"caro");
        fleft = hardwareMap.get(DcMotorEx.class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");
    }

    public void buildTrajectories() {
        goToCarouselFromStarting = mecanumDrive.trajectoryBuilder(startingPosition)
                .lineTo(carouselPos)
                .build();

        goToShippingHubFromCarousel = mecanumDrive.trajectoryBuilder(goToCarouselFromStarting.end())
                .splineToConstantHeading(test, Math.toRadians(0))
                //.splineToSplineHeading(shippingHubPose, Math.toRadians(0))
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
        fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 & 1
        bl = hardwareMap.get(Servo.class, "backleft");
        // limits: 0.034 & 1
        // tank is whole numbers
        //PoseStorage.currentPose = startingPosition;

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");

        initialize();
        buildTrajectories();
        runtime.reset();

        waitForStart();

        mecanumDrive.turnAsync(Math.toRadians(10));
        next(State.KNOCK_OFF_DUCK);

        while(opModeIsActive()) {
            intakeExtension.setPower(0.2);
            telemetry.addLine("running");
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
               case KNOCK_OFF_DUCK:
                    if (!mecanumDrive.isBusy()) {
                        while (elapsed + 5 < runtime.seconds() - time) {
                            // should spin for 6 seconds
                            carousel.setPower(-0.5);
                            fleft.setPower(-0.1);
                            fright.setPower(-0.1);
                            bleft.setPower(-0.1);
                            bright.setPower(-0.1);
                        }
                        next(State.IDLE);
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    // path to go to shipping hub
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToShippingHubFromCarousel);
                        next(State.IDLE);
                    }
                    break;
//                case TURN_AT_SHIPPING_HUB:
//                    if (!mecanumDrive.isBusy()) {
//                        double dX = Math.abs(wareHousePos.getX() - shippingHubPose.getX());
//                        double dY = Math.abs(wareHousePos.getY() - shippingHubPose.getY());
//                        double heading = Math.atan(dY / -dX);
//                        mecanumDrive.turnAsync(heading);
//                        next(State.SWITCH_TO_TANK);
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
            telemetry.addLine("out of loop");
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
