package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;


@Autonomous(name = "RedAuto")
public class RedAuto extends LinearOpMode {

    public SampleMecanumDrive mecanumDrive;
    public SampleTankDrive tankDrive;

    double time = 0.0;
    ElapsedTime runtime = new ElapsedTime();
    State currentState = State.IDLE;
    LinearOpMode op;

    Pose2d startingPosition = new Pose2d(-33, -63, Math.toRadians(0));
    Pose2d shippingHubPose = new Pose2d(5, -30, Math.toRadians(0));
    Vector2d carouselPos = new Vector2d(-55, -63);
    Vector2d wareHousePos = new Vector2d(48, -48);

    Trajectory goToCarouselFromStarting, goToShippingHubFromCarousel;

    enum State {
        GO_TO_CAROUSEL,
        KNOCK_OFF_DUCK,
        GO_TO_SHIPPING_HUB,
        TURN_AT_SHIPPING_HUB,
        IDLE,
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    public void initialize() {
        telemetry.addLine("1");

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
        tankDrive  = new SampleTankDrive(hardwareMap);
    }

    public void buildTrajectories() {
        telemetry.addLine("3");

        goToCarouselFromStarting = mecanumDrive.trajectoryBuilder(startingPosition)
                .lineTo(new Vector2d(carouselPos.getX(), carouselPos.getY()))
                .build();

        goToShippingHubFromCarousel = mecanumDrive.trajectoryBuilder(goToCarouselFromStarting.end())
                .splineToSplineHeading(shippingHubPose, Math.toRadians(0))
                .build();
    }

    public void runOpMode() throws InterruptedException {
        //PoseStorage.currentPose = startingPosition;

        initialize();
        buildTrajectories();

        runtime.reset();
        waitForStart();

        mecanumDrive.followTrajectoryAsync(goToCarouselFromStarting); // path to go to carousel
        next(State.KNOCK_OFF_DUCK);

        while(opModeIsActive()) {
            telemetry.addLine("running");
            double elapsed = runtime.seconds() - time;
            switch (currentState) {
//                case GO_TO_CAROUSEL:
//                    if (!mecanumDrive.isBusy()) {
//                        next(State.KNOCK_OFF_DUCK);
//                    }
//                    break;
                case KNOCK_OFF_DUCK:
                    if (!mecanumDrive.isBusy()) {
                        next(State.GO_TO_SHIPPING_HUB);
                    }
                    break;
                case GO_TO_SHIPPING_HUB:
                    // path to go to shipping hub
                    if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectoryAsync(goToShippingHubFromCarousel);
                        next(State.TURN_AT_SHIPPING_HUB);
                    }
                    break;
                case TURN_AT_SHIPPING_HUB:
                    if (!mecanumDrive.isBusy()) {
                        double dX = Math.abs(wareHousePos.getX() - shippingHubPose.getX());
                        double dY = Math.abs(wareHousePos.getY() - shippingHubPose.getY());
                        double heading = Math.atan(dY / -dX);
                        mecanumDrive.turnAsync(heading);
                        next(State.IDLE);
                    }
                    break;
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
