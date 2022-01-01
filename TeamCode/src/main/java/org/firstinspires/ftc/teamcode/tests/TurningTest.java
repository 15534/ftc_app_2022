package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "TurningTest")
public class TurningTest extends LinearOpMode {
    public SampleMecanumDrive mecanumDrive;

    Pose2d startingPosition = new Pose2d(-33, -63, Math.toRadians(0));


    public void initialize(){
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(startingPosition);
    }

    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();
        while (opModeIsActive()) {
            mecanumDrive.turnAsync(Math.toRadians(90));
        }
    }
}
