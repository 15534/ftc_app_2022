package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

public class TestTankDrive extends LinearOpMode {
    DcMotorEx intakeExtension, carousel, fleft, fright, bleft, bright, outtake;

    @Override
    public void runOpMode() throws InterruptedException {

        fleft = hardwareMap.get(DcMotorEx .class, "front_left");
        fright = hardwareMap.get(DcMotorEx.class, "front_right");
        bleft = hardwareMap.get(DcMotorEx.class, "rear_left");
        bright = hardwareMap.get(DcMotorEx.class, "rear_right");

        fleft.setPower(1.0);
        fright.setPower(1.0);
        bleft.setPower(1.0);
        bright.setPower(1.0);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
