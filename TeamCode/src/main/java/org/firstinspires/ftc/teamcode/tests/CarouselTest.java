package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class CarouselTest extends LinearOpMode {
    DcMotorEx carousel;

    public void runOpMode() throws InterruptedException {
        carousel = hardwareMap.get(DcMotorEx.class,"caro");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("pos: " , carousel.getCurrentPosition());
            telemetry.update();
        }
    }
}
