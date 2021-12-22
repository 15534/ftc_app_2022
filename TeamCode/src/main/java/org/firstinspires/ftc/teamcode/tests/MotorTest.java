package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fleft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor fright = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor bleft = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor bright = hardwareMap.get(DcMotor.class, "rear_right");

        waitForStart();

        while (opModeIsActive()){
            fleft.setPower(1);
            fright.setPower(1);
            bleft.setPower(1);
            bright.setPower(1);
        }
    }
}