package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Outtake Limit Test")
@Config
public class OuttakeLimitTest extends LinearOpMode {

    public static double pos = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        //Servo s = hardwareMap.get(Servo.class, "stick");
        Servo s = hardwareMap.get(Servo.class, "outtake servo");
        waitForStart();

        while (opModeIsActive()) {
            s.setPosition(pos); // max servo position is 0.9, when we want to lift servo up make servo position 0.8
            telemetry.addData("position",s.getPosition());
            telemetry.update();
        }
    }
}