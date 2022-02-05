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

    public static boolean isMec = false;

    DcMotorEx outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        //Servo s = haardwareMap.get(Servo.class, "stick");
        Servo fl = hardwareMap.get(Servo.class, "frontleft"); // lower: 0.984, upper: 0
        Servo fr = hardwareMap.get(Servo.class, "frontright"); // lower: .1, upper: 1
        Servo br = hardwareMap.get(Servo.class, "backright"); // lower: 0.954, upper: 0
        Servo bl = hardwareMap.get(Servo.class, "backleft"); // lower: 0.02, upper: 1

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

//            if (isMec) {
//                fl.setPosition(0.984);
//                fr.setPosition(.1);
//                br.setPosition(0.955);
//                bl.setPosition(0.02);
//            }
//            if (!isMec) {
//                fl.setPosition(0);
//                fr.setPosition(1);
//                br.setPosition(0);
//                bl.setPosition(1);
//            }
            //br.setPosition(pos);
            //s.setPosition(pos); // max servo position is 0.9, when we want to lift servo up make servo position 0.8
            //telemetry.addData("position",s.getPosition());
            telemetry.addData("outtake pos: ", outtake.getCurrentPosition());
            telemetry.update();

            // mec - tank
            // intake moves from mecdown to tank down before switching

            // tank
        }
    }
}