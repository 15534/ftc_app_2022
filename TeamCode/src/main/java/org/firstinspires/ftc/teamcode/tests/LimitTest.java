package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Limit Test")
public class LimitTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo s = hardwareMap.get(Servo.class, "stick");
        Servo s = hardwareMap.get(Servo.class, "stopper");

        Servo fl = hardwareMap.get(Servo.class, "frontleft");

        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "left");
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            //s.setPosition(0.76);
            //fl.setPosition(0.0);

            //motor2.setVelocity(1000);
            motor.setPower(0.1);

            if (motor.getCurrentPosition() == -330) {
                motor.setPower(0.0);
            }

            telemetry.addData("position",motor.getCurrentPosition());
            telemetry.addData("position",s.getPosition());
            telemetry.update();
        }
    }
}
