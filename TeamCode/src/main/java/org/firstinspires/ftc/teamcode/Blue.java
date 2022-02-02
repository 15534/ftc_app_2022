package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Blue")
public class Blue extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        Servo br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        Servo fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 & 1
        Servo bl = hardwareMap.get(Servo.class, "backleft");
        // limits: 0.034 & 1

        // tank is whole numbers
        boolean isMecLeft = true;
        boolean isMecRight = true;

        double scaler = 1.0;
        DcMotor fleft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor fright = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor bleft = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor bright = hardwareMap.get(DcMotor.class, "rear_right");
        DcMotor carosell = hardwareMap.get(DcMotor.class,"caro");
        ModernRoboticsI2cRangeSensor RANGE2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeee");

        br.setDirection(Servo.Direction.FORWARD);

        boolean ismechA=false;
        boolean ismechB=false;
        boolean ismechY=false;
        boolean ismechX=false;
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("a",ismechA);
            telemetry.addData("b",ismechB);
            telemetry.addData("x",ismechX);
            telemetry.addData("y",ismechY);
            telemetry.addData("pos",bl.getPosition());
            telemetry.addData("port",bl.getPortNumber());
            telemetry.update();

            if (gamepad1.a) {
                if(!ismechA) {
                    fl.setPosition(0.98);
                    ismechA=true;
                }else{
                    fl.setPosition(0);
                    ismechA=false;
                }

                while (gamepad1.a) {

                }
            }
            if(gamepad1.right_bumper){
                bl.setPosition(0.034);
                while (gamepad1.right_bumper){

                }
            }
            if(gamepad1.left_bumper){
                bl.setPosition(1);
                while (gamepad1.left_bumper){

                }
            }

            if (gamepad1.b) {
                if(!ismechB) {
                    bl.setPosition(0.034);
                    ismechB = true;
                }else{
                    bl.setPosition(1);
                    ismechB=false;
                }
                while (gamepad1.b) {

                }
            }
            if (gamepad1.y) {
                if(!ismechY) {
                    fr.setPosition(0.0175);
                    ismechY=true;
                }else{
                    fr.setPosition(1);
                    ismechY=false;
                }
                while (gamepad1.y) {

                }
            }
            if (gamepad1.x) {
                if(!ismechX){
                    br.setPosition(0.98);
                    ismechX=true;
                }else{
                    br.setPosition(0);
                    ismechX=false;
                }


                while (gamepad1.x) {

                }
            }
        }
    }

} 