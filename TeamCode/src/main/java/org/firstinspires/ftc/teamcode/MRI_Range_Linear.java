package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Range Linear", group = "MRI")
public class MRI_Range_Linear extends LinearOpMode {

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
        fl.setPosition(0.98);
        bl.setPosition(0.034);
        fr.setPosition(0.0175);
        br.setPosition(0.98);
        double scaler =1.0;
        DcMotor fleft = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor fright = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor bleft = hardwareMap.get(DcMotor.class, "rear_left");
        DcMotor bright = hardwareMap.get(DcMotor.class, "rear_right");
        DcMotor carosell = hardwareMap.get(DcMotor.class,"caro");


        waitForStart();

        while (opModeIsActive()) {
            double meow = 0;
            if(gamepad2.b){
                meow++;
            }
            if(gamepad2.a){
                meow--;
            }
            carosell.setPower(0.6*meow);
            /*
            if(gamepad1.left_trigger>0.6){
                if(isMecLeft){

                    isMecLeft=false;
                }else{
                    fl.setPosition(0.98);
                    bl.setPosition(0.034);
                    isMecLeft=true;
                }
                while (gamepad1.left_trigger>0.6){
                    double turn =gamepad1.right_stick_x;
                    double x = gamepad1.left_stick_x;
                    double y = gamepad1.left_stick_y;
                    double[] turns = turn(turn,x,y);
                    fleft.setPower(turns[0]*scaler);
                    fright.setPower(turns[1]*scaler);
                    bleft.setPower(turns[2]*scaler);
                    bright.setPower(turns[3]*scaler);
                }
            }

             */

            if(gamepad1.right_trigger>0.6){
                if(isMecRight){
                    fr.setPosition(1);
                    br.setPosition(0);
                    fl.setPosition(0);
                    bl.setPosition(1);
                    isMecRight=false;
                }else{
                    fr.setPosition(0.0175);
                    br.setPosition(0.98);
                    fl.setPosition(0.98);
                    bl.setPosition(0.034);
                    isMecRight=true;
                }
                while (gamepad1.right_trigger>0.6){
                    double turn =gamepad1.right_stick_x;
                    double x = gamepad1.left_stick_x;
                    if(!isMecRight){
                        x=0;
                    }
                    double y = gamepad1.left_stick_y;
                    double[] turns = turn(turn,x,y);
                    fleft.setPower(turns[0]*scaler);
                    fright.setPower(turns[1]*scaler);
                    bleft.setPower(turns[2]*scaler);
                    bright.setPower(turns[3]*scaler);
                }
            }

            if (gamepad1.a) {
                scaler=Math.max(0.2, scaler-0.2);
                while (gamepad1.a) {
                    double turn =gamepad1.right_stick_x;
                    double x = gamepad1.left_stick_x;
                    if(!isMecRight){
                        x=0;
                    }
                    double y = gamepad1.left_stick_y;
                    double[] turns = turn(turn,x,y);
                    fleft.setPower(turns[0]*scaler);
                    fright.setPower(turns[1]*scaler);
                    bleft.setPower(turns[2]*scaler);
                    bright.setPower(turns[3]*scaler);
                }
            }
            if (gamepad1.b) {
                scaler=Math.min(1, scaler+0.2);
                while (gamepad1.b) {
                    double turn =gamepad1.right_stick_x;
                    double x = gamepad1.left_stick_x;
                    if(!isMecRight){
                        x=0;
                    }
                    double y = gamepad1.left_stick_y;
                    double[] turns = turn(turn,x,y);
                    fleft.setPower(turns[0]*scaler);
                    fright.setPower(turns[1]*scaler);
                    bleft.setPower(turns[2]*scaler);
                    bright.setPower(turns[3]*scaler);
                }
            }
            if (gamepad1.x) {
                scaler=1;
                while (gamepad1.x) {
                    double turn =gamepad1.right_stick_x;
                    double x = gamepad1.left_stick_x;
                    if(!isMecRight){
                        x=0;
                    }
                    double y = gamepad1.left_stick_y;
                    double[] turns = turn(turn,x,y);
                    fleft.setPower(turns[0]*scaler);
                    fright.setPower(turns[1]*scaler);
                    bleft.setPower(turns[2]*scaler);
                    bright.setPower(turns[3]*scaler);
                }
            }
            while (gamepad1.dpad_up){
                double[] turns = turn(0,0,-0.4);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            while (gamepad1.dpad_down){
                double[] turns = turn(0,0,0.4);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            while (gamepad1.dpad_right&&isMecRight){
                double[] turns = turn(0,0.52,0);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            while (gamepad1.dpad_left&&isMecRight){
                double[] turns = turn(0,-0.52,0);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            while (gamepad1.left_bumper){
                double[] turns = turn(-0.4,0,0);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            while (gamepad1.right_bumper){
                double[] turns = turn(0.4,0,0);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }

            double turn =gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            if(!isMecRight){
                x=0;
            }
            double y = gamepad1.left_stick_y;
            double[] turns = turn(turn,x,y);
            fleft.setPower(turns[0]*scaler);
            fright.setPower(turns[1]*scaler);
            bleft.setPower(turns[2]*scaler);
            bright.setPower(turns[3]*scaler);



        }
    }
    public static double[] turn(double contturn, double x, double y){
        double ang = Math.atan2(y,x);
        double magnitude = Math.sqrt(x*x+y*y);
        double fleft = -1.0*(-1.0*Math.sin(ang+Math.PI*0.25)*magnitude+contturn);
        double bright = -1.0*fleft-2.0*contturn;
        double bleft = -1.0*(-1.0*Math.sin(ang-Math.PI*0.25)*magnitude+contturn);
        double fright = -1.0*bleft-2.0*contturn;
        double scale = Math.max(Math.max(Math.abs(fleft),Math.abs(bright)), Math.max(Math.abs(bleft),Math.abs(fright)));
        scale=Math.max(scale,1);
        return new double[]{fleft/scale,fright/scale,bleft/scale,bright/scale};
    }
}