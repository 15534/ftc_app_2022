package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Red")
public class Red extends LinearOpMode {


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

        fl.setPosition(0.98);
        bl.setPosition(0.034);
        fr.setPosition(0.0175);
        br.setPosition(0.98);

        waitForStart();

        while (opModeIsActive()) {
            double start = getRuntime();

            while (getRuntime()-start<0.7){
                double[] turns = turn(0,-1,0);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
            }
            double[] turns2 = turn(0,0,0);
            fleft.setPower(turns2[0]);
            fright.setPower(turns2[1]);
            bleft.setPower(turns2[2]);
            bright.setPower(turns2[3]);
            fl.setPosition(0);
            bl.setPosition(1);
            fr.setPosition(1);
            br.setPosition(0);
            while (true){
                double[] turns = turn(0,0,-1);
                fleft.setPower(turns[0]);
                fright.setPower(turns[1]);
                bleft.setPower(turns[2]);
                bright.setPower(turns[3]);
                while (RANGE2.getDistance(DistanceUnit.CM)<60.0){
                    turns = turn(0,0,0.0);
                    fleft.setPower(turns[0]);
                    fright.setPower(turns[1]);
                    bleft.setPower(turns[2]);
                    bright.setPower(turns[3]);
                }
                telemetry.addData("Distance in cm", "%.2f cm", RANGE2.getDistance(DistanceUnit.CM));
                telemetry.update();

            }

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