package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveServoLimit Test")
@Config
public class DriveServoLimits extends LinearOpMode {

    public static double flMec = 0.98;
    public static double flTank = 0;

    public static double brMec = 0.98;
    public static double brTank = 0;

    public static double frMec = 0.0175;
    public static double frTank = 1;

    public static double blMec = 0.034;
    public static double blTank = 1;

    public static boolean isMec = true;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean meow =true;
        Servo fl = hardwareMap.get(Servo.class, "frontleft");
        // limits: 0.98 & 0
        Servo br = hardwareMap.get(Servo.class, "backright");
        // limits: 0.98 & 0
        Servo fr = hardwareMap.get(Servo.class, "frontright");
        // limits: 0.0175 & 1
        Servo bl = hardwareMap.get(Servo.class, "backleft");
        waitForStart();
        while (opModeIsActive()){
            if(meow){
        if (isMec) {
            fl.setPosition(flMec);
            br.setPosition(blMec);
            fr.setPosition(frMec);
            bl.setPosition(blMec);
        } else {
            fl.setPosition(flTank);
            br.setPosition(blTank);
            fr.setPosition(frTank);
            bl.setPosition(blTank);
        }
        meow=false;
            }
        }
    }
}