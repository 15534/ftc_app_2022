package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Distance")
@Config
public class distancesense extends LinearOpMode {
    public static double constant=2.70703125;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AnalogInput dist1 = hardwareMap.get(AnalogInput.class, "dist1");
        AnalogInput dist2 = hardwareMap.get(AnalogInput.class, "dist2");
        waitForStart();

        while(opModeIsActive()){
            double raw_value = dist2.getVoltage();
            double voltage_scale_factor = 3.3/dist2.getMaxVoltage();
            double currentDistanceCentimeters = 5*voltage_scale_factor* raw_value/constant;
            double raw_2 = dist2.getVoltage();
            double vsf2 = 3.3/dist2.getMaxVoltage();
            double cureentd2 = 5*vsf2*raw_2/constant;
            telemetry.addData("meow", raw_value);
            telemetry.addData("meow2",raw_2);
            telemetry.addData("distance1", 39.3701*currentDistanceCentimeters);
            telemetry.addData("distance2",39.3701*cureentd2);
            telemetry.update();

        }
    }

}