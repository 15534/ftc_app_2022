package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Range Linear", group = "MRI")
public class MRI_Range_Linear extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ModernRoboticsI2cRangeSensor RANGE2 = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range2");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Ultra Sonic", RANGE2.rawUltrasonic());
            telemetry.addData("Raw Optical", RANGE2.rawOptical());
            telemetry.addData("Cm Optical", RANGE2.cmOptical());
            telemetry.addData("Distance in cm", "%.2f cm", RANGE2.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}