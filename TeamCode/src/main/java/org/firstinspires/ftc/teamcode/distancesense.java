package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Distance")
public class distancesense extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        AnalogInput dist1 = hardwareMap.get(AnalogInput.class, "dist1");
        AnalogInput dist2 = hardwareMap.get(AnalogInput.class, "dist2");
        waitForStart();

        while(opModeIsActive()){
            double raw_value = dist1.getVoltage();
            double voltage_scale_factor = 5/dist1.getMaxVoltage();
            double currentDistanceCentimeters = 819 * (raw_value * voltage_scale_factor * 0.125);
            telemetry.addData("distance1", currentDistanceCentimeters);
            telemetry.addData("meow",raw_value);
            telemetry.addData("meow2",dist1.getConnectionInfo());
            telemetry.update();

        }
    }
}
