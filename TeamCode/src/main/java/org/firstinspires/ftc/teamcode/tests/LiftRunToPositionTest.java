package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Created by tom on 9/26/17.
 * This assumes that you are using a REV Robotics Expansion Hub
 * as your DC motor controller.  This op mode uses the extended/enhanced
 * PID-related functions of the DcMotorEx class.  The REV Robotics Expansion Hub
 * supports the extended motor functions, but other controllers (such as the
 * Modern Robotics and Hitechnic DC Motor Controllers) do not.
 */

@Autonomous(name="Lift Run To Position")
public class LiftRunToPositionTest extends LinearOpMode {

    // our DC motor.
    DcMotorEx motorExLeft;

    public static final double NEW_P = 10;
    public static final double NEW_I = 3;
    public static final double NEW_D = 0;
    public static final double NEW_F = 0.0;
    public static int targetPosition = 180;
    public static double power = 0.5;

    public void runOpMode() {
        // get reference to DC motor.
        // since we are using the Expansion Hub,
        // cast this motor to a DcMotorEx object.
        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");

        // wait for start command.
        waitForStart();

//        // get the PID coefficients for the RUN_USING_ENCODER  modes.
//        PIDFCoefficients pidOrig = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // change coefficients using methods included with DcMotorEx class.
//        PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
//        motorExLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
//
//        // re-read coefficients and verify change.
//        PIDFCoefficients pidModified = motorExLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExLeft.setTargetPosition(targetPosition);
        motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorExLeft.setPower(power);

        // display info to user.
        while(opModeIsActive()) {
            telemetry.addData("Runtime", "%.03f", getRuntime());
//            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
//                    pidOrig.p, pidOrig.i, pidOrig.d);
//            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
//                    pidModified.p, pidModified.i, pidModified.d);
            telemetry.addData("position", motorExLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}