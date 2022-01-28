package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "IntakeSurgicalTubing")
@Config
public class IntakeSurgicalTubing extends LinearOpMode {

    DcMotorEx intakeSurgical, intakeExtension, outtake, motorExLeft;
    Servo intakePosition;

    double mecDown = 0.08;
    double intakeUp = 0.8;
    double tankDown = 0.15;

    public static int outtakeTargetPosition = 180;
    public static int outtakeDownPosition = 0;
    public static double outtakePower = 0.5;

    public static double intakeExtensionLowerLimit = -30;
    public static double intakeExtensionUpperLimit = 270;
    public static double intakePower = 0.6;

    public static double leftStickPos = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");

        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double power = 0.0;
        double scale = 0.0;

        outtake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setDirection(DcMotor.Direction.REVERSE);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeExtension = hardwareMap.get(DcMotorEx.class, "intakeExtension");
        intakeExtension.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeExtension.setDirection(DcMotor.Direction.REVERSE);
        intakeExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        //PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> kG);

        intakePosition = hardwareMap.get(Servo.class, "intakeLift");

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();
        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;

        while (!isStopRequested()) {
            telemetry.addData("upper limit: ", intakeExtensionUpperLimit);
            telemetry.addData("lower limit: ", intakeExtensionLowerLimit);
            telemetry.update();
            // surgical tubing
            if (gamepad2.right_trigger < 0.5 && gamepad2.left_trigger < 0.5) {
                intakeSurgical.setPower(0);
            }

            if (gamepad2.right_trigger > 0.5) {
                intakeSurgical.setPower(gamepad2.right_trigger);
            }

            if (gamepad2.left_trigger > 0.5) {
                intakeSurgical.setPower(-1 * gamepad2.left_trigger);
            }

            //telemetry.addData("gamepad stick position: ", gamepad2.right_stick_y);
            //telemetry.update();

            // intake servo
            //range for stick is -1 at top and 1 at bottom
            // intake comes down
            if (gamepad2.right_stick_y > 0.1) {
                intakePosition.setPosition(tankDown); // tank mode down
            }

            if (gamepad2.right_stick_y < -0.1) {
                intakePosition.setPosition(intakeUp); // general servo up position
            }



            //double function = 0.1;
            //if(pos>0){
              //  pos+=intakeExtension.getPower()*20;
                //function = Math.max(4.9*Math.sqrt(pos/300.0)*Math.exp(-1.0*pos/50.0),0.1);
            //}

            // intake extension motor
            if (gamepad2.left_stick_y > 0.1) {
                int pos = intakeExtension.getCurrentPosition() - (int) intakeExtensionLowerLimit;
                double function = Math.max((0.12 * (Math.sin((Math.PI / 270) * pos) * Math.exp(Math.PI * pos / 270))), 0.12);
                // intake should come back up
                if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit) {
                    power = 1/1.5 *(gamepad2.left_stick_y * (function));
                    scale = 1/1.5 *function;
                    intakeExtension.setPower(-power);
                } else {
                    intakeExtension.setPower(0.0);
                }
            } else if (gamepad2.left_stick_y < -0.1) {
                int pos = (int)intakeExtensionUpperLimit -intakeExtension.getCurrentPosition();
                double function = Math.max((0.12 * (Math.sin((Math.PI / 270) * pos) * Math.exp(Math.PI * pos / 270))), 0.12);
                // intake extends
                if (intakeExtension.getCurrentPosition() <= intakeExtensionUpperLimit) {
                    power = 1/1.5 * (gamepad2.left_stick_y * (function));
                    scale = 1/1.5 * (function);
                    intakeExtension.setPower(-power);
                } else {
                    intakeExtension.setPower(0.0);
                }
            } else {
                intakeExtension.setPower(0.0);
            }

            //telemetry.addData("power: ", power);
            //telemetry.addData("scale", scale);
            telemetry.addData("proposed intake power: ", power);
            telemetry.addData("proposed intake scaling factor: ", scale);
            telemetry.addData("joystick input: ", gamepad2.left_stick_y);
            telemetry.addData("intakExtension: ", intakeExtension.getCurrentPosition());

            // outtake - don't use until outtake pidf is tuned
            if (gamepad2.dpad_up) {
                motorExLeft.setTargetPosition(outtakeTargetPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
            }

            if (gamepad2.dpad_down) {
                motorExLeft.setTargetPosition(outtakeDownPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
            }

            // dpad left - tier 1
            // dpad right - open outtake servo that contains freight
            // dpad down - return to position 0 and close outtake servo as well

//
//            if (gamepad2.left_stick_y > 0.0) {
//                if (intakeExtension.getCurrentPosition() < intakeExtensionUpperLimit
//                    && intakeExtension.getCurrentPosition() > intakeExtensionLowerLimit) {
//                    intakeExtension.setPower(gamepad2.left_stick_y);
//                }
//            }
            telemetry.update();
        }
    }
}
