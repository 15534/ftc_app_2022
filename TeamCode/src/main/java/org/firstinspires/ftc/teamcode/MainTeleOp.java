package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class MainTeleOp extends LinearOpMode {
    DcMotorEx motor;

    public static double kP = -0.001; //-0.000065
    public static double kI = 0;
    public static double kD = 0;

    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = 0.35;
    public static double ROTATION_MULTIPLIER = 2.05;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        PIDFController controller = new PIDFController(coeffs, 0, 0, 0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        waitForStart();
        while (!isStopRequested()) {
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            PoseStorage.currentPose = poseEstimate;

            //telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());

            motor.setPower(gamepad1.right_stick_y * 0.5);

            Vector2d translation = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            double rotation = -ROTATION_MULTIPLIER*gamepad1.right_stick_x;

            // slow translation with dpad
            if (gamepad1.dpad_up) {
                translation = new Vector2d(DPAD_SPEED, 0);
            } else if (gamepad1.dpad_down) {
                translation = new Vector2d(-DPAD_SPEED, 0);
            } else if (gamepad1.dpad_left) {
                translation = new Vector2d(0, DPAD_SPEED);
            } else if (gamepad1.dpad_right) {
                translation = new Vector2d(0, -DPAD_SPEED);
            }

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            drive.setWeightedDrivePower(new Pose2d(translation, rotation));
            telemetry.update();
        }
    }
}