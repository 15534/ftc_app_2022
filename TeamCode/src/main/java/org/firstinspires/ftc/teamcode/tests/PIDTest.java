package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Test")
@Config
public class PIDTest extends LinearOpMode {
    DcMotorEx motor;

    //public static double speed = 1;

    public static double kP = -0.025;
    public static double kI = 0;
    public static double kD = -0.001;

    public static double targetPosition = -140;
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double kG = 0.25;

    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;

    int[] mecPositions = new int[]{70, 180, 330};

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "intake");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motor.setVelocity(1000);
        //motor.setVelocity(speed);

        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
        PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic, (x, v) -> kG);

        waitForStart();

        while(opModeIsActive()) {
            controller.setTargetPosition(targetPosition);
            double correction = controller.update(motor.getCurrentPosition());
            motor.setPower(correction);

            telemetry.addData("correction", correction);
            telemetry.addData("correction100", 100 * correction);
            telemetry.addData("position", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
