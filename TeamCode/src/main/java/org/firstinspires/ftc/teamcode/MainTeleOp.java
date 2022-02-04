package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveConstants;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp(name = "TeleOp")
@Config
public class MainTeleOp extends LinearOpMode {

    DcMotorEx intakeSurgical, intakeExtension, outtake, motorExLeft;
    Servo intakePosition, outtakeServo, fr, br, fl, bl;

    double mecDown = 0.08;
    double intakeUp = 0.8;
    double tankDown = 0.15;

    public static int outtakeThirdLevelPosition = -350;
    public static int outtakeFirstLevelPosition = -120;
    public static int outtakeDownPosition = 0;
    public static double outtakePower = 0.5;
    public static double outtakeServoClosePosition = 0.9;
    public static double outtakeServoOpenPosition = 0.8;

    public static double intakeExtensionLowerLimit = -30;
    public static double intakeExtensionUpperLimit = 270;
    public static double intakePower = 0.6;

    public static double leftStickPos = 1;

    public static double DPAD_SPEED = 0.35;
    public static double BUMPER_ROTATION_SPEED = 0.15;
    public static double ROTATION_MULTIPLIER = 0.4;

    public static boolean isMec = true;

    @Override
    public void runOpMode() throws InterruptedException {
        double lastvaluefront = -1;
        double lastvaluelast = -1;
        int startpoint = 0;
        double power = 0.0;
        double scale = 0.0;
        SampleMecanumDrive mecDrive = new SampleMecanumDrive(hardwareMap);
        mecDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);
        tankDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fl = hardwareMap.get(Servo.class, "frontleft"); // lower: 0.984, upper: 0
        fr = hardwareMap.get(Servo.class, "frontright"); // lower: .1, upper: 1
        br = hardwareMap.get(Servo.class, "backright"); // lower: 0.954, upper: 0
        bl = hardwareMap.get(Servo.class, "backleft"); // lower: 0.02, upper: 1

        motorExLeft = (DcMotorEx)hardwareMap.get(DcMotor.class, "intake");
        outtakeServo = hardwareMap.get(Servo.class, "outtake servo");

        intakeSurgical = hardwareMap.get(DcMotorEx.class, "intakeSurgical");
        intakeSurgical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeSurgical.setDirection(DcMotor.Direction.FORWARD);
        intakeSurgical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



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

        // init robot
        intakeExtensionLowerLimit = intakeExtension.getCurrentPosition();
        intakeExtensionUpperLimit = intakeExtensionLowerLimit + 270;
        intakePosition.setPosition(intakeUp);
        motorExLeft.setTargetPosition(outtakeDownPosition);
        outtakeServo.setPosition(outtakeServoClosePosition);
        double scaler = 0.6;
        fl.setPosition(0.984);
        fr.setPosition(.1);
        br.setPosition(0.955);
        bl.setPosition(0.02);
        boolean a = true;
        boolean b = true;
        boolean x = true;
        boolean y = true;
        boolean pressed = true;
        waitForStart();

        while (!isStopRequested()) {
            mecDrive.update();
            tankDrive.update();

            Pose2d poseEstimate;
            if (isMec) {
                poseEstimate = mecDrive.getPoseEstimate();
            } else {
                poseEstimate = tankDrive.getPoseEstimate();
            }
            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("scaler", scaler);
            telemetry.update();
            //telemetry.addData("upper limit: ", intakeExtensionUpperLimit);
            //telemetry.addData("lower limit: ", intakeExtensionLowerLimit);

            Vector2d translation = new Vector2d(-gamepad1.left_stick_y*scaler, gamepad1.left_stick_x*scaler);
            double rotation = -ROTATION_MULTIPLIER * gamepad1.right_stick_x*scaler;



            // slow translation with dpad
            if (gamepad1.dpad_up) {
                translation = new Vector2d(DPAD_SPEED, 0);
            } else if (gamepad1.dpad_down) {
                translation = new Vector2d(-DPAD_SPEED, 0);
            } else if (gamepad1.dpad_left) {
                translation = new Vector2d(0, -DPAD_SPEED);
            } else if (gamepad1.dpad_right) {
                translation = new Vector2d(0, DPAD_SPEED);
            }

            // slow rotation with bumpers
            if (gamepad1.left_bumper) {
                rotation = BUMPER_ROTATION_SPEED;
            } else if (gamepad1.right_bumper) {
                rotation = -BUMPER_ROTATION_SPEED;
            }

            if (isMec) {
                mecDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            } else {
                translation = new Vector2d(-gamepad1.left_stick_y*scaler, 0.0);
                tankDrive.setWeightedDrivePower(new Pose2d(translation, rotation));
            }
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
                intakePosition.setPosition(intakeUp); // tank mode down
            }

            if (gamepad2.right_stick_y < -0.1) {
                if(isMec) {
                    intakePosition.setPosition(mecDown);
                }else{
                    intakePosition.setPosition(tankDown);// general servo up position
                }
            }

            //double function = 0.1;
            //if(pos>0){
            //  pos+=intakeExtension.getPower()*20;
            //function = Math.max(4.9*Math.sqrt(pos/300.0)*Math.exp(-1.0*pos/50.0),0.1);
            //}

            // intake extension motor
            if(!gamepad2.dpad_left&&!gamepad2.dpad_up&&!gamepad2.dpad_down&&!gamepad2.dpad_right){
                if (gamepad2.left_stick_y > 0.1) {
                    if(lastvaluefront==-1){
                        startpoint = (int) intakeExtensionUpperLimit;
                    }else if(getRuntime()-lastvaluefront>0.1){
                        startpoint=intakeExtension.getCurrentPosition();

                    }
                    lastvaluefront=getRuntime();
                    int pos =  intakeExtension.getCurrentPosition();
                    pos = startpoint-pos;
                    pos = startpoint+(int)intakeExtensionLowerLimit-pos;
                    double dist = Math.abs(startpoint-intakeExtensionLowerLimit);
                    double function = Math.max((0.12 * (Math.sin((Math.PI / dist) * pos) * Math.exp(Math.PI * pos / dist))), 0.2);
                    if(function>1.5){
                        function=0.1;
                    }
                    // intake should come back up
                    if (intakeExtension.getCurrentPosition() >= intakeExtensionLowerLimit) {
                        power = 1/1.7 *(gamepad2.left_stick_y * (function));
                        intakeExtension.setTargetPosition((int) intakeExtensionLowerLimit);
                        intakeExtension.setPower(-power);
                    } else {
                        intakeExtension.setPower(0.0);
                    }
                } else if (gamepad2.left_stick_y < -0.1) {

                    if(lastvaluelast==-1){
                        startpoint = (int) intakeExtensionLowerLimit;
                    }else if(getRuntime()-lastvaluelast>0.1){
                        startpoint=intakeExtension.getCurrentPosition();

                    }
                    lastvaluelast=getRuntime();
                    int pos =intakeExtension.getCurrentPosition();
                    pos = pos-startpoint;

                    pos=startpoint+(int)intakeExtensionUpperLimit-pos;

                    double dist = Math.abs(startpoint-intakeExtensionUpperLimit);
                    double function = Math.max((0.12 * (Math.sin((Math.PI / dist) * pos) * Math.exp(Math.PI * pos / dist))), 0.2);
                    if(function>1.01){
                        function=0.1;
                    }
                    // intake extends

                    if (intakeExtension.getCurrentPosition() <= intakeExtensionUpperLimit) {
                        power = 1/1.7* (gamepad2.left_stick_y * (function));
                        intakeExtension.setPower(-power);
                    } else {
                        intakeExtension.setPower(0.0);
                    }
                } else {
                    if(Math.abs(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit)<30.0){
                        intakeExtension.setPower(-0.2);
                    }else{
                        if(motorExLeft.getCurrentPosition()==outtakeDownPosition) {
                            intakeExtension.setPower(-0.2);
                        }else{
                            intakeExtension.setPower(0.0);
                        }
                    }

                }
            }else{


            }
            if (gamepad2.dpad_up) {
                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.2);
                }
                motorExLeft.setTargetPosition(outtakeThirdLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);

            }

            if (gamepad2.dpad_left) {
                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.2);
                }
                motorExLeft.setTargetPosition(outtakeFirstLevelPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);
            }

            if (gamepad2.dpad_right) {
                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.2);
                }
                outtakeServo.setPosition(outtakeServoOpenPosition);
            }

            if (gamepad2.dpad_down) {
                if(intakeExtension.getCurrentPosition()-intakeExtensionLowerLimit<100){
                    intakeExtension.setPower(0.2);
                }

                motorExLeft.setTargetPosition(outtakeDownPosition);
                motorExLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorExLeft.setPower(outtakePower);

                outtakeServo.setPosition(outtakeServoClosePosition);
            }
            if(gamepad1.right_trigger>0.6&&pressed){
                if(isMec){
                    fr.setPosition(1);
                    br.setPosition(0);
                    fl.setPosition(0);
                    bl.setPosition(1);
                    isMec=false;
                    if(intakePosition.getPosition()==mecDown){
                        intakePosition.setPosition(tankDown);
                    }
                }else{
                    fl.setPosition(0.984);
                    fr.setPosition(.1);
                    br.setPosition(0.955);
                    bl.setPosition(0.02);
                    isMec=true;
                    if(intakePosition.getPosition()==tankDown){
                        intakePosition.setPosition(mecDown);
                    }
                }
                pressed=false;

            }
            if(gamepad1.right_trigger<0.6){
                pressed=true;
            }


            if (gamepad1.a&&a) {
                scaler=Math.max(0.2, scaler-0.2);
                a=false;

            }
            if(!gamepad1.a){
                a=true;
            }
            if (gamepad1.b&&b) {
                scaler=Math.min(1, scaler+0.2);
                b=false;
            }
            if(!gamepad1.b){
                b=true;
            }

            if (gamepad1.x&&x) {
                scaler=0.6;

            }
            if(!gamepad1.x){
                x=true;
            }
            if (gamepad1.y&&y) {
                scaler=1;

            }
            if(!gamepad1.y){
                y=true;
            }

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

        //telemetry.addData("power: ", power);
        //telemetry.addData("scale", scale);

    }
}
