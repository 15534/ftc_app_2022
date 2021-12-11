package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue")
public class Blue extends LinearOpMode {
    double time = 0.0;
    LinearOpMode op;
    ElapsedTime runtime = new ElapsedTime();

    public static final double COUNTS_PER_MOTOR_REV = 537.7;
    public static final double WHEEL_DIAMETER_INCHES = 3.78;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV)/(WHEEL_DIAMETER_INCHES * Math.PI);


    DcMotor fleft = hardwareMap.get(DcMotor.class, "front_left");
    DcMotor fright = hardwareMap.get(DcMotor.class, "front_right");
    DcMotor bleft = hardwareMap.get(DcMotor.class, "rear_left");
    DcMotor bright = hardwareMap.get(DcMotor.class, "rear_right");
    DcMotor carousel = hardwareMap.get(DcMotor.class,"caro");

    public void setRuntoPositionMode() {
        fleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    enum State {
        IDLE,
        GO_TO_CAROUSEL,
        TURN_CAROUSEL,
        PARK
    }

    void next(State s) {
        time = runtime.seconds();
        currentState = s;
    }

    State currentState = State.IDLE;

    public void runOpMode() throws InterruptedException {
        runtime.reset();

        //int currPos = fleft.getCurrentPosition();

        while (op.opModeIsActive()) {
            double elapsed = runtime.seconds() - time;

            switch (currentState) {
                case GO_TO_CAROUSEL:
                    fleft.setTargetPosition(fleft.getCurrentPosition()- (int) (20*COUNTS_PER_INCH));
                    fright.setTargetPosition(fleft.getCurrentPosition()- (int) (20*COUNTS_PER_INCH));
                    bleft.setTargetPosition(fleft.getCurrentPosition()- (int) (20*COUNTS_PER_INCH));
                    bright.setTargetPosition(fleft.getCurrentPosition()- (int) (20*COUNTS_PER_INCH));
                    setRuntoPositionMode();
                    next(State.TURN_CAROUSEL);
                    break;
                case TURN_CAROUSEL:
                    carousel.setPower(1);
                    wait(3000);
                    carousel.setPower(0);
                    next(State.PARK);
                    break;
                case PARK:
                    fleft.setTargetPosition(fleft.getCurrentPosition()+(int)(100*COUNTS_PER_INCH));
                    fright.setTargetPosition(fleft.getCurrentPosition()+(int)(100*COUNTS_PER_INCH));
                    bleft.setTargetPosition(fleft.getCurrentPosition()+(int)(100*COUNTS_PER_INCH));
                    bright.setTargetPosition(fleft.getCurrentPosition()+(int)(100*COUNTS_PER_INCH));
                    setRuntoPositionMode();
                    next(State.IDLE);
            }
        }
    }

}
