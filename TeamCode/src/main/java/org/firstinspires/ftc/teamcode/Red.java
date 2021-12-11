package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red")
public class Red extends LinearOpMode {
    double time = 0.0;
    LinearOpMode op;
    ElapsedTime runtime = new ElapsedTime();

    DcMotor fleft = hardwareMap.get(DcMotor.class, "front_left");
    DcMotor fright = hardwareMap.get(DcMotor.class, "front_right");
    DcMotor bleft = hardwareMap.get(DcMotor.class, "rear_left");
    DcMotor bright = hardwareMap.get(DcMotor.class, "rear_right");
    DcMotor carosell = hardwareMap.get(DcMotor.class,"caro");

    enum State {
        IDLE,
        GO_TO_CAROUSEL,
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
                    //while(currPos < ):

                case PARK:
            }
        }
    }

}
