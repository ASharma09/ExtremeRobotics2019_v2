//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by femukund on 10/29/2017.
 */
@TeleOp
public class ExtremeBotDriveMini extends LinearOpMode {
    RobotMini robot = new RobotMini();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, this);
        telemetry.update();

        //robot.jewelServo.setPosition(0.85);

        // Wait for game to start (driver presses PLAY
        waitForStart();

        // run until driver presses STOP
        while (opModeIsActive()) {
            drive();
        }
    }

    // drive
    public void drive()
    {
        driveMiniBot();
    }

    // drive with joysticks
    public void driveMiniBot() { {
        double speedRB = 0;
        double speedLB = 0;
        speedRB = Range.clip(speedRB, -1, 1);
        speedLB = Range.clip(speedLB, -1, 1);
        if (gamepad1.dpad_left) {
            speedRB = 0.7;
            speedLB = 0.7;
        }
        if (gamepad1.dpad_right) {
            speedRB = -0.7;
            speedLB = -0.7;
        }
        if (gamepad1.dpad_down) {
            speedRB = 0.7;
            speedLB = -0.7;
        }
        if (gamepad1.dpad_up) {
            speedRB = -0.7;
            speedLB = 0.7;
        }
        robot.rightBack.setPower(speedRB);
        robot.leftBack.setPower(speedLB);
        telemetry.addData("speedRightBack", speedRB);
        telemetry.addData("speedLeftBack", speedLB);
        telemetry.update();
    }


        /*
        double speedLF = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedLB = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedRF = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double speedRB = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
         */



        // Displaying information on the Driver Station
       /* telemetry.addData("speedLeftFront", speedLF);
        telemetry.addData("speedLeftBack", speedLB);
        telemetry.addData("speedRightFront", speedRF);
        telemetry.addData("speedRightBack", speedRB);
        telemetry.addData("Left Motor Power", robot.leftFrontMotor.getPower());
        telemetry.addData("Right Motor Power", robot.rightFrontMotor.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update(); */
    }

    //Temp
    // Rotate arm
/*    public void rotateArm() {
        double speedARM = 0;
        speedARM = Range.clip(speedARM, -1, 1);
        if (gamepad2.left_bumper) {
            speedARM = 0.4;
        }
        if (gamepad2.right_bumper) {
            speedARM = -0.4;
        }
        robot.armRightMotor.setPower(speedARM);
        robot.armLeftMotor.setPower(-speedARM);
        telemetry.addData("speedArmMotor", speedARM);
        telemetry.update();
    }
    public void rotateBackArm() {
        double speedABM = 0;
        speedABM = Range.clip(speedABM, -1, 1);
        if (gamepad2.dpad_up) {
            speedABM = 0.4;
        }
        if (gamepad2.dpad_down) {
            speedABM = -0.4;
        }
        robot.armBackMotor.setPower(speedABM);
        telemetry.addData("speedArmBackMotor", speedABM);
        telemetry.update();
    }
    // Extend and retract arm
/*    public void extendArm()
    {
        telemetry.addData("Front Arm Servo Position Before", robot.frontArmServo.getPosition());
        //telemetry.addData("Back Arm Servo Position Before", robot.backArmServo.getPosition());
        //frontArmServo = robot.frontArmServo.getPosition();
        //backArmServo = robot.backArmServo.getPosition();

        if (gamepad2.y)
        {
            frontArmServo = 1;
            backArmServo = -1;
            telemetry.addData("Inside Front Arm Servo Position After", frontArmServo);
            //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
            telemetry.update();
        }
        if (gamepad2.a)
        {
            frontArmServo = -1;
            backArmServo = 1;
            telemetry.addData("Inside Back Arm Servo Position After", backArmServo);
            telemetry.update();
        }

        if (frontArmServo > 1)
        {
            frontArmServo = 1.0;
        }

        if (frontArmServo < 0)
        {
            frontArmServo = 0;
        }

        robot.frontArmServo.setPosition(frontArmServo);
        robot.backArmServo.setPosition(backArmServo);

        //telemetry.addData("Front Arm Servo Position After", robot.frontArmServo.getPosition());
        telemetry.addData("Front Arm Servo Position After", frontArmServo);
        //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
        telemetry.update();

    }

    // Spin arm sweeper
    public void spinArmSweeper() {
        telemetry.addData("Sweep Servo Position Before", robot.sweepServo.getPosition());
        //telemetry.addData("Back Arm Servo Position Before", robot.backArmServo.getPosition());
        //frontArmServo = robot.frontArmServo.getPosition();
        //backArmServo = robot.backArmServo.getPosition();

        if (gamepad2.x) {
            sweepServo = 0.55;
            robot.sweepServo.setPosition(sweepServo);
            telemetry.addData("Inside Sweep Servo Position After", sweepServo);
            //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
            telemetry.update();
        }
        if (gamepad2.b) {
            sweepServo = 0.45;
            robot.sweepServo.setPosition(sweepServo);
            telemetry.addData("Inside Sweep Servo Position After", sweepServo);
            telemetry.update();
        }
        if (gamepad2.y) {
            sweepServo = 0.48;
            robot.sweepServo.setPosition(sweepServo);
        }


/*        if (frontArmServo > 1)
        {
            frontArmServo = 1.0;
        }

        if (frontArmServo < 0)
        {
            frontArmServo = 0;
        }*/



    }/*
    public void stopLift() {

        if (gamepad1.dpad_right) {
            stopServo = -1;
            robot.stopServo.setPosition(stopServo);
            telemetry.addData("Inside Stop Servo Position", stopServo);
            //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
            telemetry.update();
        }
        if (gamepad1.dpad_left) {
            stopServo = 1;
            robot.stopServo.setPosition(stopServo);
            telemetry.addData("Outside Stop Servo Position", stopServo);
            telemetry.update();
        }
/*    public void operateLift()
    {
        liftMotorTgtPower = 0;

        if (gamepad2.dpad_up)
        {
            liftMotorTgtPower = -0.5;
            telemetry.addData("Lift Motor UPPPPPPPPPPPPPPP", "UP");
        }
        if (gamepad2.dpad_down)
        {
            liftMotorTgtPower = 0.5;
            telemetry.addData("Lift Motor DOWNNNNNNNN", "DOWN");
        }
//        if (robot.liftTouchSensor.isPressed() && gamepad2.dpad_up)
        {
            liftMotorTgtPower = 0;
            telemetry.addData("Lift Motor TOUCHSENSOR IS PRESSED", "TOUCH");
        }
        robot.liftMotor.setPower(liftMotorTgtPower);
        telemetry.addData("Lift Motor Power", robot.liftMotor.getPower());
        telemetry.update();
    }
*/
//