//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.left;
import static android.R.attr.right;

/**
 * Created by femukund on 10/29/2017.
 */
@TeleOp
public class ExtremeBotDrive extends LinearOpMode
{
    Robot robot = new Robot();

    double leftMotorTgtPower = 0;
    double rightMotorTgtPower = 0;

    // Arm
    double liftMotorTgtPower = 0.1;
    double frontArmServo = 0.0;
    double backArmServo = 0.0;
    double sweepServo = 0.0;
    static double FRONT_ARM_SERVO_INCREMENT = 0.02;
    static double BACK_ARM_SERVO_INCREMENT = 0.02;

    @Override
    public void runOpMode ()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, this);
        telemetry.update();

        //robot.jewelServo.setPosition(0.85);

        // Wait for game to start (driver presses PLAY
        waitForStart();

        // run until driver presses STOP
        while (opModeIsActive())
        {
            drive();
        }
    }

    // drive
    public void drive()
    {
        driveWithTwoJoysticks();
        rotateArm();
        extendArm();
        spinArmSweeper();
        // operateLift();
    }

    // drive with joysticks
    public void driveWithTwoJoysticks()
    {
        double max;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back and crabs left and right,
        // the Right stick tank turns left and right
        double speedLF = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedLB = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedRF = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double speedRB = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;

       /* telemetry.addData("Before Clip speedLeftFront", speedLF);
        telemetry.addData("Before Clip speedLeftBack", speedLB);
        telemetry.addData("Before Clip speedRightFront", speedRF);
        telemetry.addData("Before Clip speedRightBack", speedRB); */

        // Clip values so that they are within -1 & +1
        speedLF = Range.clip(speedLF, -1, 1);
        speedLB = Range.clip(speedLB, -1, 1);
        speedRF = Range.clip(speedRF, -1, 1);
        speedRB = Range.clip(speedRB, -1, 1);

        // Set speed to motors
        robot.leftFrontMotor.setPower(speedLF);
        robot.leftBackMotor.setPower(speedLB);
        robot.rightFrontMotor.setPower(speedRF);
        robot.rightBackMotor.setPower(speedRB);

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

    // Rotate arm
    public void rotateArm()
    {
        double speedAM = 0;
        speedAM = Range.clip(speedAM, -1, 1);
        if (gamepad2.right_bumper)
        {
            speedAM = 0.25;
        }
        if (gamepad2.left_bumper)
        {
            speedAM = -0.25;
        }
        robot.armMotor.setPower(speedAM);
        telemetry.addData("speedArmMotor", speedAM);
        telemetry.update();
    }

    // Extend and retract arm
    public void extendArm()
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
/*
        if (frontArmServo > 1)
        {
            frontArmServo = 1.0;
        }

        if (frontArmServo < 0)
        {
            frontArmServo = 0;
        }*/

        robot.frontArmServo.setPosition(frontArmServo);
        robot.backArmServo.setPosition(backArmServo);

        //telemetry.addData("Front Arm Servo Position After", robot.frontArmServo.getPosition());
        telemetry.addData("Front Arm Servo Position After", frontArmServo);
        //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
        telemetry.update();

    }

    // Spin arm sweeper
    public void spinArmSweeper()
    {
        telemetry.addData("Sweep Servo Position Before", robot.sweepServo.getPosition());
        //telemetry.addData("Back Arm Servo Position Before", robot.backArmServo.getPosition());
        //frontArmServo = robot.frontArmServo.getPosition();
        //backArmServo = robot.backArmServo.getPosition();

        if (gamepad2.x)
        {
            sweepServo = 0.7;
            robot.sweepServo.setPosition(sweepServo);
            telemetry.addData("Inside Sweep Servo Position After", sweepServo);
            //telemetry.addData("Back Arm Servo Position After", robot.backArmServo.getPosition());
            telemetry.update();
        }
        if (gamepad2.b)
        {
            sweepServo = 0.3;
            robot.sweepServo.setPosition(sweepServo);
            telemetry.addData("Inside Sweep Servo Position After", sweepServo);
            telemetry.update();
        }
        if (gamepad2.y)
        {
            sweepServo = 0.48;
            robot.sweepServo.setPosition(sweepServo);
        }
/*
        if (frontArmServo > 1)
        {
            frontArmServo = 1.0;
        }

        if (frontArmServo < 0)
        {
            frontArmServo = 0;
        }*/


        telemetry.addData("Sweep Servo Position After", sweepServo);
        telemetry.addData("Sweep Servo Position After", robot.sweepServo.getPosition());
        telemetry.update();

    }

    public void operateLift()
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
}