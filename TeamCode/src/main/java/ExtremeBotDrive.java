//package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import static android.R.attr.left;
import static android.R.attr.right;
import static android.R.attr.wallpaperCloseEnterAnimation;

/**
 * Created by femukund on 10/29/2017.
 */
@TeleOp
public class ExtremeBotDrive extends LinearOpMode
{
    Robot robot = new Robot();

    double leftMotorTgtPower = 0;
    double rightMotorTgtPower = 0;
// this is a test lol xd l m a o
    // Arm
    double liftBackMotorTgtPower = .1;
    double frontArmServo = 0.0;
    double backArmServo = 0.0;
    double stopServo = 0.0;
    double positionSS = 0.0;
    //GoldAlignDetector goldDetector;
    SamplingOrderDetector detector;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap, this);
        telemetry.update();

        // Gold align detector
/*        goldDetector = new GoldAlignDetector();
        goldDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldDetector.useDefaults(); // Set detector to use default settings
        // Optional tuning
        goldDetector.alignSize = 600; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        //changed from 200 to 400 then 600
        // 800 does not work
        goldDetector.alignPosOffset = 5; // How far from center frame to offset this alignment zone.
        goldDetector.downscale = .6; // How much to downscale the input frames

        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldDetector.maxAreaScorer.weight = 0.005;

        goldDetector.ratioScorer.weight = 5;
        goldDetector.ratioScorer.perfectRatio = 1.0;

        goldDetector.enable();
*/
        // Detector
        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(),0, false);
        detector.useDefaults(); // Set detector to use default settings
        detector.downscale = 0.4;
        // Optional tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;
        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();

        // Wait for game to start (driver presses PLAY
        waitForStart();

        // run until driver presses STOP
        while (opModeIsActive())
        //while (opModeIsActive() && (runtime.seconds() < 31.0))
        {
            drive();
        }
    }

    // drive
    public void drive()
    {
        driveWithTwoJoysticks();
        liftRobot();
        extendArm();
        extendLift();
        moveBoot();
        spinArmSweeper();
        drawerSlideFrontLock();
        drawerSlideBackLock();

        //telemetry.addData("IsAligned" , goldDetector.getAligned()); // Is the bot aligned with the gold mineral
        //telemetry.addData("X Pos" , goldDetector.getXPosition()); // Gold X pos.
        telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result
        telemetry.update();
    }

    // drive with joysticks
    public void driveWithTwoJoysticks()
    {
        double max;
        // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
        // In this mode the Left stick moves the robot fwd and back and crabs left and right,
        // the Right stick tank turns left and right
        double speedLF = -(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        double speedLB = -(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        double speedRF = -(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        double speedRB = -(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

        /*
        double speedLF = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedLB = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        double speedRF = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double speedRB = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
         */

/*
        telemetry.addData("Before Clip speedLeftFront", speedLF);
        telemetry.addData("Before Clip speedLeftBack", speedLB);
        telemetry.addData("Before Clip speedRightFront", speedRF);
        telemetry.addData("Before Clip speedRightBack", speedRB);
*/
        // Clip values so that they are within -1 & +1
        speedLF = Range.clip(speedLF, -1, 1);
        speedLB = Range.clip(speedLB, -1, 1);
        speedRF = Range.clip(speedRF, -1, 1);
        speedRB = Range.clip(speedRB, -1, 1);
/*
        telemetry.addData("After Clip speedLeftFront", speedLF);
        telemetry.addData("After Clip speedLeftBack", speedLB);
        telemetry.addData("After Clip speedRightFront", speedRF);
        telemetry.addData("After Clip speedRightBack", speedRB);
*/
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

    //Temp

    // Rotate arm
    public void liftRobot()
    {
        double speedAM = 0;
        speedAM = Range.clip(speedAM, -1, 1);
        if (gamepad2.dpad_down)
        {
            speedAM = 0.75;
        }
        if (gamepad2.dpad_up)
        {
            speedAM = -0.75;
        }
        robot.actuatorMotor.setPower(speedAM);
        telemetry.addData("speedActuatorMotor", speedAM);
        telemetry.update();
    }
    public void extendArm()
    {
        double speedOM = 0;
        speedOM = Range.clip(speedOM, -1, 1);
        if (gamepad2.right_bumper)
        {
            speedOM = 0.8;
        }
        if (gamepad2.left_bumper)
        {
            speedOM = -0.8;
        }
        robot.omniMotor.setPower(speedOM);
        telemetry.addData("speedOmniMotor", speedOM);
        telemetry.update();
    }
    public void extendLift()
    {
        double speedCM = 0;
        speedCM = Range.clip(speedCM, -1, 1);
        if (gamepad1.right_bumper)
        {
            speedCM = 0.85;
        }
        if (gamepad1.left_bumper)
        {
            speedCM = -0.85;
        }
        robot.cascadingMotor.setPower(speedCM);
        telemetry.addData("speedCascadingMotor", speedCM);
        telemetry.update();
    }

    public void moveBoot()
    {
        double speedBM = 0;
        speedBM = Range.clip(speedBM, -1, 1);
        if (gamepad2.dpad_left)
        {
            speedBM = 0.39;
        }
        if (gamepad2.dpad_right)
        {
            speedBM = -0.39;
        }
        robot.bootMotor.setPower(speedBM);
        telemetry.addData("speedBootMotor", speedBM);
        telemetry.update();
    }

    public void drawerSlideBackLock()
    {
        if (gamepad1.b)
        {
            robot.lockBackServo.setPosition(0.4);
        }
        if (gamepad1.a)
        {
            robot.lockBackServo.setPosition(0.87);
        }

    }
    public void drawerSlideFrontLock()
    {
        if (gamepad2.b)
        {
            robot.lockFrontServo.setPosition(0.2);
        }
        if (gamepad2.a)
        {
            robot.lockFrontServo.setPosition(0.95);
        }

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
*/
    // Spin arm sweeper
    public void spinArmSweeper()
    {
        if (gamepad2.x)
        {
            positionSS = 0;
            robot.sweepServo.setPosition(positionSS);
            telemetry.addData("Sweep Servo Position:", robot.sweepServo.getPosition());
        }
        if (gamepad2.y)
        {
            positionSS = 0.5;
            robot.sweepServo.setPosition(positionSS);
            telemetry.addData("Sweep Servo Position:", robot.sweepServo.getPosition());
        }

    }
/*    public void stopLift() {

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
        }*/
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
}