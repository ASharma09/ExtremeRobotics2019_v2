//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="ExtremeBotAutoSquare")
public class ExtremeBotAutoSquare extends LinearOpMode
{
    Robot robot = new Robot();

    @Override
    public void runOpMode()
    {
        double drivePower = 0.2;

        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Say", "All systems are go!");
        //GO!!!

        // Hold glyph
        //robot.openClaw();
        //robot.jewelServo.setPosition(0.85);

        waitForStart();
        //robot.holdGlyph();
        //robot.raiseLift();

        // Lower jewel servo

        //robot.lowerJewelServo();
        //telemetry.addData("Say", "Servo is lowered.");
        //telemetry.update();

        // Sense the color of the jewel
       //boolean isJewelRed = robot.isJewelRed();
        //telemetry.addData("Is Jewel Red:", isJewelRed);

        // Knock off jewel
        long driveForwardTime = 2000;
        long driveBackwardTime = 2000;
        long driveLeftTime = 2000;
        long driveRightTime = 2000;
        /*if (isJewelRed)
        {
            robot.driveForward(drivePower, 200);
            robot.brake(500);
            driveForwardTime2 = 1050;
        }
        else
        {
            robot.driveBackwards(drivePower, 200);
            robot.brake(500);
            driveForwardTime2 = 1250;
        }

        // Raise jewel servo
//        robot.WaitMillis(3000);
        robot.raiseJewelServo();
        telemetry.addData("Say", "Servo is raised.");
        telemetry.update();
        */

        // drive to Cryptobox
        robot.driveForward(drivePower, 2800);
        robot.brake(2000);
        robot.turnRight(drivePower, 350);
        robot.brake(2000);
        robot.driveBackwards(drivePower, 3500);
        //robot.TankRight(drivePower, 2150);
        //robot.brake(500);

        // Place the glyph
        //robot.lowerLift();
        //robot.openClaw();
        //robot.driveForward(drivePower, 600);
        //robot.driveBackwards(drivePower, 300);
        // Park in the triangle

        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }
}
