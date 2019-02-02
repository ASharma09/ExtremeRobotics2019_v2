//package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
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
        double drivePower = 0.35;

        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Detector
        SamplingOrderDetector detector = new SamplingOrderDetector();
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

        telemetry.addData("Say", "All systems go!");
        //GO!!!

        waitForStart();

        robot.WaitMillis(3300);
        SamplingOrderDetector.GoldLocation goldLoc = detector.getLastOrder();
        telemetry.addData("Current Order" , goldLoc); // The current result for the frame
        //telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result
        robot.WaitMillis(300);
        if (goldLoc == SamplingOrderDetector.GoldLocation.CENTER || goldLoc == SamplingOrderDetector.GoldLocation.UNKNOWN)
        {
            telemetry.addData("Current Program: Center", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.turnRight(drivePower, 2100);
            robot.driveBackwards(drivePower, 2130);
            robot.brake(200);
            robot.turnLeft(drivePower, 630);
            robot.brake(200);
            robot.markerDrop();
            robot.brake(100);
            robot.driveBackwards(drivePower, 300);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.LEFT)
        {
            telemetry.addData("Current Program: Left", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.driveForward(drivePower, 300);
            robot.turnRight(drivePower, 1550);
            robot.driveBackwards(0.4, 1650);
            robot.driveForward(drivePower, 230);
            robot.turnRight(drivePower, 980);
            robot.brake(200);
            robot.driveBackwards(drivePower, 700);
            robot.brake(200);
            robot.markerDrop();
            robot.brake(200);
            robot.driveBackwards(drivePower, 200);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.RIGHT)
        {
            telemetry.addData("Current Program: Right", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.driveForward(drivePower, 300);
            robot.turnLeft(drivePower, 1420);
            robot.driveBackwards(0.4, 1730);
            robot.driveForward(drivePower, 230);
            robot.turnLeft(drivePower, 780);
            robot.brake(200);
            robot.markerDrop();
            robot.brake(200);
            robot.driveBackwards(drivePower, 200);
        }
        detector.disable();
        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }
}
