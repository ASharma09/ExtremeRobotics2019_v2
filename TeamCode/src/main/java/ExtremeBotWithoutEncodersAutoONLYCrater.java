//package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="ExtremeBotWithoutEncodersAutoONLYCrater")
public class ExtremeBotWithoutEncodersAutoONLYCrater extends LinearOpMode
{
    private Robot robot = new Robot();

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

        telemetry.addData("Say", "curry curry curry currrrrrry");
        //GO!!!

        waitForStart();

        robot.WaitMillis(3300);
        SamplingOrderDetector.GoldLocation goldLoc = detector.getLastOrder();
        telemetry.addData("Current Order" , goldLoc); // The current result for the frame
//        telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result
        robot.WaitMillis(300);
        if (goldLoc == SamplingOrderDetector.GoldLocation.CENTER || goldLoc == SamplingOrderDetector.GoldLocation.UNKNOWN)
        {
            telemetry.addData("Current Program: Center", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.driveForward(drivePower, 150);
            robot.turnRight(drivePower, 2000);
            robot.driveBackwards(drivePower, 2000);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.LEFT)
        {
            telemetry.addData("Current Program: Left", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.turnLeft(drivePower, 200);
            robot.turnLeft(drivePower, 1900);
            robot.driveBackwards(drivePower, 1000);
            robot.turnRight(drivePower, 200);
            robot.driveBackwards(drivePower, 500);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.RIGHT)
        {
            telemetry.addData("Current Program: Right", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.turnRight(drivePower, 200);
            robot.turnRight(drivePower, 1900);
            robot.driveBackwards(drivePower, 1000);
            robot.turnLeft(drivePower, 200);
            robot.driveBackwards(drivePower, 500);
        }
        detector.disable();
        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }
}
