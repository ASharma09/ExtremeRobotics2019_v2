//package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sun.tools.javac.comp.Flow;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * Created by femukund on 11/5/2017.
 */

@Autonomous(name="ExtremeBotAutoCrater")
public class ExtremeBotAutoCrater extends LinearOpMode
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

        telemetry.addData("Say", "All systems go!");
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
            robot.driveForward(drivePower, 1050);
            robot.driveBackwards(drivePower, 700);
            robot.turnRight(drivePower, 980);
            robot.driveBackwards(drivePower, 1800);
            robot.turnLeft(drivePower, 940);
            robot.driveBackwards(drivePower, 950);
            robot.markerDrop();
            robot.turnLeft(drivePower, 100);
            robot.driveForward(drivePower, 3350);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.LEFT)
        {
            telemetry.addData("Current Program: Left", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.turnLeft(drivePower, 400);
            robot.driveForward(drivePower, 800);
            robot.driveBackwards(drivePower, 400);
            robot.turnRight(drivePower, 600);
            robot.driveBackwards(drivePower, 1500);
            robot.turnLeft(drivePower, 500);
            robot.driveBackwards(drivePower, 2500);
            robot.markerDrop();
            robot.driveForward(0.6, 2800);
        }
        else if (goldLoc == SamplingOrderDetector.GoldLocation.RIGHT)
        {
            telemetry.addData("Current Program: Right", goldLoc);
            telemetry.update();
            robot.landRobot();
            robot.turnRight(drivePower, 200);
            robot.driveForward(drivePower, 1200);
            robot.driveBackwards(drivePower, 400);
            robot.turnRight(drivePower, 800);
            robot.driveBackwards(drivePower, 1900);
            robot.turnLeft(drivePower, 500);
            robot.driveBackwards(drivePower, 2500);
            robot.markerDrop();
            robot.driveForward(0.6, 2800);
        }
        detector.disable();
        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }
}
