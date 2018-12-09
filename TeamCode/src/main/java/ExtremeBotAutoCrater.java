//package org.firstinspires.ftc.teamcode;

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

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // Measured distance between minerals
    private static double distanceBetweenMineralsInches = 18.0;

    //Temp
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "Ab93omn/////AAABmcvezQk0/UkLnEkBq1IL0w1+OCmdHQDGKxuqvC7p82DtEhhKrSomTyP7U+FB8vlbo/YPejwG/FnJ8C4MnxxSO86rGLTG+wEDwmzKrGGeG6ATXhfoA3umFEfShouWib5UWzXHfR7JGZ9S/bqShhThpU8tgegY8DcF4KN05LL8EVxrDwJw8sLQTA/im52LwBthiDpcIbjrRRll0NX7wVHRQ1IgN9ZL13amiVo4GARHDQx8C20FdpDNi87GV0Ulpu2t6HiAccweCx45rgvSraIfCnOFj2LIAXE60Bw9IOqLijF1Y/QaZWSsr+JkWIYlwifkCXZuk3aecGlM9qmZB1VU/VW1IF7hsbUwKQkmd0sLcN6ps";
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
/*        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
*/
        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        double drivePower = 0.2;

        // drive with encoder
        autonomousDriveWithoutEncoder(drivePower);

        // drive without encoder
        //autonomousDriveWithoutEncoder(drivePower);

        telemetry.addData("Say", "I am done.");
        telemetry.update();
    }



    /**
     * Drive with encoder
     * @param drivePower Drive power
     */
    private void autonomousDriveWithEncoder(double drivePower)
    {
/*        int goldMineralPostion = 0; //detectGoldMineralPosition();

        if (goldMineralPostion == -1)
        {
            robot.encoderDriveLeft(drivePower, distanceBetweenMineralsInches, 2000);
        }
        else if (goldMineralPostion == 1)
        {
            robot.encoderDriveRight(drivePower, distanceBetweenMineralsInches, 2000);
        }*/
        robot.encoderDriveBackward(drivePower, 20, 1000);
    }

    /**
     * Drive without encoder
     * @param drivePower Drive power
     */
    private void autonomousDriveWithoutEncoder(double drivePower)
    {
        /*int goldMineralPostion = detectGoldMineralPosition();
        if (goldMineralPostion == -1)
        {
            robot.driveLeft(drivePower,2000);
        }
        else if (goldMineralPostion == 1)
        {
            robot.driveRight(drivePower, 2000);
        }*/
        robot.landRobot();
        robot.driveBackwards(drivePower, 2200);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Detect gold mineral position.
     * Return -1 if left, 0 if center, +1 if right
     */
    private int detectGoldMineralPosition()
    {
        int goldMineralPosition = 0;
        if (opModeIsActive()) {
            // Activate Tensor Flow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    goldMineralPosition = -1;
                                    telemetry.addData("Gold Mineral Position", "Left");
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    goldMineralPosition = 0;
                                    telemetry.addData("Gold Mineral Position", "Right");
                                } else {
                                    goldMineralPosition = 1;
                                    telemetry.addData("Gold Mineral Position", "Center");
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return goldMineralPosition;
    }
}
