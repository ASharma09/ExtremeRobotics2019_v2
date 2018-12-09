//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by femukund on 10/22/2017.
 */

public class RobotMini
{
    // Hardware map & op mode
    HardwareMap hwMap;
    LinearOpMode opMode;

    // Variables for Encoder driving
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: Andy Mark Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();

    // Robot wheels
    DcMotor rightBack;
    DcMotor leftBack;


    public void init(HardwareMap ProtohwMap, LinearOpMode linearOpMode)
    {
        hwMap = ProtohwMap;
        opMode = linearOpMode;

        // Wheels
        leftBack = hwMap.dcMotor.get("leftBack");
        rightBack = hwMap.dcMotor.get("rightBack");
    }

    public void WaitMillis (long millis)
    {
        try{
            Thread.sleep(millis);
        }
        catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    // drive without encoder. drive based on time.
/*    public void drive(double leftFront, double rightFront, double leftBack, double rightBack, long millis)
    {
        leftFrontMotor.setPower(leftFront);
        rightFrontMotor.setPower(rightFront);
        leftBackMotor.setPower(leftBack);
        rightBackMotor.setPower(rightBack);
        WaitMillis(millis);
    }

    // Drive forward without encoder. drive based on time.
    public void driveForward(double power, long millis)
    {
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        WaitMillis(millis);
    }

    // Drive backward without encoder. drive based on time.
    public void driveBackwards(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    // Crab left without encoder. drive based on time.
    public void driveLeft(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    // Crab right without encoder. drive based on time.
    public void driveRight(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    // Tank Turn right without encoder. drive based on time.
    public void turnRight(double power, long millis)
    {
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        WaitMillis(millis);
    }

    // Tank Turn left without encoder. drive based on time.
    public void turnLeft(double power, long millis)
    {
        leftFrontMotor.setPower(-power);
        leftBackMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
        WaitMillis(millis);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.

    public void encoderDrive(double speed,
                             double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
            leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            leftBackMotor.setTargetPosition(newLeftBackTarget);
            rightFrontMotor.setTargetPosition(newRightFrontTarget);
            rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontMotor.setPower(Math.abs(speed));
            leftBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed));
            rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to %7d :%7d :%7d :7%", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d :%7d :7% :7%",
                        leftFrontMotor.getCurrentPosition(),
                        leftBackMotor.getCurrentPosition(),
                        rightFrontMotor.getCurrentPosition(),
                        rightBackMotor.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            leftFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // drive forward with encoder. drive by distance.
    public void encoderDriveForward(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, distanceInches, distanceInches, distanceInches, distanceInches, timeout);
    }

    // drive backward with encoder. drive by distance.
    public void encoderDriveBackward(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, -distanceInches, -distanceInches, -distanceInches, -distanceInches, timeout);
    }

    // drive (Crab) left with encoder. drive by distance.
    public void encoderDriveLeft(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, distanceInches, -distanceInches, distanceInches, -distanceInches, timeout);
    }

    // drive (Crab) right with encoder. drive by distance.
    public void encoderDriveRight(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, -distanceInches, distanceInches, -distanceInches, distanceInches, timeout);
    }

    // Turn left by angle - this code needs to be changed by using gyro sensor - look at example 'PushbotAutoDriveByGyro_Linear'.
    public void encoderTurnLeft(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, -distanceInches, -distanceInches, distanceInches, distanceInches, timeout);
    }

    // Turn right by angle - this code needs to be changed by using gyro sensor - look at example 'PushbotAutoDriveByGyro_Linear'.
    public void encoderTurnRight(double power, double distanceInches, double timeout)
    {
        encoderDrive(power, distanceInches, distanceInches, -distanceInches, -distanceInches, timeout);
    }

    public void brake(long millis)
    {
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        WaitMillis(millis);
    }

    // Lift robot
    public void liftRobot()
    {
        liftMotor.setPower(-0.2);
        WaitMillis(2000);
        liftMotor.setPower(0);
    }

    public void landRobot()
    {
        stopServo.setPosition(1);
        WaitMillis(700);
        armBackMotor.setPower(0.2);
        WaitMillis(200);
        armRightMotor.setPower(0.2);
        armLeftMotor.setPower(-0.2);
        WaitMillis(2300);
        armRightMotor.setPower(0);
        armLeftMotor.setPower(0);
        turnRight(0.4, 300);
        WaitMillis(200);
        driveBackwards(0.4, 100);
        WaitMillis(200);
        turnLeft(0.4, 630);
    }

    public void markerDrop()
    {
        sideServo.setPosition(1);
        opMode.telemetry.addData("First Position", sideServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(2000);
        sideServo.setPosition(0);
        opMode.telemetry.addData("Second Position", sideServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(2000);
    }
    // Rotate arm
/*    public void rotateArm(long waitTime, double power)
    {
        armMotor.setPower(power);
        WaitMillis(waitTime);
        armMotor.setPower(-power);
        armMotor.setPower(0);
    }*/

}
