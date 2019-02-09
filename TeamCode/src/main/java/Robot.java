//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by femukund on 10/22/2017.
 */

public class Robot
{
    // Hardware map & op mode
    HardwareMap hwMap;
    LinearOpMode opMode;
    double drivePower = 0.2;
    // Variables for Encoder driving
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: Andy Mark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    private ElapsedTime runtime = new ElapsedTime();

    //Robot wheels
    DcMotor rightFrontMotor;
    DcMotor leftFrontMotor;
    DcMotor rightBackMotor;
    DcMotor leftBackMotor;

    // Arm motors & servos
    DcMotor actuatorMotor;
    DcMotor omniMotor;
    DcMotor cascadingMotor;
    DcMotor bootMotor;
    Servo sideServo;
    Servo sweepServo;
    Servo lockBackServo;
    Servo lockFrontServo;

    // Lift & Land motor


    public void init(HardwareMap ProtohwMap, LinearOpMode linearOpMode)
    {
        hwMap = ProtohwMap;
        opMode = linearOpMode;

        // Wheels
        leftFrontMotor = hwMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hwMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = hwMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = hwMap.dcMotor.get("rightBackMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Arm
        actuatorMotor = hwMap.dcMotor.get("actuatorMotor");
        omniMotor = hwMap.dcMotor.get("omniMotor");
        cascadingMotor = hwMap.dcMotor.get("cascadingMotor");
        bootMotor = hwMap.dcMotor.get("bootMotor");
        sideServo = hwMap.servo.get("sideServo");
        sweepServo = hwMap.servo.get("sweepServo");
        lockBackServo = hwMap.servo.get("lockBackServo");
        lockFrontServo = hwMap.servo.get("lockFrontServo");

        // Lift
        //liftMotor = hwMap.dcMotor.get("liftMotor");
    }

    public void WaitMillis(long millis)
    {
        try
        {
            Thread.sleep(millis);
        } catch (InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    // drive without encoder. drive based on time.
    public void drive(double leftFront, double rightFront, double leftBack, double rightBack, long millis)
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
        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        rightBackMotor.setPower(power);
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
        leftFrontMotor.setPower(-power);
        leftBackMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
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
     */
//    public void encoderDrive(double speed,
//                             double leftFrontInches, double leftBackInches,
//                             double rightFrontInches, double rightBackInches,
//                             double timeoutS)
//    {
////        int newLeftFrontTarget;
//  //      int newLeftBackTarget;
//    //    int newRightFrontTarget;
//      //  int newRightBackTarget;
//
//        // Ensure that the opmode is still active
//        if (opMode.opModeIsActive())
//        {
//
//            // Determine new target position, and pass to motor controller
//            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////            newLeftFrontTarget = leftFrontMotor.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
////            newLeftBackTarget = leftBackMotor.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);
////            newRightFrontTarget = rightFrontMotor.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
////            newRightBackTarget = rightBackMotor.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
//            leftFrontMotor.setTargetPosition((int) (leftFrontInches * COUNTS_PER_INCH));
//            leftBackMotor.setTargetPosition((int) (leftBackInches * COUNTS_PER_INCH));
//            rightFrontMotor.setTargetPosition((int) (rightFrontInches * COUNTS_PER_INCH));
//            rightBackMotor.setTargetPosition((int) (rightBackInches * COUNTS_PER_INCH));
//
//            // Turn On RUN_TO_POSITION
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            leftFrontMotor.setPower(speed * Math.signum(leftFrontInches));
//            leftBackMotor.setPower(speed * Math.signum(leftBackInches));
//            rightFrontMotor.setPower(speed * Math.signum(rightFrontInches));
//            rightBackMotor.setPower(speed * Math.signum(rightBackInches));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opMode.opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy()))
//            {
//
//                // Display it for the driver.
////                opMode.telemetry.addData("Path1", "Running to %7d :%7d :%7d :7%", newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget, newRightBackTarget);
//                opMode.telemetry.addData("Path2", "Running at %7d :%7d :7% :7%",
//                        leftFrontMotor.getCurrentPosition(),
//                        leftBackMotor.getCurrentPosition(),
//                        rightFrontMotor.getCurrentPosition(),
//                        rightBackMotor.getCurrentPosition());
//                opMode.telemetry.update();
//            }
//
//            // Stop all motion;
//            leftFrontMotor.setPower(0);
//            leftBackMotor.setPower(0);
//            rightFrontMotor.setPower(0);
//            rightBackMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }

    // drive forward with encoder. drive by distance.

    public  int convertInches(double inches)
    {
        return (int) (COUNTS_PER_INCH * inches);
    }
    public int convertDegrees(double degrees) { return (int) (degrees* 17.54385963157895); }

    public void encoderTurnRight(double power, int ticks) {
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(-power);
        while (opMode.opModeIsActive() && rightBackMotor.getCurrentPosition() >= -ticks) {
            opMode.telemetry.addData("current rightBackMotor encoder position: ", rightBackMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

       leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void encoderTurnLeft(double power, int ticks) {
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(power);
        while(opMode.opModeIsActive() && rightBackMotor.getCurrentPosition() <= ticks)
        {
            opMode.telemetry.addData("current rightBackMotor encoder position: ", rightBackMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);


    }

    public void encoderDriveForward(double power, int ticks) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightBackMotor.setPower(power);
        while(opMode.opModeIsActive() && leftFrontMotor.getCurrentPosition() >= -ticks)
        {
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        opMode.telemetry.addData("current leftFrontMotor encoder position: ", leftFrontMotor.getCurrentPosition());
        opMode.telemetry.update();
    }

    public void encoderDriveBackward(double power, int ticks) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontMotor.setPower(-power);
        rightFrontMotor.setPower(-power);
        leftBackMotor.setPower(-power);
        rightBackMotor.setPower(-power);
        while(opMode.opModeIsActive() && leftFrontMotor.getCurrentPosition() <= ticks)
        {
        }

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);

        opMode.telemetry.addData("current leftFrontMotor encoder position: ", leftFrontMotor.getCurrentPosition());
        opMode.telemetry.update();
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
/*    public void liftRobot()
    {
        liftMotor.setPower(-0.2);
        WaitMillis(2000);
        liftMotor.setPower(0);
    }*/

    public void landRobot()
    {
        actuatorMotor.setPower(0.75);
        WaitMillis(7670);
        actuatorMotor.setPower(0);
        driveRealign(0,0);
//        encoderDriveBackward(drivePower, convertInches(2) );
//        brake(300);
//        encoderTurnRight(drivePower,convertDegrees(45) );
//        brake(300);
//        encoderDriveForward(drivePower, convertInches(2));
//        brake(300);
//        encoderTurnLeft(drivePower, convertDegrees(45));
//        brake(300);


    }

    public void markerDrop()
    {
        sideServo.setPosition(0.25);
        opMode.telemetry.addData("First Position", sideServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(800);
        sideServo.setPosition(0.92);
        opMode.telemetry.addData("Second Position", sideServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(1000);
        sideServo.setPosition(0.3);
        opMode.telemetry.addData("Third Position", sideServo.getPosition());
        opMode.telemetry.update();
        WaitMillis(800);
    }
    public void driveRealign(double startMotor, double endMotor)

    {
        //Backwards to unhook from latch
        startMotor = rightFrontMotor.getCurrentPosition();
        endMotor = startMotor + 600;
        while (rightFrontMotor.getCurrentPosition() <= endMotor) {
            rightBackMotor.setPower(-drivePower);
            rightFrontMotor.setPower(-drivePower);
            leftBackMotor.setPower(-drivePower);
            leftFrontMotor.setPower(-drivePower);
        }


        //Turn right
        startMotor = rightFrontMotor.getCurrentPosition();
        endMotor = startMotor - 900;

        while (rightFrontMotor.getCurrentPosition() >= endMotor) {
            leftFrontMotor.setPower(drivePower);
            rightFrontMotor.setPower(-drivePower);
            leftBackMotor.setPower(drivePower);
            rightBackMotor.setPower(-drivePower);

            opMode.telemetry.addData("current rightFrontMotor encoder position: ", rightFrontMotor.getCurrentPosition());
            opMode.telemetry.update();
        }

        //forwards
        startMotor = rightFrontMotor.getCurrentPosition();
        endMotor = startMotor - 1300;

        while (rightFrontMotor.getCurrentPosition() >= endMotor) {
            rightBackMotor.setPower(drivePower);
            rightFrontMotor.setPower(drivePower);
            leftBackMotor.setPower(drivePower);
            leftFrontMotor.setPower(drivePower);
        }

        //Turn left
        startMotor = rightFrontMotor.getCurrentPosition();
        endMotor = startMotor + 410;

        while (rightFrontMotor.getCurrentPosition() <= endMotor) {
            leftFrontMotor.setPower(-drivePower);
            rightFrontMotor.setPower(drivePower);
            leftBackMotor.setPower(-drivePower);
            rightBackMotor.setPower(drivePower);

            opMode.telemetry.addData("current rightFrontMotor encoder position: ", rightFrontMotor.getCurrentPosition());
            opMode.telemetry.update();
        }
        //Backwards
        startMotor = rightFrontMotor.getCurrentPosition();
        endMotor = startMotor - 700;
        while (rightFrontMotor.getCurrentPosition() <= endMotor) {
            rightBackMotor.setPower(-drivePower);
            rightFrontMotor.setPower(-drivePower);
            leftBackMotor.setPower(-drivePower);
            leftFrontMotor.setPower(-drivePower);
        }

        // Rotate arm
/*    public void rotateArm(long waitTime, double power)
    {
        armMotor.setPower(power);
        WaitMillis(waitTime);
        armMotor.setPower(-power);
        armMotor.setPower(0);
    }
*/

    }
}
