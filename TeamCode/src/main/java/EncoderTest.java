
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {
    Robot robot = new Robot();
///

    public double startMotor = 0;
    public double endMotor = 0;
    public void runOpMode() {
//        try {
            robot.init(hardwareMap, this);
            waitForStart();
//            robot.leftFrontMotor.setPower(.2);
//            robot.rightFrontMotor.setPower(-.2);
//            while(opModeIsActive() && robot.leftFrontMotor.getCurrentPosition() <= 900000000)
//            {
//                telemetry.addData("current encoder position", robot.leftFrontMotor.getCurrentPosition());
//                telemetry.update();
//            }
//            //robot.encoderDriveForward(.2, 5, 20000);
//
//        } catch (Exception ex) {
//            telemetry.addData("StackTrace", ex.getStackTrace());
//            telemetry.update();
//        }


        startMotor = robot.rightFrontMotor.getCurrentPosition();
        endMotor = startMotor + 600;

        //Backwards
        while (robot.rightFrontMotor.getCurrentPosition() <= endMotor) {
            robot.rightBackMotor.setPower(-robot.drivePower);
            robot.rightFrontMotor.setPower(-robot.drivePower);
            robot.leftBackMotor.setPower(-robot.drivePower);
            robot.leftFrontMotor.setPower(-robot.drivePower);
        }


        //Turn right
        startMotor = robot.rightFrontMotor.getCurrentPosition();
        endMotor = startMotor - 555;

     while (robot.rightFrontMotor.getCurrentPosition() >= endMotor) {
         robot.leftFrontMotor.setPower(robot.drivePower);
         robot.rightFrontMotor.setPower(-robot.drivePower);
         robot.leftBackMotor.setPower(robot.drivePower);
         robot.rightBackMotor.setPower(-robot.drivePower);

         robot.opMode.telemetry.addData("current rightFrontMotor encoder position: ", robot.rightFrontMotor.getCurrentPosition());
         robot.opMode.telemetry.update();
     }

        //forwards
        startMotor = robot.rightFrontMotor.getCurrentPosition();
        endMotor = startMotor - 600;

        while (robot.rightFrontMotor.getCurrentPosition() >= endMotor) {
            robot.rightBackMotor.setPower(robot.drivePower);
            robot.rightFrontMotor.setPower(robot.drivePower);
            robot.leftBackMotor.setPower(robot.drivePower);
            robot.leftFrontMotor.setPower(robot.drivePower);
        }

        //Turn left
        startMotor = robot.rightFrontMotor.getCurrentPosition();
        endMotor = startMotor + 555;

        while (robot.rightFrontMotor.getCurrentPosition() <= endMotor) {
            robot.leftFrontMotor.setPower(-robot.drivePower);
            robot.rightFrontMotor.setPower(robot.drivePower);
            robot.leftBackMotor.setPower(-robot.drivePower);
            robot.rightBackMotor.setPower(robot.drivePower);

            robot.opMode.telemetry.addData("current rightFrontMotor encoder position: ", robot.rightFrontMotor.getCurrentPosition());
            robot.opMode.telemetry.update();
        }
    }
}
