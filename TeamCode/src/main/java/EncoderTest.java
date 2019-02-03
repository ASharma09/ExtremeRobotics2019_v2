
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {
    Robot robot = new Robot();
    @Override
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
        robot.encoderTurnLeft(0.3, 5000);
    }
}
