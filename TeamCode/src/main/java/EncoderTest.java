import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="EncoderTest")
public class EncoderTest extends LinearOpMode {
    Robot robot = new Robot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this);
        waitForStart();
        robot.encoderDriveForward(.2, 5, 20000);

    }
}
