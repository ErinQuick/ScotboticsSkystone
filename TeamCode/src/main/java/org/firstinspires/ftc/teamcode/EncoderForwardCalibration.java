package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Encoder Forward Calibration", group="Scotbotics")
public class EncoderForwardCalibration extends LinearOpMode {
    /* Declare OpMode members. */
    ScotBot robot;   // Use a Scotbot's hardware


    @Override
    public void runOpMode() {

        robot = new ScotBot(hardwareMap, telemetry, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Calibration: ", "Press start to move forward 1/2 meter");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Starting: ", "Running Test");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        robot.mecanumEncoderDrive(0.0, 500.0, 0.0, .5);
    }
}
