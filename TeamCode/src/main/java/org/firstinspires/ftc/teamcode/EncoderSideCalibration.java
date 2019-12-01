package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Encoder Side Calibration", group="Scotbotics")
public class EncoderSideCalibration extends LinearOpMode {
    /* Declare OpMode members. */
    ScotBot robot;   // Use a Scotbot's hardware


    @Override
    public void runOpMode() {

        robot = new ScotBot(hardwareMap, telemetry, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Calibration: ", "Press start to move right 1/2 meter!");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Starting: ", "Running Test");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        robot.mecanumEncoderDrive(500.0, 0.0, 0.0, .5);
    }
}
