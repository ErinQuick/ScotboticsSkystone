/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class ScotBot {
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public Servo phoneRotator = null;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_MM = 70.0;     // For figuring circumference
    public static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    private ElapsedTime encoderTimeoutTimer = new ElapsedTime();
    public static final double ENCODER_TIMEOUT = 10.0;

    public static final double MIN_SERVO = 0.0;
    public static final double MAX_SERVO = 1.0;
    public static final double SERVO_DEGREES = 360.0;
    public static final double PHONE_SERVO_START = 0.5;

    public static final boolean HARDWARE_TEAM_ADDED_PHONE_SERVO = false;

    /* local OpMode members. */
    public HardwareMap hwMap = null;
    public LinearOpMode opmode = null;
    public Telemetry telemetry = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public ScotBot(HardwareMap ahwMap, Telemetry telemetry, LinearOpMode mainopmode) { // This used to be the init() function, change any code that uses it to instead use ScotBot robot = new ScotBot(hardwareMap);
        // Save reference to Hardware map
        this.hwMap = ahwMap;
        this.opmode = mainopmode;
        this.telemetry = telemetry;
        // Define and Initialize Motors
        leftFront = hwMap.get(DcMotor.class, "lf");
        rightFront = hwMap.get(DcMotor.class, "rf");
        leftBack = hwMap.get(DcMotor.class, "lb");
        rightBack = hwMap.get(DcMotor.class, "rb");
        //lehttps://www.amazon.com/Flash-Furniture-HERCULES-Folding-Carrying/dp/B018M7VEE8/ref=sr_1_28?keywords=foldable+chair&qid=1569191837&s=gateway&sr=8-28ftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        // Define and initialize ALL installed servos.
        if (HARDWARE_TEAM_ADDED_PHONE_SERVO) {
            phoneRotator = hwMap.get(Servo.class, "phoneservo");
            phoneRotator.setPosition(PHONE_SERVO_START);
        }
    }

    //x,y: direction to move from -1,-1 to 1,1
    //turn: direction to turn from -1 to 1
    public void mecanumDrive(double x, double y, double turn) {
        double angle = getAngle(x, y);
        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        double flSpeed = speed * Math.sin(angle + Math.PI / 4) + turn;
        double brSpeed = speed * Math.sin(angle + Math.PI / 4) - turn;
        double frSpeed = speed * Math.cos(angle + Math.PI / 4) - turn;
        double blSpeed = speed * Math.cos(angle + Math.PI / 4) + turn;

        leftFront.setPower(flSpeed);
        rightBack.setPower(brSpeed);
        rightFront.setPower(frSpeed);
        leftBack.setPower(blSpeed);
    }

    //Drive to relative coordinates in millimeters
    public void mecanumEncoderDrive(double x, double y, double turn, double speed, ScotBot robot) {
        int flTarget;
        int brTarget;
        int frTarget;
        int blTarget; // Target positions for wheels

        double maxDistance = Math.max(x, y);

        double normalizedX = x / maxDistance;
        double normalizedY = y / maxDistance;

        double angle = getAngle(normalizedX, normalizedY); // Angle to drive at
        double distance = Math.sqrt(Math.pow(normalizedX, 2) + Math.pow(normalizedY, 2)); // distance from 0,0 to x,y

        double flMultiplier = (distance * Math.sin(angle + Math.PI / 4) + turn);
        double brMultiplier = (distance * Math.sin(angle + Math.PI / 4) - turn);
        double frMultiplier = (distance * Math.cos(angle + Math.PI / 4) - turn);
        double blMultiplier = (distance * Math.cos(angle + Math.PI / 4) + turn); //Distance for each wheel to turn

        double totalDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        if (robot.opmode.opModeIsActive()) {
            flTarget = robot.leftFront.getCurrentPosition() + (int) (flMultiplier * COUNTS_PER_MM * totalDistance);
            brTarget = robot.rightBack.getCurrentPosition() + (int) (brMultiplier * COUNTS_PER_MM * totalDistance);
            frTarget = robot.rightFront.getCurrentPosition() + (int) (frMultiplier * COUNTS_PER_MM * totalDistance);
            blTarget = robot.leftBack.getCurrentPosition() + (int) (blMultiplier * COUNTS_PER_MM * totalDistance);

            robot.leftFront.setTargetPosition(flTarget);
            robot.rightBack.setTargetPosition(brTarget);
            robot.rightFront.setTargetPosition(frTarget);
            robot.leftBack.setTargetPosition(blTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderTimeoutTimer.reset();

            robot.leftFront.setPower(speed * flMultiplier);
            robot.rightBack.setPower(speed * brMultiplier);
            robot.rightFront.setPower(speed * frMultiplier);
            robot.leftBack.setPower(speed * blMultiplier);

            while (robot.opmode.opModeIsActive() &&
                    (encoderTimeoutTimer.seconds() < ENCODER_TIMEOUT) &&
                    (robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.leftFront.isBusy())) {

                // Display it for the driver.
                robot.telemetry.addData("Target: ", "Running to %7d,%7d,%7d,%7d", flTarget, brTarget, frTarget, blTarget);
                robot.telemetry.addData("Current: ", "Running at %7d,%7d,%7d,%7d",
                        robot.leftBack.getCurrentPosition(),
                        robot.rightBack.getCurrentPosition(),
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                robot.telemetry.addData("targetPos: ", "Going To: %7d, %7d, and turning %7d", x, y, turn);
                robot.telemetry.update();
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static double getAngle(double x, double y) {
        return (1.5 * Math.PI - Math.atan2(y, x));
    }
}