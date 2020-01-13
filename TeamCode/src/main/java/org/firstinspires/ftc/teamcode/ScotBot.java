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
 *
 * ScotBot - the hardware file for the Scotbotics robot
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.List;

/**
 * This is NOT an opmode. This is a class for the robot.
 * All of the robot's hardware is defined and initialized here along with
 * important constants and functions.
 * important constants and functions.
 */
public class ScotBot {
    /* Public OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftBack = null;
    public DcMotor armVertical; //I am just initializing these variables here so it is useless to declare them as null.
    public Servo armGripper;
    public Servo phoneRotator = null;
    public Servo baseplatePuller0;
    public Servo baseplatePuller1;

    public BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;

    public int defaultArmPos;

    public static final double COUNTS_PER_MM = 6.518225; // don't tell Mr. Savage this has too many significant figures
    public static final double MECANUM_SIDE_MULTIPLIER = 2.0; //is this right? I thought mecanum was working but this looks wrong
    public static final int COUNTS_PER_FULL_TURN = 200;

    public static final double TURN_SPEED = 0.6;
    public static final double AUTO_SPEED = 0.5;
    public static final double ARM_TELEOP_SPEED = 100.0;
    public static final double ARM_POWER = 0.8;
    public static final int ARM_UP_POS = 100;
    public static final double ARM_OPEN_POS = 1.0;
    public static final double ARM_CLOSED_POS = 0.5;
    public static final double FOUNDATION_PULL_SPEED = 0.3;

    public static final double BASEPLATE_PULLER_0_DOWN = 0.0;
    public static final double BASEPLATE_PULLER_1_DOWN = 1.0;
    public static final double BASEPLATE_PULLER_0_UP = 1.0;
    public static final double BASEPLATE_PULLER_1_UP = 0.0;

    private ElapsedTime encoderTimeoutTimer = new ElapsedTime();
    public static final double ENCODER_TIMEOUT = 10.0;

    public static final double MIN_SERVO = 0.0;
    public static final double MAX_SERVO = 1.0;
    public static final double SERVO_DEGREES = 360.0;
    public static final double PHONE_SERVO_START = 0.5;

    public static final double LEGO_CENTER_OFFSET = 400.0;

    public static final boolean HARDWARE_TEAM_ADDED_PHONE_SERVO = false;

    /* local OpMode members. */
    public HardwareMap hwMap = null;
    public LinearOpMode opmode = null;
    public Telemetry telemetry = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public ScotBot(HardwareMap ahwMap, Telemetry telemetry, LinearOpMode mainopmode) { // This used to be the init() function, change any code that uses it to instead use ScotBot robot = new ScotBot(hardwareMap, telemetry, opmode);
        // Save reference to Hardware map
        this.hwMap = ahwMap;
        this.opmode = mainopmode;
        this.telemetry = telemetry;
        // Define and Initialize Motors
        // -- Drive Motors --
        leftFront = hwMap.get(DcMotor.class, "lf");
        rightFront = hwMap.get(DcMotor.class, "rf");
        leftBack = hwMap.get(DcMotor.class, "lb");
        rightBack = hwMap.get(DcMotor.class, "rb");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        // -- Arm Motors --
        armVertical = hwMap.get(DcMotor.class, "armRotaryMotor");
        armVertical.setDirection(DcMotor.Direction.FORWARD); //Maybe change to reverse in future
        // -- Servos --
        armGripper = hwMap.get(Servo.class, "armServoMotor");
        baseplatePuller0 = hwMap.get(Servo.class, "baseplatePuller0");
        baseplatePuller1 = hwMap.get(Servo.class, "baseplatePuller1");


        // Set all motors to zero power, otherwise we will have McInnisBot not ScotBot and we will have to use a stun
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODER if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //RUN_USING_ENCODERS is deprecated and your using it, therefore it is ok to use <center></center>
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //I was only using it because I copied it from the example, thanks for letting me know so I could fix it
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set default arm position
        defaultArmPos = armVertical.getCurrentPosition();

        // Define and initialize ALL installed servos.
        baseplatePuller0.setPosition(BASEPLATE_PULLER_0_UP);
        baseplatePuller1.setPosition(BASEPLATE_PULLER_1_UP);
        armGripper.setPosition(1);
        if (HARDWARE_TEAM_ADDED_PHONE_SERVO) {
            phoneRotator = hwMap.get(Servo.class, "phoneservo");
            phoneRotator.setPosition(PHONE_SERVO_START);
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    // public void calibrateArm(){
    //     boolean armButton = true;//Prevent errors until arm button is added (if at all)
    //     while(armButton == false) {
    //         armVertical.setPower(0.2);
    //     }
    //     armVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // }

    public void moveArmTo(int position){
        armVertical.setTargetPosition(position + defaultArmPos);
        encoderTimeoutTimer.reset();
        armVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armVertical.setPower(ARM_POWER);
        while (opmode.opModeIsActive() && (encoderTimeoutTimer.seconds() < ENCODER_TIMEOUT) && armVertical.isBusy()) {
            telemetry.addData("Arm Target:", position + defaultArmPos);
            telemetry.addData("Arm Current:", armVertical.getCurrentPosition());
        }
        armVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setArmOpen(boolean isOpen) {
        armGripper.setPosition(isOpen ? ARM_OPEN_POS : ARM_CLOSED_POS);
    }

    public void setPullerUp(boolean isUp) {
        baseplatePuller0.setPosition(isUp ? BASEPLATE_PULLER_0_UP: BASEPLATE_PULLER_0_DOWN);
        baseplatePuller1.setPosition(isUp ? BASEPLATE_PULLER_1_UP: BASEPLATE_PULLER_1_DOWN);
    }

    //x,y: direction to move from -1,-1 to 1,1
    //turn: direction to turn from -1 to 1
    public void mecanumDrive(double x, double y, double turn, boolean correctTurn) {
        x *= -1; //it is reversed for some reason
        double angle = getAngle(x, y);

        double speed = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); //Would manipulating this variable make the robot drive faster?
        speed *= Math.sqrt(2);//The comment above sounds really stupid but it actually kind of works, this line speeds it up.
        //If something tries to use this for a speed that is outside of the circle of the joystick, this will make the motor speeds
        //above their maximum, but if it is only for driving this is OK.

        if (Math.abs(turn) < 0.05 && correctTurn) {
           turn += getCorrectionAngle();
        }

        double flSpeed = speed * Math.sin(angle + Math.PI / 4) + turn;
        double brSpeed = speed * Math.sin(angle + Math.PI / 4) - turn;
        double frSpeed = speed * Math.cos(angle + Math.PI / 4) - turn;
        double blSpeed = speed * Math.cos(angle + Math.PI / 4) + turn;

        leftFront.setPower(flSpeed);
        rightBack.setPower(brSpeed);
        rightFront.setPower(frSpeed);
        leftBack.setPower(blSpeed);

    }

    public void mecanumDrive(double x, double y, double turn) {
       mecanumDrive(x,y,turn,false);
    }

    public void mecanumTurn(double degrees, double speed) {
       int flTarget;
       int brTarget;
       int frTarget;
       int blTarget; // Target positions for wheels

        if (opmode.opModeIsActive()) {
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            flTarget = (leftFront.getCurrentPosition() + COUNTS_PER_FULL_TURN * degrees/360.0;
            brTarget = (rightBack.getCurrentPosition() - COUNTS_PER_FULL_TURN * degrees/360.0;
            frTarget = (rightFront.getCurrentPosition() - COUNTS_PER_FULL_TURN * degrees/360.0;
            blTarget = (leftBack.getCurrentPosition() + COUNTS_PER_FULL_TURN * degrees/360.0;

            leftFront.setTargetPosition(flTarget);
            rightBack.setTargetPosition(brTarget);
            rightFront.setTargetPosition(frTarget);
            leftBack.setTargetPosition(blTarget);

            // Turn On RUN_TO_POSITION
            //
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderTimeoutTimer.reset();

            leftFront.setPower(speed * flMultiplier);
            rightBack.setPower(speed * brMultiplier);
            rightFront.setPower(speed * frMultiplier);
            leftBack.setPower(speed * blMultiplier);

            while (opmode.opModeIsActive() &&
                    (encoderTimeoutTimer.seconds() < ENCODER_TIMEOUT) &&
                    (rightFront.isBusy() && rightBack.isBusy() && leftBack.isBusy() && leftFront.isBusy())) {
                //do nothing
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            
            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    //Drive to relative coordinates in millimeters
    public void mecanumEncoderDrive(double x, double y, double turn, double speed) {

       int flTarget;
       int brTarget;
       int frTarget;
       int blTarget; // Target positions for wheels

        x *= MECANUM_SIDE_MULTIPLIER;

        double maxDistance = Math.max(Math.abs(x), Math.abs(y));

        double normalizedX = -x / maxDistance;
        double normalizedY = y / maxDistance;

        double angle = getAngle(normalizedX, normalizedY); // Angle to drive at
        double distance = Math.sqrt(Math.pow(normalizedX, 2) + Math.pow(normalizedY, 2)); // distance from 0,0 to x,y

        double flMultiplier = (distance * Math.cos(angle + Math.PI / 4) + turn);
        double brMultiplier = (distance * Math.cos(angle + Math.PI / 4) - turn);
        double frMultiplier = (distance * Math.sin(angle + Math.PI / 4) - turn);
        double blMultiplier = (distance * Math.sin(angle + Math.PI / 4) + turn); //Distance for each wheel to turn

        double totalDistance = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        telemetry.addData("Angle: ", angle);
        telemetry.addData("Total Distance: ", totalDistance);
        telemetry.update();

        if (opmode.opModeIsActive()) {

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            flTarget = (leftFront.getCurrentPosition() + (int) (flMultiplier * COUNTS_PER_MM * totalDistance)) * -1;
            brTarget = (rightBack.getCurrentPosition() + (int) (brMultiplier * COUNTS_PER_MM * totalDistance)) * -1;
            frTarget = (rightFront.getCurrentPosition() + (int) (frMultiplier * COUNTS_PER_MM * totalDistance)) * -1;
            blTarget = (leftBack.getCurrentPosition() + (int) (blMultiplier * COUNTS_PER_MM * totalDistance)) * -1;

            leftFront.setTargetPosition(flTarget);
            rightBack.setTargetPosition(brTarget);
            rightFront.setTargetPosition(frTarget);
            leftBack.setTargetPosition(blTarget);

            // Turn On RUN_TO_POSITION
            //
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            encoderTimeoutTimer.reset();

            leftFront.setPower(speed * flMultiplier);
            rightBack.setPower(speed * brMultiplier);
            rightFront.setPower(speed * frMultiplier);
            leftBack.setPower(speed * blMultiplier);

            while (opmode.opModeIsActive() &&
                    (encoderTimeoutTimer.seconds() < ENCODER_TIMEOUT) &&
                    (rightFront.isBusy() && rightBack.isBusy() && leftBack.isBusy() && leftFront.isBusy())) {

                // Display it for the driver.
               // telemetry.addData("Target: ", "Running to %7d,%7d,%7d,%7d", flTarget, brTarget, frTarget, blTarget);
               // telemetry.addData("Current: ", "Running at %7d,%7d,%7d,%7d",
               //         leftFront.getCurrentPosition(),
               //         rightBack.getCurrentPosition(),
               //         rightFront.getCurrentPosition(),
               //         leftBack.getCurrentPosition());
               // telemetry.addData("targetPos: ", "Going To: %7f, %7f, and turning %7f", x, y, turn);
               // telemetry.update();
            }

            // Stop all motion;
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);

            
            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public static double getAngle(double x, double y) {
        return (1.5 * Math.PI - Math.atan2(y, x));
    }
    //Reposition the foundation for autonomous. This is basically the entire foundation-side autonomous.
    //It is here and not in the autonomous program because it can be used twice, once for each side,
    //with the only difference being the horizontal direction.
    public void repositionFoundation(boolean isRedSide, VuforiaNav v) {
       double m = isRedSide ? -1.0 : 1.0; //multiplier for horizontal movement, short name because it is
       //used often... negative to move left on right side, positive to move right from blue left side
       mecanumEncoderDrive(685.8 * m, 0.0, 0.0, AUTO_SPEED); // move with encoders to be able to see a poster
       v.moveTo(-1435.15 * m, 1206.5,VuforiaNav.MoveMode.Y_THEN_X, VuforiaNav.VuforiaBackup.ENCODER_DRIVE, 520.75 * m, 0.0, this); //move to center of foundation w/ encoder backup
       // the second number  (^) is the vertical position of the foundation, it is currently trying to put the center of the this 9 in from the edge
       // but this should be changed as needed and the robot should start in the right position as a backup.
       setPullerUp(false);
       v.moveTo(-1600.2, 1206.5, VuforiaNav.MoveMode.X_THEN_Y, VuforiaNav.VuforiaBackup.ENCODER_DRIVE, -1206.55 * m, 0.0, this); // move back to starting position
       setPullerUp(true);
       mecanumEncoderDrive(1143.8 * m, 0.0, 0.0, AUTO_SPEED); //move back to poster visible
       v.moveTo(-1578.8, 1828.8, VuforiaNav.MoveMode.X_THEN_Y,VuforiaNav.VuforiaBackup.ENCODER_DRIVE, -650.0 * m, 622.3, this); //move back under bridge
    }

    public void repositionDragFoundation(boolean isRedSide, VuforiaNav v) { //This assumes it is behind the foundation and just drives forward to grab it.
       double m = isRedSide ? -1.0 : 1.0;
       mecanumEncoderDrive(0.0, -790.0, 0.0, AUTO_SPEED); //drive to foundation
       setPullerUp(false);
       mecanumEncoderDrive(0.0, 820.0, 0.0, FOUNDATION_PULL_SPEED); //drive back sloooowly
       setPullerUp(true);
       mecanumEncoderDrive(1289.05, 0.0, 0.0, AUTO_SPEED); //go under bridge
    }

    public void deliverSkystones(boolean isRedSide, VuforiaNav v) {
       double m = isRedSide ? -1.0 : 1.0;
       moveArmTo(ARM_UP_POS);
       setArmOpen(true);
       mecanumEncoderDrive(0.0, 762.0, 0.0, AUTO_SPEED); //drive in front of skystones
       goToSkystone(v); //drive in front of one skystone
       mecanumEncoderDrive(0.0, 304.0, 0.0, AUTO_SPEED); //drive forwards into the skystone
       moveArmTo(defaultArmPos);
       opmode.sleep(500);
       setArmOpen(false);
       mecanumEncoderDrive(0.0, -950.0, 0.0, AUTO_SPEED); //drive back
       mecanumTurn(-90.0 * m, TURN_SPEED);
       mecanumEncoderDrive(0.0, 2757.0, 0.0, AUTO_SPEED); //drive to foundation
       setArmOpen(true);
       moveArmTo(ARM_UP_POS);
       mecanumEncoderDrive(680.0 * m, 0.0, 0.0, AUTO_SPEED); //go to where poster is visible
       v.moveTo(-914.4 * m, -950.0, VuforiaNav.MoveMode.X_THEN_Y, VuforiaNav.VuforiaBackup.ENCODER_DRIVE, 0.0, -2750.0, this); //move back to legos
       mecanumTurn(90.0 * m, TURN_SPEED);
       goToSkystone(v);
       mecanumEncoderDrive(0.0, 304.0, 0.0, AUTO_SPEED); //drive forwards into the skystone
       moveArmTo(defaultArmPos);
       opmode.sleep(500);
       setArmOpen(false);
       mecanumEncoderDrive(0.0, -950.0, 0.0, AUTO_SPEED); //drive back
       mecanumTurn(-90.0 * m, TURN_SPEED);
       mecanumEncoderDrive(0.0, 2757.0, 0.0, AUTO_SPEED); //drive to foundation
       setArmOpen(true);
       moveArmTo(ARM_UP_POS);
       mecanumEncoderDrive(680.0 * m, 0.0, 0.0, AUTO_SPEED); //go to where poster is visible
       v.moveTo(-900.6 * m, 1828.8, VuforiaNav.MoveMode.X_THEN_Y, VuforiaNav.VuforiaBackup.ENCODER_DRIVE, 0.0, -850.9, this); //move back under bridge
    }

    public void resetIMUAngle() { //reset the angle of the IMU to the current angle. Bind to a button if using POV controls.
       lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

       globalAngle = 0;
    }

    public double getCorrectionAngle() { //get amount to correct movement to drive straight using IMU
       double correction, angle, gain = 0.10; //gain = how much to correct

       angle = getIMUAngle();

       if (angle == 0) {
          correction = 0;
       }else {
          correction = -angle;
       }

       correction *= gain;
       return correction;
    }

    public double getIMUAngle() { //get current angle of robot using IMU
       Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
       double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

       if (deltaAngle < -180) {
          deltaAngle += 360;
       }else if (deltaAngle > 180) {
          deltaAngle -= 360;
       }

       globalAngle += deltaAngle;

       lastAngles = angles;
       telemetry.addData("Delta Angle: ", deltaAngle);
       telemetry.addData("Current Global Angle: ", globalAngle);
       telemetry.addData("Current angle from angles: ", angles.firstAngle);
       telemetry.update();

       return globalAngle;
    }
    public void IMUTurn(int degrees, double power) {
       resetIMUAngle();

       if (degrees < 0) {
          mecanumDrive(0,0,TURN_SPEED); //turn right
       }else if (degrees > 0) {
          mecanumDrive(0,0,-TURN_SPEED); //turn left
       }
       else {return;} //direction is perfect

       if (degrees < 0) {
          while (opmode.opModeIsActive() && getIMUAngle() == 0) {} //turn off 0

          while (opmode.opModeIsActive() && getIMUAngle() > degrees) {
          } //continue turning
       }else {
          while (opmode.opModeIsActive() && getIMUAngle() < degrees) {
          }
       }


       mecanumDrive(0.0,0.0,0.0); //stop

       opmode.sleep(300); //wait for stop

       resetIMUAngle();
    }

    public void goToSkystone(VuforiaNav v) {
       List<Recognition> legos = v.getLegos(this);
       ElapsedTime legoTimer = new ElapsedTime(); //stop if it is running too long
       ElapsedTime visibleTimer = new ElapsedTime(); //change direction if nothing visible for a while
       double legoOffset = 0.0;
       boolean inFrontOfLego = false;
       double speed = 0.3;
       visibleTimer.reset();
       legoTimer.reset();
       while (!inFrontOfLego) {
          for (Recognition lego : legos) {
             if (lego.getLabel().equals("Skystone")) {
                telemetry.addLine("Found Skystone");
                double oldOffset = legoOffset;
                legoOffset = (lego.getLeft() - lego.getRight()) - LEGO_CENTER_OFFSET; 
                // if (oldOffset != null && legoOffset != null) { //if there is a null error we might need this check,
                // but doubles cant be null so we need to check a different way
                   if ((oldOffset - legoOffset) > 0) {
                      speed = Math.min((oldOffset - legoOffset) / 100.0, 0.5);
                   }else {
                      speed = Math.max((oldOffset - legoOffset) / 100.0, -0.5); // set speed to go to lego
                      //but not faster than 0.5, change the 100.0 to change how much it slows down
                   }
                // }
                visibleTimer.reset();
                break;
             }
          }
          if (legoTimer.seconds() > 5.0) {
             break; //stop after timer
          }
          if (visibleTimer.seconds() > 2.0) {
             speed = speed > 0 ? -0.3 : 0.3; //switch direction if not visible
          }
          mecanumDrive(speed,0.0,0.0);
       }
       mecanumDrive(0.0,0.0,0.0);
    }
}
