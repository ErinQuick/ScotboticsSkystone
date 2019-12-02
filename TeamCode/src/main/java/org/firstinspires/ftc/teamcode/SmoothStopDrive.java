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

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.File;

/**
 * 
 */

@TeleOp(name = "Smooth Stop Drive", group = "Scotbotics")

public class SmoothStopDrive extends LinearOpMode {
   /* Declare OpMode members. */
   ScotBot robot; // Use a Scotbot's hardware
   public Gamepad currentGamepad;
   // Point to sound files on the phone's drive
   private String soundPath = "/FIRST/sounds";
   private File beepSound = new File("/sdcard" + soundPath + "/beep.mp3");
   private File pipeSound = new File("/sdcard" + soundPath + "/pipes.mp3");
   private boolean firstPress;

   double driveX;
   double driveY;
   double turn;
   double arm;
   
   private static final int EXPONENT = 3;
   private static final double SMOOTH_STOP_MULTIPLIER = 0.8; //multiplier for smooth stop, lower = smoother

   @Override
   public void runOpMode() {

      robot = new ScotBot(hardwareMap, telemetry, this);
      currentGamepad = gamepad1;
      // Send telemetry message to signify robot waiting;
      telemetry.addLine("Press start to drive");
      telemetry.addData("GamePad:", currentGamepad);
      telemetry.update();

      // Wait for the game to start (driver presses PLAY)
      waitForStart();
      // This is a two-controller op-mode, press back to transfer control to the other
      // person.;
      // The robot will beep when you transfer control.;
      // Press Y for Bag-Pipes.;
      telemetry.update();
      firstPress = true;
      // run until the end of the match (driver presses STOP)
      while (opModeIsActive()) {
         if (currentGamepad.back) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, beepSound);
            if (currentGamepad == gamepad1) {
               currentGamepad = gamepad2;
               telemetry.addData("Current Controller:", "Controller 2");
            } else {
               currentGamepad = gamepad1;
               telemetry.addData("Current Controller:", "Controller 1");
            }
            telemetry.update();
         } else if (currentGamepad.y && firstPress) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, pipeSound);
            firstPress = false;
         }
         // Run wheels in POV mode (note: The joystick goes negative when pushed
         // forwards, so negate it)
         // In this mode the Left stick moves the robot fwd and back, the Right stick
         // turns left and right.
         // This way it's also easy to just drive straight, or just turn.
         double oldX = driveX;
         double oldY = driveY;

         driveX = Math.pow(currentGamepad.right_stick_x, EXPONENT);
         driveY = Math.pow(currentGamepad.right_stick_y, EXPONENT);
         turn = Math.pow(currentGamepad.left_stick_x, EXPONENT);
         arm = Math.pow(currentGamepad.left_stick_y, EXPONENT);

         if (driveX < oldX) {
            double diff = driveX - oldX;
            diff *= SMOOTH_STOP_MULTIPLIER;

            driveX += diff;
         }

         if (driveY < oldY) {
            double diff = driveY - oldY;
            diff *= SMOOTH_STOP_MULTIPLIER;

            driveY += diff;
         }

         robot.mecanumDrive(driveX, driveY, turn);

         robot.armVertical.setPower(arm);

         if(currentGamepad.dpad_up){
            robot.baseplatePuller0.setPosition(robot.BASEPLATE_PULLER_0_UP);
            robot.baseplatePuller1.setPosition(robot.BASEPLATE_PULLER_1_UP);
         } else if (currentGamepad.dpad_down){
            robot.baseplatePuller0.setPosition(robot.BASEPLATE_PULLER_0_DOWN);
            robot.baseplatePuller1.setPosition(robot.BASEPLATE_PULLER_1_DOWN);
         }

         if (currentGamepad.right_bumper) {
            robot.armGripper.setPosition(0.5);
         } else if (currentGamepad.left_bumper) {
            robot.armGripper.setPosition(1);
         }
         // robot.armVertical.setPower(arm); //Will be used when arm is added
         // Pace this loop so jaw action is reasonable speed.
         sleep(50);
      }
   }
}
