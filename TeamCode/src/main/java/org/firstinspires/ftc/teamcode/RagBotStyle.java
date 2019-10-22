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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * 
 */

@TeleOp(name="RagBot Style Controls (Similar To A Drone)", group="Scotbotics")

public class RagBotStyle extends LinearOpMode {
    /* Declare OpMode members. */
    ScotBot robot;   // Use a Scotbot's hardware
    public Gamepad currentGamepad = gamepad1;

    @Override
    public void runOpMode() {

        robot = new ScotBot(hardwareMap, telemetry, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "ScotBot Is Initialized!");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Say", "ScotBot TestDrive Started!");
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(currentGamepad.back){
                if(currentGamepad == gamepad1){
                    currentGamepad = gamepad2;
                    telemetry.addData("Current Controller:", "Controller 2");
                } else {
                    currentGamepad = gamepad1;
                    telemetry.addData("Current Controller:", "Controller 1");
                }
            }
            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            double driveX = currentGamepad.right_stick_x;
            double driveY = currentGamepad.right_stick_y;
            double turn  =  currentGamepad.left_stick_x;
            double arm = currentGamepad.left_stick_y;
            robot.mecanumDrive(driveX, driveY, turn);
            //robot.armVertical.setPower(arm); //Will be used when arm is added
            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
