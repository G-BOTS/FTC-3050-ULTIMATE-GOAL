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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Sky: TeleopSky", group = "Pushbot")
@Disabled
public class TeleopSky extends OpMode {

    /* Declare OpMode members. */
    HardwareSky robot = new HardwareSky(); // use the class created to define a Pushbot's hardware
    //double          clawOffset  = 0.0 ;                  // Servo mid position
    //final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        robot.rightElv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftElv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Say", "Hello Driver");//
        robot.rightElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    //public class Elevator implements Runnable{

    //}

    public void loop() {
        double left;
        double right;
        int leftElvEnc;
        int rightElvEnc;
        int horElvEnc;
        double target_leftE;
        double target_rightE;
        double Elspeed;
        Elspeed = 0.6;
        Thread Elevator;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;


        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        /*if (gamepad1.left_trigger > 0.1) {
            robot.armDrive.setPower(0.2);

        } else if (gamepad1.right_trigger > 0.1) {
            robot.armDrive.setPower(-0.2);
        } else {
            robot.armDrive.setPower(0); */

        if (gamepad1.right_bumper) {
            robot.left_hand.setPosition(0.31);
            robot.right_hand.setPosition(0.74); //used to be 68
        } else if (gamepad1.left_bumper) {
            robot.left_hand.setPosition(0.8);
            robot.right_hand.setPosition(0.2);
        }

        if (gamepad1.right_trigger > 0.1) { // Trigger controls the main speed of the elevators speed
            robot.leftElv.setPower(-0.95);
            robot.rightElv.setPower(-0.95);
        } else if (gamepad1.left_trigger > 0.1) {
            robot.leftElv.setPower(0.95);
            robot.rightElv.setPower(0.95);
        } else {
            robot.leftElv.setPower(0.0);
            robot.rightElv.setPower(0.0);
        }

        /*if (gamepad1.x) {// to the left
            robot.leftIntake.setPower(0.8);
            robot.rightIntake.setPower(0.8);
        } if(gamepad1.b) {//b to the right
                robot.leftIntake.setPower(-0.8);
        robot.rightIntake.setPower(-0.8);
         }*/
        if (gamepad2.right_bumper) {
            robot.leftIntake.setPower(0.8);
            robot.rightIntake.setPower(-0.8);
        } else if (gamepad2.left_bumper) {
            robot.leftIntake.setPower(-0.8);
            robot.rightIntake.setPower(0.8);
        } else {
            robot.leftIntake.setPower(0.0);
            robot.rightIntake.setPower(0.0);
        }

        if (gamepad2.right_trigger > 0.1) {
            robot.horiElv.setPower(0.95);
        } else if (gamepad2.left_trigger > 0.1) {
            robot.horiElv.setPower(-0.8);
        } else {
            robot.horiElv.setPower(0.0);
        }

        if (gamepad2.a) {
            robot.pickup.setPosition(0.28);//.25
        } else if (gamepad2.b) {
            robot.pickup.setPosition(0.7);

        }

        if (gamepad2.x) {
            robot.capstone.setPosition(0.92);
        } else {
            robot.capstone.setPosition(0.4);
        }


        leftElvEnc = robot.leftElv.getCurrentPosition();
        rightElvEnc = robot.rightElv.getCurrentPosition();
        horElvEnc=robot.horiElv.getCurrentPosition();
        telemetry.addData("Left Elevator:", leftElvEnc);
        telemetry.addData("Right Elevator:", rightElvEnc);
        telemetry.addData("Horsontal Elevator",horElvEnc);
    }
    // Use gamepad left & right Bumpers to open and close the claw
        /*if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;*/

    // Move both servos to new position.  Assume servos are mirror image of each other.
       /* clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);*/

    // Use gamepad buttons to move the arm up (Y) and down (A)
        /*if (gamepad1.y)
            robot.leftArm.setPower(robot.ARM_UP_POWER);
        else if (gamepad1.a)
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);
        else
            robot.leftArm.setPower(0.0);*/

    // Send telemetry message to signify robot running;

    //telemetry.addData("claw",  "Offset = %.2f", clawOffset);
    //telemetry.addData("left",  "%.2f", left);
    //telemetry.addData("right", "%.2f", right);
    //}
    //}
/* public void elevator(int target_leftE, int target_rightE, double Elspeed)  {
    while (robot.rightElv.getCurrentPosition()<target_rightE){
        robot.rightElv.setPower(Elspeed);
        robot.leftElv.setPower(Elspeed);
        if (robot.rightElv.getCurrentPosition() > robot.leftElv.getCurrentPosition())  {
            robot.leftElv.setPower(Elspeed + 0.1);
        } else if(robot.rightElv.getCurrentPosition() < robot.leftElv.getCurrentPosition()){
            robot.rightElv.setPower(Elspeed + 0.1);
        }else{
            robot.rightElv.setPower(Elspeed);
            robot.leftElv.setPower(Elspeed);


        }
    }


}
        /*
         * Code to run ONCE after the driver hits STOP
         */


    // @Override
//        public void stop(){
//        }

//    }
}
