package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Disabled
public class TeleopULT2 extends LinearOpMode {
    public double slowerSpeed = .2;     //This is the variable for speed adjustment using the right bumper
    public double counter = 0;
    HardwareUltimate robot = new HardwareUltimate();   // Use  ultimate goal hardware
    ElapsedTime runtime = new ElapsedTime();

    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); // "Status" and "Initialized" display on Drivers Station
        telemetry.update();

        //Set up the Hardware (get)
        robot.init(hardwareMap);

        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode((DcMotor.RunMode.RUN_WITHOUT_ENCODER));

        robot.WobbleClaw.setPosition(-.9);
        robot.Launcher.setPosition(0.4);

        waitForStart();
        runtime.reset();
        robot.Elevator.setPower(0);

        //This is where you put the code to get the robot moving
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (slowerSpeed));
                robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (slowerSpeed));
                robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (slowerSpeed));
                robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (slowerSpeed));
            } else {
                robot.leftBack.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                robot.rightBack.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                robot.leftFront.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
                robot.rightFront.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            }
            if (gamepad1.left_bumper) {
                robot.Shooter.setPower(0.9);
            } else {
                robot.Shooter.setPower(0);
            }
            if (gamepad1.y) {
                robot.Lifter.setPosition(0.44);
                sleep(250);
                robot.Lifter.setPosition(0.58);
                sleep(250);
            } //else{
            //bot.Lifter.setPosition(0.58);
            //sleep(250);}


            if (gamepad2.x) {
                robot.WobbleClaw.setPosition(0.0);
            }

            if (gamepad2.b) {
                robot.WobbleClaw.setPosition(0.4);
            }
            if (gamepad2.dpad_left) {
                robot.ExtArm.setPower(1);
            } else if (gamepad2.dpad_right) {
                robot.ExtArm.setPower(-1);
            } else {
                robot.ExtArm.setPower(0.0);
            }

            if (gamepad2.dpad_up) {
                robot.Elevator.setPower(0.8);
            } else if (gamepad2.dpad_down) {
                robot.Elevator.setPower(-0.4);
            } else {
                robot.Elevator.setPower(0.0);
            }
            if (gamepad2.left_bumper) {
                robot.inTake.setPower(0.8);
                robot.belt.setPower(1.0);
            } else {
                robot.inTake.setPower(0.0);
                robot.belt.setPower(0.0);
            }

            if (gamepad2.right_bumper) {
                robot.inTake.setPower(-0.5);
                robot.belt.setPower(-1.0);
            } else {
                robot.inTake.setPower(0.0);
                robot.belt.setPower(0.0);
            }

            if (gamepad1.a) {
                counter = counter + 1;
                robot.leftBack.setPower(0.0);
                robot.rightBack.setPower(0.0);
                robot.leftFront.setPower(0.0);
                robot.rightFront.setPower(0.0);
                if (counter == 1) {
                    robot.Lifter.setPosition(0.44);//0.42
                    sleep(500);//1000
                    robot.Launcher.setPosition(0.6);
                    sleep(450);
                    robot.Launcher.setPosition(0.4);
                    sleep(450);

                } else if (counter == 2) {
                    robot.Lifter.setPosition(0.38);//used to be 0.368
                    sleep(250);//500
                    robot.Launcher.setPosition(0.6);
                    sleep(250);//450
                    robot.Launcher.setPosition(0.4);
                    sleep(250);//450

                } else if (counter == 3) {
                    robot.Lifter.setPosition(0.343);//used to be 0.341
                    sleep(250);//500
                    robot.Launcher.setPosition(0.6);
                    sleep(250);
                    robot.Launcher.setPosition(0.4);
                    sleep(200);
                    robot.Lifter.setPosition(0.58);
                    sleep(250);
                } else {
                    counter = 0;
                    robot.Lifter.setPosition(0.6);
                    sleep(450);
                }

            }


        }
    }
}



