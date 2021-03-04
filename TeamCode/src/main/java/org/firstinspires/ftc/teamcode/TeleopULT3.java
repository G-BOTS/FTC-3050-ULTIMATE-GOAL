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
public class TeleopULT3 extends LinearOpMode {
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
        robot.Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.inTake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.belt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.WobbleClaw.setPosition(-0.9);
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
                robot.Shooter.setPower(0.92);
            } else {
                robot.Shooter.setPower(0);
            }
            if (gamepad1.y) {
                robot.Lifter.setPosition(0.44);
                sleep(250);
                robot.Lifter.setPosition(0.58);
                sleep(250);
            }
            if (gamepad2.left_bumper) {
                robot.inTake.setPower(0.6);
                robot.belt.setPower(1.0);

            }   else {
                robot.inTake.setPower(0.0);
                robot.belt.setPower(0.0);
            }

        }



    }
}



