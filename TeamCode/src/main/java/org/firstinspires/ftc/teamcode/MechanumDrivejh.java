package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class MechanumDrivejh extends LinearOpMode {
    // Declare Motors (and other Variables here)
    HardwareUltimate robot = new HardwareUltimate();   // Use  ultimate goal hardware
    ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public double slowerSpeed = .2;     //This is the variable for speed adjustment using the right bumper
    public Servo WobbleClaw;


    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized"); // "Status" and "Initialized" display on Drivers Station
        telemetry.update();

        //Set up the Hardware (get)
        robot.init(hardwareMap);

        // leftBack motor
        leftBack = hardwareMap.get(DcMotor.class, "left_Back");     // Get from Hwmap
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);        // Set Idle Behavior
        leftBack.setDirection(DcMotor.Direction.FORWARD);                      // Set Motor Rotation Direction

        // rightBack motor
        rightBack = hardwareMap.get(DcMotor.class, "right_Back");   // Get from Hwmap
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);       // Set Idle Behavior
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);               // Set Motor Rotation Direction

        // leftFront motor
        leftFront = hardwareMap.get(DcMotor.class, "left_Front");   // Get from Hwmap
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);       // Set Idle Behavior
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);               // Set Motor Rotation Direction

        // rightFront motor
        rightFront = hardwareMap.get(DcMotor.class, "right_Front");  // Get from Hwmap
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);      // Set Idle Behavior
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);              // Set Motor Rotation Direction

        //Elevator  motor
        robot.Elevator = hardwareMap.get(DcMotor.class, "ele_Vator");  // Get from Hwmap
        robot.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);      // Set Idle Behavior
        robot.Elevator.setDirection(DcMotorSimple.Direction.FORWARD);           // Set Motor Rotation Direction
        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // WobbleClaw Servo
        WobbleClaw = hardwareMap.get(Servo.class, "WobbleClaw");
        WobbleClaw.setPosition(-.9);

        waitForStart();
        runtime.reset();
        robot.Elevator.setPower(0);

        //This is where you put the code to get the robot moving
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                leftBack.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (slowerSpeed));
                rightBack.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (slowerSpeed));
                leftFront.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (slowerSpeed));
                rightFront.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (slowerSpeed));
            } else {
                leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
                rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
                leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
                rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

            }
            if (gamepad2.x) {
                WobbleClaw.setPosition(-0.9);
            }

            if (gamepad2.b) {
                WobbleClaw.setPosition(0.4);
            }
            if (gamepad2.dpad_left ){
                robot.ExtArm.setPower(1);
            } else if (gamepad2.dpad_right) {
                robot.ExtArm.setPower(-1);
            }else {
                robot.ExtArm.setPower(0.0);
            }

            if (gamepad2.dpad_up) {
                robot.Elevator.setPower(0.8);
            } else if (gamepad2.dpad_down){
                robot.Elevator.setPower(-0.4);
            }else {
                robot.Elevator.setPower(0.0);
            }


        }
    }
}


