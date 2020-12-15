package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled

public class BlueA extends LinearOpMode
{
    // Declare Motors (and other Variables here)

    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor rightFront;
    public Servo WobbleClaw;
    public CRServo ExtArm;


    @Override
    public void runOpMode() throws InterruptedException
    {
        //Set up the Hardware (get)

        // Left Back Drive Motor
        leftBack = hardwareMap.get(DcMotor.class,"left_Back"); // Get from Hwmap
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Set Idle Behavior
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);           // Set Motor Rotation Direction

        // Right Back Drive Motor
        rightBack = hardwareMap.get(DcMotor.class,"right_Back"); // Get from Hwmap
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Set Idle Behavior
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);            // Set Motor Rotation Direction

        // Left Front Drive Motor
        leftFront = hardwareMap.get(DcMotor.class,"left_Front"); // Get from Hwmap
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);    // Set Idle Behavior
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);            // Set Motor Rotation Direction

        // Right Front Drive Motor
        rightFront = hardwareMap.get(DcMotor.class,"right_Front"); // Get from Hwmap
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);     // Set Idle Behavior
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);             // Set Motor Rotation Direction

        // WobbleClaw Servo
        WobbleClaw = hardwareMap.get(Servo.class, "WobbleClaw");
        WobbleClaw.setPosition (-.9);
        ExtArm = hardwareMap.get(CRServo.class,"Ext_Arm");

        // get the robot moving here
        waitForStart();

        //Wait for 10 seconds after autonomous starts to begin moving the robot
        //sleep(10000);

        //Drive Strait forward for 1.5 seconds at a a power of .6
        leftBack.setPower(-.6);
        rightBack.setPower(-.6);
        leftFront.setPower(-.6);
        rightFront.setPower(-.6);
        sleep(1650);//previus 1850

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(1000);

        //Strafe Right for .4 seconds at a power of .3
        leftBack.setPower(-0.6);
        rightBack.setPower(0.4);
        leftFront.setPower(0.8);
        rightFront.setPower(-0.6);
        sleep(300);

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(400);

        ExtArm.setPower(0.9);
        sleep(1000);
        ExtArm.setPower(0.0);
        sleep(250);
        WobbleClaw.setPosition (0.6); //Drop off Wobble Goal in Target Zone A
        sleep(250);
        ExtArm.setPower(-0.9);
        sleep(500);
        ExtArm.setPower(0.0);

   //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(2000);


    }
}


