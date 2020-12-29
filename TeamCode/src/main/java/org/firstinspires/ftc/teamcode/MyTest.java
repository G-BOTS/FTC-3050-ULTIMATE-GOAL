package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Autonomous()
//@Disabled
public class MyTest extends LinearOpMode
{
    // Declare Motors (and other Variables here)

    public DcMotor leftBack;
    public DcMotor rightBack;
    public DcMotor leftFront;
    public DcMotor rightFront;

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

        // get the robot moving here
        waitForStart();

        //Wait for 10 seconds after autonomous starts to begin moving the robot
        sleep(10000);

        //Drive Strait forward for 1.5 seconds at a a power of .3
        leftBack.setPower(-.6);
        rightBack.setPower(-.6);
        leftFront.setPower(-.6);
        rightFront.setPower(-.6);
        sleep(2500);

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(1000);

        //Strafe Left for 2 seconds at a power of .3
        leftBack.setPower(-.6);
        rightBack.setPower(.6);
        leftFront.setPower(.6);
        rightFront.setPower(-.6);
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(1000);

        //Drive Strait Backwards for 1.5 seconds at a a power of .3
        leftBack.setPower(.6);
        rightBack.setPower(.6);
        leftFront.setPower(.6);
        rightFront.setPower(.6);
        sleep(2500);

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);
        sleep(1000);


        //Strafe Right for 2 seconds at a power of .3
        leftBack.setPower(.6);
        rightBack.setPower(-.65);
        leftFront.setPower(-.65);
        rightFront.setPower(.6);
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        leftBack.setPower(.0);
        rightBack.setPower(.0);
        leftFront.setPower(.0);
        rightFront.setPower(.0);


    }
}
