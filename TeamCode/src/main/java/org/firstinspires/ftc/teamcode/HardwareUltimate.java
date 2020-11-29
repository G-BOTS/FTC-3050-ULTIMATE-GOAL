package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareUltimate {
    /* Public OpMode members. */
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
   // public DcMotor horiElv = null;
   // public DcMotor leftIntake = null;
   // public DcMotor rightIntake = null;
    //public DcMotor  armDrive    = null;
    //public DcMotor  leftIntake  = null;

    public Servo WobbleClaw; // = null;
    //public Servo right_hand;//  = null;
    //public Servo pickup;
    //public Servo capstone;
    //public Servo dropper;
    //public Servo serv40;
//    public Servo serv41;
//    public Servo serv42;
//    public Servo serv43;
//    public Servo serv44;
//    public Servo serv45;
//    public Servo serv24;
//    public Servo serv25;

    //public ColorSensor sensorColor;
    //public ColorSensor sensorColorleft;

    // public DistanceSensor sensorRange;

    // public TouchSensor sensorTouch;

    //DigitalChannel digitalTouch;

    /* public DcMotor  leftArm     = null;
    public Servo    leftClaw    = null;
    public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;*/

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();


    /* Constructor */
    public HardwareUltimate() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


// Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        leftBack = hwMap.get(DcMotor.class, "left_Back");
        rightBack = hwMap.get(DcMotor.class, "right_Back");
        leftFront = hwMap.get(DcMotor.class, "left_Front");
        rightFront = hwMap.get(DcMotor.class, "right_Front");
        //rightIntake = hwMap.get(DcMotor.class, "right_intake");
        //leftElv = hwMap.get(DcMotor.class, "left_elevator");
        //rightElv = hwMap.get(DcMotor.class, "right_elevator");
        //armDrive = hwMap.get(DcMotor.class, "arm_drive" );

        //digitalTouch = hwMap.get(DigitalChannel.class, "sensor_digital");
        //sensorColor = hwMap.get(ColorSensor.class, "sensor_color_distance");
        //sensorColorleft = hwMap.get(ColorSensor.class,"sensor_color_left");
//
        leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // Set Idle Behavior
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //horiElv.setDirection(DcMotor.Direction.FORWARD);
        //leftIntake.setDirection(DcMotor.Direction.FORWARD);
        //rightIntake.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        //horiElv.setPower(0);
        //leftIntake.setPower(0);
        //rightIntake.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //horiElv.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        WobbleClaw = hwMap.get(Servo.class, "WobbleClaw");
        //right_hand = hwMap.get(Servo.class, "right_hand");
        //pickup = hwMap.get(Servo.class, "pick_up");
        //capstone = hwMap.get(Servo.class, "cap_stone");
//
        WobbleClaw.setPosition (-.8);
        //left_hand.setPosition(0.95);
        //right_hand.setPosition(0.1);
       //pickup.setPosition(0.8);
        //capstone.setPosition(0.4);
        // this set the servos
//
    }

}