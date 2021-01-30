package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareUltimate {
    /* Public OpMode members. */
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor Elevator = null;
    public DcMotor Shooter = null;
    //public DcMotor arm = null;
    // public DcMotor rightIntake = null;
    //public DcMotor  armDrive    = null;
    //public DcMotor  leftIntake  = null;

    public Servo WobbleClaw = null; // = null;
    public CRServo ExtArm = null;//  = null;
    public Servo Lifter = null;
    public Servo Launcher = null;
   // public DistanceSensor sDistanceL= null;
   // public DistanceSensor sDistanceR = null;
    //public Servo capston;
    //public Servo dropper;
    //public Servo serv40;
//    public Servo serv41;
//    public Servo serv42;
//    public Servo serv43;
//    public Servo serv44;
//    public Servo serv45;
//    public Servo serv24;
//    public Servo serv25;

    //public BNO055IMU imu;

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
        Elevator = hwMap.get(DcMotor.class, "ele_Vator");
        Shooter = hwMap.get(DcMotor.class, "shooter");
        //sDistanceL = hwMap.get(DistanceSensor.class, "dis_l");
       // sDistanceR = hwMap.get(DistanceSensor.class, "dis_r");


        // imu = hwMap.get(BNO055IMU.class, "imu");
        //arm = hwMap.get(DcMotor.class, "wob_Arm");
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


        Elevator.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //leftIntake.setDirection(DcMotor.Direction.FORWARD);
        //rightIntake.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        leftBack.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        Elevator.setPower(0);
        Shooter.setPower(0);
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
        ExtArm = hwMap.get(CRServo.class, "Ext_Arm");
        Lifter = hwMap.get(Servo.class, "Lift_er");
        Launcher = hwMap.get(Servo.class, "Launch");
        //capstone = hwMap.get(Servo.class, "cap_stone");
//
        WobbleClaw.setPosition(-0.8);
        Lifter.setPosition(0.58); //higer value drops the level
        Launcher.setPosition(0.5);//lower value further back 0.42
        //right_hand.setPosition(0.1);
        //pickup.setPosition(0.8);
        //capstone.setPosition(0.4);
        // this set the servos
//
    }

}
