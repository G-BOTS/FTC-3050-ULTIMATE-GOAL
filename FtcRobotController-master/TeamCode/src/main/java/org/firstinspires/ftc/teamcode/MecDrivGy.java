package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
//@Disabled

public class MecDrivGy extends LinearOpMode {

    HardwareUltimate robot = new HardwareUltimate();   // Use  ultimate goal hardware
    private ElapsedTime runtime = new ElapsedTime();


    BNO055IMU imu;
    Orientation angles = new Orientation();
    double globalAngle, power = 0.50, correction, ErAngle, heading, angle;
    boolean aButton, bButton, touched;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        robot.Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path


        gyroDrive(0.4, 0.40, 0.40, 0.40, 5);  // drive forward

         gyroDrive(-0.6, -0.6, -0.6, -0.6, 1);  // drive backward

        //gyroDrive(0.6, -0.6, -0.6, 0.6, 1);  // Strafe left

        // gyroDrive(-0.6, 0.4, 0.8, -0.6, 1); // Strafe right


        // gyroDrive(-0.6, 0.6, -0.6, 0.6, 1000);// robot rotates left
        //gyroDrive(0.6, -0.6, 0.6, -0.6, 1000);// robot rotates right

    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gyroDrive(double LFP,
                          double LBP, double RBP,
                          double RFP, int duration) {
        ErAngle = 0;
        Double LFPP = 0.0, LBPP = 0.0, RFPP = 0.0, RBPP = 0.0;

        if (opModeIsActive()) {
            runtime.reset();
            // ErAngle = 0;
            //correction = 0;

            while (runtime.seconds() < duration) {
                if ((LFP > 0 && LBP > 0) && (RFP > 0 && RBP > 0)) { //check to see that the robot is driving forward
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    ErAngle = 0; //remove this for normal opereation
                    ErAngle = angles.firstAngle - 0;
                    correction = ErAngle / 100;
                    telemetry.addData("error", ErAngle);
                    telemetry.update();
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RBPP = RBP;
                        RFPP = RFP;
                    } else {
                        LFPP = LFP * (1 + correction);
                        LBPP = LBP * (1 + correction);
                        RBPP = RBP * (1 - correction);
                        RFPP = RFP * (1 - correction);
                    }

                    ErAngle = 0;


                }
                if ((LFP < 0 && LBP < 0) && (RFP < 0 && RBP < 0)) { //check to see that the robot is driving backward
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    ErAngle = angles.firstAngle - 180;
                    correction = ErAngle / 100;
                    telemetry.addData("error", ErAngle);
                    telemetry.update();
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RBPP = RBP;
                        RFPP = RFP;
                    }

                    else {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                        RFPP = RFP * (1 + correction);
                    }
                    ErAngle = 0;


                }
                if ((LFP < 0 && LBP > 0) && (RFP > 0 && RBP < 0)) { //check to see that the robot  Strafe left
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    ErAngle = angles.firstAngle - 90;
                    correction = ErAngle / 100;
                    telemetry.addData("error", ErAngle);
                    telemetry.update();
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RBPP = RBP;
                        RFPP = RFP;
                    }

                    else {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                        RFPP = RFP * (1 + correction);
                    }
                    ErAngle = 0;


                }
                if ((LFP > 0 && LBP < 0) && (RFP < 0 && RBP > 0)) { //check to see that the robot strafe right
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    ErAngle = angles.firstAngle - 90;
                    correction = ErAngle / 100;
                    telemetry.addData("error", ErAngle);
                    telemetry.update();
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RBPP = RBP;
                        RFPP = RFP;
                    }

                    else {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                        RFPP = RFP * (1 + correction);
                    }
                    ErAngle = 0;


                }
                robot.leftBack.setPower(LBPP);
                robot.rightBack.setPower(RBPP);
                robot.leftFront.setPower(LFPP);
                robot.rightFront.setPower(RFPP);


            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);
            sleep(2000);


        }
    }
}





