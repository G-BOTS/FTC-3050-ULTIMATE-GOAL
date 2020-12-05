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
    /* Declare OpMode members. */
    HardwareUltimate robot = new HardwareUltimate();   // Use  ultimate goal hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP for 16 to 24 tooth sprockets
    static final double WHEEL_DIAMETER_INCHES = 2.83;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;
    static final double INTAKE_SPEED = 0.6;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.50, correction, ErAngle, TestCorrection;
    boolean aButton, bButton, touched;


    //DigitalChannel digitalTouch;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //digitalTouch = HardwareSky.get(DigitalChannel.class, "sensor_digital");

        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
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
        //telemetry.addData(  "direction",getAngle());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        telemetry.addData("direction", getAngle());
        telemetry.update();

        ElevatorUp(.4,200);
        robot.WobbleClaw.setPosition(0.6);
        gyroDrive(0.6, 0.6, 0.6, 0.6, 1850);  // drive forward

        //gyroDrive(-0.6, -0.6, -0.6, -0.6, 1000);  // drive backward

        //gyroDrive(0.6, -0.6, -0.6, 0.6, 1000);  // Strafe left

        gyroDrive(-0.6, 0.4, 0.8, -0.6, 400);  // Strafe right
        robot.ExtArm.setPower(1);
        sleep(500);
        ElevatorUp(-.04,0);
        robot.WobbleClaw.setPosition(0.4);
        robot.ExtArm.setPower(-1);
        sleep(500);

       // gyroDrive(-0.6, 0.6, -0.6, 0.6, 1000);// robot rotates left
        //gyroDrive(0.6, -0.6, 0.6, -0.6, 1000);// robot rotates right

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gyroDrive(double LBP,
                          double RBP, double LFP,
                          double RFP, int duration) {
        ErAngle = 0;
        Double LFPP = 0.0, LBPP = 0.0, RFPP = 0.0, RBPP = 0.0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            runtime.reset();
            ErAngle = 0;
            correction = 0;


            //robot.leftFront.setPower(Math.abs(speed));
            // robot.rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (runtime.seconds() <1) {
                if ((LFP > 0 && LBP > 0) && (RFP > 0 && RBP > 0)) { //check to see that the robot is driving forward
                    ErAngle = getAngle() - 0;
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RFPP = RFP;
                        RBPP = RBP;
                    }

                    if (ErAngle < 0) {
                        LFPP = LFP * (1 + correction);
                        LBPP = LBP * (1 - correction);
                        RFPP = RFP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                    }
                    if (ErAngle > 0) {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 + correction);
                        RFPP = RFP * (1 + correction);
                        RBPP = RBP * (1 - correction);
                    }
                    ErAngle = 0;
                    telemetry.addData("direction",getAngle());

                }
                if ((LFP < 0 && LBP < 0) && (RFP < 0 && RBP < 0)) { //check to see that the robot is driving backward
                    ErAngle = getAngle() - 180;
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RFPP = RFP;
                        RBPP = RBP;
                    }

                    if (ErAngle < 0) {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 - correction);
                        RFPP = RFP * (1 + correction);
                        RBPP = RBP * (1 + correction);
                    }
                    if (ErAngle > 0) {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 + correction);
                        RFPP = RFP * (1 + correction);
                        RBPP = RBP * (1 - correction);
                    }
                    ErAngle = 0;
                    telemetry.addData("direction",getAngle());

                }
                if ((LFP < 0 && LBP > 0) && (RFP > 0 && RBP < 0)) { //check to see that the robot  Strafe left
                    ErAngle = getAngle() - 270;
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RFPP = RFP;
                        RBPP = RBP;
                    }

                    if (ErAngle < 0) {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 + correction);
                        RFPP = RFP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                    }
                    if (ErAngle > 0) {
                        LFPP = LFP * (1 + correction);
                        LBPP = LBP * (1 - correction);
                        RFPP = RFP * (1 + correction);
                        RBPP = RBP * (1 - correction);
                    }
                    ErAngle = 0;
                    telemetry.addData("direction",getAngle());

                }
                if ((LFP > 0 && LBP < 0) && (RFP < 0 && RBP > 0)) { //check to see that the robot strafe right
                    ErAngle = getAngle() - 90;
                    if (ErAngle == 0) {
                        LFPP = LFP;
                        LBPP = LBP;
                        RFPP = RFP;
                        RBPP = RBP;
                    }

                    if (ErAngle < 0) {
                        LFPP = LFP * (1 - correction);
                        LBPP = LBP * (1 + correction);
                        RFPP = RFP * (1 - correction);
                        RBPP = RBP * (1 + correction);
                    }
                    if (ErAngle > 0) {
                        LFPP = LFP * (1 + correction);
                        LBPP = LBP * (1 - correction);
                        RFPP = RFP * (1 + correction);
                        RBPP = RBP * (1 - correction);
                    }
                    ErAngle = 0;
                    telemetry.addData("direction",getAngle());


                }
                robot.leftBack.setPower(LBPP);
                robot.rightBack.setPower(RBPP);
                robot.leftFront.setPower(LFPP);
                robot.rightFront.setPower(RFPP);


                // Display it for the driver.
                /*telemetry.addData("Path1", "Running to %7d :%7d");
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftFront.getCurrentPosition(),
                        robot.rightFront.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);


        }
    }


    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    /*private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }*/

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else return;

        // set power to rotate.

        robot.leftBack.setPower(leftPower);
        robot.rightBack.setPower(rightPower);


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void ElevatorUp(double elPower, int elTarget) {
        if (opModeIsActive() || robot.Elevator.isBusy()) {
            robot.Elevator.setTargetPosition(elTarget);
            robot.Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Elevator.setPower(elPower);
            while (opModeIsActive() && robot.Elevator.isBusy()) {
                telemetry.addData("horielv", robot.Elevator.getCurrentPosition());
                telemetry.update();
            }

        }
        robot.Elevator.setPower(0);
        robot.Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

   /* public void ArmOut(int outTarget) {
        if (opModeIsActive()) {
            robot.arm.setTargetPosition(outTarget);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.arm.setPower(0.25);


            while (opModeIsActive() && robot.arm.isBusy() ) {
                telemetry.addData("arm", robot.arm.getCurrentPosition());
                telemetry.update();
            }

        }
        robot.arm.setPower(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }*/

}

