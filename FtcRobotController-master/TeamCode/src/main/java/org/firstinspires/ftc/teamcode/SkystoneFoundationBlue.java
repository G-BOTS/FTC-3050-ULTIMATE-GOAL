package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.nio.file.Watchable;
import java.util.Locale;

@Autonomous
@Disabled

public class SkystoneFoundationBlue extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSky robot = new HardwareSky();   // Use  Skybot hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.5;     // This is < 1.0 if geared UP for 16 to 24 tooth sprockets
    static final double WHEEL_DIAMETER_INCHES = 2.83;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;
    static final double INTAKE_SPEED = 0.6;
    ColorSensor sensorColor;
    ColorSensor sensorColorleft;
    float[] hsvValues = {0F, 0F, 0F};// hsvValues is an array that will hold the hue, saturation, and value information.
    final float[] values = hsvValues; // values is a reference to the hsvValues array.
    final double SCALE_FACTOR = 255;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = 0.50, correction;
    int indicator = 1;
    int counter;
    float HueValue, aveHue;
    boolean aButton, bButton, touched;


    //DigitalChannel digitalTouch;

    //    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        //digitalTouch = HardwareSky.get(DigitalChannel.class, "sensor_digital");

        // digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.horiElv.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.horiElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.horiElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightElv.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        sensorColorleft.enableLed(false);


//        ColorSensor sensorColor;

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();
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

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)


            telemetry.update();



//        robot.left_hand.setPosition(0.31);
//        robot.right_hand.setPosition(0.69);
        encoderDrive(DRIVE_SPEED, 20, 20, 5.0);  // S1:  24 Drive forward 4 Sec timeout
//
        HueValue = 0;
        aveHue = 0;
        for (counter = 0; counter < 10; counter++) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            HueValue = HueValue + hsvValues[0];
        }
        aveHue = (HueValue / 10);
        if (aveHue > 80) {
            indicator = 1;

        } else {
            encoderDrive(DRIVE_SPEED, -4, -4, 5.0);
            rotate(-14, TURN_SPEED);
            encoderDrive(DRIVE_SPEED, 6, 6, 5);
//            sleep(500);
            HueValue = 0;
            aveHue = 0;
            for (counter = 0; counter < 10; counter++) {
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                HueValue = HueValue + hsvValues[0];
            }

            aveHue = (HueValue / 10);
            if (aveHue > 80) {
                indicator = 2;
            } else {
                indicator = 3;
            }
        }
        telemetry.addData("ind", indicator);
        telemetry.update();
        if (indicator == 1) {

                      robot.leftIntake.setPower(0.8);
            robot.rightIntake.setPower(-0.8);
            encoderDrive(0.3, 8, 8, 5.0);  // S1:  24 Drive forward 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -11, -11, 8.0);  // S1:  24 Drive forward 4 Sec timeout
            rotate(-69, TURN_SPEED);
            robot.pickup.setPosition(0.25);
            robot.leftIntake.setPower(0.0);
            robot.rightIntake.setPower(0.0);
            encoderDrive(DRIVE_SPEED,-52,-52,4);
            rotate(66,TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   6, 6,4.0);

            robot.left_hand.setPosition(0.31);
            robot.right_hand.setPosition(0.74);
            vertElevator(-800);//pick up elevator
            HorizontalElevator(0.95,-2500);//push out horelv
            vertElevator(-400);//drop elevator
            robot.pickup.setPosition(0.7);//open pickup
            vertElevator(-800);//pickup elevator
            HorizontalElevator(0.95,0);//pull in horelv
            vertElevator(0);//drop elevator
            sleep(250);
            rotate(-178,1.00); //turn foundation
            encoderDrive(DRIVE_SPEED,8,8,4);
            robot.left_hand.setPosition(0.8);
            robot.right_hand.setPosition(0.2);
            sleep (200);
            encoderDrive(DRIVE_SPEED,   -4, -4,4.0);
            rotate(66, TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   26, 26,4.0);
            HorizontalElevator(0.95, -2800);
            sleep(200);


        }
        if (indicator == 2) {

            robot.leftIntake.setPower(0.8);
            robot.rightIntake.setPower(-0.8);
//
            encoderDrive(0.3,2,2,5);
//

            encoderDrive(DRIVE_SPEED,-6,-6,5);
            rotate(-48,TURN_SPEED);

            robot.pickup.setPosition(0.25);
            robot.leftIntake.setPower(0.0);
            robot.rightIntake.setPower(0.0);
            encoderDrive(DRIVE_SPEED, -52, -52, 8);


            robot.leftIntake.setPower(0.0);
            robot.rightIntake.setPower(0.0);

            rotate(66,TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   6, 6,4.0);

            robot.left_hand.setPosition(0.31);
            robot.right_hand.setPosition(0.74);
            vertElevator(-800);//pick up elevator
            HorizontalElevator(0.95,-2500);//push out horelv
            vertElevator(-400);//drop elevator
            robot.pickup.setPosition(0.7);//open pickup
            vertElevator(-800);//pickup elevator
            HorizontalElevator(0.95,0);//pull in horelv
            vertElevator(0);//drop elevator
            sleep(250);
            rotate(-178,1.00); //turn foundation
            encoderDrive(DRIVE_SPEED,8,8,4);
            robot.left_hand.setPosition(0.8);
            robot.right_hand.setPosition(0.2);
            sleep (200);
            encoderDrive(DRIVE_SPEED,   -4, -4,4.0);
            rotate(66, TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   26, 26,4.0);
            HorizontalElevator(0.95, -2800);
            sleep(200);
        }
        if (indicator == 3) {
            //sensorColor.enableLed(false);
            rotate(14,TURN_SPEED);
//            encoderDrive(DRIVE_SPEED, -2, -2, 5.0);
//            rotate(-16, TURN_SPEED);
            encoderDrive(0.3, 4, 4, 5.0);
            encoderDrive(DRIVE_SPEED, -6, -6, 5.0);
            rotate(-26, TURN_SPEED);    // Turns Right to face 1st Skystone
            robot.leftIntake.setPower(0.8);   //Intakes 1st Skystone for Pattern A
            robot.rightIntake.setPower(-0.8); //Intakes 1st Skystone for Pattern A
//            rotate(10, TURN_SPEED);
            encoderDrive(0.3, 4, 4, 5.0);// Drives forward towards 1st Skystone
//            rotate(-10, TURN_SPEED);
            encoderDrive(0.3, 4, 4, 5.0);  // S1:  24 Drive forward 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -10, -10, 8.0);  // S1:  24 Drive forward 4 Sec timeout
            rotate(-36, TURN_SPEED);

            robot.pickup.setPosition(0.25);
            robot.leftIntake.setPower(0.0);
            robot.rightIntake.setPower(0.0);
            encoderDrive(DRIVE_SPEED,-54,-54,4);
            rotate(66,TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   6, 6,4.0);

            robot.left_hand.setPosition(0.31);
            robot.right_hand.setPosition(0.74);
            vertElevator(-800);//pick up elevator
            HorizontalElevator(0.95,-2500);//push out horelv
            vertElevator(-400);//drop elevator
            robot.pickup.setPosition(0.7);//open pickup
            vertElevator(-800);//pickup elevator
            HorizontalElevator(0.95,0);//pull in horelv
            vertElevator(0);//drop elevator
            sleep(250);
            rotate(-178,1.00); //turn foundation
            encoderDrive(DRIVE_SPEED,8,8,4);
            robot.left_hand.setPosition(0.8);
            robot.right_hand.setPosition(0.2);
            sleep (200);
            encoderDrive(DRIVE_SPEED,   -4, -4,4.0);
            rotate(66, TURN_SPEED);
            encoderDrive(DRIVE_SPEED,   26, 26,4.0);
            HorizontalElevator(0.95, -2800);
            sleep(200);
        }
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
//                        robot.leftDrive.getCurrentPosition(),
//                        robot.rightDrive.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void HorizontalElevator(double elPower, int elTarget) {
        if (opModeIsActive() || robot.horiElv.isBusy()) {
            robot.horiElv.setTargetPosition(elTarget);
            robot.horiElv.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.horiElv.setPower(elPower);
            while (opModeIsActive() && robot.horiElv.isBusy()) {
                telemetry.addData("horielv", robot.horiElv.getCurrentPosition());
                telemetry.update();
            }

        }
        robot.horiElv.setPower(0);
        robot.horiElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void vertElevator(int upTarget) {
        if (opModeIsActive()) {
            robot.leftElv.setTargetPosition(upTarget);
            robot.rightElv.setTargetPosition(upTarget);
            robot.leftElv.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightElv.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftElv.setPower(0.95);
            robot.rightElv.setPower(0.95);

            while (opModeIsActive() && robot.leftElv.isBusy() && robot.rightElv.isBusy()) {
                telemetry.addData("left", robot.leftElv.getCurrentPosition());
                telemetry.addData("right", robot.rightElv.getCurrentPosition());
                telemetry.update();
            }

        }
        robot.leftElv.setPower(0);
        robot.rightElv.setPower(0);
        robot.leftElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightElv.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    private double checkDirection() {
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
    }

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
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

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
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // wait for rotation to stop.
        sleep(200);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
