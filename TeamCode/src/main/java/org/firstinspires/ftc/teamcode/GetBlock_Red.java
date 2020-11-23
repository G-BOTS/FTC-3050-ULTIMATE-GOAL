package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous
@Disabled

public class GetBlock_Red extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSky robot   = new HardwareSky();   // Use  Skybot hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 2.83 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        encoderDrive(DRIVE_SPEED,   -18, -18, 4.0);  // S1: Drive forward 4 Sec timeout
        /*robot.armDrive.setTargetPosition(415);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armDrive.setPower(0.3);
        sleep(1500);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armDrive.setPower(0.0);
        encoderDrive(DRIVE_SPEED,   15, 15, 4.0);  // S2: hook foundationand drive backwards  with 4 Sec timeout
        robot.armDrive.setTargetPosition(0);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armDrive.setPower(0.3);
        sleep(1500);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armDrive.setPower(0.0);*/
        encoderDrive(DRIVE_SPEED,   6, 6, 5.0);  // S3: Turn Right 6 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -9.71, 9.71, 5.0);  // S4: forward 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  108,  -108 , 5.0);  // S5: Turn Left 6 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,   -32, -32, 5.0);  // S6: forward 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, 9.71, -9.71, 5.0);  // S7: Turn Left 6 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,  -6,  -6 , 5.0);  // S8: Forward 24 Inches with 5 Sec timeout
        encoderDrive(DRIVE_SPEED,   -12, -12, 5.0);  // S9: Turn left 12 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, 9.71, -9.71, 5.0);  // S10: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -24, -24, 5.0);  // S11: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   -9.71, 9.71, 5.0);  // S12:
        encoderDrive(DRIVE_SPEED, -48, -48, 5.0);  // S13: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED,   -9.71, 9.71, 5.0);  // S12:
        encoderDrive(DRIVE_SPEED, -24, -24, 5.0);  // S10: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -9.71, 9.71, 5.0);  // S11: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,   -20, -20, 5.0);  // S12:
        encoderDrive(DRIVE_SPEED, 6, 6, 5.0);  // S13: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -9.71, 9.71, 5.0);  // S11: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,   -48, -48, 5.0);  // S12:


        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
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
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
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
}

