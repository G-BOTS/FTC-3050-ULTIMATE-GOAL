package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
//@Disabled
public class BlueA2Wnew extends LinearOpMode {

    // Declare OpMode members.
    HardwareUltimate robot = new HardwareUltimate();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        int count=0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()&&count>1) {
            int Decider =2;
            if (Decider ==1) {

                MecDriv(0.6, 0.6, 0.6, 0.6, 1.2); //forward
                MecDriv(-0.6, 0.6, 0.6, -0.6, 0.4);//left
                MecDriv(0.6, 0.6, 0.6, 0.6, 0.2); //forward
                //MecDriv(0.6, -0.6, -0.6, 0.6, 1.250);//Strafe right
                // MecDriv(0.6, 0.6, 0.6, 0.6,1250); //forward
                //MecDriv(0.6, 0.6, -0.6, -0.6, 1.0);// robot rotates left
                //MecDriv(-0.6, -0.6, 0.6, 0.6, 1.0);// robot rotates right}

            } else if(Decider==2) {
                MecDriv(0.6, 0.6, 0.6, 0.6, 1.2); //forward
                MecDriv(-0.6, 0.6, 0.6, -0.6, 0.4);//left
                MecDriv(0.6, 0.6, 0.6, 0.6, 1.250); //forward
                MecDriv(0.6, -0.6, -0.6, 0.6, 1.250);//Backward
                // MecDriv(0.6, 0.6, 0.6, 0.6,1250); //forward
                //MecDriv(0.6, 0.6, -0.6, -0.6, 1.0);// robot rotates left
                //MecDriv(-0.6, -0.6, 0.6, 0.6, 1.0);// robot rotates right}
            }else{
                MecDriv(0.6, 0.6, 0.6, 0.6, 1.2); //forward
                MecDriv(0.6, -0.6, -0.6, -0.6, 0.4);//right
                MecDriv(0.6, 0.6, 0.6, 0.6, 1.250); //forward
                MecDriv(-0.6, 0.6, 0.6, -0.6, 0.4);//Strafe left
                MecDriv(-0.6, -0.6, -0.6, -0.6,0.4); //backwards
               // MecDriv(0.6, 0.6, -0.6, -0.6, 1.0);// robot rotates left
               // MecDriv(-0.6, -0.6, 0.6, 0.6, 1.0);// robot rotates right}
            }




            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            //leftPower = Range.clip(drive + turn, -1.0, 1.0);
            //rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            //leftDrive.setPower(leftPower);
            //rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

    public void MecDriv(double RFP, double RBP, double LBP, double LFP, double duration) {
        runtime.reset();
        while (opModeIsActive() && runtime.time() < duration) {
            robot.rightFront.setPower(RFP);
            robot.rightBack.setPower(RBP);
            robot.leftBack.setPower(LBP);
            robot.leftFront.setPower(LFP);
            telemetry.addData("Time expired", "Run Time:"+ runtime.toString());
            telemetry.update();
        }
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.leftFront.setPower(0.0);
        sleep(1000);

    }
}