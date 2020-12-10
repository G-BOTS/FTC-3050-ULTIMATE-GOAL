package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous
 //@Disabled
public class BlueA2Wnew extends LinearOpMode {

    // Declare OpMode members.
    HardwareUltimate robot = new HardwareUltimate();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            MecDriv(0.6, 0.6, 0.6, 0.6,1250); //forward
            MecDriv(-0.6,-0.6,-0.6,-0.6,1250);//backward
            //MecDriv(0.6, 0.6, 0.6, 0.6,1250); //forward
            //MecDriv(-0.6,-0.6,-0.6,-0.6,1250);//backwardMec
            //MecDriv(0.6, 0.6, 0.6, 0.6,1250); //forward
           // MecDriv(-0.6,-0.6,-0.6,-0.6,1250);//backward
            MecDriv(-0.6, 0.6, -0.6, 0.6, 1000);// robot rotates left
            MecDriv(0.6, -0.6, 0.6, -0.6, 1000);// robot rotates right






            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void MecDriv(double RFP, double RBP, double LBP, double LFP, double duration) {
        runtime.reset();
        while ((opModeIsActive() &&  getRuntime()<1000)) {
            robot.rightFront.setPower(RFP);
            robot.rightBack.setPower(RBP);
            robot.leftBack.setPower(LBP);
            robot.leftFront.setPower(LFP);

        }
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
    }
}