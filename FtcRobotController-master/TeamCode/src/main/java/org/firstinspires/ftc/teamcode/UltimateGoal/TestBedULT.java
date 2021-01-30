package org.firstinspires.ftc.teamcode.UltimateGoal;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareUltimate;


@Autonomous
 @Disabled
public class TestBedULT extends LinearOpMode {
    HardwareUltimate robot = new HardwareUltimate();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        robot.init(hardwareMap);
        double enc1, enc2, tim1, tim2, ShPwr;

        waitForStart();

        robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        enc1 = 0;
        enc2 = 0;

        while (opModeIsActive()) {
            ShPwr = 0.8;
            robot.Shooter.setPower(ShPwr);
            enc1 = robot.Shooter.getCurrentPosition();
            tim1 = runtime.milliseconds();
            sleep(20);
            enc2 = robot.Shooter.getCurrentPosition();
            tim2 = runtime.milliseconds();
            //motor rpm 1620 rps 27 tics per r 103.6 therfore 2797 tics per rev
            while (opModeIsActive() && runtime.seconds() < 5) {
                if ((enc2 - enc1) / (tim2 - tim1) < 2750) {
                    ShPwr = ShPwr + 0.02;
                } else if ((enc2 - enc1) / (tim2 - tim1) > 2800) {
                    ShPwr = ShPwr - 0.02;
                } else {
                }
                robot.Shooter.setPower(ShPwr);
                enc1 = robot.Shooter.getCurrentPosition();
                tim1 = runtime.milliseconds();
                sleep(20);
                enc2 = robot.Shooter.getCurrentPosition();
                tim2 = runtime.milliseconds();
            }
        }
    }
}