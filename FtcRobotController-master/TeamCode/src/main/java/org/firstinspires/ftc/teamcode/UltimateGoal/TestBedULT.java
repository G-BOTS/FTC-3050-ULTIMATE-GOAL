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
// @Disabled
public class TestBedULT extends LinearOpMode {
    HardwareUltimate robot = new HardwareUltimate();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        robot.init(hardwareMap);
        double enc1, enc2, tim1, tim2, ShPwr, disL,disR;

        waitForStart();

        robot.Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        runtime.reset();
        enc1 = 0;
        enc2 = 0;

        while (opModeIsActive()) {
            ShPwr = 0.7;
            robot.Shooter.setPower(ShPwr);
            enc1 = robot.Shooter.getCurrentPosition();
            tim1 = runtime.milliseconds();
            sleep(20);
            enc2 = robot.Shooter.getCurrentPosition();
            tim2 = runtime.milliseconds();
            //motor rpm 1620 rps 27 tics per r 103.6 therefore 2797 tics per sec
            while (opModeIsActive() && runtime.seconds() < 10) {
                if ((enc2 - enc1) / (tim2 - tim1) < 1.750) {
                    ShPwr = ShPwr + 0.01;
                } else if ((enc2 - enc1) / (tim2 - tim1) > 1.800) {
                    ShPwr = ShPwr - 0.01;
                } else {
                }
                robot.Shooter.setPower(ShPwr);
                enc1 = robot.Shooter.getCurrentPosition();
                tim1 = runtime.milliseconds();
                sleep(200);
                enc2 = robot.Shooter.getCurrentPosition();
                tim2 = runtime.milliseconds();
                telemetry.addData("", (enc2 - enc1) / (tim2 - tim1));
                telemetry.addData("power",ShPwr);
                telemetry.update();
            }
            robot.Shooter.setPower(0);
            sleep(20);
            while   (opModeIsActive()&&runtime.seconds()<20){
               // disL=robot.sDistanceL.getDistance(DistanceUnit.CM);
               // disR=robot.sDistanceR.getDistance(DistanceUnit.CM);

            }
        }
    }
}