package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous()
@Disabled
public class MyTestGy extends LinearOpMode {
    HardwareUltimate robot = new HardwareUltimate();
    BNO055IMU imu;
    Orientation angles = new Orientation();
    double angle, power = .40, correction;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }


        waitForStart();


        runtime.reset();
        while (opModeIsActive() & runtime.seconds() < 10) {
            //Drive Strait forward for 1.5 seconds at a a power of .3
            robot.leftBack.setPower(0.3);
            robot.rightBack.setPower(0.3-0.01);
            robot.leftFront.setPower(0.3);
            robot.rightFront.setPower(0.3-0.01);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("heading", angles.firstAngle);
            telemetry.addData(" roll", angles.secondAngle);
            telemetry.addData("pitch", angles.thirdAngle);
            telemetry.update();
            //sleep(100);
        }

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(500);


    }
}
