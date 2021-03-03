package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous()
@Disabled
public class MyTest extends LinearOpMode
{ 
    HardwareUltimate robot= new HardwareUltimate();
   
    BNO055IMU   imu;
    Orientation angles = new Orientation();
    double                  angle, power = .30, correction;


   
   

    @Override
    public void runOpMode() throws InterruptedException
    { 
        robot.init(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        //Orientation.angles;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        waitForStart();


       
        sleep(100);

        //Drive Strait forward for 1.5 seconds at a a power of .3
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);

        //Strafe right for 2 seconds at a power of .3
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(-power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);

        //Drive Strait Backwards for 1.5 seconds at a a power of .3
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(-power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);


        //Strafe left for 2 seconds at a power of .3
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);
        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);

        //rotate  left for 2 seconds at a power of .3
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);
        robot.leftFront.setPower(power);
        robot.rightFront.setPower(-power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);

        //rotate  left for 2 seconds at a power of .3
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
        robot.leftFront.setPower(-power);
        robot.rightFront.setPower(power);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("heading", angles.firstAngle);
        telemetry.addData(" roll",angles.secondAngle);
        telemetry.addData("pitch",angles.thirdAngle);
        telemetry.update();
        sleep(1000);

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);


    }
}
