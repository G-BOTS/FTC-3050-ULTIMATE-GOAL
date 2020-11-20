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
//@Disabled

public class BlueColorCAL extends LinearOpMode {
    /* Declare OpMode members. */
    HardwareSky robot = new HardwareSky();   // Use  Skybot hardware
    private ElapsedTime runtime = new ElapsedTime();


    ColorSensor sensorColor;
    ColorSensor sensorColorleft;
    float[] hsvValues = {0F, 0F, 0F};// hsvValues is an array that will hold the hue, saturation, and value information.
    final float[] values = hsvValues; // values is a reference to the hsvValues array.
    final double SCALE_FACTOR = 255;
int counter;

    float HueValue, aveHue;

    //    @Override
    public void runOpMode() {


        robot.init(hardwareMap);
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
//        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_left");


        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


//


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
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
            // send the info back to driver station using telemetry function.
//            telemetry.addData("Distance (cm)",
//                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("ave", aveHue);

            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
//            relativeLayout.post(new Runnable() {
//                public void run() {
//                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));


            telemetry.update();
        }




        HueValue = 0;
        aveHue = 0;
        for (counter = 0; counter < 10; counter++) {
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);
            HueValue = HueValue + hsvValues[0];
            telemetry.addData("Hue",HueValue);
            telemetry.update();
        }
    }
}

