package org.firstinspires.ftc.teamcode.UltimateGoal;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareUltimate;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below...
 */
@Autonomous
// @Disabled
public class Blue22walt extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public int Decider=1;
    //cameraName = hardwareMap.get(WebcamName.Class, "Webcam 1");
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AdKIp+T/////AAABmYPVsDPCLU20p8suBdQalWIp12ODjycJXOCcfz+/zoQ9rgkF0LSYI/Fd47Ffy4Ok2MOrj+nuH+J1/NaLHbZfno//cYR7ORMwr4ZplM6I02Ty67BT5eQ7Z4UPlb60bSkyvU+O9VpliRaoWBfGhlJvN+ZUETt6nI7WxKvZrK6mRNTZrxq/5+jpulmwQYNePSc3O1blJks8fzPVpq7CkbEJMr/DMyD/TIBcab50XvYO8UY7XGC+cdY+VxBiO2Wb8M1UCjHs7g2MHue3LWUGesHikiJXAV9izcjK+ZlCAMPXrIGjYRhrgyA3dx9nO43QjxXQ3raPYMd6c/A0ubDVa14SAp5JE0BU1EpqQIuOvq/TSbdt";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    HardwareUltimate robot = new HardwareUltimate();
    private ElapsedTime runtime = new ElapsedTime();
    public int count = 0;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //Drive Strait forward for 1.5 seconds at a a power of .6
        robot.leftBack.setPower(0.4);
        robot.rightBack.setPower(0.4);
        robot.leftFront.setPower(0.4);
        robot.rightFront.setPower(0.4);
        sleep(600);//previus 1850

        //Stop all motors and pause for 2 seconds
        robot.leftBack.setPower(.0);
        robot.rightBack.setPower(.0);
        robot.leftFront.setPower(.0);
        robot.rightFront.setPower(.0);
        sleep(1000);
        //ShootRing();



        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0) {
                            //empty list. no objects  recognized.
                            telemetry.addData("TFOD", "No Items detected.");
                            telemetry.addData("Target Zone", "A");
                            Decider=1;


                        } else {

                            // list is not empty
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                //check lable to see which target zone to go after
                                if (recognition.getLabel().equals("Single")) {
                                    telemetry.addData("Target zone", "B");
                                    Decider = 3;
                                } else if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Target zone", "C");
                                    Decider = 2;
                                } else {
                                    telemetry.addData("Target zone", "UKNOWN");
                                    Decider = 1;
                                }
                            }

                        }
                        telemetry.update();
                        // run until the end of the match (driver presses STOP)
                        while (opModeIsActive() && count < 1) {
                            // int Decider = 3;
                            if (Decider == 1) {//no rings

                                //MecDriv(-0.6, 0.6, -0.6, 0.6, 0.5); //right
                                MecDriv(0.40, 0.40, 0.43, 0.43, 1.2);//forward was 0.9
                                sleep(500);
                                ShootRing();
                                MecDriv(0.6,0.6,0.6,0.6,1.0);//forward 1.18
                                MecDriv(0.6,0.6,-0.6,-0.6,0.8);//rotate left
                                MecDriv(0.6,0.60,0.6,0.6,0.6);//forward
                                //MecDriv(0.6,-0.61,0.61,-0.6,2.15); //Strafe Left
                                //MecDriv(-0.6,-0.6,-0.62,-0.62,0.2);//backward
                                DropOfWob();
                                MecDriv(-0.6,-0.6,-0.6,-0.6,0.55);//backwords
                                MecDriv(0.6,-0.6,0.6,-0.6,2.5);//left 2.6, 2.7,
                                MecDriv(0.6,0.6,0.6,0.6,0.15);//forward
                                PickupWobbleGoal();
                               /*robot.WobbleClaw.setPosition(0.2);
                                robot.ExtArm.setPower(0.9);
                                sleep(500);
                                robot.ExtArm.setPower(0);
                                robot.WobbleClaw.setPosition(0.9);
                                robot.Elevator.setPower(0.4);
                                sleep(1000);
                                robot.Elevator.setPower(0.1);
                                robot.ExtArm.setPower(-0.9);
                                sleep(500);
                                robot.ExtArm.setPower(0);
                                robot.Elevator.setPower(0);*/
                                MecDriv(-0.6,0.6,-0.6,0.6,2.6);//right
                                MecDriv(0.6, 0.6, 0.6, 0.6, 0.3);//forward

                                DropOfWob();
                                //MecDriv(0.6, 0.6, 0.6, 0.6, 0.3);//forward
                                MecDriv(-0.6, -0.6, -0.6, -0.6, 0.1);//backward..

                            } else if (Decider == 2) {// 4 rings
                                MecDriv(-0.6, 0.6, -0.6, 0.6, 0.5);//right0.78
                                MecDriv(0.4, 0.4, 0.430, 0.430, 1.0); //forward
                                sleep(500);
                                ShootRing();
                                MecDriv(0.6,0.6,0.6,0.6,2.2);//forward
                                MecDriv(0.6,0.6,-0.6,-0.6,0.72);// rotates left
                                MecDriv(0.6, 0.6, 0.62, 0.62, 0.98); //forward 0.9
                                DropOfWob();
                                MecDriv(0.6, -0.6, 0.6, -0.6,1.60); //left
                                //MecDriv(0.6, 0.6, -0.6, -0.6, 1.0);// robot rotates left
                                //MecDriv(-0.6, -0.6, 0.6, 0.6, 1.0);// robot rotates right},,
                            } else {//1 ring

                                MecDriv(0.4, 0.4, 0.430, 0.430, 0.55); //forward0.65
                                MecDriv(0.6,-0.6,0.6,-0.6,0.19);//strafe left, was .2
                                //sleep(500);
                                ShootRing();
                                PickupRing();
                                //MecDriv(0.3,0.3,0.3,0.3,0.3);//
                                MecDriv(0.7,0.7,0.7,0.7,0.8);//forward1.75,.9,.8
                                MecDriv(0.9,-0.9,0.9,-0.9,0.15);//strafe left,was .65,0.2,0.25
                                DropOfWob();
                                MecDriv(-0.6,-0.6,-0.6,-0.6,0.3);//backwards
                                MecDriv(-0.7,-0.7,0.7,0.7,1.17);//rotate right was 1.1, 1.18
                                MecDriv(0.8, 0.8, 0.8, 0.8, 1.1); //forward was 0.9
                                //MecDriv(-0.6,0.6,-0.6,0.6,0.2);//strafe right, .1
                                //MecDriv(1.0, 1.0, -1.0, -1.0, 0.4);// robot rotates left
                                //MecDriv(0.6, 0.6, 0.6, 0.6, 0.1);// forward}
                                PickupWobbleGoal();
                                MecDriv(-1.0,-1.0,-1.0,-1.0,0.8);//backward
                                MecDriv(-1.0,-1.0,1.0,1.0,0.75);//rotates right was 0.7, 0.77
                                MecDriv(1.0,1.0,1.0,1.0,0.3);//forward
                                DropOfWob();
                                MecDriv(-1.0,-1.0,-1.0,-1.0,0.05);//backward
                            }

                        }
                    }
                }
            }

            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    public void MecDriv ( double RFP, double RBP, double LBP, double LFP, double duration){
        runtime.reset();
        while (opModeIsActive() && runtime.time() < duration) {
            robot.rightFront.setPower(RFP);
            robot.rightBack.setPower(RBP);
            robot.leftBack.setPower(LBP);
            robot.leftFront.setPower(LFP);
            telemetry.addData("Time expired", "Run Time:" + runtime.toString());
            telemetry.update();
        }
        robot.rightFront.setPower(0.0);
        robot.rightBack.setPower(0.0);
        robot.leftBack.setPower(0.0);
        robot.leftFront.setPower(0.0);
        sleep(500);
        count++;
    }
    public void DropOfWob () {
        robot.ExtArm.setPower(1.0);//.9
        sleep(600);//used to be 1s
        robot.ExtArm.setPower(0.0);
        sleep(250);
        robot.WobbleClaw.setPosition(0.2); //Drop off Wobble Goal in Target Zone A
        sleep(250);
        robot.ExtArm.setPower(-0.9);
        sleep(500);
        robot.ExtArm.setPower(0.0);
    }
    public void ShootRing(){
        //runtime.reset();
        int tel=0;
        while (opModeIsActive()  && (tel<1)) {
            robot.Shooter.setPower(0.88); //use to be .88 shot to high
            robot.Launcher.setPosition(0.4);
            sleep(2000);//2000
            robot.Lifter.setPosition(0.44);
            sleep(250);
            robot.Launcher.setPosition(0.6);
            sleep(250);
            robot.Launcher.setPosition(0.4);
            robot.Lifter.setPosition(0.58);
            sleep(200);


            robot.Lifter.setPosition(0.395);
            sleep(400);
            robot.Launcher.setPosition(0.6);
            sleep(200);
            robot.Launcher.setPosition(0.4);
            sleep(200);
            robot.Lifter.setPosition(0.58);
            sleep(200);



            robot.Lifter.setPosition(0.343);//initial value 33
            sleep(300);
            robot.Launcher.setPosition(0.6);
            sleep(200);
            robot.Launcher.setPosition(0.4);
            sleep(200);
            robot.Lifter.setPosition(0.58);
            sleep(200);
            tel=tel+1;
        }
        robot.Shooter.setPower(0.0);

    }
    public void PickupRing(){
        //while (opModeIsActive()) {
        robot.Elevator.setPower(0.7);
        sleep(200);
        robot.Elevator.setPower(0.0);
        sleep(700);
        robot.inTake.setPower(0.35);//was .65
        robot.belt.setPower(1.0);
        MecDriv(0.7,0.7,0.7,0.7,0.3);//was .lbp set to 1 instead of 0.3,.9 and .1
        sleep(2000);
        MecDriv(0.6,0.6,0.6,0.6,0.8);//0.3
        robot.inTake.setPower(0);
        robot.belt.setPower(0);
        ShootOne();

    }
    public void PickupWobbleGoal(){
        robot.WobbleClaw.setPosition(0.2);
        robot.ExtArm.setPower(0.9);
        sleep(1500);
        robot.ExtArm.setPower(0);
        robot.WobbleClaw.setPosition(0.9);
        sleep(600);
        //robot.Elevator.setPower(0.8);
        //sleep(1000);
        //robot.Elevator.setPower(0.1);
        robot.ExtArm.setPower(-0.9);
        sleep(700);
        robot.ExtArm.setPower(0);
        robot.Elevator.setPower(0);

    }
    public void ShootOne(){
        robot.Shooter.setPower(0.88); //use to be .88 shot to high
        robot.Launcher.setPosition(0.4);
        sleep(1500);//2000
        robot.Lifter.setPosition(0.343);
        sleep(300);
        robot.Launcher.setPosition(0.6);
        sleep(250);
        robot.Launcher.setPosition(0.4);
        robot.Lifter.setPosition(0.58);
        sleep(250);
        robot.Shooter.setPower(0
);

}
}

