package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous(name = "BloodBlackstageLAZER (Blocks to Java)", group = "AA")
public class BloodBlackstageLAZER extends LinearOpMode {

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private Servo arm1;
    private Servo arm2;
    private DcMotor slide;
    private Servo dump;
    private Servo lclaw;
    private Servo rclaw;
    private DistanceSensor DistanceSensor_DistanceSensor;

    VisionPortal.Builder myVisionPortalBuilder;
    double Current_Distance_from_board;
    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    AprilTagProcessor myAprilTagProcessor;
    int tgeLocation;
    double MainSpeed;
    VisionPortal myVisionPortal;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        dump = hardwareMap.get(Servo.class, "dump");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        DistanceSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");

        // Put initialization blocks here.
        Current_Distance_from_board += 0;
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.6);
        MainSpeed = 0.2;
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setPosition(0.87);
        arm2.setPosition(0.09);
        OperateClaw(0, 0);
        OperateClaw(1, 0);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        dump.setPosition(0.3);
        ResetEncoder();
        telemetry.addData("Tensor Flow", "Camera Armed");
        telemetry.addData("Billiam", "Prepared");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            telemetryTfod();
            telemetry.update();
            // Put run blocks here.
            if (tgeLocation < 150) {
                tgeLocation = 1;
                telemetry.addData("location", "1");
            } else if (151 < tgeLocation && tgeLocation < 474) {
                tgeLocation = 2;
                telemetry.addData("location", "2");
            } else if (475 < tgeLocation) {
                tgeLocation = 3;
                telemetry.addData("location", "3");
            } else {
                telemetry.addData("location", tgeLocation);
            }
            telemetry.update();
            if (tgeLocation == 1) {
                // forward 1
                MoveRobot(66.04, MainSpeed);
                StrafeRobot(-9, MainSpeed);
                TurnRobot(-46, MainSpeed);
                MoveRobot(5, MainSpeed);
                MoveRobot(-5, MainSpeed);
                StrafeRobot(20, MainSpeed);
                TurnRobot(46, MainSpeed);
                StrafeRobot(10, MainSpeed);
                // 180
                TurnRobot(92, MainSpeed);
                // strafe left to align
                StrafeRobot(-9, MainSpeed);
                MoveRobot(7, MainSpeed);
                Place_On_Board(29, 4.7);
                // strafe righ 1
                StrafeRobot(75, MainSpeed);
                // forward 1
                MoveRobot(80, MainSpeed);
                // time to brake
                sleep(1000);
            } else if (tgeLocation == 2) {
                // forward 1
                MoveRobot(66.04, MainSpeed);
                MoveRobot(11, MainSpeed);
                MoveRobot(-11, MainSpeed);
                // turn left 90
                TurnRobot(92, MainSpeed);
                // Strafe to 2nd position
                StrafeRobot(-4, MainSpeed);
                MoveRobot(30, MainSpeed);
                Place_On_Board(24, 4.5);
                sleep(700);
                // strafe righ 1
                StrafeRobot(60, MainSpeed);
                // forward 1
                MoveRobot(100, MainSpeed);
                // time to brake
                sleep(1000);
            } else {
                // location 3
                // right
                StrafeRobot(28, MainSpeed);
                // forward 1
                MoveRobot(65, MainSpeed);
                MoveRobot(-25, MainSpeed);
                StrafeRobot(27, MainSpeed);
                MoveRobot(20, MainSpeed);
                // right turn
                TurnRobot(92, MainSpeed);
                // strafe right to 3rd position
                StrafeRobot(6, MainSpeed);
                MoveRobot(-13, MainSpeed);
                Distance_Sensor_Auto_correct(20);
                Place_On_Board(20, 4.5);
                sleep(100);
                // strafe righ 1
                StrafeRobot(40, MainSpeed);
                // forward 1
                MoveRobot(60, MainSpeed);
                sleep(1000);
            }
            sleep(30000);
        }
    }

    /**
     * Describe this function...
     */
    private void OperateClaw(int side, int status) {
        // left - bigger close smaller open
        // right - bigger open smaller close
        // Side 0 = Left; Side 1 = Right; Status 0 = Closed; Status 1 = Open
        if (side == 0 && status == 0) {
            lclaw.setPosition(0.66);
        } else if (side == 0 && status == 1) {
            lclaw.setPosition(0.81);
        } else if (side == 1 && status == 0) {
            rclaw.setPosition(0.4);
        } else if (side == 1 && status == 1) {
            rclaw.setPosition(0.27);
        }
    }

    /**
     * Describe this function...
     */
    private void Place_On_Board(int Distance_From_Board, double Slide_Hieght) {
        // erect sliderail
        slide.setPower(0.5);
        slide.setTargetPosition((int) (Slide_Hieght * 385));
        sleep(1500);
        arm1.setPosition(0.4);
        arm2.setPosition(0.56);
        sleep(1500);
        dump.setPosition(0.49);
        sleep(500);
        MoveRobot(Distance_From_Board, MainSpeed);
        MoveRobot(5, MainSpeed);
        sleep(500);
        OperateClaw(0, 1);
        OperateClaw(1, 1);
        sleep(500);
        // reverse?
        MoveRobot(-Distance_From_Board, MainSpeed);
        OperateClaw(0, 0);
        OperateClaw(1, 0);
        dump.setPosition(0.3);
        sleep(1500);
        arm1.setPosition(0.87);
        arm2.setPosition(0.09);
        sleep(2500);
        slide.setPower(-0.5);
        slide.setTargetPosition(0 * 385);
    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("7258v3.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("cone"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod() {
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.
            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
            tgeLocation = (int) x;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }

    /**
     * Describe this function...
     */
    private void telemetryAprilTag() {
        List<AprilTagDetection> myAprilTagDetections;
        AprilTagDetection myAprilTagDetection;

        // Get a list of AprilTag detections.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
        // Iterate through list and call a function to display info for each recognized AprilTag.
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.
            telemetry.addLine("");
            if (myAprilTagDetection.metadata != null) {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
                telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.z, 6, 1) + "  (inch)");
                telemetry.addLine("PRY " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.pitch, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.roll, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1) + "  (deg)");
                telemetry.addLine("RBE " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.range, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.bearing, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.elevation, 6, 1) + "  (inch, deg, deg)");
            } else {
                telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
                telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    /**
     * Describe this function...
     */
    private void initAprilTag() {
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;

        // First, create an AprilTagProcessor.Builder.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        // Create an AprilTagProcessor by calling build.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Describe this function...
     */
    private void MoveRobot(double dist, double speed) {
        ResetEncoder();
        // Wheel Directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        // Set target positions to dist
        fl.setTargetPosition((int) (dist * 12.26));
        fr.setTargetPosition((int) (dist * 12.26));
        bl.setTargetPosition((int) (dist * 12.26));
        br.setTargetPosition((int) (dist * 12.26));
        RunToPos();
        // set power to speed
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);
        while (fr.isBusy()) {
        }
        ResetEncoder();
    }

    /**
     * Describe this function...
     */
    private void StrafeRobot(int dist, double speed) {
        ResetEncoder();
        // Wheel Directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        // Set target positions to dist
        fl.setTargetPosition((int) (dist * 14.18));
        fr.setTargetPosition((int) (dist * 14.18));
        bl.setTargetPosition((int) (dist * 14.18));
        br.setTargetPosition((int) (dist * 14.18));
        RunToPos();
        // set power to speed
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);
        while (fr.isBusy()) {
        }
        ResetEncoder();
    }

    /**
     * Describe this function...
     */
    private void TurnRobot(int dist, double speed) {
        ResetEncoder();
        // Wheel Directions
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Set target positions to dist
        fl.setTargetPosition((int) (dist * 8.245));
        fr.setTargetPosition((int) (dist * 8.245));
        bl.setTargetPosition((int) (dist * 8.245));
        br.setTargetPosition((int) (dist * 8.245));
        RunToPos();
        // set power to speed
        fl.setPower(speed);
        fr.setPower(speed);
        bl.setPower(speed);
        br.setPower(speed);
        while (fr.isBusy()) {
        }
        ResetEncoder();
    }

    /**
     * Describe this function...
     */
    private void RunToPos() {
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Describe this function...
     */
    private void ResetEncoder() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Describe this function...
     */
    private void Distance_Sensor_Auto_correct(int Intended_distance) {
        Current_Distance_from_board = DistanceSensor_DistanceSensor.getDistance(DistanceUnit.INCH);
        // telemetry.addData(Current_Distance_from_board, tgeLocation);
        MoveRobot(Current_Distance_from_board - Intended_distance, MainSpeed);
        telemetry.update();
    }
}