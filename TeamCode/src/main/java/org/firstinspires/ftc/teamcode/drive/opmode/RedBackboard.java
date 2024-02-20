package org.firstinspires.ftc.teamcode.drive.opmode;

// imported from blood
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;

// imported from followerPID
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class RedBackboard extends LinearOpMode {
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

    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM;
    TfodProcessor myTfodProcessor;
    float tgeLocation;
    VisionPortal myVisionPortal;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //imported from blood
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

        // Put initialization blocks here.
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Set the minimum confidence at which to keep recognitions.
        myTfodProcessor.setMinResultConfidence((float) 0.6);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setPosition(0.87);
        arm2.setPosition(0.09);
         OperateClaw(0, 0);
         OperateClaw(1, 0);
        double Distance_From_Board;
        double Slide_Hieght;
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        dump.setPosition(0.3);
        // ResetEncoder();
        telemetry.addData("Tensor Flow", "Camera Armed");
        telemetry.addData("Billiam", "Prepared");
        telemetry.update();
        // end import
        waitForStart();
        // tfod
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
        if (tgeLocation == 1) { //Location 1, Left Side

           TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(25,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                            //Limits to 30 in/s and 30 in/s^2
                    )
                    .turn(Math.toRadians(45))
                    .forward(5)
                    .back(5)
                    .strafeRight(5)
                    .turn(Math.toRadians(-135))
                    .splineTo(new Vector2d(30,-28),0)
                    .build();
            Distance_From_Board = 4; // do not set me to 0 - I will kill your code
            Slide_Hieght = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                        .addTemporalMarker(0, () -> {
                            slide.setPower(0.5);
                            slide.setTargetPosition((int) (Slide_Hieght * 385));
                        })
                        .addTemporalMarker(2, () -> {
                            arm1.setPosition(0.4);
                            arm2.setPosition(0.56);
                            dump.setPosition(0.49);
                        })
                        .waitSeconds(3)
                        .forward(Distance_From_Board,
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(10)
                                //Limits to 10 in/s and 10 in/s^2
                        )
                        .addTemporalMarker(5.5, () -> {
                            OperateClaw(0, 1);
                            OperateClaw(1, 1);
                        })
                        .waitSeconds(1.5)
                        .back(Distance_From_Board)
                        .addTemporalMarker(7.5, () -> {
                            OperateClaw(0, 0);
                            OperateClaw(1, 0);
                            arm1.setPosition(0.87);
                            arm2.setPosition(0.09);
                            dump.setPosition(0.3);
                        })
                        .addTemporalMarker(9.5, () -> {
                            slide.setPower(-0.5);
                            slide.setTargetPosition((int) (0));
                        })
                        .strafeTo(new Vector2d(30,-56))
                        .waitSeconds(4)
                        .lineTo(new Vector2d(56,-56))
                        .build();

                 drive.followTrajectorySequence(trajSeq);
                 drive.followTrajectorySequence(On_Board);
        }else if (tgeLocation == 2) { //Location 2, Middle

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(28,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                            //Limits to 30 in/s and 30 in/s^2
                    )
                    .back(2)
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(34,-34))
                    .build();
            Distance_From_Board = 4; // do not set me to 0 - I will kill your code
            Slide_Hieght = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                    .addTemporalMarker(0, () -> {
                        slide.setPower(0.5);
                        slide.setTargetPosition((int) (Slide_Hieght * 385));
                    })
                    .addTemporalMarker(2, () -> {
                        arm1.setPosition(0.4);
                        arm2.setPosition(0.56);
                        dump.setPosition(0.49);
                    })
                    .waitSeconds(3)
                    .forward(Distance_From_Board,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(10)
                            //Limits to 10 in/s and 10 in/s^2
                    )
                    .addTemporalMarker(5.5, () -> {
                        OperateClaw(0, 1);
                        OperateClaw(1, 1);
                    })
                    .waitSeconds(1.5)
                    .back(Distance_From_Board)
                    .addTemporalMarker( 7.5, () -> {
                        OperateClaw(0, 0);
                        OperateClaw(1, 0);
                        arm1.setPosition(0.87);
                        arm2.setPosition(0.09);
                        dump.setPosition(0.3);
                    })
                    .addTemporalMarker(9.5, () -> {
                        slide.setPower(-0.5);
                        slide.setTargetPosition((int) (0));
                    })
                    .strafeTo(new Vector2d(30,-56))
                    .waitSeconds(4)
                    .lineTo(new Vector2d(56,-56))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(On_Board);
        } else { //Location 3, Right Side
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .strafeRight(13,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                                    //Limits to 30 in/s and 30 in/s^2
                    )
                    .forward(24,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                                    //Limits to 30 in/s and 30 in/s^2
                    )
                    .back(15)
                    .turn(Math.toRadians(-90))
                    .lineTo(new Vector2d(38,-38))
                    .build();
            Distance_From_Board = 4; // do not set me to 0 - I will kill your code
            Slide_Hieght = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                    .addTemporalMarker(0, () -> {
                        slide.setPower(0.5);
                        slide.setTargetPosition((int) (Slide_Hieght * 385));
                    })
                    .addTemporalMarker(2, () -> {
                        arm1.setPosition(0.4);
                        arm2.setPosition(0.56);
                        dump.setPosition(0.49);
                    })
                    .waitSeconds(3)
                    .forward(Distance_From_Board,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(10)
                            //Limits to 10 in/s and 10 in/s^2
                    )
                    .addTemporalMarker(5.5, () -> {
                        OperateClaw(0, 1);
                        OperateClaw(1, 1);
                    })
                    .waitSeconds(1.5)
                    .back(Distance_From_Board)
                    .addTemporalMarker( 7.5, () -> {
                        OperateClaw(0, 0);
                        OperateClaw(1, 0);
                        arm1.setPosition(0.87);
                        arm2.setPosition(0.09);
                        dump.setPosition(0.3);
                    })
                    .addTemporalMarker(9.5, () -> {
                        slide.setPower(-0.5);
                        slide.setTargetPosition((int) (0));
                    })
                    .strafeTo(new Vector2d(30,-56))
                    .waitSeconds(4)
                    .lineTo(new Vector2d(56,-56))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(On_Board);
        }
    }
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
            tgeLocation = x;
            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }

    private void OperateClaw(int side, int status) {
        // left - bigger close smaller open
        // right - bigger open smaller close
        // Side 0 = Left; Side 1 = Right; Status 0 = Closed; Status 1 = Open
        if (side == 0 && status == 1) {
            lclaw.setPosition(0.66);
        } else if (side == 0 && status == 0) {
            lclaw.setPosition(0.81);
        } else if (side == 1 && status == 1) {
            rclaw.setPosition(0.4);
        } else if (side == 1 && status == 0) {
            rclaw.setPosition(0.27);
        }
    }
}
