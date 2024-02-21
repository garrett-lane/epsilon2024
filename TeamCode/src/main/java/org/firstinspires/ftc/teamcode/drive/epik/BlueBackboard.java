package org.firstinspires.ftc.teamcode.drive.epik;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class BlueBackboard extends LinearOpMode {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo arm1;
    private Servo arm2;
    private DcMotor slide;
    private Servo dump;
    private Servo leftClaw;
    private Servo rightClaw;

    float tgeLocation;
    boolean USE_WEBCAM;

    private static final String[] LABELS = {"cone"};
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(10, 60, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        // tensorflow initialization
        USE_WEBCAM = true;
        initTfod();

        // motors and servos
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        slide = hardwareMap.get(DcMotor.class, "slide");
        dump = hardwareMap.get(Servo.class, "dump");
        leftClaw = hardwareMap.get(Servo.class, "lclaw");
        rightClaw = hardwareMap.get(Servo.class, "rclaw");

        // Initialization behavior and positions
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setDirection(DcMotor.Direction.FORWARD);
        dump.setPosition(0.3);
        arm1.setPosition(0.87);
        arm2.setPosition(0.09);
        OperateClaw(0, 0);
        OperateClaw(1, 0);

        // variables
        double distanceFromBoard;
        double slideHeight;

        telemetry.addData("Tensor Flow", "Camera Armed");
        telemetry.addData("Billiam", "Prepared");
        telemetry.addData("3-2-9, 3-2-9","15... 20!");
        telemetry.update();

        waitForStart(); // let's roll

        // Put run blocks here.
        // gets position of recognition
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            tgeLocation = (recognition.getLeft() + recognition.getRight()) / 2 ;
        }

        // Path determinations
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
            // default
            tgeLocation = 3;
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
                    .strafeLeft(5)
                    .turn(Math.toRadians(45))
                    .lineTo(new Vector2d(30,38))
                    .build();
            distanceFromBoard = 4; // do not set me to 0 - I will kill your code
            slideHeight = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                    .addTemporalMarker(0, () -> {
                        slide.setPower(0.5);
                        slide.setTargetPosition((int) (slideHeight * 385));
                    })
                    .addTemporalMarker(2, () -> {
                        arm1.setPosition(0.4);
                        arm2.setPosition(0.56);
                        dump.setPosition(0.49);
                    })
                    .waitSeconds(3)
                    .forward(distanceFromBoard,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(10)
                            //Limits to 10 in/s and 10 in/s^2
                    )
                    .addTemporalMarker(5.5, () -> {
                        OperateClaw(0, 1);
                        OperateClaw(1, 1);
                    })
                    .waitSeconds(1.5)
                    .back(distanceFromBoard)
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
                    .strafeTo(new Vector2d(30,56))
                    .waitSeconds(4)
                    .lineTo(new Vector2d(56,56))
                    .build();

            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(On_Board);
        } else if (tgeLocation == 2) { //Location 2, Middle

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(30,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                            //Limits to 30 in/s and 30 in/s^2
                    )
                    .back(15)
                    .turn(Math.toRadians(90))
                    .lineTo(new Vector2d(30,34))
                    .build();
            distanceFromBoard = 4; // do not set me to 0 - I will kill your code
            slideHeight = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                    .addTemporalMarker(0, () -> {
                        slide.setPower(0.5);
                        slide.setTargetPosition((int) (slideHeight * 385));
                    })
                    .addTemporalMarker(2, () -> {
                        arm1.setPosition(0.4);
                        arm2.setPosition(0.56);
                        dump.setPosition(0.49);
                    })
                    .waitSeconds(3)
                    .forward(distanceFromBoard,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(10)
                            //Limits to 10 in/s and 10 in/s^2
                    )
                    .addTemporalMarker(5.5, () -> {
                        OperateClaw(0, 1);
                        OperateClaw(1, 1);
                    })
                    .waitSeconds(1.5)
                    .back(distanceFromBoard)
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
                    .strafeTo(new Vector2d(30,56))
                    .waitSeconds(4)
                    .lineTo(new Vector2d(56,56))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(On_Board);
        } else { //Location 3, Right Side
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(25,
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(30)
                            //Limits to 30 in/s and 30 in/s^2
                    )
                    .turn(Math.toRadians(-45))
                    .forward(5)
                    .back(5)
                    .strafeLeft(5)
                    .turn(Math.toRadians(135))
                    .lineTo(new Vector2d(30,30))
                    .build();
            distanceFromBoard = 4; // do not set me to 0 - I will kill your code
            slideHeight = 4.5;
            TrajectorySequence On_Board = drive.trajectorySequenceBuilder (trajSeq.end())
                    .addTemporalMarker(0, () -> {
                        slide.setPower(0.5);
                        slide.setTargetPosition((int) (slideHeight * 385));
                    })
                    .addTemporalMarker(2, () -> {
                        arm1.setPosition(0.4);
                        arm2.setPosition(0.56);
                        dump.setPosition(0.49);
                    })
                    .waitSeconds(3)
                    .forward(distanceFromBoard,
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(10)
                            //Limits to 10 in/s and 10 in/s^2
                    )
                    .addTemporalMarker(5.5, () -> {
                        OperateClaw(0, 1);
                        OperateClaw(1, 1);
                    })
                    .waitSeconds(1.5)
                    .back(distanceFromBoard)
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
                    .strafeTo(new Vector2d(30,56))
                    .waitSeconds(4)
                    .lineTo(new Vector2d(56,56))
                    .build();
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(On_Board);
        }
    }

    private void initTfod() {

        tfod = new TfodProcessor.Builder()
                .setModelAssetName("7258v3.tflite")
                .setModelLabels(LABELS)
                .setModelInputSize(300)
                .build();

        // vision portal, aka camera
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.6f);
    }

    private void OperateClaw(int side, int status) {
        // left - bigger close smaller open
        // right - bigger open smaller close
        // Side 0 = Left; Side 1 = Right; Status 0 = Closed; Status 1 = Open
        if (side == 0 && status == 1) {
            leftClaw.setPosition(0.66);
        } else if (side == 0 && status == 0) {
            leftClaw.setPosition(0.81);
        } else if (side == 1 && status == 1) {
            rightClaw.setPosition(0.4);
        } else if (side == 1 && status == 0) {
            rightClaw.setPosition(0.27);
        }
    }
}
