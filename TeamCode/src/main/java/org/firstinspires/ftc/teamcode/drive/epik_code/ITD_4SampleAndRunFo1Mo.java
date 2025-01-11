package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "main", preselectTeleOp = "Epsilon Drive Into The Deep 24-25")
public class ITD_4SampleAndRunFo1Mo extends LinearOpMode {
    // This is where we define the variables for all motors and servos. It will yell at you about how they could be local variables, ignore that, i dont care.
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo swivel;
    private Servo clawRot;
    private DcMotor slideLeft;
    private Servo freakyClaw;
    private DcMotor slideRight;
    private Servo armL;
    private Servo armR;
    private Servo outakeSwivel;
    private Servo outakeClaw;
    private DcMotor horSlide;

    private static final String[] LABELS = {"blue"};
    @Override
    public void runOpMode() {
        // Don't totally understand what this does, but it comes from the haredware map in the sample mec drive file. I think it does something with the encoder positions
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // Sets the robot's starting position and heading, assigning it to the startPose variable.
        Pose2d startPose = new Pose2d(23, 64.5, Math.toRadians(0));
        // Sets the starting pose (position)
        drive.setPoseEstimate(startPose);

        // tensorflow initialization

        // Motors and Servos - this is where the configuration is assigned. Assign motors and names in the config on the driver station first, then do it here, you will not see this in block code
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        swivel = hardwareMap.get(Servo.class, "swivel");
        freakyClaw = hardwareMap.get(Servo.class, "clawRot");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        clawRot = hardwareMap.get(Servo.class, "freakyClaw");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        outakeClaw = hardwareMap.get(Servo.class, "outakeClaw");
        outakeSwivel = hardwareMap.get(Servo.class, "outakeSwivel");
        horSlide = hardwareMap.get(DcMotor.class, "horSlide");

        // Initialization behavior and positions
        horSlide.setDirection((DcMotor.Direction.REVERSE));
        armL.setDirection(Servo.Direction.REVERSE);
        swivel.setPosition(0.04);
        armL.setPosition(0.08);
        armR.setPosition(0.04);
        outakeClaw.setPosition(.6);
        outakeSwivel.setPosition(0.4);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //slide left REVERSE here take out
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swivel.setPosition(0.04);
        freakyClaw.setPosition(0.98);
        clawRot.setPosition(0.52);
        ((DcMotorEx) slideRight).setTargetPositionTolerance(20);
        ((DcMotorEx) slideLeft).setTargetPositionTolerance(20);

        // variables

        // Trajectories
        // Left, 1
        /*
        In this section we create our trajectories during the Initilization phase. This allows the robot to run immedetly instead of taking time to build them once Auto has started.
         */
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose) //Start of a trajectory. We create a new trajectory sequence and name it "trajSeq1".
                //movement 1
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, 64.5, Math.toRadians(12.5)))
                .lineToLinearHeading(new Pose2d(51.5, 64.5, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-25, 64.75, Math.toRadians(0)))
                .waitSeconds(2.5)
                .lineToLinearHeading(new Pose2d(52, 64.5, Math.toRadians(12.5)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(51, 40.5, Math.toRadians(92)))
                .waitSeconds(1.7)
                .lineToLinearHeading(new Pose2d(60, 54.5, Math.toRadians(45)))
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(61.5, 40.5, Math.toRadians(90)))
                .waitSeconds(1.7)
                .lineToLinearHeading(new Pose2d(58, 56.5, Math.toRadians(45)))
                .waitSeconds(0.25)
                .lineToLinearHeading(new Pose2d(61.5, 40.5, Math.toRadians(132.5)))
                //block 1
                .waitSeconds(6) // Tells the Robot to wait 1.5 seconds after completing the previous movement.
                .lineToLinearHeading(new Pose2d(58, 56.5, Math.toRadians(45)))
                .waitSeconds(0.24)

                .addTemporalMarker(0,()->{//slide up
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(1.85,()->{//drop sample 1
                    outakeClaw.setPosition(.5);
                })
                .addTemporalMarker(2.25,()->{
                    clawRot.setPosition(.52);
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(2.5,()->{//slide down
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                //.addTemporalMarker(2.3,()->
                        //.lineToLinearHeading(new Pose2d(23, 62.75, Math.toRadians(0)))
                .addTemporalMarker(3,()->{//hor slides out
                    horSlide.setTargetPosition(2000);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                    swivel.setPosition(.56);
                    clawRot.setPosition(.52);
                    armR.setPosition(.13);
                    armL.setPosition(.09);
                })
                .addTemporalMarker(4.5,()->{
                    swivel.setPosition(.63);
                    armL.setPosition(.08);
                    armR.setPosition(.04);
                })
                .addTemporalMarker(4.9,()->{
                    freakyClaw.setPosition(.75);
                })
                .addTemporalMarker(5,()->{
                    armL.setPosition(.43);
                    armR.setPosition(.39);
                    swivel.setPosition(.04);
                })
                .addTemporalMarker(5.25,()->{
                    clawRot.setPosition(.52);
                    horSlide.setTargetPosition(0);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                })
                .addTemporalMarker(6,()->{
                    outakeClaw.setPosition(0.6);
                })
                .addTemporalMarker(6.175,()->{
                    freakyClaw.setPosition(.98);
                    outakeSwivel.setPosition(.5);
                })
                .addTemporalMarker(6.5,()->{
                    armL.setPosition(0.08);
                    armR.setPosition(.04);
                    outakeSwivel.setPosition(.5);
                })
                .addTemporalMarker(6.5,()->{
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(8.5,()->{
                    outakeClaw.setPosition(.5);
                })
                .addTemporalMarker(8.75,()->{
                    outakeSwivel.setPosition(.95);
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(9.25,()->{
                    clawRot.setPosition(.52);
                    horSlide.setTargetPosition(600);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                    swivel.setPosition(.56);
                    clawRot.setPosition(.52);
                    armR.setPosition(.13);
                    armL.setPosition(.09);
                })
                .addTemporalMarker(10.7,()->{
                    swivel.setPosition(.63);
                    armL.setPosition(.08);
                    armR.setPosition(.04);
                })
                .addTemporalMarker(10.95,()->{
                    freakyClaw.setPosition(.75);
                })
                .addTemporalMarker(11.05,()->{
                    armL.setPosition(.43);
                    armR.setPosition(.39);
                    swivel.setPosition(.04);
                })
                .addTemporalMarker(11.35,()->{
                    horSlide.setTargetPosition(0);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                })
                .addTemporalMarker(11.6,()->{
                    outakeClaw.setPosition(.6);
                })
                .addTemporalMarker(11.75,()->{
                    freakyClaw.setPosition(.98);
                    outakeSwivel.setPosition(.5);
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(11.85,()->{
                    armL.setPosition(.13);
                    armR.setPosition(.09);
                    swivel.setPosition(.56);
                })
                .addTemporalMarker(13.1,()->{
                    outakeClaw.setPosition(.5);
                })
                .addTemporalMarker(13.3,()->{
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(13.4,()->{
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(13.7,()->{
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(15);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(14.1,()->{
                    horSlide.setTargetPosition(600);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                    swivel.setPosition(.56);
                    clawRot.setPosition(.52);
                    armR.setPosition(.13);
                    armL.setPosition(.09);
                })
                .addTemporalMarker(14.4,()->{
                    swivel.setPosition(.63);
                    armL.setPosition(.08);
                    armR.setPosition(.04);
                })
                .addTemporalMarker(14.9,()->{
                    freakyClaw.setPosition(.75);
                })
                .addTemporalMarker(15.05,()->{
                    armL.setPosition(.43);
                    armR.setPosition(.39);
                    swivel.setPosition(.04);
                })
                .addTemporalMarker(15.35,()->{
                    horSlide.setTargetPosition(0);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                })
                .addTemporalMarker(15.6,()->{
                    outakeClaw.setPosition(.6);
                })
                .addTemporalMarker(15.75,()->{
                    freakyClaw.setPosition(.98);
                    outakeSwivel.setPosition(.5);
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(15.85,()->{
                    armL.setPosition(.13);
                    armR.setPosition(.09);
                    swivel.setPosition(.56);
                })
                .addTemporalMarker(17,()->{
                    outakeClaw.setPosition(.5);
                })
                .addTemporalMarker(17.2,()->{
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(17.4,()->{
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(17.6,()->{
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(18.1,()->{
                    horSlide.setTargetPosition(800);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                    swivel.setPosition(.56);
                    clawRot.setPosition(.72);
                    armR.setPosition(.13);
                    armL.setPosition(.09);
                })
                .addTemporalMarker(18.4,()->{
                    swivel.setPosition(.63);
                    armL.setPosition(.08);
                    armR.setPosition(.04);
                })
                .addTemporalMarker(18.9,()->{
                    freakyClaw.setPosition(.75);
                })
                .addTemporalMarker(19.05,()->{
                    armL.setPosition(.43);
                    armR.setPosition(.39);
                    swivel.setPosition(.04);
                    clawRot.setPosition(.52);
                })
                .addTemporalMarker(19.35,()->{
                    horSlide.setTargetPosition(0);
                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    horSlide.setPower(1);
                })
                .addTemporalMarker(19.6,()->{
                    outakeClaw.setPosition(.6);
                })
                .addTemporalMarker(19.75,()->{
                    freakyClaw.setPosition(.98);
                    outakeSwivel.setPosition(.5);
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })
                .addTemporalMarker(19.85,()->{
                    armL.setPosition(.13);
                    armR.setPosition(.09);
                    swivel.setPosition(.56);
                })
                .addTemporalMarker(21,()->{
                    outakeClaw.setPosition(.5);
                })
                .addTemporalMarker(21.2,()->{
                    outakeSwivel.setPosition(.95);
                })
                .addTemporalMarker(21.4,()->{
                    outakeSwivel.setPosition(.95);
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                })







//
//                .addTemporalMarker(7.75,()->{
//                    slideLeft.setTargetPosition(4000);
//                    slideRight.setTargetPosition(4000);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(1);
//                    slideRight.setPower(1);
//                })
//                .addTemporalMarker(8,()->{
//                    outakeSwivel.setPosition(.4);
//                })
//                .addTemporalMarker(10.25,()->{
//                    outakeClaw.setPosition(.5);
//                })
//                // IT STARTS HERE YOU STUPID NIGGA
//                .addTemporalMarker(11,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(1);
//                    slideRight.setPower(1);
//                })
//                .addTemporalMarker(12,()->{
//                    horSlide.setTargetPosition(400);
//                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    horSlide.setPower(1);
//                })
//                .addTemporalMarker(12.5,()->{
//                    swivel.setPosition(.63);
//                })
//                .addTemporalMarker(12.75,()->{
//                    freakyClaw.setPosition(.78);
//                })
//                .addTemporalMarker(13.5,()->{
//                    swivel.setPosition(.04);
//                    outakeSwivel.setPosition(.95);
//                    outakeClaw.setPosition(.5);
//                })
//                .addTemporalMarker(14,()->{
//                    horSlide.setTargetPosition(0);
//                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    horSlide.setPower(1);
//                })
//                .addTemporalMarker(14.75,()->{
//                    armL.setPosition(.43);
//                    armR.setPosition(.39);
//                })
//                .addTemporalMarker(15.25,()->{
//                    outakeClaw.setPosition(.6);
//                })
//                .addTemporalMarker(15.35,()->{
//                    freakyClaw.setPosition(.565);
//                })
//                .addTemporalMarker(15.5,()->{
//                    outakeClaw.setPosition(.6);
//                    armL.setPosition(.08);
//                    armR.setPosition(.04);
//                })
//                .addTemporalMarker(15.75,()->{
//                    slideLeft.setTargetPosition(4000);
//                    slideRight.setTargetPosition(4000);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(1);
//                    slideRight.setPower(1);
//                })
//                .addTemporalMarker(16,()->{
//                    outakeSwivel.setPosition(.4);
//                })
//                .addTemporalMarker(18,()->{
//                    outakeClaw.setPosition(.5);
//                })
//                // WALL BLOCK(3RD) START
//                .addTemporalMarker(18.75,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx)slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx)slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(1);
//                    slideRight.setPower(1);
//                    clawRot.setPosition(.350);
//                })
//                .addTemporalMarker(19,()->{
//                    horSlide.setTargetPosition(750);
//                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    horSlide.setPower(1);
//                })
//                .addTemporalMarker(19.75,()->{
//                    swivel.setPosition(.63);
//                })
//                .addTemporalMarker(20.25,()->{
//                    freakyClaw.setPosition(.78);
//                })
//                .addTemporalMarker(21,()->{
//                    swivel.setPosition(0.04);
//                    outakeSwivel.setPosition(.95);
//                })
//                .addTemporalMarker(22,()->{
//                    clawRot.setPosition(.52);
//                })
//                .addTemporalMarker(22.25,()->{
//                    horSlide.setTargetPosition(0);
//                    horSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    horSlide.setPower(1);
//                })
//                .addTemporalMarker(22.75,()->{
//                    armL.setPosition(.43);
//                    armR.setPosition(.39);
//                })
//                .addTemporalMarker(23.25,()->{
//                    outakeClaw.setPosition(.6);
//                })
//                .addTemporalMarker(23.35,()->{
//                    freakyClaw.setPosition(.565);
//                })
//                .addTemporalMarker(23.75,()->{
//                    armL.setPosition(.08);
//                    armR.setPosition(.04);
//                })
//                .addTemporalMarker(24,()->{
//                    slideLeft.setTargetPosition(4000);
//                    slideRight.setTargetPosition(4000);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(1);
//                    slideRight.setPower(1);
//                })
//                .addTemporalMarker(23.75,()->{
//                    outakeSwivel.setPosition(.4);
//                })
//                .addTemporalMarker(26.25,()->{
//                    outakeClaw.setPosition(.5);
//                })
//                .addTemporalMarker(17.5,()->{
//                })



//                .addTemporalMarker(0,()->{
//                    slideLeft.setTargetPosition(1350);
//                    slideRight.setTargetPosition(1350);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })
//                .addTemporalMarker(1.75,()->{
//                    slideLeft.setTargetPosition(800);
//                    slideRight.setTargetPosition(800);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                    outakeSwivel.setPosition(.15);
//                })
//                .addTemporalMarker(2,()->{
//                    outakeClaw.setPosition(.45);
//                })
//                .addTemporalMarker(3,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                    //block 2
//
//                })
//                .addTemporalMarker(10,()->{
//                    outakeClaw.setPosition(.6);
//                })
//                .addTemporalMarker(11,()->{
//                    slideLeft.setTargetPosition(1400);
//                    slideRight.setTargetPosition(1400);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })
//                .addTemporalMarker(12.25,()->{
//                    slideLeft.setTargetPosition(800);
//                    slideRight.setTargetPosition(800);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                    outakeSwivel.setPosition(.15);
//                })
//                .addTemporalMarker(12.5,()->{
//                    outakeClaw.setPosition(.45);
//                })
//                .addTemporalMarker(13,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })
//                .addTemporalMarker(13.75,()->{
//                    outakeSwivel.setPosition(.15);
//                    outakeClaw.setPosition(.45);
//                })
//                .addTemporalMarker(14,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })
//                //block 3
//                .addTemporalMarker(16.25,()->{
//                    outakeClaw.setPosition(.6);
//                })
//                .addTemporalMarker(16.5,()->{
//                    slideLeft.setTargetPosition(1400);
//                    slideRight.setTargetPosition(1400);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })
//                .addTemporalMarker(19.5,()->{
//                    slideLeft.setTargetPosition(800);
//                    slideRight.setTargetPosition(800);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                    outakeSwivel.setPosition(.15);
//                })
//                .addTemporalMarker(19.75,()->{
//                    outakeClaw.setPosition(.45);
//                })
//                .addTemporalMarker(20.5,()->{
//                    slideLeft.setTargetPosition(0);
//                    slideRight.setTargetPosition(0);
//                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
//                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
//                    slideLeft.setPower(0.75);
//                    slideRight.setPower(0.75);
//                })




//                .addTemporalMarker(12.5,()->{
                //.addTemporalMarker(12.5,()->{
                //                })
                //.addTemporalMarker(11,()->{
                //slide.setPower(0.5);
                //slide.setTargetPosition((int) (4.5 * 385));
                //})
                //.addTemporalMarker(13,()->{
                //arm1.setPosition(0.4);
                //arm2.setPosition(0.56);
                //dump.setPosition(0.49);
                //})
                //.waitSeconds(4)
                //.lineTo(new Vector2d(34, 44),
                //SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(25)
                //)
                .build();

        // Telemetry
        telemetry.addData("hey", "you look funny");
        telemetry.update();

        waitForStart(); // let's roll

        // Put run blocks here.
        // gets position of recognition


        // Path determinations

        drive.followTrajectorySequence(trajSeq1);
    }


}
