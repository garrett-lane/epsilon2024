package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "main", preselectTeleOp = "Epsilon Drive Into The Deep 24-25")
public class RRPathgen_test extends LinearOpMode {
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
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
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
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(23.58, -66.60, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-0.73, -39.38), Math.toRadians(120.35))
                .splineTo(new Vector2d(24.83, -38.55), Math.toRadians(0.92))
                .splineTo(new Vector2d(47.06, -12.78), Math.toRadians(14.04))
                .lineTo(new Vector2d(48.10, -59.95))
                .splineTo(new Vector2d(46.29, -16.20), Math.toRadians(91.19))
                .splineTo(new Vector2d(57.79, -12.85), Math.toRadians(78.44))
                .lineTo(new Vector2d(59.88, -59.25))
                .splineTo(new Vector2d(60.07, -15.28), Math.toRadians(80.36))
                .splineTo(new Vector2d(67.81, -14.65), Math.toRadians(79.91))
                .lineTo(new Vector2d(68.23, -58.60))
                .build();


        // Telemetry
        telemetry.addData("hey", "you look funny");
        telemetry.update();

        waitForStart(); // let's roll

        // Put run blocks here.
        // gets position of recognition


        // Path determinations

        drive.followTrajectorySequence(trajectory0);
    }


}
