package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class ITD_Basket_Colored_Auto extends LinearOpMode {
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
        clawRot = hardwareMap.get(Servo.class, "clawRot");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        freakyClaw = hardwareMap.get(Servo.class, "freakyClaw");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        outakeClaw = hardwareMap.get(Servo.class, "outakeClaw");
        outakeSwivel = hardwareMap.get(Servo.class, "outakeSwivel");

        // Initialization behavior and positions
        armL.setDirection(Servo.Direction.REVERSE);
        freakyClaw.setPosition(0.52);
        clawRot.setPosition(0.78);
        swivel.setPosition(0.04);
        armL.setPosition(0.06);
        armR.setPosition(0.02);
        outakeClaw.setPosition(0.6);
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
        clawRot.setPosition(0.78);
        freakyClaw.setPosition(0.52);
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

                .lineToLinearHeading(new Pose2d(54, 60.5, Math.toRadians(90)))
                .waitSeconds(7.5)
                //block 1
                .waitSeconds(0) // Tells the Robot to wait 1.5 seconds after completing the previous movement.
                .addTemporalMarker(0,()->{
                    slideLeft.setTargetPosition(4000);
                    slideRight.setTargetPosition(4000);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(3.5,()->{
                    outakeClaw.setPosition(.15);
                })
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
