package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "main", preselectTeleOp = "Epsilon Drive Into The Deep 24-25")
public class ITD_3_Spec_Auto extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-23, 64.5, Math.toRadians(-90));
        // Sets the starting pose (position)
        drive.setPoseEstimate(startPose);

        // tensorflow initialization

        // Motors and Servos - this is where the configuration is assigned. Assign motors and names in the config on the driver station first, then do it here, you will not see this in block code
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        swivel = hardwareMap.get(Servo.class, "swivel");
        clawRot = hardwareMap.get(Servo.class, "freakyClaw");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        freakyClaw = hardwareMap.get(Servo.class, "clawRot");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");
        outakeClaw = hardwareMap.get(Servo.class, "outakeClaw");
        outakeSwivel = hardwareMap.get(Servo.class, "outakeSwivel");

        // Initialization behavior and positions
        armL.setDirection(Servo.Direction.REVERSE);
        outakeClaw.setDirection(Servo.Direction.REVERSE);
        freakyClaw.setPosition(0.78);
        clawRot.setPosition(0.52);
        swivel.setPosition(0.04);
        armL.setPosition(0.06);
        armR.setPosition(0.02);
        outakeClaw.setPosition(0.7);
        outakeSwivel.setPosition(0.2);
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
                .lineTo(new Vector2d(0, 36.8))
                // Tells robot to move forward 26 inches at a slow pace.
                .waitSeconds(1)
                //movement 2
                .lineTo(new Vector2d(-7.5, 47.5))
                .setTangent(3.5)
                .splineToLinearHeading(new Pose2d(-47.5, 15.5, 0), Math.PI / 2,
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(60)
                )
                //.turn(Math.toRadians(45)) // Math.toRadians is required to make it turn exactly fourty five degrees. Postive turns left and negative turns right.
                //movement 3
                .lineTo(new Vector2d(-47.5, 50.54),
                        SampleMecanumDrive.getVelocityConstraint(225, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(225)
                )
                //movement 4
                //.splineToLinearHeading(new Pose2d(-60, 10.5, 0), 0)
                .lineToLinearHeading(new Pose2d(-36.5, 8.5, 1.5708))
                //second block strafe
                .lineTo(new Vector2d(-57, 8.5))
                .setTangent(1)
                .splineToConstantHeading(new Vector2d(-50, 50.54),1.5708,
                        SampleMecanumDrive.getVelocityConstraint(220, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(225)
                )
                .lineTo(new Vector2d(-50, 59.54),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                //movement 5
                //.lineTo(new Vector2d(-60, 55.54),
                //SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(15)
                //Limits to 30 in/s and 30 in/s^2)*/
                //)
                //movement 6
                .lineTo(new Vector2d(-50, 60.5))
                //.lineToLinearHeading(new Pose2d(31.5, 0, 90)) // strafeTo is functionally the same as lineTo. they both tell the robot to go to a position in a straight line.
                //movement 7
                .lineToLinearHeading(new Pose2d(-3.5, 45.5, -1.5708))
                .lineTo(new Vector2d(-3.5, 36.55))
                .waitSeconds(1)
                .lineTo(new Vector2d(-3.5, 45))
                .lineToLinearHeading(new Pose2d(-35, 48, 1.5708),
                        SampleMecanumDrive.getVelocityConstraint(170, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(170)
                )
                .lineTo(new Vector2d(-35, 61),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35)
                )
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, 48))
                .lineToLinearHeading(new Pose2d(-5, 45, -1.5708),
                        SampleMecanumDrive.getVelocityConstraint(170, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(170)
                )

                .lineTo(new Vector2d(-7, 35))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-7, 49))
                .lineTo(new Vector2d(-40, 60.5),
                        SampleMecanumDrive.getVelocityConstraint(170, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(170)
                )
                //.lineToLinearHeading(new Pose2d(49.5, 60, 0))
                //movement 8
                //.lineToLinearHeading(new Pose2d(31.5, 0, 90))
                //movement 9
                //.lineTo(new Vector2d(64.5, 64.5),
                //SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(30)
                //Limits to 30 in/s and 30 in/s^2)*/
                //)
                //block 1
                .waitSeconds(0) // Tells the Robot to wait 1.5 seconds after completing the previous movement.
                .addTemporalMarker(0.1,()->{
                    slideLeft.setTargetPosition(510);
                    slideRight.setTargetPosition(510);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                   ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(1.75,()->{
                    slideLeft.setTargetPosition(302);
                    slideRight.setTargetPosition(302);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(2.25,()->{
                outakeClaw.setPosition(.45);
                })
                .addTemporalMarker(3,()->{
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                    //block 2

                })
                .addTemporalMarker(10.15,()->{
                    outakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(10.6,()->{
                    slideLeft.setTargetPosition(510);
                    slideRight.setTargetPosition(510);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(13.5,()->{
                    slideLeft.setTargetPosition(302);
                    slideRight.setTargetPosition(302);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(13.75,()->{
                    outakeClaw.setPosition(0.45);
                })
                .addTemporalMarker(14.25,()->{
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                //block 3
                .addTemporalMarker(17,()->{
                    outakeClaw.setPosition(0.7);
                })
                .addTemporalMarker(17.25,()->{
                    slideLeft.setTargetPosition(510);
                    slideRight.setTargetPosition(510);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(19.5,()->{
                    slideLeft.setTargetPosition(302);
                    slideRight.setTargetPosition(302);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })
                .addTemporalMarker(19.6,()->{
                    outakeClaw.setPosition(0.45);
                })
                .addTemporalMarker(20.5,()->{
                    slideLeft.setTargetPosition(0);
                    slideRight.setTargetPosition(0);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ((DcMotorEx) slideRight).setTargetPositionTolerance(10);
                    ((DcMotorEx) slideLeft).setTargetPositionTolerance(10);
                    slideLeft.setPower(0.75);
                    slideRight.setPower(0.75);
                })




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
