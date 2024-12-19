package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
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
        clawRot = hardwareMap.get(Servo.class, "clawRot");
        slideLeft = hardwareMap.get(DcMotor.class, "slideLeft");
        freakyClaw = hardwareMap.get(Servo.class, "freakyClaw");
        slideRight = hardwareMap.get(DcMotor.class, "slideRight");

        // Initialization behavior and positions
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        swivel.setPosition(0.04);
        clawRot.setPosition(0.78);
        freakyClaw.setPosition(0.52);

        // variables

        // Trajectories
        // Left, 1
        /*
        In this section we create our trajectories during the Initilization phase. This allows the robot to run immedetly instead of taking time to build them once Auto has started.
         */
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose) //Start of a trajectory. We create a new trajectory sequence and name it "trajSeq1".
                //movement 1
                .lineTo(new Vector2d(0, 33.5))
                // Tells robot to move forward 26 inches at a slow pace.
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
                .lineToLinearHeading(new Pose2d(-38.5, 10.5, 1.5708))
                .lineTo(new Vector2d(-55, 10.5))
                .setTangent(1)
                .splineToConstantHeading(new Vector2d(-50, 50.54),1.5708,
                        SampleMecanumDrive.getVelocityConstraint(220, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(225)
                )
                //movement 5
                //.lineTo(new Vector2d(-60, 55.54),
                //SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //SampleMecanumDrive.getAccelerationConstraint(15)
                //Limits to 30 in/s and 30 in/s^2)*/
                //)
                //movement 6
                .lineTo(new Vector2d(-50, 58))
                //.lineToLinearHeading(new Pose2d(31.5, 0, 90)) // strafeTo is functionally the same as lineTo. they both tell the robot to go to a position in a straight line.
                //movement 7
                .lineToLinearHeading(new Pose2d(0, 45, -1.5708))
                .lineTo(new Vector2d(0, 35))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(0, 45))
                .lineToLinearHeading(new Pose2d(-35, 48, 1.5708),
                        SampleMecanumDrive.getVelocityConstraint(170, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(170)
                )
                .lineTo(new Vector2d(-35, 58))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(-35, 48))
                .lineToLinearHeading(new Pose2d(0, 45, -1.5708),
                        SampleMecanumDrive.getVelocityConstraint(170, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(170)
                )
                .lineTo(new Vector2d(0, 35))
                .waitSeconds(0.5)
                .lineTo(new Vector2d(0, 45))
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
                .waitSeconds(0) // Tells the Robot to wait 1.5 seconds after completing the previous movement.
                //.addTemporalMarker(7,()->{
                //dump.setPosition(0.33);
                //arm1.setPosition(0.86);
                //arm2.setPosition(0.13);
                //intake.setPower(0.7);
                //})
                //.addTemporalMarker(7.5,()->{
                //flipperL.setPosition(0.46);
                //flipperR.setPosition(0.55);
                //})
                //.addTemporalMarker(8.5,()->{
                //intake.setPower(0.0);
                //dump.setPosition(0.3);
                //arm1.setPosition(0.89);
                //arm2.setPosition(0.09);
                //flipperL.setPosition(1);
                //flipperR.setPosition(0.05);
                //})
                //.lineTo(new Vector2d(34,10))
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
