package org.firstinspires.ftc.teamcode.drive.epik_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DriveBy", group = "main")
public class DriveBy extends LinearOpMode {

    private Servo dump;
    private Servo arm1;
    private Servo arm2;
    private Servo lclaw;
    private Servo rclaw;
    private DcMotor slide;
    private DcMotor lift;
    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;
    private DcMotor hook;
    private Servo pwane;
    private DcMotor intake;
    private Servo flipperL;
    private Servo flipperR;

    @Override
    public void runOpMode() {
        boolean Claw_on_Ground;
        int turboSpeed;
        double normalSpeed;
        double slomoSpeed;
        double SlideRailSpeed;

        dump = hardwareMap.get(Servo.class, "dump");
        arm1 = hardwareMap.get(Servo.class, "arm1");
        arm2 = hardwareMap.get(Servo.class, "arm2");
        lclaw = hardwareMap.get(Servo.class, "lclaw");
        rclaw = hardwareMap.get(Servo.class, "rclaw");
        slide = hardwareMap.get(DcMotor.class, "slide");
        lift = hardwareMap.get(DcMotor.class, "lift");
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        hook = hardwareMap.get(DcMotor.class, "hook");
        pwane = hardwareMap.get(Servo.class, "pwane");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flipperL = hardwareMap.get(Servo.class, "flipperL");
        flipperR = hardwareMap.get(Servo.class, "flipperR");


        Claw_on_Ground = false;
        turboSpeed = 1;
        normalSpeed = 0.5;
        slomoSpeed = 0.2;
        dump.setPosition(0.3);
        arm1.setPosition(0.89);
        arm2.setPosition(0.09);
        lclaw.setPosition(0.81);
        rclaw.setPosition(0.27);
        slide.setTargetPosition(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotor.Direction.FORWARD);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setDirection(DcMotor.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setDirection(DcMotor.Direction.FORWARD);
        flipperL.setPosition(1);
        flipperR.setPosition(0.05);
        waitForStart();
        while (opModeIsActive()) {
            // Gamepad 1
            // slomo
            if (gamepad1.right_bumper) {
                Sticks(slomoSpeed);
            } else {
                Sticks(normalSpeed);
            }
            // turbo
            if (gamepad1.right_trigger > 0) {
                Sticks(turboSpeed);
            } else {
                Sticks(normalSpeed);
            }
            // Lift - motor
            if (gamepad1.triangle) {
                lift.setPower(1);
            } else if (gamepad1.a) {
                lift.setPower(-0.5);
            } else {
                lift.setPower(0);
            }
            // hook - motor
            if (gamepad1.circle) {
                hook.setPower(0.7);
            } else if (gamepad1.square) {
                hook.setPower(-0.7);
            } else {
                hook.setPower(0);
            }
            // plane launcher
            if (gamepad1.dpad_up) {
                pwane.setPosition(.05);
            } else if (gamepad1.dpad_down) {
                pwane.setPosition(.5);
            }

            // Gamepad 2
            // arm2 - closer to plane launcher; higher value, farther out
            // arm1 - closer to hubs; higher value, farther in
            // flippies
            if (gamepad2.dpad_right) {
                flipperL.setPosition(0.46);
                flipperR.setPosition(0.55);
            } /*else if (gamepad2.a) {
                flipperR.setPosition(0.35);
                flipperL.setPosition(0.72); //Dylon did not like this.
            }*/ else {
                flipperL.setPosition(1);
                flipperR.setPosition(0.05);
            }
            // Claw
            if (gamepad2.right_bumper) {
                // open
                rclaw.setPosition(0.4);
            }
            if (gamepad2.left_bumper) {
                // open
                lclaw.setPosition(0.66);
            } else if (gamepad2.a) {

                // open
                lclaw.setPosition(0.66);
                // open
                rclaw.setPosition(0.4);
                Claw_on_Ground = true;
            } else if (!gamepad2.a && !gamepad2.left_bumper && !gamepad2.right_bumper) {
                if (Claw_on_Ground) {
                    Claw_on_Ground = false;
                    // close
                    lclaw.setPosition(0.81);
                    // close
                    rclaw.setPosition(0.27);
                } else {
                    // close
                    lclaw.setPosition(0.81);
                    // close
                    rclaw.setPosition(0.27);
                }
            }

            // Sliderail - motor
            if (gamepad2.right_trigger > 0) {
                SlideRailSpeed = 0.5;
            } else {
                SlideRailSpeed = 1;
            }

            if (gamepad2.dpad_down) {
                // Sliderail down
                slide.setPower(-SlideRailSpeed);
                slide.setTargetPosition((int) (-8.1 * 385));
            } else if (gamepad2.dpad_up) {
                // Sliderail up
                slide.setPower(SlideRailSpeed);
                slide.setTargetPosition((int) (8.1 * 385));
            } else if (gamepad2.dpad_left) {
                // Sliderail hold
                slide.setPower(0.02);
                slide.setTargetPosition((int) (8.1 * 385));
            } else {
                slide.setPower(0);
                slide.setTargetPosition(0);
            }

            // Arm - motor
            if (gamepad2.circle) {
                arm1.setPosition(0.18);
                arm2.setPosition(0.8);
            } else if (gamepad2.square) {
                arm1.setPosition(0.38);
                arm2.setPosition(0.6);
            } else {
                if (!Claw_on_Ground) {
                    arm1.setPosition(0.87); // decrease; orig .89
                    arm2.setPosition(0.11); // increase; orig .09
                }
            }

            // Magazine - Servo
            if (gamepad2.a) {
                // Lower to Grab
                dump.setPosition(0.34);
                arm1.setPosition(0.86);
                arm2.setPosition(0.13);
                intake.setPower(0.7);
            } else if (gamepad2.square) {
                dump.setPosition(0.50);
            } else if (gamepad2.circle) {
                dump.setPosition(0.70);
            } else {
                if (!Claw_on_Ground) {
                    dump.setPosition(0.32); // orig .3
                }
                intake.setPower(0.0);
            }
            if (gamepad2.left_trigger > 0) {
                intake.setPower(-0.7);
            }
        }
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void Sticks(double speed) {
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        fl.setPower(speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        fr.setPower(-speed * gamepad1.right_stick_x + (-speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
        // We negate this value so that the topmost position corresponds to maximum forward power.
        bl.setPower(speed * gamepad1.right_stick_x + (-speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
        br.setPower(-speed * gamepad1.right_stick_x + (speed * gamepad1.left_stick_x - speed * gamepad1.left_stick_y));
    }
}