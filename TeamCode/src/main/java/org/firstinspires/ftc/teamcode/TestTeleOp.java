package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    //Dc Motor dec
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Intake = null;
    private DcMotor FlyWheel = null;
    private DcMotor Timing = null;
    //servos
    private Servo Indexing = null;
    private Servo PitchL = null;
    private Servo Kicker = null;

    //Constants
    //indexer

    //cr servo
    private static final int rDistance = 0; //the change in encoder value between each slot
    //For normal servo mode
    private static final int Left = 0;
    private static final int Center = 0;
    private static final int Right = 0;
    //Kicker
    private static final int Extended = 0;
    private static final int Retracted = 0;


    @Override
    public void runOpMode() {
        //base
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Timing = hardwareMap.get(DcMotor.class, "Timing");
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        Timing.setDirection(DcMotorSimple.Direction.FORWARD);
        Indexing = hardwareMap.get(Servo.class,"Indexing");
        Kicker = hardwareMap.get(Servo.class,"Kicker");
        Indexing.setDirection(Servo.Direction.FORWARD);
        Kicker.setDirection(Servo.Direction.FORWARD);

        //shooting
        FlyWheel = hardwareMap.get(DcMotor.class, "FlyWheel");
        FlyWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        PitchL = hardwareMap.get(Servo.class,"pitchL");
        PitchL.setDirection(Servo.Direction.FORWARD);

        //telemetry dec
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        Indexing.setPosition(Left);
        Kicker.setPosition(Retracted);
        while (opModeIsActive()) { // Loop

            // --------------------------- WHEELS --------------------------- //
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_stick_x, 3);
            double leftFrontPower = gamepad1.dpad_left ? 1 : axial + lateral + yaw;
            double rightFrontPower = gamepad1.dpad_right ? 1 : axial - lateral - yaw;
            double leftBackPower = gamepad1.dpad_down ? 1 : axial - lateral + yaw;
            double rightBackPower = gamepad1.dpad_up ? 1 : axial + lateral - yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Reduced Power Mode
            if (gamepad1.a) {
                leftFrontPower /= 2;
                rightFrontPower /= 2;
                leftBackPower /= 2;
                rightBackPower /= 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //Intake
            if(gamepad1.right_trigger > 0){
                Intake.setPower(gamepad1.right_trigger);
            }
            if(gamepad2.left_trigger>0){
                Intake.setPower(-gamepad1.left_trigger);
            }



            // --------------------------- TELEMETRY --------------------------- //
            // Show the elapsed game time and wheel power.

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("Intake  Dc/Index", "%4.2f, %4.2f",
                    Intake.getPower(), Indexing.getPosition());
            telemetry.addData("Shooting  FlyWheel/PitchL", "%4.2f, %4.2f",
                    FlyWheel.getPower(), PitchL.getPosition());

            telemetry.update();
        }
    }

    // Dedicated method for the PID loop
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}
