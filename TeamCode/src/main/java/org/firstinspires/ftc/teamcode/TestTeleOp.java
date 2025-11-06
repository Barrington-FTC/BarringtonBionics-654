package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@Config
@TeleOp(name="TestTeleOp")
public class TestTeleOp extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFlyWheel = null;
    private DcMotor rightFlyWheel = null;
    private DcMotor intake = null;
    private DcMotor lift = null;

    //servos
    private Servo pitchLeft = null;
    private Servo pitchRight = null;
    //Pid
    private int targetRPM = 0;
    private pid leftPid = null;
    private pid rightPid = null;
    //Lime Light
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private Limelight3A limelight;
    private int tagID = 0;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    // calculations
    private double verticaOffset = 0; //change when robot is built

    private double tagHeigth = 0; // should be the height of april tag

    private double targetHeight = 0;// should be a height above the edge of the rim.

    private double limLightHeight = 0;// limelight distance from the ground

    private double launcherHeight = 0;// height of launcher
    private double distance3D = 0; //distance in 3d space

    private double Distancex = 0; //Distance across the ground
    private double XA = 0;//horizontal angle in radians
    private double YA = 0;//vertical angle in radians
    private double g = 9.81;
    private double VF = 0;
    private double flyWheelRadius=0;
    private double PI = 3.1415;




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

        //Servos
        pitchLeft = hardwareMap.get(Servo.class, "pitchLeft");
        pitchRight = hardwareMap.get(Servo.class, "pitchRight");


        //Fly-wheel
        leftFlyWheel = hardwareMap.get(DcMotor.class, "leftFlyWheel");
        leftFlyWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFlyWheel = hardwareMap.get(DcMotor.class, "rightFlyWheel");
        rightFlyWheel.setDirection(DcMotor.Direction.REVERSE);
        leftFlyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Intakes
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        lift = intake = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //PID
        leftPid = new pid(0,0,0,0,leftFlyWheel);
        rightPid = new pid(0,0,0,0,rightFlyWheel);
        Thread LeftPIDThread = new Thread(this::leftFlyWheelPIDLoop);
        Thread RightPIDThread = new Thread(this::rightFlyWheelPIDLoop);
        //April Tag
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);// look here for the recommended settings.
        waitForStart();



        while (opModeIsActive()) {
            telemetry.update();
            rpm.initializeFlywheels();
            double currentTime = getRuntime();

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

            //intake for player1 does both belt and intake motor
            //forward
            if(gamepad1.right_trigger>0){
                double val;
                if(gamepad1.right_trigger>0.7){
                    val = 0.7;
                }
                else{
                    val = gamepad1.right_trigger;
                }
                intake.setPower(val);
                lift.setPower(val);

            }
            //reverse
            if(gamepad1.left_trigger>0){
                double val;
                if(gamepad1.right_trigger>0.7){
                    val = 0.7;
                }
                else{
                    val = gamepad1.right_trigger;
                }
                intake.setPower(val);
                lift.setPower(val);
            }

            //Lime Light

            if(gamepad2.a & gamepad2.b){
                LLResult result = limelight.getLatestResult();
                LLStatus status = limelight.getStatus();
                telemetry.addData("Name", "%s",
                        status.getName());
                telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                        status.getTemp(), status.getCpu(),(int)status.getFps());
                telemetry.addData("Pipeline", "Index: %d, Type: %s",
                        status.getPipelineIndex(), status.getPipelineType());

                if (result != null) {
                    // Access general information
                    double captureLatency = result.getCaptureLatency();
                    double targetingLatency = result.getTargetingLatency();
                    double parseLatency = result.getParseLatency();
                    telemetry.addData("LL Latency", captureLatency + targetingLatency);
                    telemetry.addData("Parse Latency", parseLatency);
                    telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));
                    int id;
                    if (result.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            id = fiducial.getFiducialId(); // The ID number of the fiducial
                             XA = fiducial.getTargetXDegrees(); // Where it is (left-right)
                            XA= Math.toRadians(XA);
                            double ya = fiducial.getTargetYDegrees(); // Where it is (up-down)
                            ya= Math.toRadians(YA);
                            double y = targetHeight - launcherHeight;
                            // 3D straight-line distance from limelight
                            double d = (tagHeigth - limLightHeight) * Math.cos(ya);
                            // Ground projection (distance on XY plane)
                            Distancex = (tagHeigth - limLightHeight) * Math.cos(ya);
                            distance3D = Math.sqrt(y*y + Distancex*Distancex);
                            YA = Math.acos(distance3D/Distancex);
                            double x = Distancex;
                            telemetry.addData("Fiducial " + id, "(x,y) " + "("+x+"," +y+","+")"+ " meters away from camera");
                        }
                    }
                } else {
                    telemetry.addData("Limelight", "No data available");
                }
                telemetry.update();
            }
                limelight.stop();
                VF = Math.sqrt((g*Distancex*Distancex)/(2*Math.pow(Math.cos(YA),2)+(Distancex*Math.tan(YA) + launcherHeight - targetHeight)));
                targetRPM = Math.toIntExact(Math.round((60 * VF) / (2 * PI * flyWheelRadius)));

            }

            // --------------------------- TELEMETRY --------------------------- //

            telemetry.addData("Front left/Right", "%4.2f, %4.2f",
                    leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f",
                    leftBackDrive.getPower(), rightBackDrive.getPower());

            telemetry.addData("Intake", "%4.2f", intake.getPower());

            telemetry.addData("Flywheel RPM Left/Right", "%4.2f,%4.2f",rpm.getLeftRPM(),rpm.getRightRPM());

            telemetry.update();
        }

    //PID
    private void leftFlyWheelPIDLoop() {
        while (opModeIsActive()) {
            //works when robot is not moving
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                double power = leftPid.update(targetRPM);
                leftFlyWheel.setPower(power);
            }
            sleep(1);
        }
    }

    private void rightFlyWheelPIDLoop() {
        while (opModeIsActive()) {
            //works when robot is not moving
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0) {
                double power = rightPid.update(targetRPM);
                rightFlyWheel.setPower(power);
            }
            sleep(1);
        }
    }
    // End of PID
    private void setDriveMotorsZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }
}

