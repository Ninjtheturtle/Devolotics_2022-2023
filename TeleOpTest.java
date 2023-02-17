package Provs;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Dec 10")
public class TeleOpTest extends OpMode
{
    //deals with heading not updating because motor power cap
    private double FAST_MULTIPLIER = 0.6;
    //precision/adjustment drive on right stick
    private double SLOW_MULTIPLIER = 0.25;

    //location of tip of substation
    Vector2d substation = new Vector2d(0, -60);

    private ElapsedTime runtime = new ElapsedTime();

    //region HARDWARE
    private SampleMecanumDrive drive;
    private DcMotorEx vArmLeft, vArmRight, extend, hPitch;
    private Servo hClaw, vClaw, vPitch, wiper, align;
    //endregion HARDWARE

    // region PIDFS
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    public PIDController hController;
    public PIDController vController;
    public PIDController eController;

    public double Ph = 0.005, Ih = 0, Dh = 0.0003, Fh = 0.0008;
    public double Pv = 0.003, Iv = 0, Dv = 0, Fv = 0.2;
    public double Pe = 8, Ie = 0, De = 0.8;

    public int hTarget = 0;
    public int vTarget = 0;
    public int eTarget = 0;

    private final double ticks_per_degree_tetrix = 3.84444444444444444444444444444444444444444444444444444444;
    private final double ticks_per_degree_rev = 1.8555555555555555555555555555555555555555555555555555555;
    // endregion PIDFS

    // region CONSTANTS
    final double hClawOpen = 0.97;
    final double hClawClose = 0.76;
    final double vClawOpen = 0.56;
    final double vClawClose = 0.48;
    final double vPitchIn = 0.0;
    final double vPitchMiddle = 0.45;
    final double vPitchOut = 0.685;
    final double wiperIn = 0.26;
    final double wiperOut = 0.68;
    final double alignIn = 0.0;
    final double alignOut = 0.26;
    final double alignRetract = 0.35;
    final int hIntake = 500;
    final int hTransfer = 10;
    final int hIntermediate = 150;
    final int vScore = 605;
    // endregion CONSTANTS

    int grabTransferState = 0;
    int grabState = 0;
    int highCycleState = 0;
    int homeState = 0;
    int highState = 0;
    int medState = 0;
    int lowState = 0;
    String autoProcess = "none";


    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        drive.getLocalizer().setPoseEstimate(new Pose2d(36, -60, Math.PI));
        //drive.getLocalizer().setPoseEstimate(PoseStorage.endPose);

        //makes sure we don't do dum dum when setting heading target or whatever
        headingController.setInputBounds(-Math.PI, Math.PI);

        vController = new PIDController(Pv, Iv, Dv);
        hController = new PIDController(Ph, Ih, Dh);
        eController = new PIDController(Pe, Ie, De);

        // region HARDWARE INITIALIZATION
        vArmLeft = hardwareMap.get(DcMotorEx.class, "armLeft");
        vArmRight = hardwareMap.get(DcMotorEx.class, "armRight");
        vArmLeft.setDirection(DcMotorEx.Direction.REVERSE);
        vArmRight.setDirection(DcMotorEx.Direction.FORWARD);
        vArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vArmLeft.setZeroPowerBehavior(BRAKE);
        vArmRight.setZeroPowerBehavior(BRAKE);
        extend = hardwareMap.get(DcMotorEx.class, "Extend");
        extend.setDirection(DcMotorEx.Direction.REVERSE);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(BRAKE);
        hPitch = hardwareMap.get(DcMotorEx.class, "hPitch");
        hPitch.setDirection(DcMotorEx.Direction.FORWARD);
        hPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hPitch.setZeroPowerBehavior(BRAKE);
        hClaw = hardwareMap.get(Servo.class, "hClaw");
        vClaw = hardwareMap.get(Servo.class, "vClaw");
        vPitch = hardwareMap.get(Servo.class, "vPitch");
        wiper = hardwareMap.get(Servo.class, "wiper");
        align = hardwareMap.get(Servo.class, "align");
        // endregion HARDWARE INITIALIZATION
    }

    @Override
    public void loop() {
        // region PIDFS
        int vPosition = vArmLeft.getCurrentPosition();
        int hPosition = hPitch.getCurrentPosition();
        int ePosition = extend.getCurrentPosition();

        double vPID = vController.calculate(vPosition, vTarget);
        double hPID = hController.calculate(hPosition, hTarget);
        double ePID = eController.calculate(ePosition, eTarget);

        double vFeed = Math.cos(Math.toRadians((vTarget - 240) / ticks_per_degree_tetrix)) * Fv;
        double hFeed = Math.cos(Math.toRadians((490 - hTarget) / ticks_per_degree_rev)) * Fh;

        vArmLeft.setPower(vPID + vFeed);
        vArmRight.setPower(vPID + vFeed);
        hPitch.setPower(hPID + hFeed);
        extend.setVelocity(ePID);
        // endregion PIDFS

        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        //changes location of and distance to substation aim if scoring on close junction to avoid ground junction near substation
        double closeCheckR = Math.hypot(poseEstimate.getX(), poseEstimate.getY() + 20);
        double closeCheckTheta = Math.atan2(poseEstimate.getY() + 20, poseEstimate.getX());

        if (closeCheckR < 26 && closeCheckTheta < -1.3258176636680324650592392104285 && closeCheckTheta > -1.5707963267948966192313216916398) {
            substation = new Vector2d(12, -72);
        } else if (closeCheckR < 26 && closeCheckTheta < -1.5707963267948966192313216916398 && closeCheckTheta > -1.815774989921760773403404172851) {
            substation = new Vector2d(-12, -72);
        } else {
            substation = new Vector2d(0, -60);
        }

        double distToSubstation = Math.hypot(substation.getY() - poseEstimate.getY(),substation.getX() - poseEstimate.getX());
        int extendTick = (int)Math.round(41 * distToSubstation) - 330;

        //stick direction needs to be fixed?
        Vector2d positionInput = new Vector2d(gamepad1.left_stick_x * FAST_MULTIPLIER + gamepad1.right_stick_x * SLOW_MULTIPLIER,
                -gamepad1.left_stick_y * FAST_MULTIPLIER - gamepad1.right_stick_y * SLOW_MULTIPLIER)
                .rotated(-poseEstimate.getHeading());

        //this is black magic, ignore it
        Vector2d difference = substation.minus(poseEstimate.vec());
        double targetAngle = difference.angle();
        double thetaFF = -positionInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
        headingController.setTargetPosition(targetAngle);
        double headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;
        Pose2d driveDirection = new Pose2d(positionInput, headingInput);
        drive.setWeightedDrivePower(driveDirection);
        // Update the heading controller with our current heading (i dont think we need this)
        // headingController.update(poseEstimate.getHeading());


        switch (autoProcess) {
            case "none":
                if (gamepad1.a) { autoProcess = "grabTransfer"; }
                //if (gamepad1.b) { autoProcess = "high"; }
                //if (gamepad1.x) { autoProcess = "highCycle"; }
                if (gamepad1.y) { autoProcess = "home"; }
                break;
            case "home":
                switch (homeState) {
                    case 0:
                        homeState += 1;
                        vClaw.setPosition(vClawOpen);
                        hClaw.setPosition(hClawOpen);
                        vPitch.setPosition(vPitchMiddle);
                        align.setPosition(alignIn);
                        break;
                    case 1:
                        if (vTarget > 0) {
                            vTarget -= 10;
                        } else {
                            homeState += 1;
                        }
                        break;
                    case 2:
                        if (eTarget > 0) {
                            eTarget = 0;
                        } else {
                            homeState += 1;
                        }
                        break;
                    case 3:
                        if (hTarget > 0) {
                            hTarget -= 10;
                        } else {
                            homeState = 0;
                            autoProcess = "none";
                        }
                }
                break;
            case "grabTransfer":
                switch (grabState) {
                    case 0:
                        if (vTarget > 0) {
                            vTarget -= 25;
                        } else {
                            grabState += 1;
                            vClaw.setPosition(vClawOpen);
                            vPitch.setPosition(vPitchIn);
                            hClaw.setPosition(hClawOpen);
                            align.setPosition(alignRetract);
                        }
                        break;
                    case 1:
                        if (hTarget < hIntake) {
                            hTarget = hIntake;
                            eTarget = extendTick;
                        }
                        if (ePosition + 10 > extendTick && gamepad1.a) {
                            grabState += 1;
                            hClaw.setPosition(hClawClose);
                            runtime.reset();
                        }
                        break;
                    case 2:
                        if (runtime.milliseconds() > 500) {
                            grabState = 3;
                            runtime.reset();
                        }
                        break;
                    case 3:
                        if (runtime.milliseconds() > 300 && eTarget > 350) {
                            eTarget = 350;
                        }
                        if (hTarget > hTransfer && runtime.milliseconds() > 1000) {
                            hTarget = hTransfer;
                        }
                        if (hTarget == hTransfer && eTarget <= 350) {
                            grabState += 1;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 900) {
                            hClaw.setPosition(hClawOpen);
                            grabState = 5;
                            runtime.reset();
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 400) {
                            hTarget = hIntermediate;
                            eTarget = 0;
                            grabState = 0;
                            autoProcess = "none";
                        }
                        break;
                }

                if (gamepad1.y) {
                    grabState = 0;
                    autoProcess = "home";
                    eTarget = extend.getCurrentPosition();
                    vTarget = vArmLeft.getCurrentPosition();
                    hTarget = hPitch.getCurrentPosition();
                }
                break;

        }

        // Same as drive.update i presume? just less memory consumption??
        drive.getLocalizer().update();

        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("substation x", substation.getX());
        telemetry.addData("substation y", substation.getY());
        telemetry.addData("distance to substation", distToSubstation);
        telemetry.addData("Extend target value", extendTick);
        telemetry.addData("Close check Theta", Math.toDegrees(closeCheckTheta));
        telemetry.addData("Close check r", closeCheckR);
        telemetry.update();
    }
}