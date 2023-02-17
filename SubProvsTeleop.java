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

//@Config
@TeleOp(group = "Jan 14")
public class SubProvsTeleop extends OpMode
{
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

    private DcMotorEx vArmLeft, vArmRight, extend, hPitch;
    private Servo hClaw, vClaw, vPitch, wiper, align;
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    //substation centric
    Vector2d substation = new Vector2d(0, 60);
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    public final double hClawOpen = 0.97;
    public final double hClawClose = 0.76;
    public final double vClawOpen = 0.56;
    public final double vClawClose = 0.48;
    public final double vPitchIn = 0.0;
    public final double vPitchMiddle = 0.45;
    public final double vPitchOut = 0.685;
    public final double wiperIn = 0.26;
    public final double wiperOut = 0.68;
    public final double alignIn = 0.0;
    public final double alignOut = 0.26;
    public final double alignRetract = 0.35;

    public final int hIntake = 500;
    public final int hTransfer = 10;
    public final int hIntermediate = 150;

    public final int vScore = 605;

    public double vPitchPos = 0.45;


    int autoStateNear = 0;   //cycle close junc
    boolean pNear = false;
    int autoStateFar = 0;    //cycle far junc
    boolean pFar = false;
    int autoHomeState = 0;   //return to home position
    boolean pHome = false;
    int autoProcess = 0;     //0: nothing 1: home 2: close 3: far

    boolean prevA = false, prevB = false, prevX, prevY = false;
    boolean hClawIsOpen = true, vClawIsOpen = false, wiperIsIn = true, alignIsIn = true;

    int mode = 1, driveMode = 1;
    boolean prevMode = false, prevDrive = true;


    @Override
    public void init() {
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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //substation centric
        drive.getLocalizer().setPoseEstimate(new Pose2d(-36, 60, 0));
        headingController.setInputBounds(-Math.PI, Math.PI);

        vController = new PIDController(Pv, Iv, Dv);
        hController = new PIDController(Ph, Ih, Dh);
        eController = new PIDController(Pe, Ie, De);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        //PID UPDATING
        vController.setPID(Pv, Iv, Dv);
        hController.setPID(Ph, Ih, Dh);
        eController.setPID(Pe, Ie, De);

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

        //MODE SWITCHING ON EITHER CONTROLLER
        if ((gamepad1.left_stick_button || gamepad2.left_stick_button) && !prevMode) {
            if (mode == 1) {
                mode = 0;
                autoProcess = 0;
                autoStateFar = 0;
                autoStateNear = 0;
                autoHomeState = 0;
            }
            else {mode = 1;}
        }
        prevMode = gamepad1.left_stick_button ||gamepad2.left_stick_button;

        //MODE SWITCHING FOR DRIVING ON EITHER CONTROLLER
        if ((gamepad1.right_stick_button || gamepad2.right_stick_button) && !prevDrive) {
            if (driveMode == 1) {
                driveMode = 0;
            }
            else {driveMode = 1;}
        }
        prevDrive = gamepad1.right_stick_button ||gamepad2.right_stick_button;

        //Wiper for both controllers
        if ((gamepad1.x || gamepad2.x) && !prevX) {
            wiperIsIn = !wiperIsIn;
        }
        if (wiperIsIn) {
            wiper.setPosition(wiperIn);
        } else {
            wiper.setPosition(wiperOut);
        }
        prevX = gamepad2.x || gamepad1.x;


        //if it overshoots then it will adjust
        if (vTarget > vScore){
            vTarget = vScore;
        }
        if (hTarget > hIntake){
            hTarget = hIntake;
        }

        if (driveMode == 1) {
            //Normal Mecanum Drive
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y, gamepad1.left_stick_x,
                            -0.7 * gamepad1.right_stick_x
                    )
            );

            drive.update();
        }

        if (driveMode == 0) {
            //substation centric
            // https://github.com/NoahBres/road-runner-quickstart/blob/advanced-examples/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/drive/advanced/TeleOpAlignWithPoint.java

            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            // Field centric movement
            Vector2d positionInput = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

            // Difference vector
            Vector2d difference = substation.minus(poseEstimate.vec());

            // Target angle
            double targetAngle = difference.angle();

            // Not technically omega because its power. This is the derivative of atan2
            double thetaFF = -positionInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

            // Set the target heading for the heading controller to our desired angle
            headingController.setTargetPosition(targetAngle);

            // Set desired angular velocity to the heading controller output + angular velocity feedforward
            double headingInput = (headingController.update(poseEstimate.getHeading()) * DriveConstants.kV + thetaFF) * DriveConstants.TRACK_WIDTH;

            Pose2d driveDirection = new Pose2d(positionInput, headingInput);

            drive.setWeightedDrivePower(driveDirection);

            // Update the heading controller with our current heading (i dont think we need this)
            // headingController.update(poseEstimate.getHeading());

            // Update the localizer
            drive.getLocalizer().update();

            //double distToSubstation = Math.hypot(substation.getY() - poseEstimate.getY(),substation.getX() - poseEstimate.getX());
            //int extendTick = (int)Math.round(distToSubstation) + 5;
        }

        //MANUAL MODE
        if (mode == 1) {
            //CLAWS (BOTH CONTROLLERS)
            //Vertical Claw
            if ((gamepad1.a || gamepad2.a) && !prevA) {
                vClawIsOpen = !vClawIsOpen;
            }
            if (vClawIsOpen) {
                vClaw.setPosition(vClawOpen);
            } else {
                vClaw.setPosition(vClawClose);
            }
            prevA = gamepad2.a || gamepad1.a;

            //Horizontal Claw
            if ((gamepad1.b || gamepad2.b) && !prevB) {
                hClawIsOpen = !hClawIsOpen;
            }
            if (hClawIsOpen) {
                hClaw.setPosition(hClawOpen);
            } else {
                hClaw.setPosition(hClawClose);
            }
            prevB = gamepad2.b || gamepad1.b;

            //Align
            if ((gamepad1.y || gamepad2.y) && !prevY) {
                alignIsIn = !alignIsIn;
            }
            if (alignIsIn) {
                align.setPosition(alignIn);
            } else {
                align.setPosition(alignOut);
            }
            prevY = gamepad2.y || gamepad1.y;

            //GAMEPAD 2
            //extendo, maybe change to gamepad2 sticks?
            if (gamepad2.dpad_up && eTarget < 2100) {
                eTarget += 20;
            } else if (gamepad2.dpad_down && eTarget > 0) {
                eTarget -= 20;
            }

            //h pitch
            hTarget += Math.round(5.0 * gamepad2.right_trigger);
            hTarget -= Math.round(5.0 * gamepad2.left_trigger);
            if (hTarget > hIntake) {
                hTarget = hIntake;
            }
            if (hTarget < 0) {
                hTarget = 0;
            }

            //GAMEPAD 1
//            vArmLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//            vArmRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            //v arm
            if (vTarget < vScore) {
                vTarget += Math.round(25.0 * gamepad1.right_trigger);
            }
            if (vTarget > 0) {
                vTarget -= Math.round(15.0 * gamepad1.left_trigger);
            }

            if (vTarget < 0){
                vTarget = 0;
            }

            //v pitch
            if (gamepad1.dpad_down && vPitchPos > vPitchIn) {
                vPitchPos -= 0.02;
            } else if (gamepad1.dpad_up && vPitchPos < vPitchOut) {
                vPitchPos += 0.02;
            }
            vPitch.setPosition(vPitchPos);
        }

        //Automatic Mode
        if (mode == 0) {
            boolean nearClick = !pNear && gamepad2.left_bumper, farClick = !pFar && gamepad2.right_bumper, homeClick = !pHome && gamepad2.b;
            switch (autoProcess) {
                case 0:
                    //doing nothing
                    if (homeClick) { autoProcess = 1; }
                    if (nearClick) { autoProcess = 2; }
                    if (farClick) { autoProcess = 3; }
                    break;
                case 1:
                    //home
                    switch (autoHomeState) {
                        case 0:
                            autoHomeState += 1;
                            vClaw.setPosition(vClawOpen);
                            hClaw.setPosition(hClawOpen);
                            vPitch.setPosition(vPitchMiddle);
                            align.setPosition(alignIn);
                            break;
                        case 1:
                            if (vTarget > 0) {
                                vTarget -= 10;
                            } else {
                                autoHomeState += 1;
                            }
                            break;
                        case 2:
                            if (eTarget > 0) {
                                eTarget = 0;
                            } else {
                                autoHomeState += 1;
                            }
                            break;
                        case 3:
                            if (hTarget > 0) {
                                hTarget -= 10;
                            } else {
                                autoHomeState = 0;
                                autoProcess = 0;
                            }
                    }

                    //interrupt cycle
                    if (gamepad2.a) {
                        autoHomeState = 0;
                        eTarget = extend.getCurrentPosition();
                        vTarget = vArmLeft.getCurrentPosition();
                        hTarget = hPitch.getCurrentPosition();
                    }
                    break;

                case 2:
                    //close
                    switch (autoStateNear) {
                        case 0:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            }
                            else {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                                hClaw.setPosition(hClawOpen);
                                align.setPosition(alignRetract);
                                //h out 500
                            }
                            break;
                        case 1:
                            eTarget = 700; //lock extension
                            if (hTarget < hIntake && eTarget == 700) {
                                hTarget = hIntake;
                            } else if (nearClick) {
                                autoStateNear += 1;
                                hClaw.setPosition(hClawClose);
                                runtime.reset();
                            }
                            break;
                        case 2:
                            if (runtime.milliseconds() > 300) {
                                eTarget = 325;
                                autoStateNear = 3;
                                runtime.reset();
                                //h back 30
                            }
                            break;
                        case 3:
                            if (hTarget >= hTransfer && runtime.milliseconds() > 100) {
                                hTarget = hTransfer;
                            } if (runtime.milliseconds() > 800) {
                                autoStateNear += 1;
                                hClaw.setPosition(hClawOpen);
                                runtime.reset();
                            }
                            break;
                        case 4:
                            if (runtime.milliseconds() > 200) {
                                autoStateNear = 5;
                                //h up 200
                            }
                            break;
                        case 5:
                            if (hTarget < hIntermediate) {
                                hTarget = hIntermediate;
                            } else {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawClose);
                                vPitch.setPosition(vPitchMiddle);
                                runtime.reset();
                            }
                            break;
                        case 6:
                            if (runtime.milliseconds() > 100) {
                                autoStateNear = 7;
                                runtime.reset();
                                //v arm up 625
                            }
                            break;
                        case 7:
                            if (vTarget < vScore) {
                                vTarget += 25;
                                align.setPosition(alignOut);
                            } if (vTarget >= vScore){
                                vPitch.setPosition(vPitchOut);
                            } if (nearClick && vTarget >= vScore) {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchMiddle);
                                align.setPosition(alignIn);
                                runtime.reset();
                            }
                            break;
                        case 8:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } if (vTarget <= 500){
                                align.setPosition(alignRetract);
                            } if (vTarget <= 0) {
                                autoStateNear = 0;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                            }
                            break;
                    }

                    if (homeClick) {
                        autoStateNear = 0;
                        autoProcess = 1;
                        eTarget = extend.getCurrentPosition();
                        vTarget = vArmLeft.getCurrentPosition();
                        hTarget = hPitch.getCurrentPosition();
                    }

                    //interrupt cycle
                    if (gamepad2.a) {
                        autoStateNear = 0;
                        autoProcess = 0;
                        eTarget = extend.getCurrentPosition();
                        vTarget = vArmLeft.getCurrentPosition();
                        hTarget = hPitch.getCurrentPosition();
                    }
                    break;

                case 3:
                    //far
                    switch (autoStateFar) {
                        case 0:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } else {
                                autoStateFar += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                                hClaw.setPosition(hClawOpen);
                                align.setPosition(alignRetract);
                            }
                            break;
                        case 1:
                            if (hTarget < hIntake) {
                                hTarget = hIntake;
                            } if (eTarget < 1800 && hTarget == hIntake) {
                                eTarget = 1800;
                            } else if (farClick) {
                                autoStateFar += 1;
                                hClaw.setPosition(hClawClose);
                                runtime.reset();
                            }
                            break;
                        case 2:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } if (vTarget <= 0) {
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                            } if (runtime.milliseconds() > 500) {
                                autoStateFar = 3;
                                runtime.reset();
                            }
                            break;
                        case 3:
                            hClaw.setPosition(hClawClose);
                            if (runtime.milliseconds() > 300 && eTarget > 350) {
                                eTarget = 350;
                            } if (hTarget > hTransfer && runtime.milliseconds() > 1000) {
                                hTarget = hTransfer;
                            } if (hTarget == hTransfer && eTarget <= 350) {
                                autoStateFar += 1;
                                runtime.reset();
                            }
                            break;
                        case 4:
                            if (runtime.milliseconds() > 900) {
                                hClaw.setPosition(hClawOpen);
                                autoStateFar = 5;
                                runtime.reset();
                                //h up 200
                            }
                            break;
                        case 5:
                            if (hTarget < hIntake && runtime.milliseconds() > 400) {
                                hTarget = hIntermediate;
                            } if (eTarget < 1200 && hTarget == hIntermediate) {
                                eTarget = 1200;
                            } if (runtime.milliseconds() > 800) {
                                autoStateFar += 1;
                                vClaw.setPosition(vClawClose);
                                vPitch.setPosition(vPitchMiddle);
                                runtime.reset();
                            }
                            break;
                        case 6:
                            if (runtime.milliseconds() > 100) {
                                autoStateFar = 7;
                                runtime.reset();
                                //v arm up 625
                            }
                            break;
                        case 7:
                            if (vTarget < vScore) {
                                vTarget += 25;
                                align.setPosition(alignOut);
                            } if (vTarget >= vScore) {
                                vPitch.setPosition(vPitchOut);
                            } if (farClick && vTarget >= vScore) {
                                autoStateFar += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchMiddle);
                                align.setPosition(alignIn);
                                runtime.reset();
                            }
                            break;
                        case 8:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } if (vTarget <= 400) {
                                align.setPosition(alignRetract);
                            } if (vTarget <= 100){
                                vPitch.setPosition(vPitchIn);
                            } if (vTarget <= 0 && farClick) {
                                autoStateFar += 1;
                                runtime.reset();
                            }
                            break;
                        case 9:
                            eTarget = 1800;
                            hTarget = hIntake;
                            if (runtime.milliseconds() > 500 && farClick) {
                                autoStateFar = 3;
                                runtime.reset();
                            }
                            break;
                    }

                    if (homeClick) {
                        autoStateFar = 0;
                        autoProcess = 1;
                        eTarget = extend.getCurrentPosition();
                        vTarget = vArmLeft.getCurrentPosition();
                        hTarget = hPitch.getCurrentPosition();
                    }

                    //interrupt cycle
                    if (gamepad2.a) {
                        autoStateFar = 0;
                        autoProcess = 0;
                        eTarget = extend.getCurrentPosition();
                        vTarget = vArmLeft.getCurrentPosition();
                        hTarget = hPitch.getCurrentPosition();
                    }
                    break;
            }

            pNear = gamepad2.left_bumper;
            pFar = gamepad2.right_bumper;
            pHome = gamepad2.b;
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("mode", mode);
        telemetry.addData("autoStateNear", autoStateNear);
        telemetry.addData("autoProcess", autoProcess);
        telemetry.addData("autoStateFar", autoStateFar);
        telemetry.addData("elapsedMilliseconds", runtime.milliseconds());
        telemetry.addData("vPosition", vPosition);
        telemetry.addData("hPosition", hPosition);
        telemetry.addData("ePosition", ePosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.addData("hTarget", hTarget);
        telemetry.addData("eTarget", eTarget);
        telemetry.update();
    }
}