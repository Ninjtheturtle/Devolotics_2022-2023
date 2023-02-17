package Provs;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Jan 14")
public class ProvsTeleOp extends OpMode
{
    public PIDController hController;
    public PIDController vController;
    public PIDController eController;

    public double Ph = 0.01, Ih = 0.08, Dh = 0, Fh = 0.25;
    public double Pv = 0.003, Iv = 0, Dv = 0, Fv = 0.2;
    public double Pe = 5, Ie = 0, De = 0;

    public int hTarget = 0;
    public int vTarget = 0;
    public int eTarget = 0;

    private final double ticks_per_degree_tetrix = 3.84444444444444444444444444444444444444444444444444444444;
    private final double ticks_per_degree_rev = 1.8555555555555555555555555555555555555555555555555555555;

    private DcMotorEx vArmLeft, vArmRight, extend, hPitch;
    private Servo hClaw, vClaw, vPitch, wiper;
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    public final double hClawOpen = 0.97;
    public final double hClawClose = 0.76;
    public final double vClawOpen = 0.56;
    public final double vClawClose = 0.521;
    public final double vPitchIn = 0.0;
    public final double vPitchMiddle = 0.45;
    public final double vPitchOut = 0.673;
    public final double wiperIn = 0.26;
    public final double wiperout = 0.68;

    public final int hIntake = 115;
    public final int hTransfer = 0;
    public final int hIntermediate = 50;

    public double vPitchPos = 0.45;


    int autoStateNear = 0;   //cycle close junc
    boolean pNear = false;
    int autoStateFar = 0;    //cycle far junc
    boolean pFar = false;
    int autoHomeState = 0;   //return to home position
    boolean pHome = false;
    int autoProcess = 0;     //0: nothing 1: home 2: close 3: far

    boolean prevA = false, prevB = false, prevX;
    boolean hClawIsOpen = true, vClawIsOpen = false, wiperIsIn = true;

    int mode = 1;
    boolean prevMode = false;

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

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        vController = new PIDController(Pv, Iv, Dv);
        hController = new PIDController(Ph, Ih, Dh);
        eController = new PIDController(Pe, Ie, De);

//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        //DRIVETRAIN
        drive.setWeightedDrivePower(
                new Pose2d(
                        gamepad1.left_stick_y, gamepad1.left_stick_x,
                        -0.7 * gamepad1.right_stick_x
                )
        );

        drive.update();

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
        double hFeed = Math.cos(Math.toRadians((490 - hTarget) / ticks_per_degree_rev)) * -Fh;

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

        //Wiper for both controllers
        if ((gamepad1.x || gamepad2.x) && !prevX) {
            wiperIsIn = !wiperIsIn;
        }
        if (wiperIsIn) {
            wiper.setPosition(wiperIn);
        } else {
            wiper.setPosition(wiperout);
        }
        prevX = gamepad2.x || gamepad1.x;

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

            //GAMEPAD 2
            //extendo, maybe change to gamepad2 sticks?
            if (gamepad2.dpad_up && eTarget < 1360) {
                eTarget += 20;
            } else if (gamepad2.dpad_down && eTarget > 0) {
                eTarget -= 20;
            }

            //h pitch
            hTarget += Math.round(5.0 * gamepad2.right_trigger);
            hTarget -= Math.round(5.0 * gamepad2.left_trigger);
            if (hTarget > 115) {
                hTarget = 115;
            }
            if (hTarget < 0) {
                hTarget = 0;
            }

            //GAMEPAD 1
//            vArmLeft.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//            vArmRight.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            //v arm
            if (vTarget < 625) {
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
                            vPitch.setPosition(vPitchOut);
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
                                eTarget -= 20;
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
                                vTarget -= 15;
                            }
                            else {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                                hClaw.setPosition(hClawOpen);
                                //h out 500
                            }
                            break;
                        case 1:
                            eTarget = 220; //lock extension
                            if (hTarget < hIntake && eTarget == 220) {
                                hTarget += 20;
                            } else if (nearClick) {
                                autoStateNear += 1;
                                hClaw.setPosition(hClawClose);
                                runtime.reset();
                            }
                            break;
                        case 2:
                            if (runtime.milliseconds() > 400) {
                                autoStateNear = 3;
                                //h back 30
                            }
                            break;
                        case 3:
                            if (hTarget > hTransfer) {
                                hTarget -= 10;
                            } else {
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
                                hTarget += 10;
                            } else {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawClose);
                                vPitch.setPosition(vPitchOut);
                                runtime.reset();
                            }
                            break;
                        case 6:
                            if (runtime.milliseconds() > 100) {
                                autoStateNear = 7;
                                //v arm up 625
                            }
                            break;
                        case 7:
                            if (vTarget < 625) {
                                vTarget += 25;
                            } else if (nearClick) {
                                autoStateNear += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchMiddle);
                                //v arm down 0
                            }
                            break;
                        case 8:
                            if (vTarget > 0) {
                                vTarget -= 15;
                            } else {
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
                                vTarget -= 15;
                            } else {
                                autoStateFar += 1;
                                vClaw.setPosition(vClawOpen);
                                vPitch.setPosition(vPitchIn);
                                hClaw.setPosition(hClawOpen);
                            }
                            break;
                        case 1:
                            if (hTarget < hIntake) {
                                hTarget += 10;
                            } if (eTarget < 900 && hTarget >= 80) {
                            eTarget += 50;
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
                        } if (runtime.milliseconds() > 300) {
                            autoStateFar = 3;
                            //h back 30
                        }
                            break;
                        case 3:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } if (vTarget <= 0) {
                            vClaw.setPosition(vClawOpen);
                            vPitch.setPosition(vPitchIn);
                        } if (eTarget > 220) {
                            eTarget = 220;
                        } if (hTarget > hTransfer && runtime.milliseconds() > 400) {
                            hTarget -= 10;
                        } if (hTarget <= hTransfer && eTarget <= 220) {
                            autoStateFar += 1;
                            runtime.reset();
                        }
                            break;
                        case 4:
                            if (runtime.milliseconds() > 500) {
                                hClaw.setPosition(hClawOpen);
                                autoStateFar = 5;
                                //h up 200
                            }
                            break;
                        case 5:
                        if (hTarget < hIntake && runtime.milliseconds() > 900) {
                            hTarget = hIntermediate;
                        } if (eTarget < 900 && hTarget >= hIntermediate) {
                            eTarget = 900;
                        } if (eTarget == 900 && hTarget > hIntermediate) {
                            autoStateFar += 1;
                            vClaw.setPosition(vClawClose);
                            vPitch.setPosition(vPitchOut);
                            runtime.reset();
                        }
                            break;
                        case 6:
                            if (runtime.milliseconds() > 100) {
                                autoStateFar = 7;
                                //v arm up 625
                            }
                            break;
                        case 7:
                            if (vTarget < 675) {

                                vTarget += 25;
                            } if (hTarget < hIntake) {
                            hTarget += 20;
                        } else if (farClick) {
                            autoStateFar += 1;
                            vClaw.setPosition(vClawOpen);
                            vPitch.setPosition(vPitchMiddle);
                            //v arm down 0
                        }
                            break;
                        case 8:
                            if (vTarget > 0) {
                                vTarget -= 25;
                            } if (vTarget <= 400) {
                            autoStateFar = 2;
                            hClaw.setPosition(hClawClose);
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