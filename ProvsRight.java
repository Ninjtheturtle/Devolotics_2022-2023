package Provs;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "Jan 14")
public class ProvsRight extends OpMode
{
    //HARDWARE CONSTANTS
    private final double ticks_per_degree_tetrix = 3.84444444444444444444444444444444444444444444444444444444;
    private final double ticks_per_degree_rev = 1.8555555555555555555555555555555555555555555555555555555;

    private DcMotorEx vArmLeft, vArmRight, extend, hPitch;
    private Servo hClaw, vClaw, vPitch, wiper, align;
    private SampleMecanumDrive drive;

    //HARDWARE CONSTANTS
    public final int hClawOpen = 970;
    public final int hClawClose = 740;
    public final int vClawOpen = 560;
    public final int vClawClose = 480;
    public final int vPitchIn = 0;
    public final int vPitchMiddle = 450;
    public final int vPitchOut = 690;
    public final int VPITCH = 0, VARM = 0, VCLAW = 1, HPITCH = 1, HCLAW = 2, EXTEND = 2, ALIGN = 3, WIPER = 4;

    public final int wiperIn = 260;
    public final int wiperOut = 680;
    public final int alignIn = 0;
    public final int alignOut = 260;
    public final int alignRetract = 350;
    public final int vScore = 605;
    public final int hTransfer = 10;
    public final int hIntermediate = 150;
    public final int ExtendTransfer = 350;
    public final int ExtendIntake = 1980;

    //PIDF CONSTANTS
    public PIDController hController;
    public PIDController vController;
    public PIDController eController;
    public double Ph = 0.005, Ih = 0, Dh = 0.0003, Fh = 0.0008;
    public double Pv = 0.003, Iv = 0, Dv = 0, Fv = 0.2;
    public double Pe = 8, Ie = 0, De = 0.8;
    public int hTarget = 0;
    public int vTarget = 0;
    public int eTarget = 0;

    //OTHER
    private ElapsedTime runtime = new ElapsedTime();
    public int prevLine = -1;
    public int line = 0;
    public int hTargetTarget = 0;
    public int vTargetTarget = 0;
    public int eTargetTarget = 0;
    public int hSpeed = 0;
    public int vSpeed = 0;
    public int eSpeed = 0;

    Pose2d startPose = new Pose2d(-31.5, 60.2, Math.toRadians(270));
    Trajectory traj1 = null;
    Trajectory traj4 = null;
    Trajectory traj5 = null;
    Trajectory traj6 = null;

    //AUTONOMOUS PROGRAM
    public List<Integer[]> program = new ArrayList<>();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int LEFT = 173;
    int MIDDLE = 319;
    int RIGHT = 576;
    int finalTag = 0;
    AprilTagDetection tagOfInterest = null;

    @Override
    public void init() {
        //HARDWARE INITIALIZATION
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

        vController = new PIDController(Pv, Iv, Dv);
        hController = new PIDController(Ph, Ih, Dh);
        eController = new PIDController(Pe, Ie, De);

        //CAMERA HSEILUFH
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);

        wiper.setPosition(0.26);
        vClaw.setPosition(0.48);
        hClaw.setPosition(0.97);
        vPitch.setPosition(0.3);
    }

    @Override
    public void init_loop() {
        if (runtime.milliseconds() > 20) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine(Integer.toString(tagOfInterest.id));
                }
            }

            telemetry.update();
            runtime.reset();
        }
    }

    @Override
    public void start() {
        if (tagOfInterest == null) {
            finalTag = 0;
        } else if (tagOfInterest.id == LEFT) {
            finalTag = 1;
        } else if (tagOfInterest.id == RIGHT) {
            finalTag = 3;
        } else if (tagOfInterest.id == MIDDLE){
            finalTag = 2;
        }

        drive.setPoseEstimate(startPose);
        buildProgram();
//old shit
//        traj1 = drive.trajectoryBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(-24.5, 60))
//                .splineToSplineHeading(new Pose2d(-19, 24, Math.toRadians(183)), Math.toRadians(190))
//                .build();
//        traj4 = drive.trajectoryBuilder(traj1.end())
//                .splineToLinearHeading(new Pose2d(-12, 15, Math.toRadians(180)), Math.toRadians(0))
//                .build();
//        traj5 = drive.trajectoryBuilder(traj4.end())
//                .forward(22)
//                .build();
//        traj6 = drive.trajectoryBuilder(traj4.end())
//                .forward(46)
//                .build();

        traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-18, 55))
                .splineToSplineHeading(new Pose2d(-13, 16, Math.toRadians(188.8)), Math.toRadians(270))
                .build();
        traj4 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(-13, 9, Math.toRadians(180)), Math.toRadians(188.8))
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(24)
                .build();
        traj6 = drive.trajectoryBuilder(traj4.end())
                .forward(48)
                .build();
    }

    @Override
    public void loop() {

        //MOTOR POSITION VARIABLES
        int vPosition = vArmLeft.getCurrentPosition();
        int hPosition = hPitch.getCurrentPosition();
        int ePosition = extend.getCurrentPosition();

        //TARGET SETTING
        if (vTarget != vTargetTarget) { vTarget += vSpeed; }
        if (Math.abs(vTarget - vTargetTarget) < Math.abs(vSpeed)) { vTarget = vTargetTarget; vSpeed = 0; }
        if (hTarget != hTargetTarget) { hTarget = hTargetTarget; }
        if (Math.abs(hTarget - hTargetTarget) < Math.abs(hSpeed)) { hTarget = hTargetTarget; hSpeed = 0; }
        if (eTarget != eTargetTarget) { eTarget = eTargetTarget;}
        if (Math.abs(eTarget - eTargetTarget) < Math.abs(eSpeed)) { eTarget = eTargetTarget; eSpeed = 0; }

        //UPDATE PIDs
        double vPID = vController.calculate(vPosition, vTarget);
        double hPID = hController.calculate(hPosition, hTarget);
        double ePID = eController.calculate(ePosition, eTarget);
        double vFeed = Math.cos(Math.toRadians((vTarget - 240) / ticks_per_degree_tetrix)) * Fv;
        double hFeed = Math.cos(Math.toRadians((490 - hTarget) / ticks_per_degree_rev)) * Fh;
        vArmLeft.setPower(vPID + vFeed);
        vArmRight.setPower(vPID + vFeed);
        hPitch.setPower(hPID + hFeed);
        extend.setVelocity(ePID);

        if (line < program.size()) {
            int func = program.get(line)[0];
            int arg1 = program.get(line)[1];
            int arg2 = program.get(line)[2];
            int arg3 = program.get(line)[3];

            boolean changeLine = false;

            switch (func) {
                case 1:
                    //setServoPos(int servo, int target1k)
                    switch (arg1) {
                        case 0:
                            vPitch.setPosition((double)arg2 / 1000);
                            break;
                        case 1:
                            vClaw.setPosition((double)arg2 / 1000);
                            break;
                        case 2:
                            hClaw.setPosition((double)arg2 / 1000);
                            break;
                        case 3:
                            align.setPosition((double)arg2 / 1000);
                            break;
                        case 4:
                            wiper.setPosition((double)arg2 / 1000);
                            break;
                    }
                    changeLine = true;
                    break;
                case 2:
                    //setMotorPos(int motor, int target, int speed)
                    switch (arg1) {
                        case 0:
                            vTargetTarget = arg2;
                            vSpeed = arg3;
                            break;
                        case 1:
                            hTargetTarget = arg2;
//                            hSpeed = arg3;
                            break;
                        case 2:
                            eTargetTarget = arg2;
//                            eSpeed = arg3;
                            break;
                    }
                    changeLine = true;
                    break;
                case 3:
                    //followTraj(trajno)
                    switch (arg1) {
                        case 1:
                            drive.followTrajectoryAsync(traj1);
                            break;
                        case 4:
                            drive.followTrajectoryAsync(traj4);
                            break;
                        case 5:
                            drive.followTrajectoryAsync(traj5);
                            break;
                        case 6:
                            drive.followTrajectoryAsync(traj6);
                            break;
                    }
                    changeLine = true;
                    break;
                case 4:
                    //waitMotorTarget(int motor, int waitUntilPosition, int sign)
                    switch (arg1) {
                        case 0:
                            if ((vTarget >= arg2 && arg3 > 0) || (vTarget <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 1:
                            if ((hTarget >= arg2 && arg3 > 0) || (hTarget <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 2:
                            if ((eTarget >= arg2 && arg3 > 0) || (eTarget <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                    }
                    break;
                case 5:
                    //waitTime(int ms)
                    if (prevLine != line) {
                        runtime.reset();
                    }
                    if (runtime.milliseconds() > arg1) {
                        changeLine = true;
                    }
                    break;
                case 6:
                    //waitMotorTick(int motor, int waitUntilPosition, int sign)
                    switch (arg1) {
                        case 0:
                            if ((vTarget >= arg2 && arg3 > 0) || (vPosition <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 1:
                            if ((hTarget >= arg2 && arg3 > 0) || (hPosition <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 2:
                            if ((eTarget >= arg2 && arg3 > 0) || (ePosition <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                    }
                    break;
                case 7:
                    //waitTrajDone()
                    if (!drive.isBusy()) {
                        changeLine = true;
                    }
                    break;
            }
            prevLine = line;
            if (changeLine) {
                line += 1;
            }
        }

        //the single most fucking idiotic dumbass line of code in the world -evan
        drive.update();

        //TELEMETRY
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("busy", drive.isBusy());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("line", line);
        telemetry.addData("prevLine", prevLine);
        telemetry.addData("elapsedMilliseconds", runtime.milliseconds());
        telemetry.addData("vPosition", vPosition);
        telemetry.addData("hPosition", hPosition);
        telemetry.addData("ePosition", ePosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.addData("hTarget", hTarget);
        telemetry.addData("eTarget", eTarget);
        telemetry.addData("vTargetTarget", vTargetTarget);
        telemetry.addData("hTargetTarget", hTargetTarget);
        telemetry.addData("eTargetTarget", eTargetTarget);
        telemetry.addData("vSpeed", vSpeed);
        telemetry.addData("hSpeed", hSpeed);
        telemetry.addData("eSpeed", eSpeed);
        telemetry.update();

    }

    //PROGRAM BUILDING FUNCTIONS
    public void setServoPos(int servo, int target1k) {
        program.add(new Integer[] {1, servo, target1k, 0});
    }
    public void setMotorPos(int motor, int target, int speed) {
        program.add(new Integer[] {2, motor, target, speed});
    }
    public void followTraj(int trajno) {
        program.add(new Integer[] {3, trajno, 0, 0});
    }
    public void waitMotorTarget(int motor, int waitUntilPosition, int sign) {
        program.add(new Integer[] {4, motor, waitUntilPosition, sign});
    }
    public void waitTime(int ms) {
        program.add(new Integer[] {5, ms, 0, 0});
    }
    public void waitMotorTick(int motor, int waitUntilPosition, int sign) {
        program.add(new Integer[] {6, motor, waitUntilPosition, sign});
    }
    public void waitTrajDone() {
        program.add(new Integer[] {7, 0, 0, 0});
    }

    public void buildProgram() {
        setMotorPos(HPITCH, hIntermediate, 0);
        waitTime(200);
        setServoPos(VPITCH, 300);
        followTraj(1);
        waitTrajDone();
        setMotorPos(HPITCH, 435, 0);
        setServoPos(ALIGN, alignOut);
        setServoPos(HCLAW, hClawClose);
        setMotorPos(EXTEND, ExtendIntake - 300, 0);
        setMotorPos(VARM, vScore,30);
        waitMotorTick(VARM, vScore, 1);
        setServoPos(VPITCH, vPitchOut);
        waitTime(200);
        setServoPos(VCLAW, vClawOpen);
        setServoPos(VPITCH, vPitchMiddle);
        setServoPos(HCLAW, hClawOpen);
        setServoPos(ALIGN, alignIn);
        waitTime(300);
        setMotorPos(VARM, 0 ,-25);
        setServoPos(ALIGN, alignRetract);
        waitTime(500);
        setServoPos(VPITCH, vPitchIn);
        setMotorPos(EXTEND, ExtendIntake, 0);
        waitTime(300);
        setServoPos(HCLAW, hClawClose);
        waitTime(500);
        setMotorPos(HPITCH, 200, 0);
        waitTime(200);

        for (int i = 1; i < 5; i++) {
            setMotorPos(EXTEND, ExtendTransfer, 0);
            waitMotorTick(EXTEND, 400, -1);
            setMotorPos(HPITCH, hTransfer, 0);
            waitTime(500);
            setServoPos(HCLAW, hClawOpen);
            waitTime(350);
            setServoPos(HCLAW, hClawClose);
            setMotorPos(HPITCH, 435 + 20 * i, 0);
            setMotorPos(EXTEND, ExtendIntake - 300, 0);
            setServoPos(VPITCH, vPitchMiddle);
            setServoPos(VCLAW, vClawClose);
            waitTime(200);
            setServoPos(HCLAW, hClawOpen);
            setMotorPos(VARM, vScore, 30);
            setServoPos(ALIGN, alignOut);
            waitMotorTick(VARM, vScore, 1);
            setServoPos(VPITCH, vPitchOut);
            setMotorPos(EXTEND, ExtendIntake, 0);
            waitTime(200);
            setServoPos(VCLAW, vClawOpen);
            setServoPos(VPITCH, vPitchMiddle);
            setServoPos(ALIGN, alignIn);
            waitTime(200);
            setMotorPos(VARM, 0, -25);
            setServoPos(ALIGN, alignRetract);
            setServoPos(HCLAW, hClawClose);
            waitTime(300);
            setMotorPos(HPITCH, 200, 0);
            setServoPos(VPITCH, vPitchIn);
            waitTime(200);
        }

        setMotorPos(EXTEND, ExtendTransfer, 0);
        waitMotorTick(EXTEND, 400, -1);
        setMotorPos(HPITCH, hTransfer, 0);
        waitTime(500);
        setServoPos(HCLAW, hClawOpen);
        waitTime(350);
        setServoPos(VCLAW, vClawClose);
        setMotorPos(HPITCH, hIntermediate, 0);
        setServoPos(VPITCH, vPitchMiddle);
        waitTime(200);
        setMotorPos(VARM, vScore, 30);
        setMotorPos(EXTEND, 0, 0);
        setServoPos(ALIGN, alignOut);
        waitMotorTick(VARM, vScore, 1);
        setServoPos(VPITCH, vPitchOut);
        waitTime(200);
        setServoPos(VCLAW, vClawOpen);
        setServoPos(VPITCH, vPitchMiddle);
        setServoPos(ALIGN, alignIn);
        waitTime(200);
        setMotorPos(VARM, 0, -25);
        setServoPos(ALIGN, alignRetract);
        waitTime(300);
        followTraj(4);
        setMotorPos(HPITCH, 0, 0);
        waitTime(200);
        setServoPos(ALIGN, alignIn);

        switch (finalTag) {
            case 2:
                followTraj(5);
                break;
            case 3:
                followTraj(6);
                break;
        }
    }
}