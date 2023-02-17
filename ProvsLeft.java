package Provs;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

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
public class ProvsLeft extends OpMode
{
    //HARDWARE CONSTANTS
    private final double ticks_per_degree_tetrix = 3.84444444444444444444444444444444444444444444444444444444;
    private final double ticks_per_degree_rev = 2.22222222222222222222222222222222222222222222222222222222222;

    private DcMotorEx vArmLeft, vArmRight, extend, hPitch;
    private Servo hClaw, vClaw, vPitch;
    private SampleMecanumDrive drive;

    //HARDWARE CONSTANTS
    public final int hClawOpen = 960;
    public final int hClawClose = 740;
    public final int vClawOpen = 500;
    public final int vClawClose = 582;
    public final int vPitchIn = 750;
    public final int vPitchMiddle = 500;
    public final int vPitchOut = 210;
    public final int VPITCH = 0, VARM = 0, VCLAW = 1, HPITCH = 1, HCLAW = 2, EXTEND = 2;

    //PIDF CONSTANTS
    public PIDController hController;
    public PIDController vController;
    public PIDController eController;
    public double Ph = 0.008, Ih = 0, Dh = 0, Fh = 0.007;
    public double Pv = 0.005, Iv = 0, Dv = 0, Fv = 0.15;
    public double Pe = 10, Ie = 0, De = 0;
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

    Pose2d startPose = new Pose2d(39.5, 61, Math.toRadians(90));
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
        vArmRight.setDirection(DcMotorEx.Direction.REVERSE);
        vArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vArmLeft.setZeroPowerBehavior(BRAKE);
        vArmRight.setZeroPowerBehavior(BRAKE);

        extend = hardwareMap.get(DcMotorEx.class, "horizontalExtend");
        extend.setDirection(DcMotorEx.Direction.FORWARD);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(BRAKE);

        hPitch = hardwareMap.get(DcMotorEx.class, "horizontalPitch");
        hPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hPitch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hPitch.setZeroPowerBehavior(BRAKE);

        hClaw = hardwareMap.get(Servo.class, "horizontalClaw");
        vClaw = hardwareMap.get(Servo.class, "verticalClaw");
        vPitch = hardwareMap.get(Servo.class, "verticalPitch");

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

        vPitch.setPosition(0.78);
        vClaw.setPosition(0.63);
        hClaw.setPosition(hClawOpen);
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

        traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(24.5, 60))
                .splineToSplineHeading(new Pose2d(15.5, 21.5, Math.toRadians(-14.5)), Math.toRadians(10))
                .build();
        traj4 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(10, 15, Math.toRadians(0)), Math.toRadians(180))
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(22)
                .build();
        traj6 = drive.trajectoryBuilder(traj4.end())
                .forward(46)
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
        if (hTarget != hTargetTarget) { hTarget += hSpeed; }
        if (Math.abs(hTarget - hTargetTarget) < Math.abs(hSpeed)) { hTarget = hTargetTarget; hSpeed = 0; }
        if (eTarget != eTargetTarget) { eTarget += eSpeed;}
        if (Math.abs(eTarget - eTargetTarget) < Math.abs(eSpeed)) { eTarget = eTargetTarget; eSpeed = 0; }

        //UPDATE PIDs
        double vPID = vController.calculate(vPosition, vTarget);
        double hPID = hController.calculate(hPosition, hTarget);
        double ePID = eController.calculate(ePosition, eTarget);
        double vFeed = Math.cos(Math.toRadians((vTarget - 240) / ticks_per_degree_tetrix)) * Fv;
        double hFeed = Math.cos(Math.toRadians((hTarget + 270) / ticks_per_degree_tetrix)) * Fh;
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
                            hSpeed = arg3;
                            break;
                        case 2:
                            eTargetTarget = arg2;
                            eSpeed = arg3;
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
        setServoPos(HCLAW, hClawClose);
        followTraj(1);
        waitTime(1000);
        setMotorPos(HPITCH, 20, 5);
        waitTime(1500);
        setMotorPos(EXTEND, 200, 30);
        waitTrajDone();
        waitTime(200);
        setServoPos(VCLAW, vClawOpen);
        setServoPos(HCLAW, hClawOpen);
        waitTime(300);

        for (int i = 0; i < 5; i++) {
            setMotorPos(HPITCH, 415 + 15 * i, 15);
            setMotorPos(EXTEND, 1000, 50);
            waitTime(200);
            setServoPos(HCLAW, hClawClose);
            waitTime(200);
            setServoPos(VPITCH, vPitchMiddle);
            setServoPos(VCLAW, vClawClose);
            setServoPos(HCLAW, hClawOpen);
            waitTime(200);
            setMotorPos(VARM, 500, 30);
            waitMotorTick(VARM, 250, 1);
            setServoPos(VPITCH, vPitchOut);
            waitTime(200);
            waitMotorTick(VARM, 500, 1);
            waitTime(200);
            setServoPos(VCLAW, vClawOpen);
            setServoPos(HCLAW, hClawClose);
            setServoPos(VPITCH, vPitchMiddle);
            waitTime(300);

            setMotorPos(HPITCH, 100, -15);
            setMotorPos(VARM, 0, -25);
            waitMotorTick(HPITCH, 300, -1);
            setMotorPos(EXTEND, 210, -50);
            waitMotorTick(VARM, 200, -1);
            setServoPos(VPITCH, vPitchIn);
            waitTime(300);
            waitMotorTick(EXTEND, 500, -1);
            setMotorPos(HPITCH, 30, -10);
            waitMotorTick(EXTEND, 220, -1);
            waitMotorTick(HPITCH, 40, -1);
            waitTime(150);
            setServoPos(HCLAW, hClawOpen);
            waitTime(300);
        }

        setMotorPos(HPITCH, 120, 15);
        waitTime(300);
        setServoPos(VPITCH, vPitchMiddle);
        setServoPos(VCLAW, vClawClose);
        waitTime(200);
        setMotorPos(EXTEND, 0, -20);
        setMotorPos(VARM, 500, 25);
        waitMotorTick(VARM, 250, 1);
        setServoPos(VPITCH, vPitchOut);
        waitTime(400);
        waitMotorTick(VARM, 500, 1);
        setServoPos(VCLAW, vClawOpen);
        setServoPos(VPITCH, vPitchMiddle);
        waitTime(100);
        followTraj(4);
        setMotorPos(VARM, 0, -25);
        waitTrajDone();
        setMotorPos(HPITCH, 0, -10);
        switch (finalTag) {
            case 2:
                followTraj(5);
                break;
            case 1:
                followTraj(6);
                break;
        }
    }
}