package Provs;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Dec 10")
public class SubstationTest extends OpMode
{
    private SampleMecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    Vector2d substation = new Vector2d(0, 60);

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(-36, 60, 0));
        headingController.setInputBounds(-Math.PI, Math.PI);
    }

    @Override
    public void loop() {

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

        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.update();
    }
}
