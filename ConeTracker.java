package Provs;

import static org.opencv.core.Core.inRange;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Autonomous
public class ConeTracker extends OpMode {
    public boolean ConeIsLeft = false;
    public boolean ConeIsRight = false;
    public boolean ConeIsCenter = false;

    OpenCvCamera camera;
    ConeTrackerPipeline coneTrackerPipeline;

    @Override
    public void init() {
        //CAMERA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        coneTrackerPipeline = new ConeTrackerPipeline(telemetry);
        camera.setPipeline(coneTrackerPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    //890 is the center of cone stack
    //
    @Override
    public void loop() {
        if (coneTrackerPipeline.BlueRect.x > 950) {
            ConeIsLeft = true;
        } else{
            ConeIsLeft = false;
        }

        if (coneTrackerPipeline.BlueRect.x < 830) {
            ConeIsRight = true;
        } else{
            ConeIsRight = false;
        }

        if (coneTrackerPipeline.BlueRect.x > 830 && coneTrackerPipeline.BlueRect.x < 950 ) {
            ConeIsCenter = true;
        } else{
            ConeIsCenter = false;
        }

        telemetry.addData("Cone is left", ConeIsLeft);
        telemetry.addData("Cone is right", ConeIsRight);
        telemetry.addData("Cone is center", ConeIsCenter);

        telemetry.addData("Blue X", coneTrackerPipeline.BlueRect.x);
        telemetry.addData("Red X", coneTrackerPipeline.RedRect.x);
        telemetry.update();
    }

    public class ConeTrackerPipeline extends OpenCvPipeline {
        Telemetry telemetry;

        final Scalar GREEN = new Scalar(0, 255, 0);

        boolean RED = true;
        boolean BLUE = true;

        public int redContourCount = 0;
        public int blueContourCount = 0;

        public List<Rect> redRect;
        public List<Rect> blueRect;

        public Rect RedRect;
        public Rect BlueRect;

        public List<MatOfPoint> redContours;
        public List<MatOfPoint> blueContours;
        public List<MatOfPoint> circleContours;

        public MatOfPoint biggestRedContour;
        public MatOfPoint biggestBlueContour;

        public ConeTrackerPipeline(Telemetry telemetry) {
            redContours = new ArrayList<MatOfPoint>();
            redRect = new ArrayList<Rect>();
            RedRect = new Rect();
            biggestRedContour = new MatOfPoint();

            blueContours = new ArrayList<MatOfPoint>();
            blueRect = new ArrayList<Rect>();
            BlueRect = new Rect();
            biggestBlueContour = new MatOfPoint();

            this.telemetry = telemetry;
        }

        // Filters the contours to be greater than a specific area in order to be tracked
        public boolean filterContours(MatOfPoint contour) {
            return Imgproc.contourArea(contour) > 50;
        }

        // Red masking thresholding values:
        Scalar lowRed = new Scalar(10, 100, 50); //10, 100, 50
        Scalar highRed = new Scalar(35, 255, 255); //35, 255, 255

        // Blue masking thresholding values:
        Scalar lowBlue = new Scalar(10, 100, 50); //10, 100, 50
        Scalar highBlue = new Scalar(35, 255, 255); //35, 255, 255

        // Mat object for the red and blue mask
        Mat maskRed = new Mat();
        Mat maskBlue = new Mat();

        // Mat object for YCrCb color space
        Mat YCrCb = new Mat();

        // Kernel size for blurring
        Size kSize = new Size(5, 5);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size((2 * 2) + 1, (2 * 2) + 1));

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Imgproc.erode(YCrCb, YCrCb, kernel);
            Imgproc.GaussianBlur(YCrCb, YCrCb, kSize, 0);

            if (RED) {
                // Finds the pixels within the thresholds and puts them in the mat object "maskRed"
                inRange(YCrCb, lowRed, highRed, maskRed);

                // Clears the arraylists
                redContours.clear();
                redRect.clear();

                // Finds the contours and draws them on the screen
                Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.drawContours(input, redContours, -1, GREEN); //input

                // Iterates through each contour
                for (int i = 0; i < redContours.size(); i++) {

//                Imgproc.HoughCircles(redContours.get(i), circles, Imgproc.HOUGH_GRADIENT, Math.PI/180, 50, 1, 50);
                    // Filters out contours with an area less than 50 (defined in the filter contours method)
                    if (filterContours(redContours.get(i))) {

                        // TODO: move outside of for loop to optimize potentially
                        biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                            return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                        });

                        // Creates a bounding rect around each contourand the draws it
                        RedRect = Imgproc.boundingRect(biggestRedContour);

                        Imgproc.rectangle(input, RedRect, GREEN, 2);
                    }
                }

                // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
                telemetry.addData("Red Contour ", "%7d,%7d", RedRect.x + (RedRect.width / 2), RedRect.y + (RedRect.height / 2));

                maskRed.release();
            }

            if (BLUE) {
                // Finds the pixels within the thresholds and puts them in the mat object "maskBlue"
                inRange(YCrCb, lowBlue, highBlue, maskBlue);

                // Clears the arraylists
                blueContours.clear();
                blueRect.clear();

                //TODO: Canny edge detection?

                // Finds the contours and draws them on the screen
                Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.drawContours(input, blueContours, -1, GREEN); //input

                // Iterates through each contour
                for (int i = 0; i < blueContours.size(); i++) {
                    // Filters out contours with an area less than 50 (defined in the filter contours method)
                    if (filterContours(blueContours.get(i))) {
                        biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
                            return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
                        });

                        // Creates a bounding rect around each contourand the draws it
                        BlueRect = Imgproc.boundingRect(biggestBlueContour);
                        Imgproc.rectangle(input, BlueRect, GREEN, 2);
                    }
                }

                // Displays the position of the center of each bounding rect (rect.x/y returns the top left position)
                telemetry.addData("Blue Contour ", "%7d,%7d", BlueRect.x + (BlueRect.width / 2), BlueRect.y + (BlueRect.height / 2));

                maskBlue.release();
            }

            redContourCount = 0;
            blueContourCount = 0;

            YCrCb.release();

            //TODO: move this when actually using code (this is just for EasyOpenCV sim)
            telemetry.update();

            return input;
        }
    }
}