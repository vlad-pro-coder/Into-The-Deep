package org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class YellowSampleDetectionPipeline extends OpenCvPipeline {
    public enum TARGET{
        CENTER,
        LOW_CENTER
    }
    public static TARGET returnType = TARGET.LOW_CENTER;
    private SparkFunOTOS.Pose2D poseWhenSnapshoted;
    public static boolean showMask = false;
    public static Size morphologicalKernel = new Size(3, 3),
                    erodeKernel = new Size(3, 3),
                    dilateKernel = new Size(3, 3);
    public static int erodeSteps = 1, dilateSteps = 2, morphologySteps = 2;

    // daca nu cuprinde toate sampleurile din cauza luminii mai scade putin din rosu
    public static Scalar lowerYellow = new Scalar(10, 150, 50), higherYellow = new Scalar(30, 255, 255);
    public static final double cameraFOV = 78;

    // daca nu recunoaste pachuri de sample uri posibil ca sunt prea mici, mareste treshold ul
    // sau fa-l mai mic daca ia in calcul noise ul din background
    public static double SizeTreshold = 50, maxBlur = 10;
    private Mat mask = new Mat(), tmp = new Mat(),
            labels = new Mat(), stats = new Mat(), centroids = new Mat();
    private List<Double> tx, ty;
    private int biggestDetectionID = 0;

    @Override
    public Mat processFrame(Mat input) {
        //test for blureness
        Mat gray = new Mat(), laplacian = new Mat(), cpy;
        cpy = input.clone();
        Imgproc.cvtColor(cpy, gray, Imgproc.COLOR_RGB2GRAY);
        Imgproc.Laplacian(gray, laplacian, CvType.CV_64F);
        MatOfDouble mean = new MatOfDouble(), standardDeviation = new MatOfDouble();

        Core.meanStdDev(laplacian, mean, standardDeviation);
        double variance = standardDeviation.get(0, 0)[0] * standardDeviation.get(0, 0)[0]; // blurrness

        gray.release();
        mean.release();
        standardDeviation.release();
        laplacian.release();

        if(variance > maxBlur){
            return input;
        }

        if(tx == null){
            tx = new ArrayList<>(2);
            ty = new ArrayList<>(2);
        }
        poseWhenSnapshoted = Localizer.getCurrentPosition();

        double largestContour = -1;

        // Daca tot nu merge schimva fov urile sa fie cele bune (cauta pe net)
        //double cameraFOV_Y = Math.sqrt(2 * input.rows() * (1 - Math.cos(cameraFOV)) / (input.rows() * input.rows() + input.cols() * input.cols()));
        //double cameraFOV_X = Math.sqrt(2 * input.cols() * (1 - Math.cos(cameraFOV)) / (input.rows() * input.rows() + input.cols() * input.cols()));
        double cameraFOV_Y = 0.81;
        double cameraFOV_X = 1.08;
        RobotInitializers.Dashtelemetry.addData("FOVY",cameraFOV_Y);
        RobotInitializers.Dashtelemetry.addData("FOVX",cameraFOV_X);


        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        //make treshold
        Core.inRange(mask, lowerYellow, higherYellow, tmp);

        //apply filters
        Imgproc.morphologyEx(tmp, mask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphologicalKernel), new Point(-1, -1), morphologySteps);
        Imgproc.erode(mask, tmp, Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, erodeKernel), new Point(-1, -1), erodeSteps);
        Imgproc.dilate(tmp, mask, Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, dilateKernel), new Point(-1, -1), dilateSteps);


        int noSamples = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8);
        int maxId = 0;

        // make virtual plane at distance 1 from focal point and compute its with and height
        double vpw = 2.d * Math.tan(cameraFOV_X / 2.d);
        double vph = 2.d * Math.tan(cameraFOV_Y / 2.d);

        if(showMask){
            input = mask.clone();
        }

        // initialize empty list for tx and ty
//        tx = new double[noSamples];
//        ty = new double[noSamples];
        List<Double> ttx = new ArrayList<>(noSamples);
        List<Double> tty = new ArrayList<>(noSamples);

        ttx.add(0.0);
        tty.add(0.0);
        for(int i = 1; i < noSamples; i++){
            if(stats.get(i, Imgproc.CC_STAT_AREA)[0] < SizeTreshold) continue;
            if(stats.get(i, Imgproc.CC_STAT_AREA)[0] > largestContour) {
                largestContour = stats.get(i, Imgproc.CC_STAT_AREA)[0];
                maxId = i;
            }

            double x = stats.get(i, Imgproc.CC_STAT_LEFT)[0],
                    y = stats.get(i, Imgproc.CC_STAT_TOP)[0],
                    w = stats.get(i, Imgproc.CC_STAT_WIDTH)[0],
                    h = stats.get(i, Imgproc.CC_STAT_HEIGHT)[0];

            Point target;
            Point targetMiddleLow = new Point(x + w/2.d, y+h);
            Point targetMiddle = new Point(x + w/2.d, y + h/2.d);

            switch (returnType){
                case CENTER:
                    target = targetMiddle;
                    break;
                case LOW_CENTER:
                    target = targetMiddleLow;
                    break;
                default:
                    target = targetMiddle;
            }

            // convert from top-left (0, 0) to middle of the image (0, 0)
            double nx = (target.x - input.cols() / 2.d - 0.5d) * 2.d / input.cols(),
                    ny = (input.rows() / 2.d - 0.5d - target.y) * 2.d / input.rows();


            // convert from pixel distance to virtual plane coordonates
            target = new Point(vpw / 2.d * nx, vph / 2.d * ny);

            // get the angle
            ttx.add(i, Math.atan2(target.x, 1));
            tty.add(i, Math.atan2(target.y, 1));

        }
        for(int i = 1; i < noSamples; i++) {
            if (stats.get(i, Imgproc.CC_STAT_AREA)[0] < SizeTreshold) continue;

            // draw image for debugging

            double x = stats.get(i, Imgproc.CC_STAT_LEFT)[0],
                    y = stats.get(i, Imgproc.CC_STAT_TOP)[0],
                    w = stats.get(i, Imgproc.CC_STAT_WIDTH)[0],
                    h = stats.get(i, Imgproc.CC_STAT_HEIGHT)[0];

            Imgproc.putText(input, Integer.toString(i), new Point(x, y - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 0.2, new Scalar(255, 255, 255), 1);
            Scalar rectColor = new Scalar(255, 0, 0);
            if (stats.get(i, Imgproc.CC_STAT_AREA)[0] < largestContour) {
                rectColor = new Scalar(0, 255, 0);
            }
            //bounding box
            Imgproc.rectangle(input, new Point(x, y), new Point(x + w, y + h), rectColor, 2);
        }
        tx = ttx;
        ty = tty;
        biggestDetectionID = maxId;
        mask.release();
        tmp.release();

        return input;
    }

    public synchronized double getTx(int id){
        return tx.get(id);
    }
    public synchronized double getTy(int id){
        return ty.get(id);
    }
    public synchronized int getLargetDetectionId(){
        return biggestDetectionID;
    }
    public synchronized int getNODetections(){
        return tx.size() - 1;
    }
    public synchronized SparkFunOTOS.Pose2D getPoseAtDetectionTime(){
        return poseWhenSnapshoted;
    }
}
