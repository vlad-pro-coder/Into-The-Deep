package org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.ExtendoToDistance;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import kotlin.Pair;

@Config
public class YellowSampleDetectionPipeline extends OpenCvPipeline {
    public static SparkFunOTOS.Pose2D poseWhenSnapshoted = new SparkFunOTOS.Pose2D(0,0,0);
    public static boolean showMask = false;
    private ArrayList<SparkFunOTOS.Pose2D> failedAttempts = new ArrayList<>();
    public static Size morphologicalKernel = new Size(3, 3),
            erodeKernel = new Size(3, 3),
            dilateKernel = new Size(3, 3);
    public static int erodeSteps = 1, dilateSteps = 4, morphologySteps = 1;
    public static double diminuator = 4.9;
            public static double startingY = 380;

    // daca nu cuprinde toate sampleurile din cauza luminii mai scade putin din rosu
    public static Scalar lowerYellow = new Scalar(15, 165, 130), higherYellow = new Scalar(30, 255, 255);
    public static final double cameraFOV = 78;

    public static double cameraFOV_Ydegree = 51.8;
    public static double cameraFOV_Xdegree = 66;

    // daca nu recunoaste pachuri de sample uri posibil ca sunt prea mici, mareste treshold ul
    // sau fa-l mai mic daca ia in calcul noise ul din background
    public static double SizeTreshold = 200;
    private Mat mask = new Mat(), tmp = new Mat(),
            labels = new Mat(), stats = new Mat(), centroids = new Mat();

    private static Point getMiddleTargetPoint(Mat stat, int i){
        double xM = (stat.get(i, Imgproc.CC_STAT_LEFT)[0] + stat.get(i, Imgproc.CC_STAT_WIDTH)[0]) / 2.d;
        double yM = (stat.get(i, Imgproc.CC_STAT_TOP)[0] + stat.get(i, Imgproc.CC_STAT_HEIGHT)[0]) / 2.d;

        return new Point(xM, yM);
    }

    private static Point getLowerTargetPoint(Mat stat, int i){
        double xM = (stat.get(i, Imgproc.CC_STAT_LEFT)[0] + stat.get(i, Imgproc.CC_STAT_WIDTH)[0]) / 2.d;
        double y = (stat.get(i, Imgproc.CC_STAT_TOP)[0] + stat.get(i, Imgproc.CC_STAT_HEIGHT)[0]);

        return new Point(xM, y);
    }
    private List<Double> tx, ty;
    private int biggestDetectionID = 0;
    public boolean newResultsReady = false;

    @Override
    public Mat processFrame(Mat input) {
        if(tx == null){
            tx = new ArrayList<>();
            ty = new ArrayList<>();
        }

        double largestContour = -1;

        // Daca tot nu merge schimva fov urile sa fie cele bune (cauta pe net)
        //double cameraFOV_Y = Math.sqrt(2 * input.rows() * (1 - Math.cos(cameraFOV)) / (input.rows() * input.rows() + input.cols() * input.cols()));
        //double cameraFOV_X = Math.sqrt(2 * input.cols() * (1 - Math.cos(cameraFOV)) / (input.rows() * input.rows() + input.cols() * input.cols()));
        double cameraFOV_Y = Math.toRadians(cameraFOV_Ydegree);
        double cameraFOV_X = Math.toRadians(cameraFOV_Xdegree);


        Imgproc.cvtColor(input, mask, Imgproc.COLOR_RGB2HSV);

        //make treshold
        Core.inRange(mask, lowerYellow, higherYellow, tmp);

        //apply filters
        Imgproc.morphologyEx(tmp, mask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, morphologicalKernel), new Point(-1, -1), morphologySteps);
        Imgproc.erode(mask, tmp, Imgproc.getStructuringElement(Imgproc.MORPH_ERODE, erodeKernel), new Point(-1, -1), erodeSteps);
        Imgproc.dilate(tmp, mask, Imgproc.getStructuringElement(Imgproc.MORPH_DILATE, dilateKernel), new Point(-1, -1), dilateSteps);


        int noSamples = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8);

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
                biggestDetectionID = i;
            }

            double x = stats.get(i, Imgproc.CC_STAT_LEFT)[0],
                    y = stats.get(i, Imgproc.CC_STAT_TOP)[0],
                    w = stats.get(i, Imgproc.CC_STAT_WIDTH)[0],
                    h = stats.get(i, Imgproc.CC_STAT_HEIGHT)[0];

            Point target = new Point(x + w/2.d, y + h/2.d);

            // convert from top-left (0, 0) to middle of the image (0, 0)
            double nx = (target.x - input.cols() / 2.d - 0.5d) * 2.d / input.cols(),
                    ny = (input.rows() / 2.d - 0.5d - target.y) * 2.d / input.rows();


            // convert from pixel distance to virtual plane coordonates
            target = new Point(vpw / 2.d * nx, vph / 2.d * ny);

            // get the angle
            ttx.add(Math.atan2(target.x, 1));
            tty.add(Math.atan2(target.y, 1));

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
        mask.release();
        tmp.release();

        poseWhenSnapshoted = Localizer.getCurrentPosition();

        return input;
    }

    public static class SamplePoint {
        public double x;
        public double y;

        public SamplePoint(double x, double y) {
            this.x = x;
            this.y = y;
        }

        // For displaying the point
        public String toString() {
            return "(" + x + ", " + y + ")";
        }
    }

    Comparator<SamplePoint> pointComparator = (p1, p2) -> {
        int yComparison = Double.compare(p2.y, p1.y); // higher y first
        if (yComparison != 0) {
            return yComparison;
        }
        return Double.compare(p1.x, p2.x); // lower x first
    };

    public class ClosestToOriginComparator implements Comparator<SamplePoint>{
        public int compare(SamplePoint p1,SamplePoint p2){

            double dist1 = p1.x * p1.x + p1.y*p1.y;
            double dist2 = p2.x * p2.x + p2.y*p2.y;

            return Double.compare(dist1,dist2);
        }
    }
    public static final double XLimit = 1400,YLimit = 1670;
    public static final double XLowLimit = 850;

    public synchronized PriorityQueue<SamplePoint> getSamplesPrioritized(){
        PriorityQueue<SamplePoint> orderedSamples = new PriorityQueue<SamplePoint>(pointComparator);
        for(int i=1;i<tx.size();i++)
        {
            SparkFunOTOS.Pose2D globalSample = GetSamplePosition.GetGlobalSamplePosition(Math.toDegrees(tx.get(i)), Math.toDegrees(ty.get(i)));
            RobotInitializers.Dashtelemetry.addLine(new SamplePoint(globalSample.x,globalSample.y).toString());
            if(globalSample.x > XLimit || globalSample.y > YLimit || globalSample.x < XLowLimit)
                continue;
            orderedSamples.add(new SamplePoint(globalSample.x,globalSample.y));
        }
        return orderedSamples;
    }

    public SparkFunOTOS.Pose2D calculateWhereExtendoEndUp(int extendoDist,double wantedH){
        double z = ExtendoToDistance(extendoDist);
        double h = Localizer.normalizeRadians(wantedH);
        double x_contribution = z * Math.cos(h);
        double y_contribution = z * Math.sin(h);
        SparkFunOTOS.Pose2D normalized = GetSamplePosition.normalize_pos(poseWhenSnapshoted);
        return new SparkFunOTOS.Pose2D(x_contribution + normalized.x,y_contribution + normalized.y,0);
    }

    public synchronized SamplePoint getBestTxTy(){
        double mini = 1e9;
        double besttx = 0,bestty = 0;
        if(tx == null)
            return new SamplePoint(besttx,bestty);
        for(int i=1;i<tx.size();i++)
        {
            RobotLog.ii("tx" + i,"" + Math.toDegrees(tx.get(i)));
            RobotLog.ii("ty" + i,"" + Math.toDegrees(ty.get(i)));
            SparkFunOTOS.Pose2D data = GetSamplePosition.getExtendoRotPair(Math.toDegrees(tx.get(i)),Math.toDegrees(ty.get(i)));
            SparkFunOTOS.Pose2D distances = GetSamplePosition.getPositionRelativeToRobot(Math.toDegrees(tx.get(i)),Math.toDegrees(ty.get(i)));

            double accepted_distance = startingY + Math.toDegrees(Math.abs(Localizer.getAngleDifference(Math.toRadians(0),data.h))) * diminuator;

            if(IsIgnored(data) || distances.y < accepted_distance || data.x > Extendo.MaxExtension+100 || Math.abs(Localizer.getAngleDifference(Math.toRadians(0),data.h)) > Math.toRadians(40))
                continue;
            if(data.x < mini){
                mini = data.x;
                besttx = Math.toDegrees(tx.get(i));
                bestty = Math.toDegrees(ty.get(i));
            }
        }
        return new SamplePoint(besttx,bestty);
    }

    private final double MaxAngleDifference = Math.toRadians(4);
    private final double maxExtensionDifference = 50;

    private boolean IsIgnored(SparkFunOTOS.Pose2D result){
        for(SparkFunOTOS.Pose2D ignored:failedAttempts){
            if(Math.abs(ignored.x - result.x)<=maxExtensionDifference && Math.abs(ignored.h - result.h) <= MaxAngleDifference)
                return true;
        }
        return false;
    }

    public void AddResultToIgnored(SparkFunOTOS.Pose2D result){
        failedAttempts.add(result);
    }

    public void resetIgnoredSample(){
        failedAttempts.clear();
    }

    public synchronized List<Double> getTx(){
        return tx;
    }
    public synchronized List<Double> getTy(){
        return ty;
    }
    public synchronized int getLargetDetectionId(){
        return biggestDetectionID;
    }
    public synchronized int getNODetections(){
        return centroids.rows();
    }
    public synchronized SparkFunOTOS.Pose2D getPoseAtDetectionTime(){
        return poseWhenSnapshoted;
    }
}
