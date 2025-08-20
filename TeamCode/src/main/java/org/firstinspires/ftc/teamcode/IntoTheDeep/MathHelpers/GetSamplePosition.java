package org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline.poseWhenSnapshoted;

import android.graphics.Point;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;


@Config
public class GetSamplePosition {
    public static double spool = 16,RA=4.75;
    public static int CPR = 28;
    public static double ExtendoToDistance(int e){
        return (2.0*Math.PI*spool/RA) * ((double)e/CPR);
    }
    public static double initialAngle = Math.toRadians(25), h = 261.817, centerToExtendo = 162;// COY = 140
    public static double cameraOffsetX = 142.621, cameraOffsetY = 107.053;
    public static double CameraAngleFromCenterPoint = Math.atan(cameraOffsetY/cameraOffsetX);//de schimbat
    public static int MMToEncoderTicks(double distance){
        return (int)( distance * CPR * RA / ( 2.0 * Math.PI * spool ));
    }

    public static boolean hasId(LLResult res, int id){
        for(LLResultTypes.DetectorResult r : res.getDetectorResults()){
            if(r.getClassId() == id) return true;
        }
        return false;
    }

    public static double middleX = 630, middleY = -1600;
    public static double XBarSub = 0, halfYTerrain = -1500;
    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(SparkFunOTOS.Pose2D fieldPos){
        double d = Localizer.getDistanceFromTwoPoints(fieldPos, Localizer.getCurrentPosition());
        double t = Localizer.getCurrentPosition().h - Math.atan2(fieldPos.y, fieldPos.x);
        double x = d * Math.cos(t);
        double y = d * Math.sin(t);
        return new SparkFunOTOS.Pose2D(x, y, 0);
    }

    public static SparkFunOTOS.Pose2D getExtendoRotPairByField(SparkFunOTOS.Pose2D s, SparkFunOTOS.Pose2D R){
//        s = new SparkFunOTOS.Pose2D(R.x - s.x, R.y - s.y, 0);
//        double h = Math.atan2((s.y - R.y), (s.x - R.x));
        double r = Localizer.getDistanceFromTwoPoints(s, R);
        double h = Math.PI - Math.acos((s.x - R.x) / r);
        if(s.y - R.y < 0) h = Math.PI * 2 - h;

        h = Localizer.normalizeRadians(h);

        double e = Localizer.getDistanceFromTwoPoints(s, R) - centerToExtendo;
        double forward = e - ExtendoToDistance(Extendo.MaxExtension);
        if(forward < 0) forward = 0;
        return new SparkFunOTOS.Pose2D(e, forward, -h);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToRobot(double tx, double ty){

        //double z = Math.hypot(cameraOffsetX,cameraOffsetY);
        //double XcameraRobot = z * Math.cos((Localizer.normalizeRadians(poseWhenSnapshoted.h) + CameraAngleFromCenterPoint)%(2*Math.PI));
        //double YcameraRobot = z * Math.sin((Localizer.normalizeRadians(poseWhenSnapshoted.h) + CameraAngleFromCenterPoint)%(2*Math.PI));
        //double ErrorX = - (Localizer.getCurrentPosition().x - poseWhenSnapshoted.x);
        //double ErrorY = - (Localizer.getCurrentPosition().y - poseWhenSnapshoted.y);
        double Y = h * Math.tan(Math.PI / 2 - initialAngle + Math.toRadians(ty));
        double X = Math.tan(Math.toRadians(tx)) * Y - cameraOffsetX;
        Y += cameraOffsetY;
        return new SparkFunOTOS.Pose2D(X, Y, 0);
    }
    public static SparkFunOTOS.Pose2D getExtendoRotPair(double tx, double ty){
        double robotHwhenphototaken = Localizer.normalizeRadians(poseWhenSnapshoted.h);
        SparkFunOTOS.Pose2D pose = getPositionRelativeToRobot(tx, ty);
        double extendoDist = MMToEncoderTicks(Math.sqrt(pose.x * pose.x + pose.y * pose.y) - centerToExtendo);
        double rot = robotHwhenphototaken + Math.atan(-pose.x / pose.y);
        double fwd = pose.x - Extendo.MaxExtension;
        if(fwd < 0) fwd = 0;
        return new SparkFunOTOS.Pose2D(extendoDist, fwd, rot);
    }
    public static SparkFunOTOS.Pose2D getPositionRelativeToFiled(double tx, double ty, SparkFunOTOS.Pose2D R){
        SparkFunOTOS.Pose2D p = getPositionRelativeToRobot(tx, ty);
        p.x *= -1;
        double d = Math.hypot(p.x, p.y);
        double t = Math.atan(p.y / p.x);
        double A = R.h + t - Math.PI;

        double x = d * Math.sin(A);
        double y = d * Math.cos(A);

        return new SparkFunOTOS.Pose2D(R.x + x, R.y + y, 0);
    }

    /* vlad */

    public static SparkFunOTOS.Pose2D getSampleRelativeToField(SparkFunOTOS.Pose2D cameraPos, double angleRobot, SparkFunOTOS.Pose2D samplePos) {
        double angleTotal = angleRobot - Math.tan(samplePos.x / samplePos.y);
        if(angleTotal < 0)
            angleTotal += 2*Math.PI;
        else if(angleTotal > Math.PI * 2)
            angleTotal -= Math.PI * 2;
        double ipo = Math.hypot(samplePos.x, samplePos.y);
        SparkFunOTOS.Pose2D translatedToCameraPos;

        if(angleTotal > 0 && angleTotal <= Math.PI/2)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(ipo*Math.cos(angleTotal),ipo*Math.sin(angleTotal),0);
        else if(angleTotal > Math.PI/2 && angleTotal <= Math.PI)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(-ipo*Math.sin(angleTotal-Math.PI/2),ipo*Math.cos(angleTotal-Math.PI/2),0);
        else if(angleTotal > Math.PI && angleTotal <= Math.PI*3/2)
            translatedToCameraPos = new SparkFunOTOS.Pose2D(-ipo*Math.cos(angleTotal-Math.PI),-ipo*Math.sin(angleTotal-Math.PI),0);
        else
            translatedToCameraPos = new SparkFunOTOS.Pose2D(ipo*Math.sin(angleTotal-Math.PI*3/2),-ipo*Math.cos(angleTotal-Math.PI*3/2),0);

        return new SparkFunOTOS.Pose2D(/*centerOffsetToDownRightX +*/ translatedToCameraPos.x + cameraPos.x,/*centerOffsetToDownRightY +*/ translatedToCameraPos.y + cameraPos.y,0);
    }

    public static SparkFunOTOS.Pose2D getSamplePositionRelativeToCamera(double tx, double ty){

        double Y = h * Math.tan(Math.PI / 2 - initialAngle + Math.toRadians(ty));//doesnt work
        double X = Math.tan(Math.toRadians(tx)) * Y;
        return new SparkFunOTOS.Pose2D(X, Y, 0);
    }

    public static SparkFunOTOS.Pose2D normalize_pos(SparkFunOTOS.Pose2D posRobot){
        double normalizedDegrees = 0;
        if(Math.signum(posRobot.h) == 1)
            normalizedDegrees =  posRobot.h;
        else
            normalizedDegrees =  Math.PI*2 + posRobot.h;

        //to change if wrong
        double NaturalDisplacement = 0;

        normalizedDegrees += NaturalDisplacement;
        if(normalizedDegrees > Math.PI*2)
            normalizedDegrees -= Math.PI*2;

        return new SparkFunOTOS.Pose2D(-posRobot.x,-posRobot.y, normalizedDegrees) ;
    }

    public static SparkFunOTOS.Pose2D CameraRelativeToField(SparkFunOTOS.Pose2D positionRobot){

        SparkFunOTOS.Pose2D NormalizepositionRobot = normalize_pos(positionRobot);

        double z = Math.hypot(cameraOffsetX,cameraOffsetY);

        double XcameraField = NormalizepositionRobot.x + z * Math.cos((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));
        double YcameraField = NormalizepositionRobot.y + z * Math.sin((positionRobot.h + CameraAngleFromCenterPoint)%(2*Math.PI));

        return new SparkFunOTOS.Pose2D(XcameraField,YcameraField,0);
    }

    public static SparkFunOTOS.Pose2D GetExtendoTicksToTravelAndNeededAngle(double tx,double ty) {

        SparkFunOTOS.Pose2D pos = poseWhenSnapshoted;
        SparkFunOTOS.Pose2D normalizedPos = normalize_pos(pos);

        SparkFunOTOS.Pose2D poscamera = CameraRelativeToField(pos);

        SparkFunOTOS.Pose2D pos_sample = getSamplePositionRelativeToCamera(tx,ty);

        SparkFunOTOS.Pose2D SampleFieldPos = getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x) - Math.PI/2.f;
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - ExtendoToDistance(Extendo.MaxExtension);
        if(forward < 0) forward = 0;


        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D GetExtendoTicksToTravelAndNeededAngleFromSample(SparkFunOTOS.Pose2D SampleFieldPos){

        SparkFunOTOS.Pose2D pos = poseWhenSnapshoted;
        SparkFunOTOS.Pose2D normalizedPos = normalize_pos(pos);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x);
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - ExtendoToDistance(Extendo.MaxExtension);
        if(forward < 0) forward = 0;


        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D getContinuosTrackingData(SparkFunOTOS.Pose2D SampleFieldPos){
        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();
        SparkFunOTOS.Pose2D normalizedPos = normalize_pos(pos);

        double delta_x = SampleFieldPos.x - normalizedPos.x;
        double delta_y = SampleFieldPos.y - normalizedPos.y;
        double angle = Math.atan2(delta_y,delta_x) - Math.PI/2.f;
        angle = Localizer.normalizeRadians(angle);

        double e = Localizer.getDistanceFromTwoPoints(SampleFieldPos, normalizedPos) - centerToExtendo;
        double forward = e - ExtendoToDistance(Extendo.MaxExtension);
        if(forward < 0) forward = 0;

        return new SparkFunOTOS.Pose2D(MMToEncoderTicks(e),forward,angle);
    }

    public static SparkFunOTOS.Pose2D GetGlobalSamplePosition(double tx,double ty){
        SparkFunOTOS.Pose2D pos = poseWhenSnapshoted;
        SparkFunOTOS.Pose2D normalizedPos = normalize_pos(pos);

        SparkFunOTOS.Pose2D poscamera = CameraRelativeToField(pos);

        SparkFunOTOS.Pose2D pos_sample = getSamplePositionRelativeToCamera(tx,ty);

        return getSampleRelativeToField(poscamera,normalizedPos.h,pos_sample);

    }
}