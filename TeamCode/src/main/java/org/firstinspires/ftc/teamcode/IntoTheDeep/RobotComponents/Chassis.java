package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;

@Config
public class Chassis {

    public static CachedMotor FL, FR, BL, BR;
    public static SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0);
    public static PIDController Strafe = new PIDController(-0.015,0,-0.002),
            Forward = new PIDController(0.006,0,0.0015),
            Heading = new PIDController(1.1,0,0.06);

    static{
        Strafe.setFreq(30);
        Forward.setFreq(30);
        Heading.setFreq(30);

        Strafe.kS = -0.0;
        Forward.kS = 0.00;
        Heading.kS = -0.045; //-0.035

    }

    public static void drive(double x, double y, double r){
        double d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1);
        double fl, bl, fr, br;

        fl = (y + x + r) / d;
        bl = (y - x + r) / d;
        fr = (y - x - r) / d;
        br = (y + x - r) / d;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public static void setTargetPosition(SparkFunOTOS.Pose2D pos){

        pos.h = Localizer.normalizeRadians(pos.h);
        Strafe.setTargetPosition(0);
        Forward.setTargetPosition(0);
        Heading.setTargetPosition(0);
        target = pos;
    }

    public static SparkFunOTOS.Pose2D getTargetPosition(){
        return target;
    }

    public static boolean IsPositionDone(double error_distance){
        return Localizer.getDistanceFromTwoPoints(getTargetPosition(),Localizer.getCurrentPosition()) <= error_distance;
    }
    public static boolean IsHeadingDone(double error_heading){
        return Localizer.getAngleDifference(getTargetPosition().h,Localizer.getCurrentPosition().h) <= error_heading;
    }


    public static void update(){
        SparkFunOTOS.Pose2D normal = new SparkFunOTOS.Pose2D(
                getTargetPosition().x - Localizer.getCurrentPosition().x,
                getTargetPosition().y - Localizer.getCurrentPosition().y,
                getTargetPosition().h - Localizer.getCurrentPosition().h);

        double Herror = Localizer.normalizeRadians(normal.h);
        double h = Localizer.getCurrentPosition().h;
        while(h < 0) h += Math.PI * 2;
        while(h > 2*Math.PI) h -= Math.PI * 2;
        SparkFunOTOS.Pose2D error = new SparkFunOTOS.Pose2D(
                Math.cos(h) * normal.x + Math.sin(h) * normal.y,
                Math.sin(h) * normal.x - Math.cos(h) * normal.y,
                Herror
        );

        double xP = Forward.calculatePower(error.x);
        double yP = Strafe.calculatePower(error.y);
        double hP = Heading.calculatePower(error.h);

        RobotInitializers.Dashtelemetry.addData("xError", error.x);
        RobotInitializers.Dashtelemetry.addData("yError", error.y);
        RobotInitializers.Dashtelemetry.addData("hError", Math.toDegrees(error.h));
        double p = 1;
        /*if(RobotInitializers.VOLTAGE > 12.8){
            p *= 12.8 / RobotInitializers.VOLTAGE;
        }*/
        drive(yP * p, -xP * p, hP * p);
    }
}
