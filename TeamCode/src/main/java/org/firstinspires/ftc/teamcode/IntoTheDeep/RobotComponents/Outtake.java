package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.AsymmetricMotionProfile;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.ServoPlus;

@Config
public class Outtake {

    public enum OverHeadStates{
        TAKESAMPLE,
        OVERBASKET,
        TAKESPECIMEN,
        SCORESPECIMEN,
    }

    public static ServoPlus Overheads1,Overheads2,Claw,Extension;

    public static AsymmetricMotionProfile armProfile,ExtensionProfile;
    public static double extendoPos = 115, retractPos = 315;
    public static double ClawOpenPos = 100, ClawClosePos = 55, ClawCloseTight = 45;
    public static double OverHeadTakeSampPos = 349,OverHeadOverBasketPos = 90, OverHeadTakeSpecPos = 0,
            OverHeadScoreSpecPos = 0,OverHeadBasketMovingSafePos = 180;
    private static double tmp = 0;
    static {
        armProfile = new AsymmetricMotionProfile(7500, 2200, 2000);
        ExtensionProfile = new AsymmetricMotionProfile(500, 100000, 100000);
    }

    public static void closeClawTight(){
        Claw.setAngle(ClawCloseTight);
        tmp = System.currentTimeMillis() / 1000.0;
    }

    public static void closeClaw(){
        Claw.setAngle(ClawClosePos);
        tmp = System.currentTimeMillis() / 1000.0;
    }
    public static void openClaw(){
        Claw.setAngle(ClawOpenPos);
        tmp = System.currentTimeMillis() / 1000.0;
    }
    public static boolean IsClawDone(){
        return System.currentTimeMillis() / 1000.0 - tmp > 0.1;
    }

    public static void setOverHeadPos(double targetPos) {
        armProfile.startMotion(armProfile.getPosition(),targetPos);
    }

    public static boolean OverHeadDoneness(double OffsetFromTarget){
        return Math.abs(armProfile.getPosition() - armProfile.getTargetPosition()) <= OffsetFromTarget;
    }
    public static boolean OverHeadDoneness(){
        return armProfile.motionEnded();
    }
    public static void setExtensionPos(double procentage){//from 0-1
        double motion = retractPos - procentage * (retractPos - extendoPos);
        ExtensionProfile.startMotion(ExtensionProfile.getPosition(),motion);
    }
    public static boolean ExtensionDoneness(double OffsetFromTarget) {
        return Math.abs(ExtensionProfile.getPosition() - ExtensionProfile.getTargetPosition()) <= OffsetFromTarget;
    }
    public static boolean ExtensionDoneness() {
        return ExtensionProfile.motionEnded();
    }
    public static void OverHead_TAKESAMPLE(){
        setOverHeadPos(OverHeadTakeSampPos);
    }
    public static void OverHead_OVERBASKET(){
        setOverHeadPos(OverHeadOverBasketPos);
    }
    public static void OverHead_TAKESPECIMEN(){
        setOverHeadPos(OverHeadTakeSpecPos);
    }
    public static void OverHead_SCORESPECIMEN(){
        setOverHeadPos(OverHeadScoreSpecPos);
    }
    public static void OverHead_BASKETMOVINGSAFEPOS(){
        setOverHeadPos(OverHeadBasketMovingSafePos);
    }

    public static void update(){
        Overheads1.setAngle(armProfile.getPosition());
        Overheads2.setAngle(armProfile.getPosition());

        Extension.setAngle(ExtensionProfile.getPosition());

        armProfile.update();
        ExtensionProfile.update();

        RobotInitializers.Dashtelemetry.addData("overhead angle",armProfile.getPosition());
    }

}
