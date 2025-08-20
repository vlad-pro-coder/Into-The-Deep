package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    public static double ClawOpenPos = 90, ClawClosePos = 55, ClawCloseTight = 45,ClawCloseTightTeleop = 65;
    public static double OverHeadTakeSampPos = 348,OverHeadOverBasketPos = 90, OverHeadTakeSpecPos = 0,
            OverHeadScoreSpecPos = 0,OverHeadBasketMovingSafePos = 180;
    private static double tmp = 0;

    public static double OverBasketFreq = 20;
    public static ElapsedTime timeBeforeLastOverheadUpdate = new ElapsedTime(),
                                timeBeforeLastExtensionUpdate = new ElapsedTime();
    static {
        armProfile = new AsymmetricMotionProfile(9000, 2200, 1850);
        ExtensionProfile = new AsymmetricMotionProfile(6000, 4000, 4000);
    }

    public static void OverheadMoveWhenOverBasket(double val){
        if(timeBeforeLastOverheadUpdate.seconds() > 1.0/OverBasketFreq) {
            double new_angle = Math.max(0, Math.min(armProfile.getPosition() + val, 355));
            setOverHeadPos(new_angle);
            timeBeforeLastOverheadUpdate.reset();
        }
    }

    public static void ExtensionMoveWhenOverBasket(double val){
        double multiplier = 0.005;
        if(timeBeforeLastExtensionUpdate.seconds() > 1.0/OverBasketFreq) {
            val *= multiplier;
            double now_procentage = (retractPos - ExtensionProfile.getTargetPosition()) / (retractPos - extendoPos);
            now_procentage = Math.max(0, Math.min(now_procentage + val, 1));
            setExtensionPos(now_procentage);
            timeBeforeLastOverheadUpdate.reset();
        }
    }

    public static void closeClawTight(){
        Claw.setAngle(ClawCloseTight);
        tmp = System.currentTimeMillis() / 1000.0;
    }
    public static void closeClawTightTeleop(){
        Claw.setAngle(ClawCloseTightTeleop);
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
