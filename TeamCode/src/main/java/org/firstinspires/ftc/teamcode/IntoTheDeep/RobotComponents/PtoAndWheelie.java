package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.ServoPlus;

@Config
public class PtoAndWheelie {

    public static ServoPlus W1,W2,PTO;

    public static CachedMotor FL, FR, BL, BR;

    public static double LiftingRobotWheeliePosW1 = 320, IdleRobotWheeliePosW1 = 190;
    public static double LiftingRobotWheeliePosW2 = 70, IdleRobotWheeliePosW2 = 200;
    public static double EngagePTO = 150, DisengagePTO = 280;
    public static PIDController EngagedLift= new PIDController(0.02, 0, 0);

    static {
        EngagedLift.setFreq(30);
    }

    public static void setPosChassisDrivenLift(double pos){
        EngagedLift.setTargetPosition(pos);
    }

    public static void TiltRobot(){
        W1.setAngle(LiftingRobotWheeliePosW1);
        W2.setAngle(LiftingRobotWheeliePosW2);
    }
    public static void IdleWheeliePos(){
        W1.setAngle(IdleRobotWheeliePosW1);
        W2.setAngle(IdleRobotWheeliePosW2);
    }

    public static void engagePTO(){
        PTO.setAngle(EngagePTO);
    }
    public static void disengagePTO(){
        PTO.setAngle(DisengagePTO);
    }

    public static void powerToChassis(double power){
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);
    }

    public static void UpdateChassisDrivenLift(){
        powerToChassis(EngagedLift.calculatePower(Lift.getPosition()));
    }

}
