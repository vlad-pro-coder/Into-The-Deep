package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

//import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm2;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.Dashtelemetry;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.opencv.core.Mat;

@Config
public class Lift {

    public static double MaxHighBasketPos = 850;
    public static double MaxLowBasketPos = 350;
    public static CachedMotor motor1,motor2,encoder;
    public static double LowBasketPos = 300,HighBasketPos = 800;
    private static double currentPos = 0;
    private static double PowerToMotors = 0;
    public static PIDController pidController = new PIDController(0.0065,0.0003,0.0003);
    public static boolean DoingAuto = false;

    public static double powerDown = -0.1,powerUp = 0.2;
    public static double intervalStart = 0,intervalEnd = 800;
    public enum LIFTSTATES{
        OFF,
        FREEWILL,
        LOWBASKET,
        HIGHBASKET,
        RETRACTING,
        CUSTOMMOTORPOWER,
    }

    public static LIFTSTATES state = LIFTSTATES.RETRACTING;

    public static void setLiftPos(double Pos){
        Pos = Math.max(0,Math.min(Pos,MaxHighBasketPos));
        pidController.setTargetPosition(Pos);
    }
    public static double getPosition(){
        return encoder.getCurrentPosition();
    }
    public static boolean IsLiftDone(double Error){
        return Math.abs(pidController.getTargetPosition() - getPosition()) <= Error;
    }

    public static void LowBasketPosition(){
        state = LIFTSTATES.LOWBASKET;
        setLiftPos(LowBasketPos);
        currentPos = LowBasketPos;
    }
    public static void HighBasketPosition(){
        state = LIFTSTATES.HIGHBASKET;
        setLiftPos(HighBasketPos);
        currentPos = HighBasketPos;
    }
    public static void CustomPowerToMotors(double power){
        PowerToMotors = power;
        state = LIFTSTATES.CUSTOMMOTORPOWER;
    }

    private static void setLiftPower(double power){
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public static void update(){
        Dashtelemetry.addData("lift state",state);
        switch (state){
            case OFF:
                setLiftPower(0);
                break;
            case FREEWILL:
                setLiftPower(pidController.calculatePower(getPosition()) + LinearFunction.getOutput(powerDown,powerUp,intervalStart,intervalEnd,getPosition()));
                break;
            case CUSTOMMOTORPOWER:
                setLiftPower(PowerToMotors);
                break;
            case RETRACTING:
                setLiftPower(-1);
                if(motor1.getCurrent(CurrentUnit.AMPS) >= 4 && Math.abs(encoder.getVelocity()) <= 5)
                {
                    setLiftPower(0);
                    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    state = LIFTSTATES.OFF;
                    setLiftPos(0);
                }
                break;
            case LOWBASKET:
                setLiftPower(pidController.calculatePower(getPosition()) + LinearFunction.getOutput(powerDown,powerUp,intervalStart,intervalEnd,getPosition()));
                if(-gm2.left_stick_y > 0.05 && currentPos + -gm2.left_stick_y <= MaxLowBasketPos){
                    currentPos += -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                else if(-gm2.left_stick_y < -0.05 && currentPos - -gm2.left_stick_y >= LowBasketPos){
                    currentPos -= -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                break;
            case HIGHBASKET:
                setLiftPower(pidController.calculatePower(getPosition()) + LinearFunction.getOutput(powerDown,powerUp,intervalStart,intervalEnd,getPosition()));
                if(-gm2.left_stick_y > 0.05 && currentPos + -gm2.left_stick_y <= MaxHighBasketPos){
                    currentPos += -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                else if(-gm2.left_stick_y < -0.05 && currentPos - -gm2.left_stick_y >= HighBasketPos){
                    currentPos -= -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                break;
        }
    }
}
