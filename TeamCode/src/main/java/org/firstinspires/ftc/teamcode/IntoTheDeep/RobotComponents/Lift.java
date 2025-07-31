package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;

@Config
public class Lift {

    public static double MaxHighBasketPos = 0;
    public static double MaxLowBasketPos = 0;
    public static CachedMotor motor1,motor2,encoder;
    public static double LowBasketPos = 0,HighBasketPos = 0;
    private static double currentPos = 0;
    private static double PowerToMotors = 0;
    public static PIDController pidController = new PIDController(0,0,0);
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

        switch (state){
            case OFF:
                setLiftPower(0);
                break;
            case FREEWILL:
                setLiftPower(pidController.calculatePower(getPosition()));
                break;
            case CUSTOMMOTORPOWER:
                setLiftPower(PowerToMotors);
            case RETRACTING:
                setLiftPower(-1);
                if(motor1.getCurrent(CurrentUnit.AMPS) >= 3 && encoder.getVelocity() < 5)
                {
                    setLiftPower(0);
                    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    state = LIFTSTATES.OFF;
                }
                break;
            case LOWBASKET:
                setLiftPower(pidController.calculatePower(getPosition()));
                if(-gm2.left_stick_y > 0.05 && currentPos + -gm2.left_stick_y <= MaxLowBasketPos){
                    currentPos += -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                else if(-gm2.left_stick_y < -0.05 && currentPos - -gm2.left_stick_y >= LowBasketPos){
                    currentPos -= -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
            case HIGHBASKET:
                setLiftPower(pidController.calculatePower(getPosition()));
                if(-gm2.left_stick_y > 0.05 && currentPos + -gm2.left_stick_y <= MaxHighBasketPos){
                    currentPos += -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
                else if(-gm2.left_stick_y < -0.05 && currentPos - -gm2.left_stick_y >= HighBasketPos){
                    currentPos -= -gm2.left_stick_y;
                    setLiftPos(currentPos);
                }
        }
    }
}
