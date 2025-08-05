package org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.Dashtelemetry;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers.ExpansionHubDigital;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.TeleopsStarter.gm1;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.LimitSwitch;

@Config
public class Extendo {

    public enum ExtendoStates{
        RETRACTING,
        FREEWILL,
        GOTOPOS,
    }

    public static CachedMotor motor,encoder;
    public static PIDController pidController = new PIDController(0.01,0,0.0002);
    public static LimitSwitch lm;
    public static final int MaxExtension = 880;
    public static ExtendoStates state = ExtendoStates.RETRACTING;
    public static boolean DoingAuto = false;
    private static double lastRegisteredPos = 0;

    public static void setExtendoPos(double Pos){
        Pos = Math.max(0,Math.min(Pos,MaxExtension));
        pidController.setTargetPosition(Pos);
    }
    public static double getPosition(){
        return encoder.getCurrentPosition();
    }
    public static boolean IsExtendoDone(double Error){
        return Math.abs(pidController.getTargetPosition() - getPosition()) <= Error;
    }

    public static void update(){
        Dashtelemetry.addData("Extendo state",state);
        switch (state) {
            case FREEWILL:
                Dashtelemetry.addData("joystick power",-gm1.right_stick_y);
                Dashtelemetry.addData("extendo pos",getPosition());
                Dashtelemetry.addData("motor power consumption",motor.getCurrent(CurrentUnit.AMPS));
                if(-gm1.right_stick_y >= 0.05 && getPosition() <= MaxExtension-30){
                motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    lastRegisteredPos = getPosition();
                    }
                    else if(-gm1.right_stick_y <= -0.05 && getPosition() >= 30)
                {
                    motor.setPower(-Math.signum(gm1.right_stick_y) * (gm1.right_stick_y * gm1.right_stick_y));
                    lastRegisteredPos = getPosition();
                }
                    else {
                        setExtendoPos(lastRegisteredPos);
                    motor.setPower(pidController.calculatePower(getPosition()));
                }
                    break;

            case RETRACTING:

                motor.setPower(-1);
                boolean isLMactive = true;
                try{
                    lm.getState();
                }catch (Exception e){
                    isLMactive = false;
                    Dashtelemetry.addLine("lm not operational");
                }
                if((isLMactive && lm.getState()) || (motor.getCurrent(CurrentUnit.AMPS) >= 4 && Math.abs(encoder.getVelocity()) <= 5))
                {
                    motor.setPower(0);
                    encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    lastRegisteredPos = 0;
                    if (!DoingAuto)
                        state = ExtendoStates.FREEWILL;
                    else {
                        setExtendoPos(0);
                        state = ExtendoStates.GOTOPOS;
                    }
                }
                break;
            case GOTOPOS:
                motor.setPower(pidController.calculatePower(getPosition()));
                break;
        }
    }

}
