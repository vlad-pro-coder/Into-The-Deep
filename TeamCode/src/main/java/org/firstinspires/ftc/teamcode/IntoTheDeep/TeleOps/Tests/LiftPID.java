package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOps.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.LinearFunction;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class LiftPID extends LinearOpMode {

    public static double pos = 0,lastpos = 0;
    public static PIDCoefficients coefs= new PIDCoefficients(0,0,0);
    public static boolean Doingfeed = true;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Lift.state = Lift.LIFTSTATES.CUSTOMMOTORPOWER;

        waitForStart();

        while(opModeIsActive()) {
            RobotInitializers.clearCache();
            if (Doingfeed) {
                Lift.CustomPowerToMotors(LinearFunction.getOutput(Lift.powerDown,Lift.powerUp,Lift.intervalStart,Lift.intervalEnd,Lift.getPosition()));
                RobotInitializers.Dashtelemetry.addData("funcresults",LinearFunction.getOutput(Lift.powerDown,Lift.powerUp,Lift.intervalStart,Lift.intervalEnd,Lift.getPosition()));

            } else {
                Lift.state = Lift.LIFTSTATES.FREEWILL;
                if (lastpos != pos) {
                    Lift.setLiftPos(pos);
                    lastpos = pos;
                }
                Lift.pidController.setPidCoefficients(coefs);
            }
            Lift.update();
            RobotInitializers.Dashtelemetry.addData("pos lift",Lift.encoder.getCurrentPosition()    );
            RobotInitializers.Dashtelemetry.addData("current m1",Lift.motor1.getCurrent(CurrentUnit.AMPS));
            RobotInitializers.Dashtelemetry.addData("current m2",Lift.motor2.getCurrent(CurrentUnit.AMPS));
            RobotInitializers.Dashtelemetry.addData("current state",Lift.state);
            RobotInitializers.Dashtelemetry.update();
        }
    }
}
