package org.firstinspires.ftc.teamcode.boboci;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControler {

    private PIDCoefficients coefs = new PIDCoefficients(0,0,0);
    private double target = 0;
    private double lasterror = 0;
    private double i_sum = 0;
    private ElapsedTime time = new ElapsedTime();

    public PIDControler(double p,double i,double d){
        this.coefs = new PIDCoefficients(p,i,d);
    }

    public double getPower(double currpos){

        double p_p = (target - currpos) * coefs.p;
        double p_d = ((target - currpos) - lasterror)/time.seconds() * coefs.d;
        lasterror = target - currpos;
        i_sum += target - currpos;
        double p_i = i_sum * coefs.i;
        time.reset();

        return p_d + p_p + p_i;
    }

        public void  setTarget(double target){
            this.target = target;
        }

    public void setCoefs(PIDCoefficients coefs){
        this.coefs = coefs;
    }

}
