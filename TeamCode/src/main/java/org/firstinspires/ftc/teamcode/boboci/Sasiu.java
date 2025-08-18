package org.firstinspires.ftc.teamcode.boboci;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sasiu {

    private DcMotorEx fs;
    private DcMotorEx fd;
    private DcMotorEx ss;
    private DcMotorEx sd;

    public static double malin = 1;

    public Sasiu(HardwareMap gigel){
        fs = gigel.get(DcMotorEx.class, "stangafata");
        fd = gigel.get(DcMotorEx.class, "dreaptafata");
        ss = gigel.get(DcMotorEx.class, "stangaspate");
        sd = gigel.get(DcMotorEx.class, "dreaptaspate");
    }

    public void Drive(double y, double x, double rot){
        double d = Math.max( Math.abs(x) + Math.abs(y) + Math.abs(rot), 1);
        double fsp, fdp, ssp, sdp;
        fsp = (y + x - rot) / d;
        fdp = (y - x + rot) / d;
        ssp = (y - x - rot) / d;
        sdp = (y + x + rot) / d;
        fs.setPower(fsp);
        fd.setPower(fdp);
        ss.setPower(ssp);
        sd.setPower(sdp);
    }

}
