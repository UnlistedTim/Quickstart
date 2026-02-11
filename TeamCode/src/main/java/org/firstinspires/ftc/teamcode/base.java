
package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.StaterobotDebug.turretPos;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;

public class base {


//     public final double ticksPerDegree = 957.0/180.0;
    public final double ticksPerDegree =114.02;;

    public double redGoalX = 140.86;
   // public double redGoalY = 140.86;
    public double blueGoalX = 0;
   // public double blueGoalY = 140.86;
    public double targetGoalY = 140.86;
    public double targetGoalX = 140.86;
    public double Pi=Math.PI,intakeflypower1=0.3,intakeflypower2=0.6;
    public double rfxoffset=93.83-7.5,rfyoffset=11.17,bfxoffset=47.52-7.5,bfyoffset=11.17;//23.17-12







    public final double YOFFSET = 7.25; //TODO

    public final double HEADINGOFFSET = Math.PI/2; // TODO


    public final double REDXOFFSET = 144-7.5; //TODO

    public final double BLUEXOFFSET = 7.5; // TODO


    public final double  ledgreen=0.5, ledred=0.28;
    private Timer beamtimer;



    public boolean limelocked=false ,intakefirst=false,intakelast=false,beamcheck=false;
    InterpLUT Flylut = new InterpLUT();

    InterpLUT FlylutPP = new InterpLUT();
    InterpLUT Hoodlut = new InterpLUT();
    InterpLUT HoodlutPP = new InterpLUT();
    //public static double turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;//
    public final double turretkP = 0.025, turretkI = 0.0, turretkD = 0.001;//
    public final double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;
    public final double intakep= 0.0006, intakei = 0, intaked = 0, intakef = 0.0004;

    public double flyspeedgap=500,Txgap=50,  turnMax=0.8,dist=50;


    public double targetVel=0;
    public double blockClose = 0.35, blockOpen = 0.46;
    public double tripodIdle = 1.0, tripodPark = 0.35;
    public int step=0,beamoncount=0,ballcount=0,beamoffcount=0,beamoncount1=0,step1=0,beamtimecount=0,beamscancount=0;


    //double turretPower=0;
    PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);
    public PIDController intakePID=new PIDController(intakep, intakei, intaked);
    double Tx_offset=0;
    int turretCwlim=-12000;
    int turretCcwlim=16000;
   // public MedianFilter intakeCurrentFilter = new MedianFilter(10);

    public final int intakeVel = 1500,outtakVel=1500;








    public void init()

    {
        beamtimer=new Timer();
        flyPID.setPID(flyp, flyi, flyd);
        turretPID.setPID(turretkP, turretkI, turretkD);
        intakePID.setPID(intakep, intakei, intaked);


        Flylut.add(-13.5,1720); //far old
        Flylut.add(-12.7,1680); //far
        Flylut.add(-12.2,1630); //far
        Flylut.add(-11.85,1610); //far old

        Flylut.add(-9.26, 1440); //close
        Flylut.add(-6.208,1380); //close
        Flylut.add(-2.31,1420); //close
        Flylut.add(5.078,1250); //close
        Flylut.add(15.21 , 1120); // close
        Flylut.add(16 , 800); // close


        Hoodlut.add(-13.5,0.8);   //far old
        Hoodlut.add(-12.7,0.8);   //far
        Hoodlut.add(-12.2,0.8);   //far
        Hoodlut.add(-11.8,0.8);   //far old

        Hoodlut.add(-9.26,0.62); //close
        Hoodlut.add(-6.208,0.6);
        Hoodlut.add(-2.31,0.57);
        Hoodlut.add(5.078,0.32);
        Hoodlut.add(15.21,0.15);
        Hoodlut.add(16,0.16); //close


        Flylut.createLUT();
        Hoodlut.createLUT();

        FlylutPP.add(0,800); //close
        FlylutPP.add(57.724,1120);
        FlylutPP.add(69.795,1250);
        FlylutPP.add(85.513,1420);
        FlylutPP.add(100.717,1380);
        FlylutPP.add(115.923,1440); //cloe

        FlylutPP.add(129.20,1540); //far old
        FlylutPP.add(144.66,1630); //far old
        FlylutPP.add(162.00,1720); //far old




        HoodlutPP.add(0,0.15);

        HoodlutPP.add(57.724,0.15); //close
        HoodlutPP.add(69.795,0.32);
        HoodlutPP.add(85.513,0.57);
        HoodlutPP.add(100.717,0.6);
        HoodlutPP.add(115.923,0.62);

        HoodlutPP.add(129.20,0.8);//far  old
        HoodlutPP.add(144.66,0.8);//far old
        HoodlutPP.add(162.00,0.8); //far old



        FlylutPP.createLUT();
        HoodlutPP.createLUT();
   }



    public boolean beamscanouttake (boolean topon, boolean midon,boolean boton)

    {
        ballcount=0;
      if(topon&&midon&&boton) beamscancount++; else beamscancount=0;
      if(beamscancount>4) {beamscancount=0;return true;}

      return false;//no result

    }


    public boolean beamscanintake(boolean topon, boolean midon,boolean boton)

    {

        if(!topon&&!midon&&!boton) beamscancount++; else beamscancount=0;
        if(beamscancount>4) {beamscancount=0;return true;}
        return false;

    }





    public double  turretturn(boolean outtake , boolean valid,int target, int turretPos, double tx, double offset){
        double turretPower;
        if (outtake)  {
            if(valid) {
               // turretPower = turretPID.calculate(tx, offset);
                turretPower=  Math.signum (offset-tx)*Math.pow(Math.abs(tx-offset)*0.1,1.5);
                limelocked=true;
                Txgap=Math.abs(tx-offset);
                if (turretPos > turretCcwlim- 300 &&  turretPower  > 0) {
                    turretPower= -0.5;
//                    target = 0;
                    limelocked = false;
                }
                if (turretPos < (turretCwlim + 300) && turretPower< 0) {
                    turretPower= 0.5;
//                    target = 0;
                    limelocked = false;
                }

//                turretPower = Range.clip(turretPower,-turnMax,turnMax);
                return  turretPower ;
            }

            if (limelocked) {
                turretPower=0;
                return turretPower;
            }


        }

        turretPower = turretPID.calculate( turretPos*0.35,  target*0.35);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);

        return turretPower;


    }

    public double turrettickpower(int currentpos,double targetpos)


    {

        double posgap=Math.abs(targetpos - currentpos);
        double powerrate=1;

        if ((posgap) > 2000){
            powerrate= Math.signum (targetpos - currentpos);

        }

        else if (posgap > 250){
            powerrate= 0.3*Math.signum (targetpos - currentpos);
        }
        else{
           powerrate=0;
        }

        return powerrate;

    }



    public double  turretturnPP(boolean outtake , Pose2D p, int turretTicks){

        double dx = targetGoalX- p.getX(DistanceUnit.INCH);
        double dy = targetGoalY -p.getY(DistanceUnit.INCH);

        double targetAngle= Math.atan2(dy, dx) - (p.getHeading(AngleUnit.RADIANS) -Pi);
        if(Math.abs(targetAngle) >Pi) targetAngle=-Math.signum(targetAngle)*(2*Pi-Math.abs(targetAngle));
        dist=Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));


        double turretPower;
        if (outtake)  {
           // turretPower = turretPID.calculate(turretTicks, Math.toDegrees(targetAngle) * ticksPerDegree);
            turretPower = turrettickpower(turretTicks,Math.toDegrees(targetAngle) * ticksPerDegree);
            Txgap=Math.abs( (Math.toDegrees(targetAngle)  - (turretTicks / ticksPerDegree)))*0.7;

            if (turretTicks> turretCcwlim- 300 &&  turretPower  > 0) {
                turretPower= -0.4;
            }
            if (turretTicks < (turretCwlim + 300) && turretPower< 0) {
                turretPower= 0.4;
            }

        }

       else  turretPower = turrettickpower(turretTicks,0);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);
        return turretPower;



    }

    public double flyhood(double Ty) {

        double hoodLutGet;
        if (Ty < 11 && Ty > -13.5) {


            hoodLutGet = Hoodlut.get(Ty);
        }

        else hoodLutGet=0;

        return hoodLutGet;


    }

    public double flyhoodPP() {

        double hoodLutGet;
        if (dist < 162 && dist >= 0) {


            hoodLutGet = HoodlutPP.get(dist);
        }

        else hoodLutGet=0.8;

        return hoodLutGet;


    }

    public double  flyspeed(double currentVel,double ty) {


        if (ty < 11 && ty > -13.5) {

            targetVel = Flylut.get(ty);// Tx offset

//            Tx_offset=0;

        }
        else targetVel = 1600;



        targetVel = Math.round(targetVel / 0.001) * 0.001;

        flyspeedgap=Math.abs(currentVel-targetVel);


        //   double power = flyPID.calculate(flyCurrentVel, targetVel) + flyf * targetVel;

        return(flyPID.calculate(currentVel, targetVel) + flyf * targetVel);


    }

    public double  flyspeedPP(double currentVel) {


        if (dist < 162 && dist >= 0) {

            targetVel = FlylutPP.get(dist);// Tx offset


        }
        else targetVel = 1500;



        targetVel = Math.round(targetVel / 0.001) * 0.001;

        flyspeedgap=Math.abs(currentVel-targetVel);


        //   double power = flyPID.calculate(flyCurrentVel, targetVel) + flyf * targetVel;

        return(flyPID.calculate(currentVel, targetVel) + flyf * targetVel);


    }



}


