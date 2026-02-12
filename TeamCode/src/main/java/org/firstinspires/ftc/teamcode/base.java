
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


    public final double  ledgreen=0.5, ledred=0.28,hoodfarpos=0.31,hoodnearpose=0.15;




    public boolean limelocked=false ,beamcheck=false;
    InterpLUT Flylut = new InterpLUT();

    InterpLUT FlylutPP = new InterpLUT();
//    InterpLUT Hoodlut = new InterpLUT();
//    InterpLUT HoodlutPP = new InterpLUT();
    //public static double turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;//
    public final double turretkP = 0.0002, turretkI = 0.0, turretkD = 0,turretkS = 0.04,turretkPtx=0.025;//
    public final double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;
    public final double intakep= 0.0006, intakei = 0, intaked = 0, intakef = 0.0004;

    public double flyspeedgap=500,Txgap=50,  turnMax=0.8,dist=50;


    public double targetVel=0;
    public double blockClose = 0.35, blockOpen = 0.46;
    public double tripodIdle = 1.0, tripodPark = 0.35;
    public int beamscancount=0;


    //double turretPower=0;
    //PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);
    public PIDController intakePID=new PIDController(intakep, intakei, intaked);
    double Tx_offset=0;
    int turretCwlim=-12000;
    int turretCcwlim=16000;
    public final int intakeVel = 1500,outtakVel=1500;

    public void init()

    {

        flyPID.setPID(flyp, flyi, flyd);
      //  turretPID.setPID(turretkP, turretkI, turretkD);
        intakePID.setPID(intakep, intakei, intaked);


        Flylut.add(-13.59,1640); //far
        Flylut.add(-12.79,1600); //far

        Flylut.add(-11.65,1460); //far

        Flylut.add(-9.26, 1340); //close
        Flylut.add(-5.5,1260); //close
        Flylut.add(-1.19,1160); //close
        Flylut.add(5.82,1140); //close
        Flylut.add(14.07 , 1180); // close
        Flylut.add(16 , 1200); // close


//        Hoodlut.add(-13.5,0.8);   //far old
//        Hoodlut.add(-12.7,0.8);   //far
//        Hoodlut.add(-12.2,0.8);   //far
//        Hoodlut.add(-11.8,0.8);   //far old
//
//        Hoodlut.add(-9.26,0.62); //close
//        Hoodlut.add(-6.208,0.6);
//        Hoodlut.add(-2.31,0.57);
//        Hoodlut.add(5.078,0.32);
//        Hoodlut.add(15.21,0.15);
//        Hoodlut.add(16,0.16); //close


        Flylut.createLUT();
//        Hoodlut.createLUT();

        FlylutPP.add(0,1200); // only for data leakclose
        FlylutPP.add(43.66,1180);
        FlylutPP.add(53.3,1140);
        FlylutPP.add(67.27,1160);
        FlylutPP.add(81.75,1260);
        FlylutPP.add(102.064,1340); //cloe



        FlylutPP.add(124.33,1460); //far
        FlylutPP.add(140.74,1600); //far
        FlylutPP.add(156.09,1640); //far
        FlylutPP.add(200,1700);// only for data leak





//        HoodlutPP.add(0,0.15);
//
//        HoodlutPP.add(57.724,0.15); //close
//        HoodlutPP.add(69.795,0.32);
//        HoodlutPP.add(85.513,0.57);
//        HoodlutPP.add(100.717,0.6);
//        HoodlutPP.add(115.923,0.62);
//
//        HoodlutPP.add(129.20,0.8);//far  old
//        HoodlutPP.add(144.66,0.8);//far old
//        HoodlutPP.add(162.00,0.8); //far old



        FlylutPP.createLUT();
      //  HoodlutPP.createLUT();
   }



    public boolean beamscanouttake (boolean topon, boolean midon,boolean boton)

    {

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


    public double hoodposition(boolean pinpoint , double ty)

    {
        double hoodpos;
        if(pinpoint)
        {
        if(dist>110) hoodpos=hoodfarpos;
        else hoodpos=hoodnearpose;
        }

        else {
            if(ty<-10.5) hoodpos=hoodfarpos;
            else hoodpos=hoodnearpose;
        }


        return hoodpos;
    }



    public double  turretturn(boolean outtake , boolean valid,int target, int turretPos, double tx, double offset){
        double turretPower;
        if (outtake)  {
            if(valid) {
               // turretPower = turretPID.calculate(tx, offset);
                turretPower= turretpid(turretPos,0,tx,offset,false);

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

       turretPower = turretpid(turretPos,0,tx,0,true);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);

        return turretPower;


    }


    public double turrettxpower(int currentpos,double targetpos) {

       // turretkP = 0.025, turretkI = 0.05, turretkD = 0.002;
        double error = targetpos - currentpos;

        return error * turretkPtx + Math.signum(error) * turretkS;

    }


    public double turretpid(int currentpos,double targetpos,double tx, double offset, boolean pinpoint)


    {


        double error;

       if (pinpoint){
           error = targetpos-currentpos;
           return error * turretkP + Math.signum(error) * turretkS;
       }
       else
       {
           error = offset-tx;
           return error * turretkPtx + Math.signum(error) * turretkS;
       }


//
//        double posgap=Math.abs(targetpos - currentpos);
//        double powerrate=1;
//
//        if ((posgap) > 2000){
//            powerrate= Math.signum (targetpos - currentpos);
//
//        }
//
//        else if (posgap > 250){
//            powerrate= 0.3*Math.signum (targetpos - currentpos);
//        }
//        else{
//           powerrate=0;
//        }
//
//        return powerrate;

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
            turretPower = turretpid(turretTicks,Math.toDegrees(targetAngle) * ticksPerDegree,0,0,true);
            Txgap=Math.abs( (Math.toDegrees(targetAngle)  - (turretTicks / ticksPerDegree)));

            if (turretTicks> turretCcwlim- 300 &&  turretPower  > 0) {
                turretPower= -0.4;
            }
            if (turretTicks < (turretCwlim + 300) && turretPower< 0) {
                turretPower= 0.4;
            }

        }

       else  turretPower = turretpid(turretTicks,0,0,0,true);

        turretPower= Range.clip(turretPower,-turnMax,turnMax);
        return turretPower;



    }



//    public double flyhood(double Ty) {
//
//        double hoodLutGet;
//        if (Ty < 11 && Ty > -13.5) {
//
//
//            hoodLutGet = Hoodlut.get(Ty);
//        }
//
//        else hoodLutGet=0;
//
//        return hoodLutGet;
//
//
//    }

//    public double flyhoodPP() {
//
//        double hoodLutGet;
//        if (dist < 162 && dist >= 0) {
//
//
//            hoodLutGet = HoodlutPP.get(dist);
//        }
//
//        else hoodLutGet=0.8;
//
//        return hoodLutGet;
//
//
//    }

    public double  flyspeed(double currentVel,double ty) {


        if (ty < 11 && ty > -13.58) {

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


