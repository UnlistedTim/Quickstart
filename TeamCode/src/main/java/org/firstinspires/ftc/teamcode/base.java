
package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.StaterobotDebug.turretPos;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import com.qualcomm.robotcore.util.Range;

public class base {


//     public final double ticksPerDegree = 957.0/180.0;
    public final double ticksPerDegree =113.7777; // 114.02

    public double redGoalX = 140.86;
   // public double redGoalY = 140.86;
    public double blueGoalX = 0;
   // public double blueGoalY = 140.86;
    public double targetGoalY = 140.86;
    public double targetGoalX = 140.86;
    public double Pi=Math.PI,intakeflypower1=0.3,intakeflypower2=0.65,turret_offset=0;
    public double rfxoffset=93.83-7.5,rfyoffset=11.17,bfxoffset=47.52-7.5,bfyoffset=11.17;//23.17-12
    public double  rcalxoffset=140.86-7.5, calheading=0.5*Math.PI;
    public double  bcalxoffset=7.5,calyoffset=7.25,txoffset=0,flypower1=0.4,flypower2=0.75;//todo





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
    public final double turretkP = 0.0002, turretkI = 0.0, turretkD = 0,turretkS = 0.035,turretkPtx=0.015;// 0.025
    public final double flyp = 0.002, flyi = 0, flyd = 0, flyf = 0.0005;
    public final double intakep= 0.0006, intakei = 0, intaked = 0, intakef = 0.0004;

    public double flyspeedgap=500,Txgap=50,  turnMax=0.9,dist=50;


    public double targetVel=0,targetpos;
    public double blockClose = 0.35, blockOpen = 0.46;
    public double tripodIdle = 1.0, tripodPark = 0.35;
    public int beamscancount=0, turrettarget =0;



    //double turretPower=0;
    //PIDController turretPID = new PIDController(turretkP, turretkI, turretkD);
    PIDController flyPID = new PIDController(flyp, flyi, flyd);
    public PIDController intakePID=new PIDController(intakep, intakei, intaked);
 //   double Tx_offset=0;
    int turretCwlim=-14000;
    int turretCcwlim=14000;
    public final int intakeVel = 1500,outtakVel=1500;

    public void init()

    {

        flyPID.setPID(flyp, flyi, flyd);

        intakePID.setPID(intakep, intakei, intaked);
        Flylut.add(-13.59,1720); //far         1700
        Flylut.add(-12.79,1680); //far    1660
        Flylut.add(-11.65,1540); //far    1520
        Flylut.add(-9.26, 1380); //close   1340
        Flylut.add(-5.5,1260); //close
        Flylut.add(-1.19,1160); //close
        Flylut.add(5.82,1140); //close
        Flylut.add(14.07 , 1180); // close
        Flylut.add(16 , 1200); // close

        Flylut.createLUT();


        FlylutPP.add(0,1200); // only for data leakclose
        FlylutPP.add(43.66,1180);
        FlylutPP.add(53.3,1140);
        FlylutPP.add(67.27,1160);
        FlylutPP.add(81.75,1260);
        FlylutPP.add(102.064,1340); //cloe



        FlylutPP.add(124.33,1460); //far   1480
        FlylutPP.add(140.74,1600); //far  1620
        FlylutPP.add(156.09,1654); //far 1660
        FlylutPP.add(200,1700);// only for data leak





        FlylutPP.createLUT();

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
                targetpos=turretPos-tx*ticksPerDegree;


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

                turretPower= turretpid(turretPos,targetpos,tx,offset,true);
                Txgap=Math.abs(offset+(targetpos-turretPos)/ticksPerDegree);
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
                return turretPower;
            }


        }

       turretPower = turretpid(turretPos,target,tx,0,true);

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


         //  if(Math.abs(error)<50) error=0.0001;

           error = targetpos+offset*ticksPerDegree-currentpos;

           if (Math.abs(currentpos) > 10000) return error * (turretkP + 0.00013)  + Math.signum(error) * turretkS;
           if(Math.abs(error)<100) error=0.0001;

           return error * turretkP + Math.signum(error) * turretkS;
       }
       else
       {
           error = offset-tx;
           if (Math.abs(error) < 1.5) return 0;
           if (Math.abs (currentpos) > 10000) return error * (turretkPtx + 0.008) + Math.signum(error) * turretkS;
           else return error * turretkPtx + Math.signum(error) * turretkS;
       }
    }



    public double  turretturnPP(boolean outtake , Pose2D p, int turretTicks,boolean red){

        double dx = targetGoalX- p.getX(DistanceUnit.INCH);
        double dy = targetGoalY -p.getY(DistanceUnit.INCH);
        double goalangle=Math.atan2(dy, dx);

      if(red) txoffset=-(goalangle-Pi/4)/(Pi/4)*3;
      else  txoffset=(goalangle-Pi*3/4)/(Pi/4)*3;

        double targetAngle= goalangle- (p.getHeading(AngleUnit.RADIANS) -Pi);
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

    public double  turret(boolean outtake , Pose2D p, int turretTicks, boolean red, boolean nav, boolean valid , double tx, boolean shooting){

        double turretPower=0;
        if (outtake)  {

                        if(nav&&!shooting){//pinpoint aiming
                            double dx = targetGoalX- p.getX(DistanceUnit.INCH);
                            double dy = targetGoalY -p.getY(DistanceUnit.INCH);
                            double goalangle=Math.atan2(dy, dx);
                            if(red) txoffset=-(goalangle-Pi/4)/(Pi/4)*5;
                            else  txoffset=(goalangle-Pi*3/4)/(Pi/4)*5;
                            double targetAngle= goalangle- (p.getHeading(AngleUnit.RADIANS) -Pi);
                            if(Math.abs(targetAngle) >Pi) targetAngle=-Math.signum(targetAngle)*(2*Pi-Math.abs(targetAngle));
                            dist=Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
                            turretPower = turretpid(turretTicks,Math.toDegrees(targetAngle) * ticksPerDegree,0,turret_offset,true);
                            Txgap=Math.abs( (Math.toDegrees(targetAngle) + turret_offset  - (turretTicks/ticksPerDegree)));
                              }
                         else {//Limelight aiming
                                 if (valid) {
                                    turretPower = turretpid(0, 0, tx, txoffset, false);
                                    limelocked = true;
                                    Txgap = Math.abs(tx - txoffset);
                                     }
                                 else   if (limelocked) turretPower=0;

                              if (turretPos > turretCcwlim- 300 &&  turretPower  > 0) {
                                turretPower= -0.5;//
                                limelocked = false;
                                  }
                               if (turretPos < (turretCwlim + 300) && turretPower< 0) {
                                turretPower= 0.5;//
                                limelocked = false;
                                  }
                          }

              }

        else  turretPower = turretpid(turretTicks, turrettarget,0,0,true);
        turretPower= Range.clip(turretPower,-turnMax,turnMax);
        return turretPower;


    }


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


