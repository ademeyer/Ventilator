#pragma once

//#include <iostream>
//#include <cmath>

//using namespace std;

//#define PI  3.1428571428571428571428571428571
#define e 2.7182818284590452353602874713527

//Patient paramter
#define sv  0.000007      //specific gravity

//motor parameter
#define hm  0.04        //Height in meter
#define S   0.01175       //Separation in meter
#define W   0.0605        //Width in meter
#define Jm  0.0000214     //Rotor inertia in Kgm^2
#define Bm  0.00000955      //Damping factor in Ns/m
#define g   58          //gear ratio
#define Ra  4.32        //Terminal resistance
#define k   0.1         //Electric constant

//Ambu Bag Parameter
float R     = 0.06;     //Radius in meter
float L     = 0.24;     //Length in meter
float Lf    = 0.084;    //Truncated left length in meter
float Lr    = 0.092;    //Truncated right length in meter
float hb    = 0.052;    //height in meter

/*
//clinical paramters for variant patient
int   wg    = 75;     //patient weight in kg
int   BPM   = 40;     //required breath per minute
int   IE    = 4;      //IE ratio needed
*/

/**********program function needed for computation*********************************************/

//compute patient needed air volume for patient
float p_vol(float w)
{
  return (w * sv);
}

//compute ambu bag volume
float ambu_vol(float rr, float ll) //radius and length
{
  return (PI * pow(rr, 2) * ((Lr+Lf) - ((4/(3*pow(ll, 2))) * (pow(Lr, 3) + pow(Lf, 3)))));
}

//compute arm length La
float arm_len(float rr)
{
  return (pow((pow(rr, 2) + (2 * rr * (hb - hm + S + (W/2))) + (pow((hb - hm), 2)) + (pow((S+(W/2)), 2))), 0.5));
}

//compute volume of air expelled out from the ambu bag i.e change in volume 
float delta_vol(float rr, float ll, float la)
{
  return ((16.0/3.0 * PI * pow(rr, 2) * la * (pow(Lr, 3) + pow(Lf, 3))/pow(ll, 3)));
}

//compute inhalation time, tau
float tau(float bpm, float ie)
{
  return (60.0/((1+ie)*bpm));
}

//compute voltage
float Volt(float mp1, float mp2, float th, float tu)
{
  return ((g * (mp1) * th)/(k * (1 - (pow(e, -(mp1 * tu/mp2))))));
} 
