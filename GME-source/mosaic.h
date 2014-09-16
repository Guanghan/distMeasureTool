/**************************************************************************
* Program Name: uavOFMA
*
* Filename: mosaic.h
*
* Description:
*  
*  Compute registrated frames and/or create mosaic image.
*   
*  
*  
*
* Authors: Yu-chia(York) Chung, Dr. Zhihai He
*
*
* Copyright (C) 2006-2007. Yu-chia(York) Chung, Dr. Zhihai He,
* and Curators of the University of Missouri, a public corporation.
* All Rights Reserved.
*
* Created by
* Dr. Zhihai He
* Department of Electrical and Computer Engineering,
* University of Missouri - Columbia
*
* For more information, please contact:
*
* Dr. Zhihai He
* 225 Engineering Building West
* University of Missouri-Columbia
* Columbia, MO 65211
* (573) 882-3495
* HeZhi@missouri.edu
*
*
**************************************************************************/

#ifndef MOZAIC_H
#define MOZAIC_H

#include "global.h"

void	 writeMozaic();

void     renewMap(int i, double rposproj, double cposproj);	// renew map with position projections.
void	 drawWindow(double ** O,double ** P,double ** Q,double ** R);
void	 putStats();										// Put stats to the bottom right corner.
void	 putSubwin();										// Put a sub-window for video in bottom left corner.


double   getPSNR(int i, double rposproj, double cposproj);	// get PSNR from one single pixel (Squared Error).
double ** getinverse4landscape( double r, double c, int transMxCnt );

int myfloor (double numin);
int myceil  (double numin);




void writeMozaic()
{
	
	ImageChar * currFrame;
	

//	int pixcnt;									// # of pixels cnt for PSNR.
//	int lines_ext = line_ext_factor;
//	int pels_ext  = pel_ext_factor;
	
	int RN, CN;
	int lines, pels;
	int r, c, i;
	int Mptr = 0;
	
	int transMxCnt;									// # of transform coefficients set stored.
	int r_left, r_right, c_upper, c_bottom;			// the landscape ( r,c ) boundary

	double ** O;
	double ** P;
	double ** Q;
	double ** R;
	double rMin, rMax, cMin, cMax, OR, RQ, QP, OP;

	double rposproj, cposproj;						// warping position in the current frame.
	double aa, bb, cc, dd, ee, ff, gg, hh;			// global transform coefficients.
	double O00, O01, P00, P01, Q00, Q01, R00, R01;	// warped bounding area.

	unsigned char lowerL, lowerR, upperL, upperR;
	double r_weight, c_weight;
	double weighted_L, weighted_R;

													// 2009-04-06 Use interpolation for raster-scan matching.

	//double Aproj, Bproj, Cproj, Dproj, R_ratio, C_ratio;

	//double SE;			// accumulated Squared Error for PSNR check.
	//double MSE;			// Mean Squared Error for PSNR check.

	VectorXd Ce(8);		// coefficients.
	Matrix3d C;	// coefficient matrix.
	Matrix3d Cg;		// global coefficient matrix.
	MatrixXd Cd(1,3);
	MatrixXd xyRfc(3,1);	// coordinate matrix in Refernce frame [x y 1]'.
	MatrixXd xyCrt(3,1);	// coordinate matrix in Current frame  [X Y 1]'.

	currFrame = OFM->currFrame;
	lines	  = currFrame->lines;
	pels	  = currFrame->pels;
	RN		  = lines / BLOCKSIZE;
	CN		  = pels / BLOCKSIZE;

	transMxCnt = transMatrix->cnt;

		aa	= transMatrix->glbTRFM[0][0];
		bb	= transMatrix->glbTRFM[0][1];
		cc	= transMatrix->glbTRFM[0][2];
		dd	= transMatrix->glbTRFM[0][3];
		ee	= transMatrix->glbTRFM[0][4];
		ff	= transMatrix->glbTRFM[0][5];
		gg	= transMatrix->glbTRFM[0][6];
		hh	= transMatrix->glbTRFM[0][7];
									

											
	//--------------------------------- "alterable method" II ---------------------------------//
	

	// Find window 'OPQR' explored in Landscape from current frame.
	O = getinverse4landscape(		0 ,		0, transMxCnt );	//	O-----R
	P = getinverse4landscape((lines-1),		0, transMxCnt );	//	|     |
	Q = getinverse4landscape((lines-1), (pels-1), transMxCnt );	//	|     |
	R = getinverse4landscape(		0 , (pels-1), transMxCnt );	//	P-----Q

	rMin = min( min(O[0][0],P[0][0]), min(Q[0][0],R[0][0]) );
	rMax = max( max(O[0][0],P[0][0]), max(Q[0][0],R[0][0]) );

	cMin = min( min(O[0][1],P[0][1]), min(Q[0][1],R[0][1]) );
	cMax = max( max(O[0][1],P[0][1]), max(Q[0][1],R[0][1]) );

	//printf("\n\nrMax = %f rMin = %f \n\n",rMax,rMin);

	/*-----------------------------------------*/
	// write O-R-Q-P coordinates to header file.
	/*-----------------------------------------*/
	O00	= O[0][0] + ( lines * (lines_ext_factor-1)/2 );	O01	= O[0][1] + (pels  * (pels_ext_factor -1)/2);
	P00	= P[0][0] + ( lines * (lines_ext_factor-1)/2 );	P01	= P[0][1] + (pels  * (pels_ext_factor -1)/2);
	Q00	= Q[0][0] + ( lines * (lines_ext_factor-1)/2 );	Q01	= Q[0][1] + (pels  * (pels_ext_factor -1)/2);
	R00	= R[0][0] + ( lines * (lines_ext_factor-1)/2 );	R01	= R[0][1] + (pels  * (pels_ext_factor -1)/2);

	//printf("\nO = %5f %5f\n\n", *(O[0] + 0), *(O[0] + 1) );

	//--------------------------------- PSNR check ---------------------------------//
	
	//--------------------------------- PSNR check ---------------------------------//

	Landscape->useNewframe = 0;
	//if ( MSE < MSE_threshold )	// PSNR threshold check.
	if(0<1)
	{
		Landscape->useNewframe = 1;
	}

	//-------------------------------- Update Landscape if pass PSNR check.
	
	if ( Landscape->useNewframe )	// PSNR threshold is 20 dB.
	{

		OR			= (R[0][0]-O[0][0])/(R[0][1]-O[0][1]);	// slopes of boundary lines.
		RQ			= (Q[0][0]-R[0][0])/(Q[0][1]-R[0][1]);
		QP			= (P[0][0]-Q[0][0])/(P[0][1]-Q[0][1]);
		OP			= (P[0][0]-O[0][0])/(P[0][1]-O[0][1]);

		r_left		= - lines * (lines_ext_factor-1)/2 ;
		r_right		= (lines-1) + lines * (lines_ext_factor-1)/2 ;
		c_upper		= - pels  * (pels_ext_factor -1)/2 ;
		c_bottom	= (pels-1) + pels  * (pels_ext_factor -1)/2 ;

		O00			= O[0][0];		O01	=	O[0][1];
		P00			= P[0][0];		P01	=	P[0][1];
		Q00			= Q[0][0];		Q01	=	Q[0][1];
		R00			= R[0][0];		R01	=	R[0][1];


	for (r = myfloor(rMin); r < myceil(rMax); r++)
	{
		for (c = myfloor(cMin); c < myceil(cMax); c++)
		{
			// check if (r,c) is inside the landscape.
			if( ( r>= r_left   )	&&
				( r<= r_right  )	&&
				( c>= c_upper  )	&&
				( c<= c_bottom )		)
			{
				i = ( lines * (lines_ext_factor-1)/2 ) * ( pels  * pels_ext_factor )	+ 
					r * ( pels  * pels_ext_factor )							+ 
					( c + pels  * (pels_ext_factor -1)/2 );

				//if ( *(xyMap->pixF + i) == 0 )	// the positioin that has not been explored.
				if (1)
				{
					
					
					// check if (r,c) is inside the current explored window.
					// 4 IFs check the 4 edges of the window.
					if ( (( r - O00 - OR*(c-O01) )*( Q00 - O00 - OR*(Q01 -O01) ) >= 0)&&
						 (( r - R00 - RQ*(c-R01) )*( O00 - R00 - RQ*(O01 -R01) ) >= 0)&&
						 (( r - Q00 - QP*(c-Q01) )*( O00 - Q00 - QP*(O01 -Q01) ) >= 0)&&
						 (( r - O00 - OP*(c-O01) )*( R00 - O00 - OP*(R01 -O01) ) >= 0)	)
					{
							
									// find forward mapping.
									{
										
											/*	 Calculate warping straight forwardly	*/
											
											rposproj			  = ( r * aa +
																	  c * bb +
																	  cc )		/
																	( r * gg +
																	  c * hh +
																	  1 );

											cposproj			  = ( r * dd +
																	  c * ee +
																	  ff )		/
																	( r * gg +
																	  c * hh +
																	  1);


									}// end of forward mapping.

									// Update landscape.
									if ( ( rposproj	>=0 )				&&
										 ( rposproj	<=(lines-1) )		&&
										 ( cposproj	>=0 )				&&
										 ( cposproj	<=(pels-1) ) )
									{
										//renewMap( i, xyCrt(0,0), xyCrt(1,0) );
										

	

										lowerL    = *(currFrame->data + (int)(rposproj +1 ) *pels	+ (int)(cposproj +1 ) );
										upperL	  = *(currFrame->data + (int)(rposproj)*pels		+ (int)(cposproj) );
										upperR    = *(currFrame->data + (int)(rposproj)*pels		+ (int)(cposproj) );
										lowerR    = *(currFrame->data + (int)(rposproj +1 ) *pels	+ (int)(cposproj +1) );

	
    									r_weight  = 1 - ( rposproj - (int)(rposproj) );
    									c_weight  = 1 - ( cposproj - (int)(cposproj) );

										weighted_L						= upperL * r_weight + lowerL * ( 1 - r_weight );
										weighted_R						= upperR * r_weight + lowerR * ( 1 - r_weight );

										*(Landscape->data + i ) = weighted_L * c_weight + weighted_R * ( 1 - c_weight);
	
										//*(Landscape->data + i ) = (int)( ( lowerL+upperL+upperR+lowerR ) / 4 );



										// Set the pixel flag for explored map.
										//*(xyMap->pixF + i)    = 1;	                                     //2009 05 25	
									}// end of updating landscape.

								
							
						
					}//end if in the current explored window.

				}// end of if not been explored.

			}// end of if-(r,c)-inside-the-landscape.

			else
			{
				*(Landscape->Reset_flag)	= 1;
			}


		}// end of 'c' for-loop.	

	}// end of 'r' for-loop.




	}// end of IF pass PSNR check.






	

}//end: writeMozaic.



double ** getinverse4landscape( double X, double Y, int transMxCnt )
{
	double a, b, c, d, e, f, g, h, yD, xD, x, y;
	double ** xy;
	//int	   i, j;
	int	j;

	xy    = (double **)malloc( sizeof(double*) );
	xy[0] = (double*)malloc(2*sizeof(double));

	//for (i=0; i<=transMxCnt; i++)
	{
		

		j = 0;							//use global transform, 03072007.
		a = transMatrix->glbTRFM[j][0];
		b = transMatrix->glbTRFM[j][1];
		c = transMatrix->glbTRFM[j][2];
		d = transMatrix->glbTRFM[j][3];
		e = transMatrix->glbTRFM[j][4];
		f = transMatrix->glbTRFM[j][5];
		g = transMatrix->glbTRFM[j][6];
		h = transMatrix->glbTRFM[j][7];

		yD = (d - g*Y)*(b - h*X) - (e - h*Y)*(a - g*X);
		xD = (a - g*X);

		if (yD == 0)
			yD = 0.00000001;
		if (xD == 0)
			xD = 0.00000001;

		y  = ( (d - g*Y)*(X - c) - (a - g*X)*(Y - f) ) / yD;
		x  = ( (X - c) - (b - h*X)*y ) / xD;

		X  = x;
		Y  = y;

	}		// end inverse trace loop.
	xy[0][0] = x;
	xy[0][1] = y;
	//--------------------------------- Display ---------------------------------//
		//printf("\n\nx = %f y = %f \n\n",x,y);

	return xy;
}//end: double ** getinverse4landscape.



#endif