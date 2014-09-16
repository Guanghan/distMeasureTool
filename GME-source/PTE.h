/**************************************************************************
* Program Name: uavOFMA
*
* Filename: pte.h
*
* Description:
*  
*  Compute perspective transform martix for video registration.
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
* Yu-chia(York) Chung
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
**************************************************************************/
#include "global.h"

#include "mosaic.h"

#include "Eigen/Core"
#include "Eigen/Array"
#include "Eigen/LU"
#include "Eigen/Cholesky"

USING_PART_OF_NAMESPACE_EIGEN


void doPTE ()
{
	ImageChar * currFrame;

//	int lines_ext = line_ext_factor;
//	int pels_ext  = pel_ext_factor;
	
	int RN, CN;
	int lines, pels;
	int r, c, i;
	int Mptr = 0;
	int StructBlkCnt;	// # of structure blks in current frame.
	int transMxCnt;		// # of transform coefficients set stored.


	BlockClassStat * classMap;

	currFrame = OFM->currFrame;
	lines	  = currFrame->lines;
	pels	  = currFrame->pels;
	RN		  = lines / BLOCKSIZE;
	CN		  = pels / BLOCKSIZE;

	// Get Coordinates.

	StructBlkCnt = 0;

	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		classMap = OFM->classMap + r * CN + c;
		if(classMap->useForMotion == 1)
		{
			*(frameMapXY->x + Mptr) = r * BLOCKSIZE;
			*(frameMapXY->y + Mptr) = c * BLOCKSIZE;

			*(frameMapxy->x + Mptr) = r * BLOCKSIZE + classMap->MVr;
			*(frameMapxy->y + Mptr) = c * BLOCKSIZE + classMap->MVc;

			Mptr ++;
			StructBlkCnt ++;
		}

	}

	// Get Transform Coefficients.
	MatrixXd A(StructBlkCnt*2,8);          //Pm:ning
	MatrixXd B(StructBlkCnt*2,1);          //Qm:ning
	VectorXd Ce(8);			// coefficients.    //W. should be all 1: ning
	VectorXd Bb(8);			// A.transpose() * B.
	Matrix3d C;				// coefficient matrix.
	Matrix3d Cg;			// global coefficient matrix.
	MatrixXd Cd(1,3);
	MatrixXd xyRfc(3,1);	// coordinate matrix in Refernce frame [x y 1]'.
	MatrixXd xyCrt(3,1);	// coordinate matrix in Current frame  [X Y 1]'.
	MatrixXd temp(8,8);

	for(i=0; i<StructBlkCnt; i++)
	{
		A(2*i,0) = *(frameMapxy->x + i);							  A(2*i+1,0) = 0;
		A(2*i,1) = *(frameMapxy->y + i);							  A(2*i+1,1) = 0;
		A(2*i,2) = 1;												  A(2*i+1,2) = 0;
		A(2*i,3) = 0;												  A(2*i+1,3) = *(frameMapxy->x + i);
		A(2*i,4) = 0;												  A(2*i+1,4) = *(frameMapxy->y + i);
		A(2*i,5) = 0;												  A(2*i+1,5) = 1;
		A(2*i,6) = *(frameMapxy->x + i)*(-1)*( *(frameMapXY->x + i) );A(2*i+1,6) = *(frameMapxy->x + i)*(-1)*( *(frameMapXY->y + i) );
		A(2*i,7) = *(frameMapxy->y + i)*(-1)*( *(frameMapXY->x + i) );A(2*i+1,7) = *(frameMapxy->y + i)*(-1)*( *(frameMapXY->y + i) );

		B(2*i,0)   = *(frameMapXY->x + i);
		B(2*i+1,0) = *(frameMapXY->y + i);
	}	//end of creating matrice A B.
	
	//temp = (~A)*A;
	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce);

	//temp /= 1e+200;
	//Ce   = A.Solve(B); 

	//--------------------------------- Display ---------------------------------//
	//cout << "\n\nMatrix A'*A = \n\n" << temp << endl;	// display the Matrix for inversion check.
	//cout << "\n\nMatrix    A = \n\n" << A    << endl;	// display the Matrix for inversion check.
	//cout << "\n\nMatrix    B = \n\n" << B    << endl;	// display the Matrix for inversion check.
	//printf("\n\nB(190) = %f \n\n",*(frameMapXY->x + 95) );
	//printf("\n\nB(189) = %f \n\n",*(frameMapXY->y + 94) );
	//--------------------------------- Display ---------------------------------//
	//temp = ! temp;
	//Ce   = temp  * (~A) * B;

	// Load Global Transform Coefficients.
	C(0,0) = Ce(0);	Cg(0,0) = transMatrix->glbTRFM[0][0];
	C(0,1) = Ce(1);	Cg(0,1) = transMatrix->glbTRFM[0][1];
	C(0,2) = Ce(2);	Cg(0,2) = transMatrix->glbTRFM[0][2];	
	C(1,0) = Ce(3);	Cg(1,0) = transMatrix->glbTRFM[0][3];
	C(1,1) = Ce(4);	Cg(1,1) = transMatrix->glbTRFM[0][4];
	C(1,2) = Ce(5);	Cg(1,2) = transMatrix->glbTRFM[0][5];
	C(2,0) = Ce(6);	Cg(2,0) = transMatrix->glbTRFM[0][6];
	C(2,1) = Ce(7);	Cg(2,1) = transMatrix->glbTRFM[0][7];
	C(2,2) = 1;			Cg(2,2) = transMatrix->glbTRFM[0][8];

	Cg	= Cg * C;

	transMatrix->glbTRFM[0][0]	=Cg(0,0);
	transMatrix->glbTRFM[0][1]	=Cg(0,1);
	transMatrix->glbTRFM[0][2]	=Cg(0,2);
	transMatrix->glbTRFM[0][3]	=Cg(1,0);
	transMatrix->glbTRFM[0][4]	=Cg(1,1);
	transMatrix->glbTRFM[0][5]	=Cg(1,2);
	transMatrix->glbTRFM[0][6]	=Cg(2,0);
	transMatrix->glbTRFM[0][7]	=Cg(2,1);
	transMatrix->glbTRFM[0][8]	=Cg(2,2);

	// Save Transform coefficients for future use.
	transMxCnt = transMatrix->cnt;
	for (i=0; i<9; i++)
	{
		transMatrix->data[transMxCnt][i] = transMatrix->glbTRFM[0][i];

		// write Transform to the header.

		//fwrite( transMatrix->glbTRFM[0] + i, 1, 8, Landscape->VideoHeaderFP);
		//if (i==0) printf("\nC[0 0] %5f\n\n", transMatrix->glbTRFM[0][0] );
	}
	transMatrix->cnt = transMxCnt + 1;
	
	
	//writeMozaic();



}	// end of doPTE.


int myfloor (double numin)
{
	int	   temp;

	temp   = (int) numin;

	return temp;
}


int myceil (double numin)
{
	int	   temp;

	temp   = (int) numin + 1;

	return temp;
}


void   renewMap(int i, double rposproj, double cposproj)
{
	int pels;
	unsigned char lowerL, lowerR, upperL, upperR;
	unsigned char r_weight, c_weight;
	unsigned char weighted_L, weighted_R;
	
	ImageChar * frameBuf;
	frameBuf  = OFM->currFrame;
	pels	  = frameBuf->pels;

	lowerL    = *(frameBuf->data + myceil(rposproj) *pels + myfloor(cposproj) );
	upperL	  = *(frameBuf->data + myfloor(rposproj)*pels + myfloor(cposproj) );
	upperR    = *(frameBuf->data + myfloor(rposproj)*pels + myceil(cposproj) );
	lowerR    = *(frameBuf->data + myceil(rposproj) *pels + myceil(cposproj) );

    r_weight  = 1 - ( rposproj - myfloor(rposproj) );
    c_weight  = 1 - ( cposproj - myfloor(cposproj) );

	weighted_L						= upperL * r_weight + lowerL * ( 1 - r_weight );
	weighted_R						= upperR * r_weight + lowerR * ( 1 - r_weight );

	*(Landscape->data + i ) = weighted_L * c_weight + weighted_R * ( 1 - c_weight);
}


double   getPSNR(int i, double rposproj, double cposproj)
{
	int pels;
	double		  PSNR;
	unsigned char lowerL, lowerR, upperL, upperR;
	unsigned char r_weight, c_weight;
	unsigned char weighted_L, weighted_R;
	
	ImageChar * frameBuf;
	frameBuf  = OFM->currFrame;
	pels	  = frameBuf->pels;

	lowerL    = *(frameBuf->data + myceil(rposproj) *pels + myfloor(cposproj) );
	upperL	  = *(frameBuf->data + myfloor(rposproj)*pels + myfloor(cposproj) );
	upperR    = *(frameBuf->data + myfloor(rposproj)*pels + myceil(cposproj) );
	lowerR    = *(frameBuf->data + myceil(rposproj) *pels + myceil(cposproj) );

    r_weight  = 1 - ( rposproj - myfloor(rposproj) );
    c_weight  = 1 - ( cposproj - myfloor(cposproj) );

	weighted_L						= upperL * r_weight + lowerL * ( 1 - r_weight );
	weighted_R						= upperR * r_weight + lowerR * ( 1 - r_weight );

	PSNR	  = *(Landscape->data + i ) - ( weighted_L * c_weight + weighted_R * ( 1 - c_weight) );
	PSNR	  = PSNR * PSNR;

	return PSNR;
}



void	drawWindow(double ** O,double ** P,double ** Q,double ** R)
{
	int	lines	  = Landscape->lines;
	int	pels	  = Landscape->pels;
//	int lines_ext = line_ext_factor;
//	int pels_ext  = pel_ext_factor;	
	int	nSteps, MVr, MVc, rposCenter, cposCenter, k, rpos,cpos;
	double **	A;
	double **	B;	
	double		stepR, stepC;

	//--------------------------------------------------------------------------------------------------------------------------//
	A			= O;	B = P;
	MVr			= myceil( B[0][0] ) - myfloor( A[0][0] );
	MVc			= myceil( B[0][1] ) - myfloor( A[0][1] );
	nSteps		= (int)( sqrt( (double)MVr * MVr + MVc * MVc) );
	stepR		= (MVr * 1.0)/nSteps;
	stepC		= (MVc * 1.0)/nSteps;
	rposCenter	= myfloor( A[0][0] );
	cposCenter	= myfloor( A[0][1] );

				for(k=0; k<nSteps; k++)
			{
				rpos = (int)(rposCenter + stepR * k);
				cpos = (int)(cposCenter + stepC * k);
				rpos = rpos + 240 * (lines_ext_factor-1)/2;
				cpos = cpos + 320 * (pels_ext_factor -1)/2;

				if(rpos < 0)		rpos = 0;
				if(rpos > lines - 1)	rpos = lines - 1;
				if(cpos < 0)		cpos = 0;
				if(cpos > pels - 1)		cpos = pels - 1;

				*(Landscape->data_buf + ( rpos )* pels + ( cpos ) ) = 250;
			}
	//--------------------------------------------------------------------------------------------------------------------------//
	A			= P;	B = Q;
	MVr			= myceil( B[0][0] ) - myfloor( A[0][0] );
	MVc			= myceil( B[0][1] ) - myfloor( A[0][1] );
	nSteps		= (int)( sqrt( (double)MVr * MVr + MVc * MVc) );
	stepR		= (MVr * 1.0)/nSteps;
	stepC		= (MVc * 1.0)/nSteps;
	rposCenter	= myfloor( A[0][0] );
	cposCenter	= myfloor( A[0][1] );

				for(k=0; k<nSteps; k++)
			{
				rpos = (int)(rposCenter + stepR * k);
				cpos = (int)(cposCenter + stepC * k);
				rpos = rpos + 240 * (lines_ext_factor-1)/2;
				cpos = cpos + 320 * (pels_ext_factor -1)/2;

				if(rpos < 0)		rpos = 0;
				if(rpos > lines - 1)	rpos = lines - 1;
				if(cpos < 0)		cpos = 0;
				if(cpos > pels - 1)		cpos = pels - 1;

				*(Landscape->data_buf + ( rpos )* pels + ( cpos ) ) = 250;
			}
	//--------------------------------------------------------------------------------------------------------------------------//
	A			= Q;	B = R;
	MVr			= myceil( B[0][0] ) - myfloor( A[0][0] );
	MVc			= myceil( B[0][1] ) - myfloor( A[0][1] );
	nSteps		= (int)( sqrt( (double)MVr * MVr + MVc * MVc) );
	stepR		= (MVr * 1.0)/nSteps;
	stepC		= (MVc * 1.0)/nSteps;
	rposCenter	= myfloor( A[0][0] );
	cposCenter	= myfloor( A[0][1] );

				for(k=0; k<nSteps; k++)
			{
				rpos = (int)(rposCenter + stepR * k);
				cpos = (int)(cposCenter + stepC * k);
				rpos = rpos + 240 * (lines_ext_factor-1)/2;
				cpos = cpos + 320 * (pels_ext_factor -1)/2;

				if(rpos < 0)		rpos = 0;
				if(rpos > lines - 1)	rpos = lines - 1;
				if(cpos < 0)		cpos = 0;
				if(cpos > pels - 1)		cpos = pels - 1;

				*(Landscape->data_buf + ( rpos )* pels + ( cpos ) ) = 250;
			}
	//--------------------------------------------------------------------------------------------------------------------------//
	A			= R;	B = O;
	MVr			= myceil( B[0][0] ) - myfloor( A[0][0] );
	MVc			= myceil( B[0][1] ) - myfloor( A[0][1] );
	nSteps		= (int)( sqrt( (double)MVr * MVr + MVc * MVc) );
	stepR		= (MVr * 1.0)/nSteps;
	stepC		= (MVc * 1.0)/nSteps;
	rposCenter	= myfloor( A[0][0] );
	cposCenter	= myfloor( A[0][1] );

				for(k=0; k<nSteps; k++)
			{
				rpos = (int)(rposCenter + stepR * k);
				cpos = (int)(cposCenter + stepC * k);
				rpos = rpos + 240 * (lines_ext_factor-1)/2;
				cpos = cpos + 320 * (pels_ext_factor -1)/2;

				if(rpos < 0)		rpos = 0;
				if(rpos > lines - 1)	rpos = lines - 1;
				if(cpos < 0)		cpos = 0;
				if(cpos > pels - 1)		cpos = pels - 1;

				*(Landscape->data_buf + ( rpos )* pels + ( cpos ) ) = 250;
			}
	//--------------------------------------------------------------------------------------------------------------------------//

}// end draw windows.

void putStats()
{
	int i, j;
	FILE *fp;
	fp = Landscape->display_stats;

	fseek(fp, 0, SEEK_CUR);
	fread(Landscape->inerStats, 144*120, 1, fp);
	
	for (i=0; i<144; i++)	// orientation.
	{
		for (j=0; j<120; j++)
		{
			*( Landscape->data_buf + (Landscape->lines - 146 + i)*(Landscape->pels)  + (Landscape->pels - 123+j) ) = *(Landscape->inerStats + i*120 + j);
			*(xyMap->pixF  + (Landscape->lines - 146 + i)*(Landscape->pels)  + (Landscape->pels - 123+j) ) = 1;	// mark the writen pixels.
		}
	}

}// end put stats.

void putSubwin()
{
	int i, j;
	ImageChar * frameBuf;
	frameBuf  = OFM->currFrame;

	for (i=0; i<120; i++)
	{
		for (j=0; j<160; j++)
		{
			*(Landscape->data_buf + (Landscape->lines - 121 + i)*(Landscape->pels)  + (j+1) ) = ( *(frameBuf->data + (2*i)*320+ (2*j) )+ *(frameBuf->data + (2*i+1)*320+ (2*j) )+ *(frameBuf->data + (2*i)*320+ (2*j+1) )+ *(frameBuf->data + (2*i+1)*320+ (2*j+1) )  )/4; 
			*(xyMap->pixF + (Landscape->lines - 121 + i)*(Landscape->pels)  + (j+1)  ) = 1;
		}
	}
}// end put video window.


