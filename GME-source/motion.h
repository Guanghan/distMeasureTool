#ifndef MOTION_H
#define MOTION_H

#include "image.h"
#include "defines.h"

#define ABS(x) (x<0)?(-(x)):(x)

int SADMB(
		  MBInt *currMB, 
		  Image * ref, 
		  int xpos, 
		  int ypos
		  );

int fullSearch(
			   MBInt *currMB, 
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *MV,
			   int *minSAD
			   );

int levelSearch(
			   MBInt *currMB,			
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *predMV,
			   MotionVector *MV,
			   int *minSAD
			   );
int threeStepMotionPredictionMB(
			   MBInt *currMB, 
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *MV,
			   int *meSAD,
			   MBInt * predMB,
			   EncoderHandle * enc
			   );

/*-----------------------------------------------------------------*/





int SADMB(
		  MBInt *currMB, 
		  Image * ref, 
		  int xpos, 
		  int ypos
		  )
{
	
	unsigned char *refPtr;
	int i;
	int tmp, sad;

	sad = 0;

	for(i=0; i<16; i++)
	{
		refPtr = ref->lum + (ypos + i) * ref->epels + xpos;

		tmp = (currMB->lum[i][0]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][1]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][2]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][3]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][4]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][5]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][6]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][7]) - (*refPtr++); sad += ABS(tmp);

		tmp = (currMB->lum[i][8]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][9]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][10]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][11]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][12]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][13]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][14]) - (*refPtr++); sad += ABS(tmp);
		tmp = (currMB->lum[i][15]) - (*refPtr++); sad += ABS(tmp);
	}

	return sad;

}


int SADMBIntra(
		  MBInt *currMB
		  )
{
	
	int i;
	int tmp, sad;

	sad = 0;

	for(i=0; i<16; i++)
	{

		tmp = (currMB->lum[i][0]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][1]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][2]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][3]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][4]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][5]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][6]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][7]) - 128; sad += ABS(tmp);

		tmp = (currMB->lum[i][8]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][9]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][10]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][11]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][12]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][13]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][14]) - 128; sad += ABS(tmp);
		tmp = (currMB->lum[i][15]) - 128; sad += ABS(tmp);
	}

	return sad;

}


int fullSearch(
			   MBInt *currMB, 
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *MV,
			   int *meSAD
			   )
{
	int xpos, ypos;
	int mx, my;
	int nSearchRange, pSearchRange;
	int lines, pels;
	int sad, minSAD;
	int minMVX, minMVY;

	
	nSearchRange = (-1) * searchRange + 1;
	pSearchRange = searchRange;

	lines = ref->lines;
	pels = ref->pels;

	if(searchRange > ref->edge)
	{
		errorMessage((unsigned char *)"Search range is larger than padding edge");
	}


	minSAD = 65526;
	minMVX = 0;
	minMVY = 0;

	for(mx=nSearchRange; mx<pSearchRange; mx++)
	for(my=nSearchRange; my<pSearchRange; my++)
	{
		xpos = currX + mx;
		ypos = currY + my;

		sad = SADMB(currMB, ref, xpos, ypos);

		if (sad < minSAD)
		{
			minMVX = mx;
			minMVY = my;
			minSAD = sad;
		}

	}


	MV->x = minMVX;
	MV->y = minMVY;
	MV->x_half = 0;
	MV->y_half = 0;
	*meSAD = minSAD;


	return 0;

}


int threeStepMotionPredictionMB(
			   MBInt *currMB, 
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *MV,
			   int *meSAD,
			   MBInt *predMB,
			   EncoderHandle * enc
			   )
{
	int xpos, ypos;
	int mx, my, m;
	int lines, pels;
	int sad, minSAD;
	int minMVX, minMVY;
	int stepSize;
	int centerX, centerY;
	int sadIntra;
	int numSAD;

	int POS[8][2]={{0, 1}, {0, -1},  {1, 0}, {-1, 0},  {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}; 


	lines = ref->lines;
	pels = ref->pels;
	numSAD = 0;

	if(searchRange > ref->edge)
	{
		errorMessage((unsigned char *)"Search range is larger than padding edge");
	}

	/*--Intra Mode--*/
	sadIntra = SADMBIntra(currMB);

	/*-- (0, 0) position --*/
	mx = 0;
	my = 0; 
	xpos = currX + mx;
	ypos = currY + my;
	sad = SADMB(currMB, ref, xpos, ypos);

	minSAD = sad;
	minMVX = mx;
	minMVY = my;

	/*--three steps--*/

	stepSize = searchRange / 2;
	centerX = 0;
	centerY = 0;

	while (stepSize >= 1)
	{
		for(m=0; m<8; m++)
		{
			mx = centerX + POS[m][0] * stepSize;
			my = centerY + POS[m][1] * stepSize;
			xpos = currX + mx;
			ypos = currY + my;
			sad = SADMB(currMB, ref, xpos, ypos);
			numSAD++;

			if (sad < minSAD)
			{
				minMVX = mx;
				minMVY = my;
				minSAD = sad;
			}

		}

		centerX = minMVX;
		centerY = minMVY;

		stepSize = stepSize / 2;

	}

	/*--mode decision--*/
	if(minSAD < sadIntra)
	{
		/*inter mode*/
		MV->x = minMVX;
		MV->y = minMVY;
		MV->x_half = 0;
		MV->y_half = 0;
		*meSAD = minSAD;

		/*set the prediction reference*/
		fillMB(ref, currX+MV->x, currY+MV->y, predMB);
		enc->mbType = MODE_INTER;
		(enc->currMV).x = MV->x;
		(enc->currMV).y = MV->y;
	}
	else
	{
		/*intra mode*/
		MV->x = 0;
		MV->y = 0;
		MV->x_half = 0;
		MV->y_half = 0;
		*meSAD = sadIntra;

		enc->mbType = MODE_INTRA;
		(enc->currMV).x = MV->x;
		(enc->currMV).y = MV->y;
	}
	


	/*printf("[%d %d]  ", minMVX, minMVY); */

	return numSAD;

}



int diamondMotionPredictionMB(
			   MBInt *currMB, 
			   Image *ref,
			   int currX,
			   int currY,
			   int searchRange,
			   MotionVector *MV,
			   int *meSAD,
			   MBInt *predMB,
			   EncoderHandle * enc
			   )
{
	int xpos, ypos;
	int mx, my, m, k;
	int lines, pels;
	int sad, minSAD;
	int minMVX, minMVY;
	int centerX, centerY;
	int sadIntra;
	int predMVX, predMVY;
	int diamond[4][2]={{1,0}, {-1, 0}, {0, 1}, {0, -1}};
	/*used to save the searched positions*/
	int pastMVS[5][2]={{-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}};
	int pastMVSize = 5;
	int pastMVPtr = 0;
	int stopNow = 0;
	int hasBeenSearched;
	int diamondHasMinSAD;
	int numSAD;

	lines = ref->lines;
	pels = ref->pels;

	if(searchRange > ref->edge)
	{
		errorMessage((unsigned char *)"Search range is larger than padding edge");
	}

	/*--Intra Mode--*/
	sadIntra = SADMBIntra(currMB);

	/*-- (0, 0) position --*/
	mx = 0;
	my = 0; 
	xpos = currX + mx;
	ypos = currY + my;
	sad = SADMB(currMB, ref, xpos, ypos);
	numSAD = 1;
	minSAD = sad;
	minMVX = mx;
	minMVY = my;
	pastMVS[pastMVPtr][0] = xpos;
	pastMVS[pastMVPtr][1] = ypos;
	pastMVPtr++;



	/*--the predicted position--*/
	predMVX = (enc->predMV)->x;
	predMVY = (enc->predMV)->y;
	if((predMVX !=0) || (predMVY != 0))
	{
		xpos = currX + mx;
		ypos = currY + my;
		sad = SADMB(currMB, ref, xpos, ypos);
		numSAD++;
		pastMVS[pastMVPtr][0] = xpos;
		pastMVS[pastMVPtr][1] = ypos;
		pastMVPtr++;

		if(sad < minSAD)
		{
			minSAD = sad;
			minMVX = predMVX;
			minMVY = predMVY;
		}
	}

	stopNow = 0;
	centerX = currX + minMVX;
	centerY = currY + minMVY;
	
	while(stopNow == 0)
	{
		diamondHasMinSAD = 0;
		for(m=0; m<4; m++)
		{
			xpos = centerX + diamond[m][0];
			ypos = centerY + diamond[m][1];

			/*check if it has been searched*/
			hasBeenSearched = 0;
			for(k=0; k<pastMVSize; k++)
			{
				if((pastMVS[k][0]== xpos) && (pastMVS[k][1]==ypos))
					hasBeenSearched = 1;
			}

			if(hasBeenSearched == 0)
			{
				/*find SAD*/
				sad = SADMB(currMB, ref, xpos, ypos);
				numSAD++;

				pastMVS[pastMVPtr][0] = xpos;
				pastMVS[pastMVPtr][1] = ypos;
				pastMVPtr++;
				if(pastMVPtr == pastMVSize) pastMVPtr = 0;


				if(sad < minSAD)
				{
					diamondHasMinSAD = 1;
					minSAD = sad;
					minMVX = xpos - currX;
					minMVY = ypos - currY;
				}
			}


		}

		/*if not center min and not reach edge continue search*/ 
		if((diamondHasMinSAD == 1)
			&& (minMVX < (searchRange-1)) && (minMVX > (-searchRange+1))
			&& (minMVY < (searchRange-1)) && (minMVY > (-searchRange+1)) )
		{
			centerX = currX + minMVX;
			centerY = currY + minMVY;
			stopNow = 0;
		}
		else
		{
			stopNow = 1;
		}

		if(numSAD > MAX_SAD_PER_MB)
			stopNow = 1;	
	}
	
	/*--mode decision--*/
	if(minSAD < sadIntra)
	{
		/*inter mode*/
		MV->x = minMVX;
		MV->y = minMVY;
		MV->x_half = 0;
		MV->y_half = 0;
		*meSAD = minSAD;

		/*set the prediction reference*/
		fillMB(ref, currX+MV->x, currY+MV->y, predMB);
		enc->mbType = MODE_INTER;
		(enc->currMV).x = MV->x;
		(enc->currMV).y = MV->y;
	}
	else
	{
		/*intra mode*/
		MV->x = 0;
		MV->y = 0;
		MV->x_half = 0;
		MV->y_half = 0;
		*meSAD = sadIntra;

		enc->mbType = MODE_INTRA;
		(enc->currMV).x = MV->x;
		(enc->currMV).y = MV->y;
	}
	


	/*printf("[%d %d]  ", minMVX, minMVY); */

	return numSAD;

}





#endif
