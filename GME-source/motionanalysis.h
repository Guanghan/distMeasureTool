/**************************************************************************
* Program Name: uavOFMA
*
* Filename: motionanalysis.h
*
* Description:
*  
*  Camera motion estimation.
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

#ifndef MOTION_ANALYSIS_H
#define MOTION_ANALYSIS_H
	
//#include "feature.h"
#include "blockmotion.h"
//#include "LMSE.h"
//#include "range.h"

//-----function prototype
void extendedTranslationMotionSearch(BlockInt * currBlk, int rpos, int cpos, ImageChar * refFrame, 
							 BlockClassStat * classMap);
//-----

void invariantMotionEstimation()
{
	ImageChar * currFrame;
	ImageChar * refFrame;
	FeatureSetStruct * currBlkFeature;
	FeatureSetStruct * refBlkFeature;
	BlockInt * currBlk;


	int RN, CN, rpos, cpos;
	int r, c;
	int lines, pels;
	BlockClassStat * classMap;
	
	currFrame = OFM->currFrame;
	refFrame = OFM->refFrame;
	currBlkFeature = OFM->currBlockFeature;
	refBlkFeature = OFM->refBlockFeature;

	lines = currFrame->lines;
	pels = currFrame->pels;
	RN = lines / BLOCKSIZE;
	CN = pels / BLOCKSIZE;
	currBlk = OFM->currBlk;



	
	//find the motion match for the structural blocks
	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		classMap = OFM->classMap + r * CN + c;
		if(classMap->useForMotion == 1)
		{
			//if the block is structural
			rpos = r * BLOCKSIZE;
			cpos = c * BLOCKSIZE;

			//get block data
			fillBlockFromCharImage(currFrame, rpos, cpos, currBlk);
			extendedTranslationMotionSearch(currBlk, rpos, cpos, refFrame, classMap);

		}
		else
		{
			classMap->MVc = 0;
			classMap->MVr = 0;
		}
	}


}


/*---------------------------------------------------------
 * This function will perform extended motion search. 
 *--------------------------------------------------------*/

void extendedTranslationMotionSearch(BlockInt * currBlk, int rpos, int cpos, ImageChar * refFrame, 
									 BlockClassStat * classMap)
{
	BlockInt  * refBlk;
	ImageChar * currFrame = OFM->currFrame;
	int r, c, MVr, MVc;
	int SAD, minSAD;
	int searchRange;
	int k;
	int maxSAD, maxSADIndex; 
	int sumMVr, sumMVc, ct, meanMinMVr, meanMinMVc;
	int SADminus, SADplus, SADzero, SADave;
	// 3-Step MV search.
	int POS[8][2]={{0, 1}, {0, -1},  {1, 0}, {-1, 0},  {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	int stepSize, m, centerX, centerY;									
	int predMVX, predMVY;
	// Diamond MV search.
	int diamond[4][2]={{1,0}, {-1, 0}, {0, 1}, {0, -1}};
	// to save the searched positions*/
	int pastMVS[5][2]={{-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}, {-100, -100}};
	int pastMVSize = 5;
	int pastMVPtr = 0;
	int stopNow = 0;
	int hasBeenSearched;
	int diamondHasMinSAD;
	int numSAD = 0;
	// serching step limit for Diamond.
	int MAX_SAD_PER_MB = 100;

	double reliability;
	


	BlockMotionStat * MVCandidates;
	int nMVCandidates; 


	//nMVCandidates = OFM->nMVCandidates;          2009 05 27
	//MVCandidates = classMap->MVCandidates;

	// initialize, set them to the maximum SAD;
	//maxSAD = 1000000000;
	//maxSADIndex = 0;


	searchRange = MV_RANGE;
	minSAD = 10000000;
	refBlk = OFM->refBlk;
	r	= 0;
	c	= 0;
	
	
	/*--the predicted position with 3-Step search--*/

	//predMVX = MVr;														//2009-05-27
	//predMVY = MVc;

	/*--the predicted position using origin--*/

	predMVX = 0;
	predMVY = 0;
	//SAD		= blockSAD(currBlk, refFrame, rpos, cpos);
	SAD		= FastBlockSAD(currFrame, refFrame, rpos, cpos, rpos, cpos);

	//SAD	= minSAD;														//2009-05-27
	pastMVS[pastMVPtr][0] = predMVX;
	pastMVS[pastMVPtr][1] = predMVY;
	pastMVPtr++;

	if(SAD < minSAD)
		{
			MVr = r;
			MVc = c;
			minSAD = SAD;
		}

	numSAD++;

	/*-- Diamond searching loop --*/
	stopNow = 0;
	centerX = MVr;
	centerY = MVc;

	while(stopNow == 0)
	{
		diamondHasMinSAD = 0;
		for(m=0; m<4; m++)
		{
			r = centerX + diamond[m][0];
			c = centerY + diamond[m][1];

			// check if it has been searched
			hasBeenSearched = 0;
			for(k=0; k<pastMVSize; k++)
			{
				if((pastMVS[k][0]== r) && (pastMVS[k][1]==c))
					hasBeenSearched = 1;
			}

			if(hasBeenSearched == 0)
			{
				// find SAD
				//fillBlockFromCharImage(refFrame, rpos + r, cpos + c, refBlk);
				//SAD = blockSAD(currBlk, refBlk);
				SAD		= FastBlockSAD(currFrame, refFrame, rpos + r, cpos + c, rpos, cpos);
				//SAD = blockSAD(currBlk, refFrame, rpos + r, cpos + c);
				numSAD++;

				pastMVS[pastMVPtr][0] = r;
				pastMVS[pastMVPtr][1] = c;
				pastMVPtr++;
				if(pastMVPtr == pastMVSize) pastMVPtr = 0;

				

				if(SAD < minSAD)
				{
					diamondHasMinSAD = 1;
					MVr = r;
					MVc = c;
					minSAD = SAD;
				}
			}


		}//end: for(m=0; m<4; m++)

		/*if not center min and not reach edge continue search*/ 
		if((diamondHasMinSAD == 1)
			&& (r < (searchRange-1)) && (r > (-searchRange+1))
			&& (c < (searchRange-1)) && (c > (-searchRange+1)) )
		{
			centerX = MVr;
			centerY = MVc;
			stopNow = 0;
		}
		else
		{
			stopNow = 1;
		}

		if(numSAD > MAX_SAD_PER_MB)
			stopNow = 1;	
	}

	// End of Diamond.

	

	classMap->MVr = MVr;
	classMap->MVc = MVc;
	classMap->minSAD = minSAD;



#if 0

	//print out the candidate motion vectors
	printf("\n MB [%4d %4d] %4d %8.3f = ", rpos, cpos, ct, reliability);
	for(k=0; k<nMVCandidates; k++)
	{
		
		if((MVCandidates + k)->minSAD == minSAD)
			printf("*");
		printf("[%d %d %d] ", (MVCandidates + k)->minSAD, (MVCandidates + k)->MVr, (MVCandidates + k)->MVc);
	}
#endif



	return;

}







#endif