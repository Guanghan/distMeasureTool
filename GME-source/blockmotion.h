/**************************************************************************
* Program Name: uavOFMA
*
* Filename: blockmotion.h
*
* Description:
*  
*  Image block motion estimation.
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

#ifndef MOTION_H
#define MOTION_H
#include <math.h>

#define ABS(x) (x<0)?(-(x)):(x)

void blockTranslationMEFullSearch();
void writeMotionFieldToVideo();

//---------------------------------------------------//

void prepareReferenceFrame()
{
	ImageChar *tmp;

	//switch the curr and reference frames
	tmp = OFM->currFrame;
	OFM->currFrame = OFM->refFrame;
	OFM->refFrame = tmp;
}

void motionEstimation()
{
	blockTranslationMEFullSearch();
	writeMotionFieldToVideo();
}


int blockSAD(
		  BlockInt * currBlk, 
		  ImageChar *image, 
		  int rpos, 
		  int cpos
		  )
{
	int i, r;
	int tmp, sad;
	int lines, pels;
	unsigned char *refPtr;
	lines	= image->lines;
	pels	= image->pels;
	sad		= 0;


	//r = rpos + i;
	r = rpos;
	refPtr = image->data + r * pels + cpos;

	for(i=0; i< currBlk->size ; i++)
	{

		tmp = (currBlk->data[i][0]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][1]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][2]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][3]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][4]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][5]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][6]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][7]) - (int)*(refPtr++); sad += ABS(tmp);

		tmp = (currBlk->data[i][8])  - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][9])  - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][10]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][11]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][12]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][13]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][14]) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = (currBlk->data[i][15]) - (int)*(refPtr++); sad += ABS(tmp);

		refPtr	= refPtr + pels - currBlk->size;

	}

	return sad;

}

int FastBlockSAD2(
		  ImageChar *imageCur, 
		  ImageChar *imageRef, 
		  int rposRef, 
		  int cposRef,
		  int rposCur,
		  int cposCur
		  )
{
	int i, r, rCurr;
	int tmp, sad;
	int lines, pels;
	int BlkSize		= OFM->currBlk->size;
	unsigned char *refPtr;
	unsigned char *curPtr;
	lines	= imageRef->lines;
	pels	= imageRef->pels;
	sad		= 0;


	//r = rposRef + i;
	
	refPtr	= imageRef->data + rposRef * pels + cposRef;
	
	curPtr	= imageCur->data + rposCur * pels + cposCur;

	for(i=0; i< BlkSize ; i++)
	{

		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);

		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - (int)*(refPtr++); sad += ABS(tmp);

		refPtr	= refPtr + pels - BlkSize;
		curPtr	= curPtr + pels - BlkSize;

	}

	return sad;

}

int FastBlockSAD(
		  ImageChar *imageCur, 
		  ImageChar *imageRef, 
		  int rposRef, 
		  int cposRef,
		  int rposCur,
		  int cposCur
		  )
{
	int i, r, rCurr;
	int tmp, sad;
	int lines, pels;
	int BlkSize		= OFM->currBlk->size;
	unsigned char *refPtr;
	unsigned char *curPtr;
	lines	= imageRef->lines;
	pels	= imageRef->pels;
	sad		= 0;


	//r = rposRef + i;
	
	refPtr	= imageRef->data + rposRef * pels + cposRef;
	
	curPtr	= imageCur->data + rposCur * pels + cposCur;

	for(i=0; i< BlkSize ; i++)
	{

		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);

		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);
		tmp = *(curPtr++) - *(refPtr++); sad += ABS(tmp);

		refPtr	= refPtr + pels - BlkSize;
		curPtr	= curPtr + pels - BlkSize;

	}

	return sad;

}


void blockTranslationMEFullSearch()
{
	int RN, CN, rMV, cMV;
	int rpos, cpos, r, c;
	int searchRange;
	int SAD, minSAD;
	int MV[2];
	BlockMotionStat * mStat;

	ImageChar * currFrame;
	ImageChar * refFrame;
	BlockInt * currBlk;
	BlockInt * refBlk;

	currFrame = OFM->currFrame;
	refFrame = OFM->refFrame;
	currBlk = OFM->currBlk;
	refBlk = OFM->refBlk;

	RN = currFrame->lines / BLOCKSIZE;
	CN = currFrame->pels / BLOCKSIZE;
	searchRange = OFM->SEARCH_RANGE;

	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		rpos = r * BLOCKSIZE;
		cpos = c * BLOCKSIZE;

		//fill the block
		fillBlockFromCharImage(currFrame, rpos, cpos, currBlk);
		
		minSAD = BLOCKSIZE * BLOCKSIZE * 1024;

		for(rMV=-searchRange; rMV<searchRange; rMV++)
		for(cMV=-searchRange; cMV<searchRange; cMV++)
		{
			//fillBlockFromCharImage(refFrame, rpos+rMV, cpos+cMV, refBlk);
			//SAD = blockSAD(currBlk, refBlk);
			SAD		= FastBlockSAD(currFrame, refFrame, rpos + r, cpos + c, rpos, cpos);
			//SAD = blockSAD(currBlk, refFrame, rpos + r, cpos + c);

			//save the minimum
			if(SAD < minSAD)
			{
				MV[0] = cMV;	//horizontal motion
				MV[1] = rMV;	//vertical motion
				minSAD = SAD;
			}

		}

		//save the results
		mStat = OFM->motionStat + r * CN + c;
		mStat->MVc = MV[0];
		mStat->MVr = MV[1];
		mStat->minSAD = minSAD;

	}
	

}


void writeMotionFieldToVideo()
{
	ImageChar * MFF;
	int i;
	int lines, pels;
	int RN, CN, rMV, cMV;
	int rpos, cpos, r, c, rposCenter, cposCenter;
	int searchRange;
	BlockMotionStat * mStat;
	double stepr, stepc, mag;
	int k, TK;
	int level;
	int scale;

	scale = 3;
	MFF = OFM->MFFrame;
	lines = MFF->lines;
	pels = MFF->pels;


	RN = lines / BLOCKSIZE;
	CN = pels / BLOCKSIZE;
	searchRange = OFM->SEARCH_RANGE;
	TK = searchRange * scale;

	for(i=0; i<lines*pels; i++)
		*(MFF->data + i) = 255;

	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		rposCenter = r * BLOCKSIZE + (BLOCKSIZE / 2);
		cposCenter = c * BLOCKSIZE + (BLOCKSIZE / 2);
	
		mStat = OFM->motionStat + (r * CN + c);
		rMV = scale * mStat->MVr;
		cMV = scale * mStat->MVc;

		stepr = (rMV * 1.0) / TK;
		stepc = (cMV * 1.0) / TK;

		mag = sqrt( (double)rMV * rMV + cMV * cMV);
		level = 128 + (int)(mag / (searchRange * 1.414) * 127);
		if(level > 255) level = 255;

#if 0
		for(i=r*BLOCKSIZE; i<(r+1)*BLOCKSIZE; i++)
		for(j=c*BLOCKSIZE; j<(c+1)*BLOCKSIZE; j++)
			*(MFF->data + i * pels + j) = (unsigned char)level;
#endif

		for(k=0; k<TK; k++)
		{
			rpos = (int)(rposCenter + stepr * k + 0.5);
			cpos = (int)(cposCenter + stepc * k + 0.5);
			if(rpos < 0) rpos = 0;
			if(rpos > lines) rpos = lines;
			if(cpos < 0) cpos = 0;
			if(cpos > pels) cpos = pels;
			*(MFF->data + rpos * pels + cpos) = (unsigned char)32;
		}

	}

	writeFrame(MFF, OFM->motionVideoFP);

}

#endif