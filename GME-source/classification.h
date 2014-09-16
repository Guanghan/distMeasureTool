/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *	File:	
 *	Author:	Zhihai He
 *	Date:	06/08/2005
 *	AFRL
 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

#ifndef CLASS_H
#define CLASS_H

#include "global.h"
#include "DCT.h"

void blockClassification()
{
	int inBlock[64], outBlock[64];
	int RN, CN;
	int r, c, i, j, k;
	ImageChar *currFrame;
	unsigned char *inputFrame;
	int lines, pels;
	double dcValue, var, lowACEnergy, dd;
	int AC_Index[8]={1, 2, 8, 9, 10, 16, 17, 18};
	BlockClassStat * classMap;
	double reliability, maxReliability, minReliability, rStep;

	int selectRatio = MBselectRatio; 						// # of structure blocks from all.
	
	double threshold;
	int hist[100], ind, numGoodBlocks, count;
	int nBins = 50;
	int downSampleSize, sr, sc;
	

	currFrame = OFM->currFrame;
	inputFrame = currFrame->data;
	lines = currFrame->lines;
	pels = currFrame->pels;
	minReliability = 100000000;
	maxReliability = -1;

	RN = lines / BLOCKSIZE;
	CN = pels / BLOCKSIZE;
	downSampleSize = BLOCKSIZE / 8;
	

	//DCT
	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{

		//down sample and get data
		k=0;
		for(i=0; i<8; i++)
		for(j=0; j<8; j++)
		{
			sr = r * BLOCKSIZE + i * downSampleSize;
			sc = c * BLOCKSIZE + j * downSampleSize;
			inBlock[k]=*(inputFrame + sr*pels + sc);
			k++;
		}

		DCT(inBlock, outBlock);

		//DC value
		dcValue = *(outBlock+0);

		//total AC energy
		var = 0;
		for(i=1; i<64; i++)
		{
			dd = *(outBlock + i);
			var = var + dd * dd;
		}

		//low-frequency AC energy;
		lowACEnergy = 0;
		for(i=0; i<8; i++)
		{
			dd = *(outBlock + AC_Index[i]);
			lowACEnergy = lowACEnergy + dd * dd;
		}

		var = var + dcValue;
		reliability = (lowACEnergy+0.01) / (var + 0.01);
		if((r==0) || (r==RN-1)) reliability = 0;
		if((c==0) || (c==CN-1)) reliability = 0;

		//save the statistics
		classMap = OFM->classMap + r * CN + c;
		classMap->DC = dcValue;
		classMap->var = var;
		classMap->lowACEnergy = lowACEnergy;
		classMap->reliability = reliability;

		//find the max and min
		if(reliability > maxReliability) maxReliability = reliability;
		if(reliability < minReliability) minReliability = reliability;
	}
	
	//assure stability
	if(maxReliability < minReliability + 0.0001)
		maxReliability = minReliability + 0.0001;

	//classification
	rStep = (maxReliability - minReliability) / nBins;

	//get historgram of reliability
	for(i=0; i<nBins; i++) hist[i] = 0;	// int nBins = 50;
	for(k=0; k<RN*CN; k++)
	{
		classMap = OFM->classMap + k;
		reliability = classMap->reliability;
		ind = (int)((reliability - minReliability) / rStep);
		if(ind > (nBins-1)) ind = nBins-1;

		hist[ind] = hist[ind] + 1;

		classMap->useForMotion = 0;

	}

	// Select the top 
	numGoodBlocks = (selectRatio * RN * CN / 100);
	//numGoodBlocks = numStructBlks;

	count = 0;
	k = nBins - 1;
	while((count < numGoodBlocks) && (k>=0)) 
	{
		count = count + hist[k];
		k--;
	}

	k = k + 1;
	if(k<1) k = 1;

	threshold = minReliability + (k+1) * rStep;

	count = 0;
	k = 0;
	while((count < numGoodBlocks) && (k<RN*CN))
	{
		classMap = OFM->classMap + k;
		reliability = classMap->reliability;

		if(reliability > threshold)
		{
			count++;
			classMap->useForMotion = 1;
		}

		k++;
	}
			

}


void viewClassMap()
{
	int RN, CN;
	int r, c, i, j, k;
	ImageChar *currFrame;
	unsigned char *inputFrame;
	unsigned char *cbuf;
	int lines, pels;
	BlockClassStat * classMap;
	int MVr, MVc, MVScale;
	double stepR, stepC;
	int nSteps;
	int rpos, cpos, rposCenter, cposCenter;
	
	currFrame = OFM->currFrame;
	inputFrame = currFrame->data;
	lines = currFrame->lines;
	pels = currFrame->pels;

	RN = lines / BLOCKSIZE;
	CN = pels / BLOCKSIZE;

	MVScale = -1;
	
	//copy the data
	cbuf = OFM->tmpBuf;
	for(i=0; i<lines*pels; i++)
		*(cbuf + i) = *(inputFrame + i);

	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		classMap = OFM->classMap + r * CN + c;
		if(classMap->useForMotion == 1)
		{

			for(i=r*BLOCKSIZE; i<(r+1)*BLOCKSIZE; i++)
			{
				*(cbuf + i * pels + c*BLOCKSIZE) = 250;
				*(cbuf + i * pels + c*BLOCKSIZE+BLOCKSIZE) = 250;

			}

			for(j=c*BLOCKSIZE; j<(c+1)*BLOCKSIZE; j++)
			{
				*(cbuf + (r*BLOCKSIZE) * pels + j) = 250;
				*(cbuf + (r*BLOCKSIZE+BLOCKSIZE) * pels + j) = 250;
			}


			//display the motion vector;
			MVr = classMap->MVr * MVScale;
			MVc = classMap->MVc * MVScale;
			nSteps = (int)(sqrt( (double)MVr * MVr + MVc * MVc) + 2);
			stepR = (MVr * 1.0)/nSteps;
			stepC = (MVc * 1.0)/nSteps;
			rposCenter = r * BLOCKSIZE + BLOCKSIZE / 2;
			cposCenter = c * BLOCKSIZE + BLOCKSIZE / 2;

			for(k=0; k<nSteps; k++)
			{
				rpos = (int)(rposCenter + stepR * k + 0.5);
				cpos = (int)(cposCenter + stepC * k + 0.5);
				if(rpos < 0) rpos = 0;
				if(rpos > lines) rpos = lines;
				if(cpos < 0) cpos = 0;
				if(cpos > pels) cpos = pels;
				*(cbuf + rpos * pels + cpos) = 250;
			}

		}
	}

	//fwrite(inputFrame, lines*pels, 1, OFM->classMapFP);
	fwrite(cbuf, lines*pels, 1, OFM->classMapFP);
	fwrite(currFrame->chroma, lines*pels/2, 1, OFM->classMapFP);
}



#endif