/**************************************************************************
* Program Name: videoStabilization
*
* Filename: utilities.h
*
* Description:
*  
*  Program initialization, memory allocation and utilitiy functions.
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
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "global.h"

#ifndef INIT_H
#define INIT_H

void readFrame(FILE *fp, int skip, ImageChar * frameBuf)
{
	int frameSize;
	int offset;
	int lines, pels;

	lines = frameBuf->lines;
	pels = frameBuf->pels;
	frameSize = lines * pels;

	offset = skip * frameSize * 3 / 2;
	
	fseek(fp, offset, SEEK_CUR);
	fread(frameBuf->data, frameSize, 1, fp);

	fread(frameBuf->chroma, frameSize / 2, 1, fp);



	return;
}


/*---------------------------------------------------------
 * Write image into file 
 *
 *--------------------------------------------------------*/
void writeFrame(ImageChar *frameBuf, FILE *fp)
{
	int lines, pels;

	lines = frameBuf->lines;
	pels = frameBuf->pels;

	fwrite(frameBuf->data, lines*pels, 1, fp);

	//set the chrominance to be 128;
	fwrite(frameBuf->chroma, lines*pels / 2, 1, fp);


}

/*---------------------------------------------------------
 * This function allocate memomry for an image of type char
 *
 *--------------------------------------------------------*/
ImageChar * newImageChar(int lines, int pels)
{
	ImageChar *img;

	img = (ImageChar *)malloc(sizeof(ImageChar));

	img->data = (unsigned char *)malloc(lines * pels * sizeof(unsigned char));
	img->chroma = (unsigned char *)malloc(lines * pels / 2 * sizeof(unsigned char));
	img->lines = lines;
	img->pels = pels;

	return img;
}


/*---------------------------------------------------------
 * This function allocate memomry for an image of type int
 *
 *--------------------------------------------------------*/
ImageInt * newImageInt(int lines, int pels)
{
	ImageInt *img;

	img = (ImageInt *)malloc(sizeof(ImageInt));
	img->data = (int *)malloc(lines * pels * sizeof(int));
	img->lines = lines;
	img->pels = pels;

	return img;
}



/*---------------------------------------------------------
 * This function allocate memomry for a block of type int
 *
 *--------------------------------------------------------*/
BlockInt * newBlockInt(int size)
{
	BlockInt *block;
	int i;

	block = (BlockInt *)malloc(sizeof(BlockInt));
	block->data = (int **)malloc(size * sizeof(int *));

	for(i=0; i<size; i++)
		block->data[i] = (int *)malloc(size * sizeof(int));

	block->size = size;
	return block;
}


/*---------------------------------------------------------
 * This function allocate memomry for a block of type int
 *
 *--------------------------------------------------------*/
BlockFloat * newBlockFloat(int size)
{
	BlockFloat *block;
	int i;

	block = (BlockFloat *)malloc(sizeof(BlockFloat));
	block->data = (double **)malloc(size * sizeof(double *));

	for(i=0; i<size; i++)
		block->data[i] = (double *)malloc(size * sizeof(double));

	block->size = size;

	return block;
}



/*---------------------------------------------------------
 * This function
 * Need to deal with the outside-of-the-boundary situation
 *--------------------------------------------------------*/

void fillBlockFromCharImage(ImageChar *image, int rpos, int cpos, BlockInt * block)
{
	int i, j, r, c;
	int lines, pels;

	lines = image->lines;
	pels = image->pels;

	for(i=0; i<block->size; i++)
	for(j=0; j<block->size; j++)
	{
		r = rpos + i;
		c = cpos + j;

		if(r < 0) r = 0;
		if(r > (lines-1)) r = lines - 1;
		if(c < 0) c = 0;
		if(c > (pels-1)) c = pels - 1;

		block->data[i][j] = (int)*(image->data + r * pels + c);

	}

	return;
}




/*---------------------------------------------------------
 * This function writes a block (int) into an image (char)
 * Need to deal with the outside-of-the-boundary situation
 *--------------------------------------------------------*/

void writeBlockToCharImage(BlockInt * block, ImageChar *image, int rpos, int cpos)
{
	int i, j, r, c;
	int lines, pels;
	int dd;

	lines = image->lines;
	pels = image->pels;

	for(i=0; i<block->size; i++)
	for(j=0; j<block->size; j++)
	{
		r = rpos + i;
		c = cpos + j;

		if(r < 0) r = 0;
		if(r > (lines-1)) r = lines - 1;
		if(c < 0) c = 0;
		if(c > (pels-1)) c = pels - 1;

		dd = block->data[i][j];
		if(dd < 0) dd = 0;
		if(dd > 255) dd = 255;

		*(image->data + r * pels + c) = block->data[i][j];

	}

	return;
}



/*---------------------------------------------------------
 * This function writes a block (int) into an image (char)
 * Need to deal with the outside-of-the-boundary situation
 *--------------------------------------------------------*/

void writeBlockToIntImage(BlockInt * block, ImageInt *image, int rpos, int cpos)
{
	int i, j, r, c;
	int lines, pels;

	lines = image->lines;
	pels = image->pels;

	for(i=0; i<block->size; i++)
	for(j=0; j<block->size; j++)
	{
		r = rpos + i;
		c = cpos + j;

		if(r < 0) r = 0;
		if(r > (lines-1)) r = lines - 1;
		if(c < 0) c = 0;
		if(c > (pels-1)) c = pels - 1;

		*(image->data + r * pels + c) = block->data[i][j];

	}

	return;
}

/*---------------------------------------------------------
 * This function
 *
 *--------------------------------------------------------*/


float PSNR_Char_Char(unsigned char * image1, int lines, int pels,
					unsigned char * image2)
{
	int i;
	double MSE, mean;
	int coeffA, coeffB, diff;
	double PSNR;

	MSE = 0;
	mean = 0;

	for(i=0; i<lines * pels; i++)
	{
		coeffA = *(image1 + i);
		coeffB = *(image2 + i);
		diff = (int)coeffA - (int)coeffB;

		MSE = MSE + diff * diff;
		mean = mean + diff;
	}

	MSE = MSE / (lines * pels);
	mean = mean / (lines * pels);
	MSE = MSE - mean * mean;

	if(MSE < 0.01) return 99;
	else
	{
		PSNR = 10 * log10(255*255 / MSE);
		return (float)PSNR;

	}


}

/*---------------------------------------------------------
 * This function
 *
 *--------------------------------------------------------*/
float PSNR_Char_Int(unsigned char * image1, int lines, int pels,
					int * image2)
{
	int i;
	double MSE, mean;
	int coeffA, coeffB, diff;
	double PSNR;

	MSE = 0;
	mean = 0;

	for(i=0; i<lines * pels; i++)
	{
		coeffA = *(image1 + i);
		coeffB = *(image2 + i);
		diff = coeffA - coeffB;

		MSE = MSE + diff * diff;
		mean = mean + diff;
	}

	MSE = MSE / (lines * pels);
	mean = mean / (lines * pels);
	MSE = MSE - mean * mean;

	if(MSE < 0.01) return 99;
	else
	{
		PSNR = 10 * log10(255 * 255 / MSE);
		return (float)PSNR;

	}


}


double findDiffVar(int *image0, int *image1, int lines, int pels)
{
	int i, dd;
	double mean, var;

	mean = 0;
	var = 0;
	for(i=0; i<lines*pels; i++)
	{
		dd = *(image0+i) - *(image1+i);
		mean = mean + dd;
		var = var + dd * dd;
	}

	mean = mean / (lines*pels);
	var = var / (lines*pels);
	var = var - mean * mean;

	return var;

}

double ** createMatrix(int N, int M)
{
	double ** A;
	int i;

	A = (double **)malloc(N * sizeof(double *));
	for(i=0; i<N; i++)
		A[i] = (double *)malloc(M * sizeof(double));

	return A;

}



//-----------------utilities

void createNeighborPoints();

void OFEstimationInit(int lines, int pels, char *fname)
{
	int i;
	BlockClassStat * classMap;

	OFM = (OFMStruct *)malloc(sizeof(OFMStruct));

	OFM->currFrame = newImageChar(lines, pels);
	OFM->refFrame = newImageChar(lines, pels);

	OFM->DCTBuffer = (int *)malloc(lines * pels * sizeof(int));

	OFM->currBlk = newBlockInt(BLOCKSIZE);
	OFM->refBlk = newBlockInt(BLOCKSIZE);;

	OFM->SEARCH_RANGE = MV_RANGE;
	OFM->picSizeInBlocks = (lines * pels) / (BLOCKSIZE * BLOCKSIZE);

	OFM->motionStat = (BlockMotionStat *)malloc(OFM->picSizeInBlocks * sizeof(BlockMotionStat));

	OFM->classMap = (BlockClassStat *)malloc(OFM->picSizeInBlocks * sizeof(BlockClassStat));

	//choose a line of motion vector candidates
	OFM->nMVCandidates = OFM->SEARCH_RANGE * 2;
	for(i=0; i<OFM->picSizeInBlocks; i++)
	{
		classMap = OFM->classMap + i; 
		classMap->MVCandidates = (BlockMotionStat *)malloc(OFM->nMVCandidates * sizeof(BlockMotionStat));
	}

	//double matrix A;
	OFM->A = createMatrix(OFM->picSizeInBlocks, 3);
	OFM->ATA = createMatrix(3, 3);
	OFM->invATA = createMatrix(3, 3);

	OFM->b = (double *)malloc(OFM->picSizeInBlocks * sizeof(double));

	//for feature extraction
	OFM->currBlockFeature = (FeatureSetStruct *)malloc(sizeof(FeatureSetStruct));
	OFM->refBlockFeature = (FeatureSetStruct *)malloc(sizeof(FeatureSetStruct));

	OFM->videoFP = fopen(fname, "rb");
	//OFM->outVideoFP = fopen("out.yuv", "wb");
	//OFM->motionVideoFP = fopen("motion.yuv", "wb");
	
	//find video length.
	fseek(OFM->videoFP, 0, SEEK_END);
	OFM->EndofVideo	= ftell(OFM->videoFP);
	fseek(OFM->videoFP, 0, SEEK_SET);
	//	printf("\n EoF = %5d\n\n", ftell(OFM->videoFP) );

	#ifdef WRITE_CLASSIFICATION_MAP
		OFM->classMapFP = fopen("classmap.yuv", "wb");
	#endif

	OFM->MFFrame = newImageChar(lines, pels);
	for(i=0; i<lines*pels/2; i++)
		*((OFM->MFFrame)->chroma + i) = (unsigned char)128;

	OFM->tmpBuf = (unsigned char *)malloc(lines * pels);

	OFM->neighborPoints = (CirclePoints *)malloc(NEIGHBOR_SIZE * NEIGHBOR_SIZE * 4 * sizeof(CirclePoints));
	createNeighborPoints();
	



	

}


void createNeighborPoints()
{
	int i, j;
	int ind;
	int ptr;
	CirclePoints * tmpPtr;
	int * hist;

	OFM->nCircles = NEIGHBOR_SIZE;
	OFM->neighborHist = (int *)malloc(OFM->nCircles * sizeof(int));
	OFM->avgIntensity = (int *)malloc(OFM->nCircles * sizeof(int));

	hist = OFM->neighborHist;
	for(i=0; i<OFM->nCircles; i++)
		*(hist + i) = 0;

	ptr = 0;
	for(i=-NEIGHBOR_SIZE; i<=NEIGHBOR_SIZE; i++)
	for(j=-NEIGHBOR_SIZE; j<=NEIGHBOR_SIZE; j++)
	{
		ind = (int)sqrt( (double)i*i + j*j);
		
		if(ind <=NEIGHBOR_SIZE)
		{
			ind = ind - 1;
			if(ind < 0) ind = 0;

			tmpPtr = OFM->neighborPoints + ptr;
			tmpPtr->dr = i;
			tmpPtr->dc = j;
			tmpPtr->bin = ind;
			ptr++;

			hist[ind] = hist[ind] + 1;
		}
		
	}

	OFM->nNeighborPoints = ptr;
	
	return;
}

void OFEstimationClose()
{
	#ifdef WRITE_WARPED_FRAME
	fclose(OFM->videoFP);
	#endif 

//	fclose(OFM->outVideoFP);
//	fclose(OFM->motionVideoFP);

	#ifdef WRITE_CLASSIFICATION_MAP
		fclose(OFM->classMapFP);
	#endif
}

//-----------------

//-----------------




#endif 