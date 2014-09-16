/**************************************************************************
* Program Name: videoStabilization
*
* Filename: global.h
*
* Description:
*  
*  Program structures initializations.
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


#ifndef GLOBAL_H
#define GLOBAL_H

#include <stdlib.h>
#include <stdio.h>

//#include "mosaic.h"

#include "Eigen/Core"
#include "Eigen/Array"
#include "Eigen/LU"
#include "Eigen/Cholesky"

USING_PART_OF_NAMESPACE_EIGEN

#define NOMINMAX

#define WRITE_WARPED_FRAME					//write stabilized video frames.
#define BLOCKSIZE		16
#define MBselectRatio	50
#define stableWindow	200
#define MV_RANGE		16
#define DCT_BLOCKSIZE 8
#define numStructBlks 1000					//# of structure blocks.

#define NEIGHBOR_SIZE	12					//specify the size of neighbor for matching
#define MAX_NEIGHBOR_RANGE 64

#define lines_ext_factor 1.5				// Extension factors; must be odd numbers.
#define pels_ext_factor  1.5

#define length_of_transform 1000			// Number of motion transform stored.


typedef struct tagImageChar{
	unsigned char *data;
	unsigned char *chroma;
	int lines;
	int pels;
}ImageChar;


typedef struct tagImageInt{
	int *data;
	int lines;
	int pels;
}ImageInt;


typedef struct tagBlockInt{
	int **data;
	int size;
}BlockInt;


typedef struct tagBlockFloat{
	double **data;
	int size;
}BlockFloat;

typedef struct tagBlockMotionStat{
	int MVr;
	int MVc;
	int minSAD;
} BlockMotionStat;


typedef struct tagBlockClassStat{
	double DC;
	double var;
	double lowACEnergy;
	double reliability;
	int useForMotion;
	int MVr;
	int MVc;
	int minSAD;
	double xVel;
	double yVel;
	BlockMotionStat *MVCandidates;
	int nCandidateMVS;
	double range;

} BlockClassStat;


typedef struct tagCircles{
	int dr;
	int dc;
	int bin;
} CirclePoints;


typedef struct tagFeatureSetStruct{
	int avgIntensity[MAX_NEIGHBOR_RANGE];
	int featureSize;
}FeatureSetStruct;



typedef struct tagOFM{
	ImageChar * currFrame;
	ImageChar * refFrame;

	BlockInt * currBlk;
	BlockInt * refBlk;

	int currFrameNumber;
	int time;
	int SEARCH_RANGE;
	int picSizeInBlocks;

	BlockMotionStat * motionStat;
	BlockClassStat * classMap;
	int nMVCandidates;

	//input and output files
	FILE *videoFP;
	FILE *outVideoFP;
	FILE *motionVideoFP;
	FILE *classMapFP;
	ImageChar *MFFrame;
	long EndofVideo;

	//feature extraction
	FeatureSetStruct * currBlockFeature;
	FeatureSetStruct * refBlockFeature;
	int featureQuantStepSize;
	CirclePoints * neighborPoints;
	int nNeighborPoints;
	int nCircles;
	int * neighborHist;
	int * avgIntensity;


	//memory buffers for computation and storage
	unsigned char *currFrameChroma;
	int *DCTBuffer;
	unsigned char *tmpBuf;

	//for least mean square fitting
	double ** A;
	double * b;
	double ** ATA, ** invATA;
	double Omega[3];
	double Vb1, Vb2, Vb3;
	


	
}OFMStruct;

OFMStruct * OFM;

//strudef
//------------------------- Exploring Methods Switch ------------------------//
//#define _full_search_landscape
#define _inverse_mapping_search


#define MSE_threshold 6502.5		// PSNR = 10dB
//#define MSE_threshold 2056.3		// PSNR = 15dB
//#define MSE_threshold 650.25		// PSNR = 20dB
//#define MSE_threshold 205.6271		// PSNR = 25dB
//#define MSE_threshold 65.025		// PSNR = 30dB
//#define MSE_threshold 20.5627		// PSNR = 35dB
//#define MSE_threshold 6.5025		// PSNR = 40dB
//#define MSE_threshold 2.0563		// PSNR = 45dB
//#define MSE_threshold 0.6503		// PSNR = 50dB



#define ABS(x) (x<0)?(-(x)):(x)

#ifndef __MINMAX_DEFINED
#  define max(a,b)    (((a) > (b)) ? (a) : (b))
#  define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif



typedef struct LandscapeF{
	//unsigned char	*data;
	float			*data;
	float			*data_flux;
	unsigned		*flux_flag;
	unsigned		*Reset_flag;
	unsigned char *data_buf;
	unsigned char *chroma;
	unsigned char *inerStats;	// Buffer for displaying GPS/Orientation stats.
	int lines;
	int pels;
	int useNewframe;			// flag for if using new frame for PTE.
	FILE *exploreVideoFP;
	FILE *FluxVideoFP;
	FILE *VideoHeaderFP;
	FILE *display_stats;		// For displaying GPS/Orient. info.
}LandscapeF;


typedef struct mapPix{
	double *rposMap;	// row. position in Landscape Map.
	double *cposMap;	// col. position in Landscape Map.
	int *pixF;			// flag for already searched pixels.
}mapPix;

typedef struct frameMap{
	double *x;
	double *y;
}frameMap;

typedef struct PSNRstat{
	double MSE;
	double RMSE;
	double *PSNRrec;		// PSNR at each frame.

}PSNRstat;

typedef struct transMatrix_storage{
	double ** data;
	double ** glbTRFM;		// Global Transform Matrix.
	int		  cnt;
}transMatrix_storage;		// Storage for the Transform Matrice.


LandscapeF * Landscape;
mapPix	   * xyMap;
frameMap   * frameMapxy;				// coordinates in Reference frame.
frameMap   * frameMapXY;				// coordinates in Current frame.

transMatrix_storage	* transMatrix;		// Storage for the Transform Matrice.

FILE *outLandscape; 

#endif