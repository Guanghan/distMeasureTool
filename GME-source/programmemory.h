/**************************************************************************
* Program Name: uavOFMA
*
* Filename: programmemory.h
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

#include <stdlib.h>
#include <stdio.h>
#include "global.h"


void LandscapeInit(ImageChar * frameBuf)
{
	int lines, pels;
	int offset;
	int i, j;
//	int lines_ext = line_ext_factor;
//	int pels_ext  = pel_ext_factor;	


	lines				   = frameBuf->lines;
	pels				   = frameBuf->pels;

	Landscape			   = (LandscapeF * )malloc(sizeof(LandscapeF));
	Landscape->lines	   = lines * lines_ext_factor;		
	Landscape->pels		   = pels  * pels_ext_factor;
	

	Landscape->useNewframe = 1;
	//Landscape->data        = (unsigned char *)malloc(Landscape->lines * Landscape->pels * sizeof(unsigned char));
	Landscape->data        = (float *)malloc(Landscape->lines * Landscape->pels * sizeof(float));
	Landscape->data_flux   = (float *)malloc(Landscape->lines * Landscape->pels * sizeof(float));
	Landscape->flux_flag   = (unsigned *)malloc(sizeof(unsigned));
	Landscape->Reset_flag  = (unsigned *)malloc(sizeof(unsigned));
	Landscape->data_buf    = (unsigned char *)malloc(Landscape->lines * Landscape->pels * sizeof(unsigned char));
	Landscape->chroma      = (unsigned char *)malloc( (Landscape->lines * Landscape->pels) / 2 * sizeof(unsigned char));
	Landscape->inerStats   = (unsigned char *)malloc( 144*120*sizeof(unsigned char) );
	
	Landscape->exploreVideoFP	= fopen("Stabilized_video_.yuv", "wb");
	//Landscape->FluxVideoFP		= fopen("Explored_Flux_.yuv", "wb");
	//Landscape->VideoHeaderFP	= fopen("Explore_Landscape_.hdr","wb");

	xyMap			 = (mapPix * )malloc(sizeof(mapPix));
	xyMap->cposMap	 = (double *)malloc(Landscape->lines * Landscape->pels * sizeof(double));
	xyMap->rposMap	 = (double *)malloc(Landscape->lines * Landscape->pels * sizeof(double));
	xyMap->pixF 	 = (int *)malloc(Landscape->lines * Landscape->pels * sizeof(int));

	offset			 = lines * pels * (lines_ext_factor-1)/2 * pels_ext_factor + pels * (pels_ext_factor-1)/2;

	//	Initialize the Landscape map.
	for (i=0; i< Landscape->lines * Landscape->pels; i++)
	{
		*(Landscape->data + i) = 0;
	}

	for (i=0; i< (Landscape->lines * Landscape->pels)/2 ; i++)
	{
		*(Landscape->chroma + i ) = (unsigned char)128;
	}

	//	Initialize xy coordinate map, xyMap(for current frame), for Landscape.
	for (i=0; i< Landscape->lines; i++)
	{
		for (j=0; j< Landscape->pels; j++)
		{
			*(xyMap->rposMap + i*Landscape->pels + j) = (double)( i - lines * (lines_ext_factor-1)/2 );
			*(xyMap->cposMap + i*Landscape->pels + j) = (double)( j - pels  * (pels_ext_factor -1)/2 );
			*(xyMap->pixF    + i*Landscape->pels + j) = 0;
		}
	}

	
	// Memory Allocation for coordinate mapping.

	frameMapxy = (frameMap *)malloc( sizeof(frameMap) );
	frameMapXY = (frameMap *)malloc( sizeof(frameMap) );

	frameMapxy->x = (double *)malloc( lines * pels /BLOCKSIZE / BLOCKSIZE * sizeof(double) );	// the # of structure blks are changing.
	frameMapxy->y = (double *)malloc( lines * pels /BLOCKSIZE / BLOCKSIZE * sizeof(double) );
	frameMapXY->x = (double *)malloc( lines * pels /BLOCKSIZE / BLOCKSIZE * sizeof(double) );
	frameMapXY->y = (double *)malloc( lines * pels /BLOCKSIZE / BLOCKSIZE * sizeof(double) );
	
	
	
}



void TransMatrixInit()
{
	// Memory Allocation for transform matrice storage.
	int i;
	transMatrix				= (transMatrix_storage * )malloc(sizeof(transMatrix_storage));
	transMatrix->data		= (double **)malloc(length_of_transform * sizeof(double*) );

	for (i=0; i<length_of_transform; i++)
		transMatrix->data[i] = (double*)malloc( 9 * sizeof(double) );

	transMatrix->glbTRFM		= (double **)malloc(sizeof(double*) );
	transMatrix->glbTRFM[0]		= (double*)malloc( 9 * sizeof(double) );

	transMatrix->glbTRFM[0][0]	= 1;
	transMatrix->glbTRFM[0][1]	= 0;
	transMatrix->glbTRFM[0][2]	= 0;
	transMatrix->glbTRFM[0][3]	= 0;
	transMatrix->glbTRFM[0][4]	= 1;
	transMatrix->glbTRFM[0][5]	= 0;
	transMatrix->glbTRFM[0][6]	= 0;
	transMatrix->glbTRFM[0][7]	= 0;
	transMatrix->glbTRFM[0][8]	= 1;

	transMatrix->cnt = 0;
}


void TransMatrixReset()
{
	transMatrix->glbTRFM[0][0]	= 1;
	transMatrix->glbTRFM[0][1]	= 0;
	transMatrix->glbTRFM[0][2]	= 0;
	transMatrix->glbTRFM[0][3]	= 0;
	transMatrix->glbTRFM[0][4]	= 1;
	transMatrix->glbTRFM[0][5]	= 0;
	transMatrix->glbTRFM[0][6]	= 0;
	transMatrix->glbTRFM[0][7]	= 0;
	transMatrix->glbTRFM[0][8]	= 1;

	transMatrix->cnt = 0;
}



void writeLandscape()
{
	outLandscape = fopen("Landscape.out","wb");
	fwrite( Landscape->data, Landscape->lines * Landscape->pels, 1,outLandscape );

}



void LandscapeClose()
{
	//fclose(outLandscape);
	//fclose(display_stats);
	fclose(Landscape->exploreVideoFP);
	//fclose(Landscape->FluxVideoFP);
	//fclose(Landscape->VideoHeaderFP);

	free(Landscape->data);
	free(Landscape->data_flux);
	free(Landscape);
	free(xyMap->pixF);
	free(xyMap->rposMap);
	free(xyMap->cposMap);
	free(xyMap);
	free(frameMapxy->x);
	free(frameMapxy->y);
	free(frameMapXY->x);
	free(frameMapXY->y);
	free(frameMapxy);
	free(frameMapXY);
	free(transMatrix->data);
	free(transMatrix);

}




void writeVideo_flux_tensor()
{
	int	i;

	if ( *(Landscape->flux_flag) == 1 )
	
		//flag is unsigned, if it's 1 you have output frame placed in "data_flux".
	{
		for (i=0; i<Landscape->lines * Landscape->pels; i++)

		{
			if (*(Landscape->data + i) > 254.0)
				{
					*(Landscape->data_buf + i) = 255;
				}else
			*(Landscape->data_buf + i) = (unsigned char) (*(Landscape->data_flux + i));
		}
	}

	else

	{
		for (i=0; i<Landscape->lines * Landscape->pels; i++)
		{
				if (*(Landscape->data + i) > 255.0)
				{
					*(Landscape->data_buf + i) = 255;
				}else
				{

		        	*(Landscape->data_buf + i) = (unsigned char)(*(Landscape->data + i));
				}
		}
	}


	

	//drawWindow(O,P,Q,R);	// Draw white frame edges.
	//putStats();			// Put on inerStats chart.
	//putSubwin();			// Put a half-size window for original video(not adaptive sizing).

	//fwrite( Landscape->data_buf, Landscape->lines * Landscape->pels, 1, Landscape->FluxVideoFP);
	//fwrite( Landscape->chroma,(Landscape->lines * Landscape->pels)/2, 1, Landscape->FluxVideoFP);

	//	Clean up the Landscape map.
	for (i=0; i< Landscape->lines * Landscape->pels; i++)
	{
		*(Landscape->data + i)	= 0;
		*(xyMap->pixF + i)		= 0;
	}
}




void writeVideo()
{
	int	i;

	
	for (i=0; i<Landscape->lines * Landscape->pels; i++)
	{
		if (*(Landscape->data + i) > 255.0)
		{
			*(Landscape->data_buf + i) = 255;
		}else
		*(Landscape->data_buf + i) = *(Landscape->data + i);
	}
	

	fwrite( Landscape->data_buf, Landscape->lines * Landscape->pels, 1, Landscape->exploreVideoFP);
	fwrite( Landscape->chroma,(Landscape->lines * Landscape->pels)/2, 1, Landscape->exploreVideoFP);

	//	Clean up the Landscape map.
	for (i=0; i< Landscape->lines * Landscape->pels; i++)
	{
		*(Landscape->data + i)	= 0;
		//*(xyMap->pixF + i)		= 0;
	}

	//
}


/*---------------------------------------------------------------------------*/
/* frameTell(): retrieve the number for registed frame for batch processing. */
/*---------------------------------------------------------------------------*/
int frameTell()
{
	FILE		*HandOffReg;
	int			frameReset;

	HandOffReg	= fopen("HOFF.cfg", "r");

	if ( HandOffReg != NULL )
	{
		fscanf( HandOffReg, "%d", &frameReset );
		fclose( HandOffReg );
	}
	else
	{
		frameReset	= 0;
	}
	

	return frameReset;

}

/*---------------------------------------------------------------------------*/
/*			CloseUp(): write file EXIT.out to end the batch processing.		 */
/*---------------------------------------------------------------------------*/
void CloseUp()
{
	FILE		*endbatch;

	endbatch	= fopen("EXIT.out", "w");

	fclose(endbatch);
}


/*---------------------------------------------------------------------------*/
/*			FrameRegister: record frame number done for batch processing.	 */
/*---------------------------------------------------------------------------*/
void FrameRegister( int frameReset )
{
	FILE		*HandOffReg;

	HandOffReg	= fopen("HOFF.cfg", "w");

	fprintf( HandOffReg, "%d", frameReset);

	fclose( HandOffReg );
}


