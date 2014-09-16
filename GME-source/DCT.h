/**************************************************************************
* Program Name: uavOFMA
*
* Filename: dct.h
*
* Description:
*  
*  Discrete cosine transform.
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

#ifndef DCT_H
#define DCT_H

#define mnint(a)	((a) < 0 ? (int)(a - 0.5) : (int)(a + 0.5))


int DCT (int *block, int *coeff)
{
  int j1, i, j, k;
  float b[8];
  float b1[8];
  float d[8][8];
  float f0 = (float) .7071068;
  float f1 = (float) .4903926;
  float f2 = (float) .4619398;
  float f3 = (float) .4157348;
  float f4 = (float) .3535534;
  float f5 = (float) .2777851;
  float f6 = (float) .1913417;
  float f7 = (float) .0975452;

  for (i = 0, k = 0; i < 8; i++, k += 8)
  {
    for (j = 0; j < 8; j++)
    {
      b[j] = (float) block[k + j];
    }
    /* Horizontal transform */
    for (j = 0; j < 4; j++)
    {
      j1 = 7 - j;
      b1[j] = b[j] + b[j1];
      b1[j1] = b[j] - b[j1];
    }
    b[0] = b1[0] + b1[3];
    b[1] = b1[1] + b1[2];
    b[2] = b1[1] - b1[2];
    b[3] = b1[0] - b1[3];
    b[4] = b1[4];
    b[5] = (b1[6] - b1[5]) * f0;
    b[6] = (b1[6] + b1[5]) * f0;
    b[7] = b1[7];
    d[i][0] = (b[0] + b[1]) * f4;
    d[i][4] = (b[0] - b[1]) * f4;
    d[i][2] = b[2] * f6 + b[3] * f2;
    d[i][6] = b[3] * f6 - b[2] * f2;
    b1[4] = b[4] + b[5];
    b1[7] = b[7] + b[6];
    b1[5] = b[4] - b[5];
    b1[6] = b[7] - b[6];
    d[i][1] = b1[4] * f7 + b1[7] * f1;
    d[i][5] = b1[5] * f3 + b1[6] * f5;
    d[i][7] = b1[7] * f7 - b1[4] * f1;
    d[i][3] = b1[6] * f3 - b1[5] * f5;
  }
  /* Vertical transform */
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 4; j++)
    {
      j1 = 7 - j;
      b1[j] = d[j][i] + d[j1][i];
      b1[j1] = d[j][i] - d[j1][i];
    }
    b[0] = b1[0] + b1[3];
    b[1] = b1[1] + b1[2];
    b[2] = b1[1] - b1[2];
    b[3] = b1[0] - b1[3];
    b[4] = b1[4];
    b[5] = (b1[6] - b1[5]) * f0;
    b[6] = (b1[6] + b1[5]) * f0;
    b[7] = b1[7];
    d[0][i] = (b[0] + b[1]) * f4;
    d[4][i] = (b[0] - b[1]) * f4;
    d[2][i] = b[2] * f6 + b[3] * f2;
    d[6][i] = b[3] * f6 - b[2] * f2;
    b1[4] = b[4] + b[5];
    b1[7] = b[7] + b[6];
    b1[5] = b[4] - b[5];
    b1[6] = b[7] - b[6];
    d[1][i] = b1[4] * f7 + b1[7] * f1;
    d[5][i] = b1[5] * f3 + b1[6] * f5;
    d[7][i] = b1[7] * f7 - b1[4] * f1;
    d[3][i] = b1[6] * f3 - b1[5] * f5;
  }
  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      *(coeff + j + i * 8) = (int) (d[i][j]);
    }
  }
  return 0;
}


int IDCT (int *coeff, int *block)
{
  int j1, i, j;
  double b[8], b1[8], d[8][8];
  double f0 = .7071068;
  double f1 = .4903926;
  double f2 = .4619398;
  double f3 = .4157348;
  double f4 = .3535534;
  double f5 = .2777851;
  double f6 = .1913417;
  double f7 = .0975452;
  double e, f, g, h;

  /* Horizontal */

  for (i = 0; i < 8; i++)
  {

    for (j = 0; j < 8; j++)
      b[j] = coeff[j + i * 8];

    e = b[1] * f7 - b[7] * f1;
    h = b[7] * f7 + b[1] * f1;
    f = b[5] * f3 - b[3] * f5;
    g = b[3] * f3 + b[5] * f5;

    b1[0] = (b[0] + b[4]) * f4;
    b1[1] = (b[0] - b[4]) * f4;
    b1[2] = b[2] * f6 - b[6] * f2;
    b1[3] = b[6] * f6 + b[2] * f2;
    b[4] = e + f;
    b1[5] = e - f;
    b1[6] = h - g;
    b[7] = h + g;

    b[5] = (b1[6] - b1[5]) * f0;
    b[6] = (b1[6] + b1[5]) * f0;
    b[0] = b1[0] + b1[3];
    b[1] = b1[1] + b1[2];
    b[2] = b1[1] - b1[2];
    b[3] = b1[0] - b1[3];

    for (j = 0; j < 4; j++)
    {
      j1 = 7 - j;
      d[i][j] = b[j] + b[j1];
      d[i][j1] = b[j] - b[j1];
    }
  }


  /* Vertical */

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      b[j] = d[j][i];
    }
    e = b[1] * f7 - b[7] * f1;
    h = b[7] * f7 + b[1] * f1;
    f = b[5] * f3 - b[3] * f5;
    g = b[3] * f3 + b[5] * f5;

    b1[0] = (b[0] + b[4]) * f4;
    b1[1] = (b[0] - b[4]) * f4;
    b1[2] = b[2] * f6 - b[6] * f2;
    b1[3] = b[6] * f6 + b[2] * f2;
    b[4] = e + f;
    b1[5] = e - f;
    b1[6] = h - g;
    b[7] = h + g;

    b[5] = (b1[6] - b1[5]) * f0;
    b[6] = (b1[6] + b1[5]) * f0;
    b[0] = b1[0] + b1[3];
    b[1] = b1[1] + b1[2];
    b[2] = b1[1] - b1[2];
    b[3] = b1[0] - b1[3];

    for (j = 0; j < 4; j++)
    {
      j1 = 7 - j;
      d[j][i] = b[j] + b[j1];
      d[j1][i] = b[j] - b[j1];
    }
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      *(block + i * 8 + j) = mnint (d[i][j]);
    }
  }
  return 0;
}



void DCTFrame(unsigned char * inputFrame, int lines, int pels, 
			  int *outputFrame)
{

	int inBlock[64], outBlock[64];
	int RN, CN;
	int r, c, i, j, k;

	RN = lines / 8;
	CN = pels / 8;
	

	//DCT
	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{

		k=0;
		for(i=0; i<8; i++)
		for(j=0; j<8; j++)
		{
			inBlock[k]=*(inputFrame + (r*8+i)*pels + c*8+j);
			k++;
		}

		DCT(inBlock, outBlock);

		k=0;
		for(i=0; i<8; i++)
		for(j=0; j<8; j++)
		{
			*(outputFrame + (r*8+i)*pels + c*8+j) = outBlock[k];
			k++;
		}

	}

}

void IDCTFrame(int * inputFrame, int lines, int pels, 
			   int *outputFrame)
{

	int inBlock[64];
	int outBlock[64];
	int RN, CN;
	int r, c, i, j, k;

	RN = lines / 8;
	CN = pels / 8;
	

	//DCT
	for(r=0; r<RN; r++)
	for(c=0; c<CN; c++)
	{
		k=0;
		for(i=0; i<8; i++)
		for(j=0; j<8; j++)
		{
			inBlock[k] = *(inputFrame + (r*8+i)*pels + c*8+j);
			k++;
		}


		IDCT(inBlock, outBlock);
		

		k=0;
		for(i=0; i<8; i++)
		for(j=0; j<8; j++)
		{
			*(outputFrame + (r*8+i)*pels + c*8+j)=outBlock[k];
			k++;
		}
	}
}



void CopyFrameCharInt(unsigned char * inputFrame, int lines, int pels, 
			  int *outputFrame)
{
	int i;
	for(i=0; i<lines*pels; i++)
		*(outputFrame + i) = *(inputFrame + i);

}

void CopyFrameIntInt(int * inputFrame, int lines, int pels, 
			  int *outputFrame)
{
	int i;
	for(i=0; i<lines*pels; i++)
		*(outputFrame + i) = *(inputFrame + i);

}

#endif