//Header Declaration
#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <string>
#include <fstream>
//#include <Clarkson-Delaunay.h>
#include "planeFitting.h"
#include "PTE.h"

#include <stdio.h>
#include <conio.h>
#include "libxl.h"

#pragma comment( linker, "/subsystem:\"windows\" /entry:\"mainCRTStartup\"")

#define WORD  unsigned int

//#define SHOWLINES               //Show the delaunay triangles
//#define SHOW_UNDISTORTED      //Show undistorted image in clicking image
//#define SHOW_UNDISTORTED2     //Show undistorted image with click dots test in another window
//#define PLANE_FITTING
//#define COMPENSATE              //compensate based on undistorted image. (x,y) projection to (x', y') in undistorted image. Use (x', y') on fitted plane.
//#define COMPENSATE2             //Artificial compensation on the plane, make it curvy.

#define CURVE_FITTING
#define Z_MODE_4

//#define CURVE_FITTING3D
//#define MODE_3D_3

#define SAVE_XLS

WORD *BuildTriangleIndexList (void *pointList, float factor, int numberOfInputPoints,
                              int numDimensions, int clockwise, int *numTriangleVertices);
int drawlines_drag( int point, IplImage* selectedImg );
void onMouseDrag(int event,int x,int y,int flags,void* param);
//*****************************structure**************************************************//
typedef struct data_Struct
{
	int		cursor_x_position[1024];
	int		cursor_y_position[1024];
	double	distance_from_camera[1024];
	double  distance_from_previous_point[1024];
	double  sum_distance_sequence;
	double  angle_animal_camera;
	double  angle_animal_animal;
}data_Struct; 

data_Struct *gdata;
//*******************************Global Variables*************************************************// 
WORD *triangleIndexList;   // this does not need initialization
int n_pos;
int distance_array[1024][3];
int success= 0;                   //whether loading input image is successful
int width_ground, height_ground;   //background image width and height
int width_img, height_img;      //foreground image width and height
CvSize img_size;
/*const*/ char background[]= "background";  //window name    
IplImage *groundImg_org=0, *groundImg_org_text= 0, *image_org=0, *groundImg= 0, *image= 0, *standard= 0, *foreImg= 0, *groundImg_copy= 0;  //background image and foreground image
int img_num = 0;        // the nth input image
int folder_num= 0;
char inputName[128];    //input image name
int pushed_Button;     //the value  of the button that is pushed
int delay1= 100, delay2;                 //the delay by ms
static int times=0;      //time could be at most 3, choose dot by mouse 3 times
int pointarray[1024][2];//= { { 0,0 },{ 0,0 },{ 0,0 } };  //record dots chosen
int pointarrayAll[1024][6];
int offset_x= 34, offset_y= 80;             //offset coordinate of the foreground Image
int offsetNumber_x, offsetNumber_y;      //offset coordinate of the number image
int step1, step2, step3;               //WidthStep
int x,y,z,xx,yy,m,n;
int doExit = 0;
int c, channels1, channels2, channels3;
uchar* data1, *data2, *data3; 
int key;
IplImage *one= 0, *two= 0, *three= 0, *number= 0;
CvScalar CvColorRed = CV_RGB(255,0,0);
int dotx, doty;     
int changePic= 0;
//CvMat *intrinsic, *distortion ;
int mouseParam= 5;
float scale;

char folder[128];
char text[128];

//CvMat objectPoints, imagePoints, pointCounts;

float Next[2][2]= {
	{270, 958},
	{333, 1145}
  };
float Prev[2][2]= {
	{357, 958},
	{421, 1146}
  };
float NextFolder[2][2]= {
	{182, 959},
	{246, 1147}
  };
float NewPath[2][2]= {
	{452, 960},
	{514, 1148}
  };
float Exit[2][2]= {
	{43, 1045},
	{105, 1141}
  };
float Save[2][2]= {
	{539, 963},
	{603, 1149}
  };

float Drag[2][2]= {
	{625, 960},
	{690, 1148}
  };

float Image[2][2]= {
	{75, 33},
	{578, 702}
};

float BlackBoard[2][2]= {
	{637, 51},
	{690, 943}
};

//************************Parameters for calibration************************************//
int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;
int board_n;
int img_num_calibration= 1; //158
char inputName_calibration[128];    //input image name
CvMat* image_points, *object_points, *point_counts, *intrinsic_matrix, *distortion_coeffs, *rotation_vectors, *translation_vectors; 
	int corner_count;
	int successes = 0;
	int step, frame = 0;
	CvPoint2D32f* corners;
	IplImage *image_calibration = 0;
CvMat *intrinsic, *distortion,  *rotation, *translation;
CvSize board_sz;

//Parameters for output
char *txt_name= "..//stick2D_camera3D.txt";
FILE *fp_DATA;
BookHandle book; 
SheetHandle sheet;
int which_row;

float xxx1, yyy1, zzz1;
float xxx2, yyy2, zzz2;

float *xx1,*yy1,*zz1;
float *xx2,*yy2,*zz2;
/***************************************************************************************/
float x_scale;    //selected points, their coordinates tranlated back to original image size
/***************************************************************************************/
//Flags
int flag_NI= 1;
int flag_PI= 0;
int flag_NF= 1;
int flag_EXIT= 0;
int flag_else= 1;
int flag_NP= 1;
int flag_SV= 1;
int flag_DG= 1;
int flag_inaccurate= 0;
/***************************************************************************************/
float inputNum[1280];
char *filelist= "..//stick2D_camera3D.txt";
int Max= 100*5;      //100 poles maximum
int count= 0;
//int best_three_index[3];
/***************************************************************************************/
//For InitFolderPicture2
int flag_folder, flag_image;
int Num_folder= 0;
int Num_image= 0;	
char **folder_name;
char **PicName;
char **temp;
char cmd_arg[128];
char PictureList_dir[128];
	
/***************************************************************************************/
/**************************For Drag and Drop********************************************/
//added in v0.7.0
IplImage *selectedImg= 0;
int point= -1;
IplImage* tempImg;
/***************************************************************************************/
int temp_img_num= 0;
/***********************For planeFitting************************************************/
//These variables should be global.
tagPoint data_xyXX[256];   //256*5= 1280, the number of inputNum[]
tagPoint data_xyYY[256];   //256*5= 1280, the number of inputNum[]
tagPoint data_xyZZ[256];   //256*5= 1280, the number of inputNum[]
tagPlane Plane1, Plane2, Plane3;
int flag_planeFitting= 0;

IplImage* mapx= 0, *mapy= 0; //for compensation 
CvMat* mapX, *mapY;
CvMat* remapX, *remapY;
IplImage *t= 0;

/************************For curve fitting***********************************************/
int flag_curveFitting= 0;
VectorXd Ce(8);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
VectorXd Ce2(8);
VectorXd Ce3(8);

int flag_curveFitting3D= 0;
VectorXd Ce3D(15);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
VectorXd Ce3D_2(5);	
VectorXd Ce3D_3(11);

tagPoint data_XYZ[256];   //256*5= 1280, the number of inputNum[]
tagPlane Plane3D;


//Read in points for plane fitting
void readPoints(tagPoint data_xyXX[], tagPoint data_xyYY[], tagPoint data_xyZZ[], int count){
	for(int i= 0; i< count-1; i++){
		data_xyXX[i].x= inputNum[5*i + 1];
		data_xyYY[i].x= inputNum[5*i + 1];
		data_xyZZ[i].x= inputNum[5*i + 1];
		data_xyXX[i].y= inputNum[5*i + 2];
		data_xyYY[i].y= inputNum[5*i + 2];
		data_xyZZ[i].y= inputNum[5*i + 2];

		data_xyXX[i].z= inputNum[5*i + 3];
		data_xyYY[i].z= inputNum[5*i + 4];
		data_xyZZ[i].z= inputNum[5*i + 5];
		//printf("xyXX[%d].z= %f\n", i, data_xyXX[i].z);
		//printf("xyYY[%d].z= %f\n", i, data_xyYY[i].z);
		//printf("xyZZ[%d].z= %f\n", i, data_xyZZ[i].z);
#ifdef COMPENSATE
		data_xyXX[i].x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
	    data_xyXX[i].y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
		data_xyYY[i].x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
	    data_xyYY[i].y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
		data_xyZZ[i].x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
	    data_xyZZ[i].y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*i+2]), int(inputNum[5*i + 1]));
#endif
	}
}

//Plane fitting
void planeFitting(){
	/*
	PlaneSet2(data_xyXX, Plane1, count);
	PlaneSet2(data_xyZZ, Plane2, count);
	PlaneSet2(data_xyYY, Plane3, count);*/

	PlaneSet2(data_xyXX, Plane1, count);
	PlaneSet2(data_xyYY, Plane2, count);
	PlaneSet2(data_xyZZ, Plane3, count);
	
}

//Calculate Coordinates using plane fitting
void Cal_Coordinates_planeFitting(int x, int y, float *xx, float *yy, float *zz )
{
	flag_planeFitting= 1;

	double XX, YY, ZZ;
	XX= (-1)*( Plane1.d + Plane1.ax * x + Plane1.by * y ) / Plane1.cz;
	YY= (-1)*( Plane2.d + Plane2.ax * x + Plane2.by * y ) / Plane2.cz;
	ZZ= (-1)*( Plane3.d + Plane3.ax * x + Plane3.by * y ) / Plane3.cz;

#ifdef COMPENSATE2
	float param_1s, param_1b, param_2, param_3;
	param_1s= 0.4; //0.5  //from y<param_1s we do compensation
	param_1b= 0.6;  //from y>param_1b we do compensation

	if( y> (image_org->height)*param_1b ) {
	    param_2= 0.8;   //0.8  //when y biggest, how much compensation( y*= how much percent) 
	    param_3= (1- param_2)/(1- param_1b)- 0.0001;
		//printf("compensate2!= %f\n", (float(image_org->height)- y )/float(image_org->height)+ param_2   );
	}
	else if ( y< (image_org->height)*param_1s ) {
	    param_2= 1.6;     //when y biggest, how much compensation( y*= how much percent) 
	    param_3= (1- param_2)/(1- param_1s)- 0.0001;
		//printf("compensate2!= %f\n", (float(image_org->height)- y )/float(image_org->height)+ param_2   );
	}
	else { //no compensation
		param_2= 1;   
	    param_3= 0;
	}
	XX= (-1)*( Plane1.d + Plane1.ax * x + Plane1.by * y * (  param_3*( float(image_org->height)- y )/float(image_org->height)+ param_2   )     ) / Plane1.cz;
	YY= (-1)*( Plane2.d + Plane2.ax * x + Plane2.by * y * (  param_3*( float(image_org->height)- y )/float(image_org->height)+ param_2   )     ) / Plane2.cz;
	ZZ= (-1)*( Plane3.d + Plane3.ax * x + Plane3.by * y * (  param_3*( float(image_org->height)- y )/float(image_org->height)+ param_2   )     ) / Plane3.cz;
#endif
	//printf("XX= (-1)*( Plane1.d + Plane1.ax * x + Plane1.by * y ) / Plane1.cz;\n");
	//printf("XX= (-1)*( %f + %f * x + %f * y ) / %f;\n", Plane1.d,   Plane1.ax,  Plane1.by,  Plane1.cz);
	//printf("YY= (-1)*( Plane2.d + Plane2.ax * x + Plane2.by * y ) / Plane2.cz;\n");
	//printf("YY= (-1)*( %f + %f * x + %f * y ) / %f;\n", Plane2.d,   Plane2.ax,  Plane2.by,  Plane2.cz);
	//printf("ZZ= (-1)*( Plane3.d + Plane3.ax * x + Plane3.by * y ) / Plane3.cz;\n");
	//printf("ZZ= (-1)*( %f + %f * x + %f * y ) / %f;\n", Plane3.d,   Plane3.ax,  Plane3.by,  Plane3.cz);
	//printf("(XX,YY,ZZ)= (%f, %f, %f)\n", XX, YY, ZZ);

    *xx= XX;
	*yy= YY;
	*zz= ZZ;
}


//Curve fitting
//Write my own function to do PTE for calibration. Use this in window.cpp.
void curveFitting(float inputNum[], int count)
{
	// Get Transform Coefficients.
	MatrixXd A((count-1)*2,8);          //Pm:ning
	MatrixXd B((count-1)*2,1);          //Qm:ning
	//VectorXd Ce(8);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
	VectorXd Bb(8);			// A.transpose() * B.
	Matrix3d C;				// coefficient matrix.
	Matrix3d Cg;			// global coefficient matrix.

	//MatrixXd Cd(1,3);
	//MatrixXd xyRfc(3,1);	// coordinate matrix in Refernce frame [x y 1]'.
	//MatrixXd xyCrt(3,1);	// coordinate matrix in Current frame  [X Y 1]'.
	MatrixXd temp(8,8);

	//change here!! Not block but points  
	for(int i=0; i<count-1; i++)
	{
		A(2*i,0) = inputNum[5*i+1];							          A(2*i+1,0) = 0;
		A(2*i,1) = inputNum[5*i+2];								      A(2*i+1,1) = 0;
		A(2*i,2) = 1;												  A(2*i+1,2) = 0;
		A(2*i,3) = 0;												  A(2*i+1,3) = inputNum[5*i+1];
		A(2*i,4) = 0;												  A(2*i+1,4) = inputNum[5*i+2];
		A(2*i,5) = 0;												  A(2*i+1,5) = 1;
		A(2*i,6) = inputNum[5*i+1]*(-1)*inputNum[5*i+3];              A(2*i+1,6) = inputNum[5*i+1]*(-1)*inputNum[5*i+4];
		A(2*i,7) = inputNum[5*i+2]*(-1)*inputNum[5*i+3];              A(2*i+1,7) = inputNum[5*i+2]*(-1)*inputNum[5*i+4];

		B(2*i,0)   = inputNum[5*i+3];
		B(2*i+1,0) = inputNum[5*i+4]; 
	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce);

#ifdef Z_MODE_4
	//Read in (XX, YY, ZZ) for plane fitting
    for(int i= 0; i< count-1; i++){
		data_XYZ[i].x= inputNum[5*i + 3];
		data_XYZ[i].y= inputNum[5*i + 4];
		data_XYZ[i].z= inputNum[5*i + 5];
	}
	//Do plane fitting for aXX + bYY + cZZ + d=0
    PlaneSet2(data_XYZ, Plane3D, count);
#endif

#ifdef Z_MODE_1
	//do plane fitting for ZZ.
	readPoints(data_xyXX, data_xyYY, data_xyZZ, count);
	PlaneSet2(data_xyZZ, Plane3, count);  //previously wrong: PlaneSet2(data_xyYY, Plane3, count);
#endif

#ifdef Z_MODE_2
	//do curve fitting for ZZ. with YY
	for(int i=0; i<count-1; i++)
	{
		A(2*i,0) = inputNum[5*i+2];							          A(2*i+1,0) = 0;
		A(2*i,1) = 0;								                  A(2*i+1,1) = 0;
		A(2*i,2) = 1;												  A(2*i+1,2) = 0;
		A(2*i,3) = 0;												  A(2*i+1,3) = inputNum[5*i+2];
		A(2*i,4) = 0;												  A(2*i+1,4) = 0;
		A(2*i,5) = 0;												  A(2*i+1,5) = 1;
		A(2*i,6) = inputNum[5*i+2]*(-1)*inputNum[5*i+4];              A(2*i+1,6) = inputNum[5*i+2]*(-1)*inputNum[5*i+5];
		A(2*i,7) = 0;                                                 A(2*i+1,7) = 0;

		B(2*i,0)   = inputNum[5*i+4];
		B(2*i+1,0) = inputNum[5*i+5]; 
	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce2);
#endif

#ifdef Z_MODE_3
	//do curve fitting for ZZ. With XX
	for(int i=0; i<count-1; i++)
	{
		A(2*i,0) = inputNum[5*i+1];							          A(2*i+1,0) = 0;
		A(2*i,1) = 0;								                  A(2*i+1,1) = 0;
		A(2*i,2) = 1;												  A(2*i+1,2) = 0;
		A(2*i,3) = 0;												  A(2*i+1,3) = inputNum[5*i+1];
		A(2*i,4) = 0;												  A(2*i+1,4) = 0;
		A(2*i,5) = 0;												  A(2*i+1,5) = 1;
		A(2*i,6) = inputNum[5*i+1]*(-1)*inputNum[5*i+3];              A(2*i+1,6) = inputNum[5*i+1]*(-1)*inputNum[5*i+5];
		A(2*i,7) = 0;                                                 A(2*i+1,7) = 0;

		B(2*i,0)   = inputNum[5*i+3];
		B(2*i+1,0) = inputNum[5*i+5]; 
	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce3);

#endif


}

//Calculate Coordinates using curve fitting
void Cal_Coordinates_curveFitting(int x, int y, float *xx, float *yy, float *zz )
{
	flag_curveFitting= 1;

	double XX, YY, ZZ;
	XX= ( Ce(0)*x + Ce(1)*y + Ce(2) )/( Ce(6)*x + Ce(7)*y + 1);
	YY= ( Ce(3)*x + Ce(4)*y + Ce(5) )/( Ce(6)*x + Ce(7)*y + 1);
	ZZ=  0;  //Now it is not correct, the result must be wrong. But don't know how to assign ZZ yet.

	//printf("XX= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n", Ce(0), Ce(1), Ce(2), Ce(6), Ce(7));
	//printf("YY= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n\n", Ce(3), Ce(4), Ce(5), Ce(6), Ce(7));

#ifdef Z_MODE_4
    ZZ= (-1)*( Plane3D.d + Plane3D.ax * XX + Plane3D.by * YY ) / Plane3D.cz;
#endif

#ifdef Z_MODE_1
	ZZ= (-1)*( Plane3.d + Plane3.ax * x + Plane3.by * y ) / Plane3.cz;  //use plane fitting for ZZ
#endif
#ifdef Z_MODE_2
	ZZ= ( Ce2(3)*x + Ce2(4)*y + Ce2(5) )/( Ce2(6)*x + Ce2(7)*y + 1);
	printf("YY= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n", Ce2(0), Ce2(1), Ce2(2), Ce2(6), Ce2(7));
	printf("ZZ= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n", Ce2(3), Ce2(4), Ce2(5), Ce2(6), Ce2(7));
	printf("(YY,YY)= (%f, %f)\n", ( Ce(3)*x + Ce(4)*y + Ce(5) )/( Ce(6)*x + Ce(7)*y + 1), ( Ce2(0)*x + Ce2(1)*y + Ce2(2) )/( Ce2(6)*x + Ce2(7)*y + 1));
#endif

#ifdef Z_MODE_3
    ZZ= ( Ce3(3)*x + Ce3(4)*y + Ce3(5) )/( Ce3(6)*x + Ce3(7)*y + 1);
	//printf("XX= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n", Ce3(0), Ce3(1), Ce3(2), Ce3(6), Ce3(7));
	//printf("ZZ= ( %f *x + %f*y +  %f )/(  %f*x +  %f*y + 1);\n", Ce3(3), Ce3(4), Ce3(5), Ce3(6), Ce3(7));
	//printf("(YY,YY)= (%f, %f)\n", ( Ce(3)*x + Ce(4)*y + Ce(5) )/( Ce(6)*x + Ce(7)*y + 1), ( Ce3(0)*x + Ce3(1)*y + Ce3(2) )/( Ce3(6)*x + Ce3(7)*y + 1));
#endif
	
	//printf("(XX,YY,ZZ)= (%f, %f, %f)\n", XX, YY, ZZ);
     
    *xx= XX;
	*yy= YY;
	*zz= ZZ;
}


void curveFitting3D(float inputNum[], int count)
{

#ifdef MODE_3D_1
	 printf("num of poles in the list is:%d\n", count-1);
	// Get Transform Coefficients.
	MatrixXd A((count-1)*3,15);          //Pm:ning
	MatrixXd B((count-1)*3,1);          //Qm:ning
	//VectorXd Ce(15);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
	VectorXd Bb(15);			// A.transpose() * B.

	MatrixXd temp(15,15);

	//change here!! Not block but points  
	for(int i=0; i<count-1; i++)
	{
		A(3*i,0) = inputNum[5*i+1];							          A(3*i+1,0) = 0;						                               A(3*i+2,0) = 0;
		A(3*i,1) = inputNum[5*i+2];								      A(3*i+1,1) = 0;						                               A(3*i+2,1) = 0;
		A(3*i,2) = 0.0;												  A(3*i+1,2) = 0;						         	         	       A(3*i+2,2) = 0;
		A(3*i,3) = 1;												  A(3*i+1,3) = 0;						         	         	       A(3*i+2,3) = 0;

		A(3*i,4) = 0;												  A(3*i+1,4) = inputNum[5*i+1];						 	               A(3*i+2,4) = 0;
		A(3*i,5) = 0;												  A(3*i+1,5) = inputNum[5*i+2];							               A(3*i+2,5) = 0;
		A(3*i,6) = 0;                                                 A(3*i+1,6) = 0.0;						        	         	       A(3*i+2,6) = 0;
		A(3*i,7) = 0;                                                 A(3*i+1,7) = 1;						        	                   A(3*i+2,7) = 0;

		A(3*i,8) = 0;                                                 A(3*i+1,8) = 0;						        	                   A(3*i+2,8) = inputNum[5*i+1];
		A(3*i,9) = 0;                                                 A(3*i+1,9) = 0;						        	                   A(3*i+2,9) = inputNum[5*i+2];
		A(3*i,10) = 0;                                                A(3*i+1,10) = 0;						        	                   A(3*i+2,10) = 0.0;
		A(3*i,11) = 0;                                                A(3*i+1,11) = 0;						        	         	       A(3*i+2,11) = 1;

		A(3*i,12) = inputNum[5*i+1]*(-1)*inputNum[5*i+3];             A(3*i+1,12) = inputNum[5*i+1]*(-1)*inputNum[5*i+4];		           A(3*i+2,12) = inputNum[5*i+1]*(-1)*inputNum[5*i+5];
		A(3*i,13) = inputNum[5*i+2]*(-1)*inputNum[5*i+3];             A(3*i+1,13) = inputNum[5*i+2]*(-1)*inputNum[5*i+4];		           A(3*i+2,13) = inputNum[5*i+2]*(-1)*inputNum[5*i+5];
		A(3*i,14) = 0.0;                                              A(3*i+1,14) = 0.0;						         	         	   A(3*i+2,14) = 0.0;

		B(3*i,0)   = inputNum[5*i+3];
		B(3*i+1,0) = inputNum[5*i+4]; 
		B(3*i+2,0) = inputNum[5*i+5];
	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce3D);

	printf("Ce3D= (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n", Ce3D(0),Ce3D(1),Ce3D(2), Ce3D(3), Ce3D(4), Ce3D(5), Ce3D(6), Ce3D(7), Ce3D(8), Ce3D(9), Ce3D(10), Ce3D(11), Ce3D(12), Ce3D(13), Ce3D(14)); 
#endif

#ifdef MODE_3D_2  //X£¬Y comes from curveFitting, Z comes from here  

	MatrixXd A((count-1)*1,5);          //Pm:ning
	MatrixXd B((count-1)*1,1);          //Qm:ning
	//VectorXd Ce(5);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
	VectorXd Bb(5);			// A.transpose() * B.

	MatrixXd temp(5,5);

	//change here!! Not block but points  
	for(int i=0; i<count-1; i++)
	{
		A(i,0) = inputNum[5*i+1];							       
		A(i,1) = inputNum[5*i+2];								     
		A(i,2) = 1;												  
		A(i,3) = inputNum[5*i+1]*(-1)*inputNum[5*i+5];												 
		A(i,4) = inputNum[5*i+2]*(-1)*inputNum[5*i+5];												 
	
		B(i,0)   = inputNum[5*i+5];

	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce3D_2);
	printf("Ce3D_2= (%f, %f, %f, %f, %f)\n", Ce3D_2(0),Ce3D_2(1),Ce3D_2(2), Ce3D_2(3), Ce3D_2(4));
#endif

#ifdef  MODE_3D_3
	MatrixXd A((count-1)*3,11);          //Pm:ning
	MatrixXd B((count-1)*3,1);          //Qm:ning
	//VectorXd Ce(11);			// coefficients.    //Ce is what i want. Put it in windows.cpp as global
	VectorXd Bb(11);			// A.transpose() * B.

	MatrixXd temp(11,11);

	//change here!! Not block but points  
	for(int i=0; i<count-1; i++)
	{
		A(3*i,0) = inputNum[5*i+1];							          A(3*i+1,0) = 0;                                           A(3*i+2,0) = 0;
		A(3*i,1) = inputNum[5*i+2];								      A(3*i+1,1) = 0;                                           A(3*i+2,1) = 0;
		A(3*i,2) = 1;												  A(3*i+1,2) = 0;                                           A(3*i+2,2) = 0;

		A(3*i,3) = 0;												  A(3*i+1,3) = inputNum[5*i+1];                             A(3*i+2,3) = 0;
		A(3*i,4) = 0;												  A(3*i+1,4) = inputNum[5*i+2];                             A(3*i+2,4) = 0;
		A(3*i,5) = 0;												  A(3*i+1,5) = 1;                                           A(3*i+2,5) = 0;

        A(3*i,6) = 0;												  A(3*i+1,6) = 0;                                           A(3*i+2,6) = inputNum[5*i+1];
		A(3*i,7) = 0;												  A(3*i+1,7) = 0;                                           A(3*i+2,7) = inputNum[5*i+2];
		A(3*i,8) = 0;												  A(3*i+1,8) = 0;                                           A(3*i+2,8) = 1;

		A(3*i,9) = inputNum[5*i+1]*(-1)*inputNum[5*i+3];              A(3*i+1,9) = inputNum[5*i+1]*(-1)*inputNum[5*i+4];        A(3*i+2,9) = inputNum[5*i+1]*(-1)*inputNum[5*i+5];
		A(3*i,10) = inputNum[5*i+2]*(-1)*inputNum[5*i+3];             A(3*i+1,10) = inputNum[5*i+2]*(-1)*inputNum[5*i+4];       A(3*i+2,10) = inputNum[5*i+2]*(-1)*inputNum[5*i+5];

		B(3*i,0)   = inputNum[5*i+3];
		B(3*i+1,0) = inputNum[5*i+4]; 
		B(3*i+2,0) = inputNum[5*i+5];

	}	//end of creating matrice A B.

	temp = A.transpose() * A;//A conjugate * A.
	Bb	 = A.transpose() * B;
	temp.ldlt().solve(Bb,&Ce3D_3);
	printf("Ce3D_3= (%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f)\n", Ce3D_3(0),Ce3D_3(1),Ce3D_3(2), Ce3D_3(3), Ce3D_3(4), Ce3D_3(5), Ce3D_3(6), Ce3D_3(7), Ce3D_3(8), Ce3D_3(9), Ce3D_3(10));
#endif
}

void Cal_Coordinates_curveFitting3D(int x, int y, float *xx, float *yy, float *zz )
{
	flag_curveFitting3D= 1;

	double XX, YY, ZZ;

	
#ifdef MODE_3D_1
	XX= ( Ce3D(0)*x + Ce3D(1)*y + Ce3D(3) )/( Ce3D(12)*x + Ce3D(13)*y + 1);
	YY= ( Ce3D(4)*x + Ce3D(5)*y + Ce3D(7) )/( Ce3D(12)*x + Ce3D(13)*y + 1);
	ZZ= ( Ce3D(8)*x + Ce3D(9)*y + Ce3D(11) )/( Ce3D(12)*x + Ce3D(13)*y + 1);
	//XX= 10000*( Ce3D(0)*x + Ce3D(1)*y + Ce3D(3) )/( Ce3D(12)*x + Ce3D(13)*y + 1);
	//YY= 10000*( Ce3D(4)*x + Ce3D(5)*y + Ce3D(7) )/( Ce3D(12)*x + Ce3D(13)*y + 1);
	//ZZ= 10000*( Ce3D(8)*x + Ce3D(9)*y + Ce3D(11) )/( Ce3D(12)*x + Ce3D(13)*y + 1); 
#endif
#ifdef MODE_3D_2
	XX= ( Ce(0)*x + Ce(1)*y + Ce(2) )/( Ce(6)*x + Ce(7)*y + 1);
	YY= ( Ce(3)*x + Ce(4)*y + Ce(5) )/( Ce(6)*x + Ce(7)*y + 1);
	ZZ= ( Ce3D_2(0)*x + Ce3D_2(1)*y + Ce3D_2(2) )/( Ce3D_2(3)*x + Ce3D_2(4)*y + 1); 
#endif
#ifdef MODE_3D_3
	XX= ( Ce3D_3(0)*x + Ce3D_3(1)*y + Ce3D_3(2) )/( Ce3D_3(9)*x + Ce3D_3(10)*y + 1); 
	YY= ( Ce3D_3(3)*x + Ce3D_3(4)*y + Ce3D_3(5) )/( Ce3D_3(9)*x + Ce3D_3(10)*y + 1);
	ZZ= ( Ce3D_3(6)*x + Ce3D_3(7)*y + Ce3D_3(8) )/( Ce3D_3(9)*x + Ce3D_3(10)*y + 1);
#endif

	//printf("(XX,YY,ZZ)= (%f, %f, %f)\n", XX, YY, ZZ);
     
    *xx= XX;
	*yy= YY;
	*zz= ZZ;
}


void ChangeFolderFunc(int NextOrPrev)
	{
		        if ( NextOrPrev )
				{
		            folder_num++; 
				}
				else
				{
					folder_num--;
				}
				sprintf( folder, "..\\data\\%s", folder_name[folder_num]);                      //derivbe first folder's directory
				//sprintf( cmd_arg, "dir %s\\ /B /O:D >%s\\PictureList.txt", folder, folder);   //modify command argument
				sprintf( cmd_arg, "dir %s\\ /B /O:N >%s\\PictureList.txt", folder, folder);   //modify command argument
				system(cmd_arg);                                                      //generate Picture list in that folder

				sprintf( PictureList_dir, "..\\data\\%s\\PictureList.txt", folder);   //derive 
				FILE* fp_PictureList = fopen( PictureList_dir, "r" );
				

               
				for(int i= 0; i<256; i++)
				{
				  flag_image= fscanf(fp_PictureList, "%s", PicName[i]);                          //derive first folder's name
				  if (flag_image==1)
				  {
					 Num_image++;
				  }
				}
				


				fclose(fp_PictureList );
				Num_image--;  //exclude the txt file. The total number of images in this folder
				sprintf( inputName, "%s\\%s", folder, PicName[0]);
				printf("Current folder is: %s\n", folder);
				printf("First Pic is: %s\n", PicName[0]);
	}
int* find_3_nearest( int x, int y)
{
	
	float u[30],v[30],dist[30],temp[30];
	float best,second,third;
	int best_three_index[3];

	for (int PoleNum = 0; PoleNum<= count-1; PoleNum++)
	{
        u[PoleNum]= inputNum[5*(PoleNum)+1];
		v[PoleNum]= inputNum[5*(PoleNum)+2];
		dist[PoleNum]= (x- u[PoleNum])*(x- u[PoleNum])+ (y- v[PoleNum])*(y- v[PoleNum]);
		temp[PoleNum]= dist[PoleNum];
	}
    

    best_three_index[0]= 0;
	best_three_index[1]= 1;
	best_three_index[2]= 2;
	best= dist[0];
    second= dist[0];
	third= dist[0];
	for (int PoleNum = 0; PoleNum<= count-1; PoleNum++)
	{
		if (dist[PoleNum]< best)
		{
			best= dist[PoleNum];
            best_three_index[0]= PoleNum;
		}
	}
    dist[best_three_index[0]]= 10000000000;

    for (int PoleNum = 0; PoleNum<= count-1; PoleNum++)
	{
		if (dist[PoleNum]< second)
		{
			second= dist[PoleNum];
            best_three_index[1]= PoleNum;
		}
	}
    dist[best_three_index[1]]= 10000000000;

	for (int PoleNum = 0; PoleNum<= count-1; PoleNum++)
	{
		if (dist[PoleNum]< third)
		{
			third= dist[PoleNum];
            best_three_index[2]= PoleNum;
		}
	}
	dist[best_three_index[2]]= 10000000000;


	  for (int Num = 0; Num<= 2; Num++)
	{
		//printf("best_three_index[%d] is:%d\n", Num, best_three_index[Num]);  
    }

 
 return best_three_index;
}

int* find_nearest( int x, int y)
{
	
	float u[30],v[30],dist[30],temp[30];
	float min;
	int minPtr, k, i;
	int best_index[10];

	for (int PoleNum = 0; PoleNum<= count-1; PoleNum++)
	{
        u[PoleNum]= inputNum[5*(PoleNum)+1];
		v[PoleNum]= inputNum[5*(PoleNum)+2];
		dist[PoleNum]= (x- u[PoleNum])*(x- u[PoleNum])+ (y- v[PoleNum])*(y- v[PoleNum]);
		temp[PoleNum]= dist[PoleNum];
	}

	for(k=0; k<6; k++)
	{
		min = 1000000000;
		minPtr = 0;
		for(i=0; i<count; i++)
		{
			if(dist[i] < min)
			{
				min = dist[i];
				minPtr = i;
			}
		}

		dist[minPtr] = 1000000000;
		best_index[k] = minPtr;
	}
    
	 for (int Num = 0; Num< 6; Num++)
	{
		//printf("best_six_index[%d] is:%d\n", Num, best_index[Num]);  
    }

 
 return best_index;
}



void find_nearest_N( int N, int x, int y, int best_index[])
{
	//int N= 10; //ning
	//int best_index[10];
	//float u[30],v[30],dist[30],temp[30];
	float u[100],v[100],dist[100],temp[100];  //ning
	float min;
	int minPtr, k, i;

	for (int PoleNum = 0; PoleNum< count; PoleNum++)
	{
        u[PoleNum]= inputNum[5*(PoleNum)+1];
		v[PoleNum]= inputNum[5*(PoleNum)+2];
		dist[PoleNum]= (x- u[PoleNum])*(x- u[PoleNum])+ (y- v[PoleNum])*(y- v[PoleNum]);
		temp[PoleNum]= dist[PoleNum];
	}

	for(k=0; k<N; k++)
	{
		min = 1000000000;
		minPtr = 0;
		for(i=0; i< count-1; i++)
		{
			if(dist[i] < min)
			{
				min = dist[i];
				minPtr = i;
			}
		}

		dist[minPtr] = 1000000000;
		best_index[k] = minPtr;
		//printf("Best Index[%d]: %d\n", k, best_index[k]);
	}
    
//	 for (int Num = 0; Num< N; Num++)
	//{
	//	printf("best_N_index[%d] is:%d\n", Num, best_index[Num]);  
  //  }

 
 //return best_index;
}
int Cal_Coordinates3(/*char *txt_name,*/ int x, int y, int *xx, int *yy, int *zz )
{
   int *best_index;

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct, sXX, sYY, sZZ;
   int k1, k2, k3;

   double dx21, dy21, dx31, dy31;

   best_index= find_nearest(x, y);

   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1<=3; k1++)
   for(k2=(k1+1); k2<=4; k2++)
   for(k3=(k2+1); k3<=5; k3++)
   {
		P1_x= inputNum[5*(best_index[k1])+1];
		P1_y= inputNum[5*(best_index[k1])+2];
		P1_XX= inputNum[5*(best_index[k1])+3];
		P1_YY= inputNum[5*(best_index[k1])+4];
		P1_ZZ= inputNum[5*(best_index[k1])+5];
		P2_x= inputNum[5*(best_index[k2])+1];
		P2_y= inputNum[5*(best_index[k2])+2];
		P2_XX= inputNum[5*(best_index[k2])+3];
		P2_YY= inputNum[5*(best_index[k2])+4];
		P2_ZZ= inputNum[5*(best_index[k2])+5];
		P3_x= inputNum[5*(best_index[k3])+1];
		P3_y= inputNum[5*(best_index[k3])+2];
        P3_XX= inputNum[5*(best_index[k3])+3];
		P3_YY= inputNum[5*(best_index[k3])+4];
		P3_ZZ= inputNum[5*(best_index[k3])+5];	

		//difference vectors
		dx21 = P2_x - P1_x;
		dy21 = P2_y - P1_y;

		dx31 = P3_x - P1_x;
		dy31 = P3_y - P1_y;
		
		cosTheta = (dx21 * dx31 + dy21 * dy31)/sqrt( (dx21*dx21 + dy21*dy21) * (dx31*dx31 + dy31*dy31) + 0.0001);
		if(cosTheta < 0) cosTheta = -cosTheta;

		if(cosTheta < 0.8)
		{
	
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );
	      
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			sXX = sXX + (int)(P_XX);
			sYY = sYY + (int)(P_YY);
			sZZ = sZZ + (int)(P_ZZ);

			ct++;
		}

   }

   if(ct > 0)
   {
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
   }
   else
   {
	   printf("\n\nERROR!!!");
	   return -1;
   }

   *xx= sXX;
   *yy= sYY;
   *zz= sZZ;

   return 1;



}




int Cal_Coordinates4(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k1, k2, k3;

   double dx21, dy21, dx31, dy31;

   //best_index= find_nearest(x, y);
   int N= 10;
  find_nearest_N(N, x, y, best_index);
  
  // printf("Best Index[%d]: %d\n", 0, best_index[0]);  
  // printf("Best Index[%d]: %d\n", 1, best_index[1]); 
  // printf("Best Index[%d]: %d\n", 2, best_index[2]); 

   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1<N; k1++)
	  for(k2= k1+1; k2<N-1; k2++)     //for(k2=0; k2<N-1; k2++) 
        for(k3=k2+1; k3<N; k3++)
   //for(k2=(k1+1); k2<=4; k2++)
  // for(k3=(k2+1); k3<=5; k3++)
   {
	   if ((k1==k2)||(k2==k3)||(k1==k3)) continue;
	     //printf("[k1]: %d\n", k1);  
	     //printf("[k2]: %d\n", k2); 
		 //printf("[k3]: %d\n", k3); 
	   //printf("Best Index[%d]: %d\n", k1, best_index[k1]);  
	   // printf("Best Index[%d]: %d\n", k2, best_index[k2]); 
		// printf("Best Index[%d]: %d\n", k3, best_index[k3]); 
		P1_x= inputNum[5*(best_index[k1])+1];
		P1_y= inputNum[5*(best_index[k1])+2];
		P1_XX= inputNum[5*(best_index[k1])+3];
		P1_YY= inputNum[5*(best_index[k1])+4];
		P1_ZZ= inputNum[5*(best_index[k1])+5];
		P2_x= inputNum[5*(best_index[k2])+1];
		P2_y= inputNum[5*(best_index[k2])+2];
		P2_XX= inputNum[5*(best_index[k2])+3];
		P2_YY= inputNum[5*(best_index[k2])+4];
		P2_ZZ= inputNum[5*(best_index[k2])+5];
		P3_x= inputNum[5*(best_index[k3])+1];
		P3_y= inputNum[5*(best_index[k3])+2];
        P3_XX= inputNum[5*(best_index[k3])+3];
		P3_YY= inputNum[5*(best_index[k3])+4];
		P3_ZZ= inputNum[5*(best_index[k3])+5];	

		//difference vectors
		/*
		dx21 = P2_x - P1_x;
		dy21 = P2_y - P1_y;

		dx31 = P3_x - P1_x;
		dy31 = P3_y - P1_y;
		
		cosTheta = (dx21 * dx31 + dy21 * dy31)/sqrt( (dx21*dx21 + dy21*dy21) * (dx31*dx31 + dy31*dy31) + 0.0001);
		if(cosTheta < 0) cosTheta = -cosTheta;
        */
		//if(cosTheta < 1)
		//{
	
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

			//if ( (w1>-4)&(w1<4)&&(w2>-4)&&(w2<4) )  //!!!
			//if ( (w1>-3)&(w1<3)&&(w2>-3)&&(w2<3) ) 
			if ( (w1>-2)&&(w1<2)&&(w2>-2)&&(w2<2) ) 
				//if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1)&&(w1+w2>=-1)&&(w1+w2<=1) ) 
			//if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1)/*&&(w1+w2>=-1)&&(w1+w2<=1)*/ ) 
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			//sXX = sXX + (int)(P_XX);
			//sYY = sYY + (int)(P_YY);
			//sZZ = sZZ + (int)(P_ZZ);
			sXX = sXX + (P_XX);
			sYY = sYY + (P_YY);
			sZZ = sZZ + (P_ZZ);

			ct++;
			}
		//}

   }
   //printf("count is:%d\n", ct);
   if(ct > 0)
   {
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
   }
   else
   {
	   printf("\n\nERROR!!!");
	   return -1;
   }

   *xx= sXX;
   *yy= sYY;
   *zz= sZZ;

   //return 1;
   //printf("count is: %d\n", ct);
   return ct;


}

int Cal_Coordinates_nearestTriangle(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k1, k2, k3;

   double dx21, dy21, dx31, dy31;

   int N= 3;   //N= 3 means we want only 3 vertices near this point to form one triangle.
   find_nearest_N(N, x, y, best_index);

   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1<N; k1++)
	  for(k2= k1+1; k2<N-1; k2++)     //for(k2=0; k2<N-1; k2++) 
        for(k3=k2+1; k3<N; k3++)
   {
	   if ((k1==k2)||(k2==k3)||(k1==k3)) continue;
	     //printf("[k1]: %d\n", k1);  
	     //printf("[k2]: %d\n", k2); 
		 //printf("[k3]: %d\n", k3); 
	   //printf("Best Index[%d]: %d\n", k1, best_index[k1]);  
	   // printf("Best Index[%d]: %d\n", k2, best_index[k2]); 
		// printf("Best Index[%d]: %d\n", k3, best_index[k3]); 
		P1_x= inputNum[5*(best_index[k1])+1];
		P1_y= inputNum[5*(best_index[k1])+2];
		P1_XX= inputNum[5*(best_index[k1])+3];
		P1_YY= inputNum[5*(best_index[k1])+4];
		P1_ZZ= inputNum[5*(best_index[k1])+5];
		P2_x= inputNum[5*(best_index[k2])+1];
		P2_y= inputNum[5*(best_index[k2])+2];
		P2_XX= inputNum[5*(best_index[k2])+3];
		P2_YY= inputNum[5*(best_index[k2])+4];
		P2_ZZ= inputNum[5*(best_index[k2])+5];
		P3_x= inputNum[5*(best_index[k3])+1];
		P3_y= inputNum[5*(best_index[k3])+2];
        P3_XX= inputNum[5*(best_index[k3])+3];
		P3_YY= inputNum[5*(best_index[k3])+4];
		P3_ZZ= inputNum[5*(best_index[k3])+5];	

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
		//printf("w1 and w2 are: %f %f \n", w1, w2 );

		P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
		P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
		P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
		
		sXX = sXX + (P_XX);
		sYY = sYY + (P_YY);
		sZZ = sZZ + (P_ZZ);

		ct++;

		//show the triangle that covers the dot in thick lines
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
		 3, 8, 0 );  
   }
   //printf("count is:%d\n", ct);
   if(ct > 0)  //ct should be 1, meaning there is only one nearest triangle.
   {
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
   }
   else
   {
	   printf("\n\nERROR!!!");
	   return -1;
   }

   *xx= sXX;
   *yy= sYY;
   *zz= sZZ;

   return ct;
}

int Cal_Coordinates_Delaunay(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count; k0++)
   {
	   PolesXY[k0+0]=  inputNum[5*k0+1];
	   PolesXY[k0+1]=  inputNum[5*k0+2];
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  (float)32000,
							  count,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	



   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	   //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	   //printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

		//difference vectors

			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				//if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1)&&(w1+w2>=-1)&&(w1+w2<=1) )  //this should make it unique
				//if ( (w1>=-2)&&(w1<=2)&&(w2>=-2)&&(w2<=2) ) 
			if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1) )     //only one vertex left
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			//sXX = sXX + (int)(P_XX);
			//sYY = sYY + (int)(P_YY);
			//sZZ = sZZ + (int)(P_ZZ);
			sXX = sXX + (P_XX);
			sYY = sYY + (P_YY);
			sZZ = sZZ + (P_ZZ);

			ct++;
			}
   }
   //printf("count is:%d\n", ct);
   if(ct > 0)
   {
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
   }
   else
   {
	   printf("\n ERROR!!!ct= 0!!! \n");
	   for(k1=0; k1< numTriangleVertices; k1+=3)
     {
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

		//difference vectors

			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			
			if ( (w1>=-2)&&(w1<=2)&&(w2>=-2)&&(w2<=2) )  //make the condition loose to give inaccurate result.
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);

			sXX = sXX + (P_XX);
			sYY = sYY + (P_YY);
			sZZ = sZZ + (P_ZZ);

			ct++;
			}
	   }
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
	   ct= 0;
   }

   *xx= sXX;
   *yy= sYY;
   *zz= sZZ;

   //return 1;
   //printf("count is: %d\n", ct);
   //free ( triangleIndexList );
   return ct;


}

int Cal_Coordinates_Delaunay_improved(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	// printf("numTriangleVertices is:%d\n", numTriangleVertices);
	 //printf("num of poles in the list is:%d\n", count-1);




   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );

		//difference vectors

			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
				//if ( (w1>=-2)&&(w1<=2)&&(w2>=-2)&&(w2<=2) ) 
			//if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1) )     //only one vertex left
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			/*
			sXX = sXX + (P_XX);
			sYY = sYY + (P_YY);
			sZZ = sZZ + (P_ZZ);
            */
            
			// printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			// printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	        // printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	        // printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
					 
					 /* 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 ); 
					 */
				
			 }
			/*
			printf("(P1_x ,P1_y)= (%f, %f)\n", P1_x ,P1_y);
			printf("(P2_x ,P2_y)= (%f, %f)\n", P2_x ,P2_y);
			printf("(P3_x ,P3_y)= (%f, %f)\n", P3_x ,P3_y);
            */

			ct++;
			}
   }
   //printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   /*
	   sXX = sXX / ct;
	   sYY = sYY / ct;
	   sZZ = sZZ / ct;
	   */
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   
   else ct= Cal_Coordinates4(x,y,xx,yy,zz);
   //return 1;
   //printf("count is: %d\n", ct);
   //free ( triangleIndexList );
   return ct;


}

int Cal_Coordinates_Delaunay_improved_2ndVersion(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	 //printf("numTriangleVertices is:%d\n", numTriangleVertices);
	 //printf("num of poles in the list is:%d\n", count-1);




   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
        
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			// printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	         //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	         //printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
					 
#ifdef SHOWLINES
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
   printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //ct= Cal_Coordinates_nearestTriangle(x,y,xx,yy,zz);
   {
	 float u[100],v[100],dist[100],temp[100];  //ning
	 float min;
	 int minPtr, k, i;
	 int best_index[3];
	 int N= 3;
	 int PoleNum= 0;

	 for(k1=0; k1< numTriangleVertices; k1+=3)
	 {
		PoleNum= k1/3;

		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		if ( (w1>=-1)&&(w1<=1)&&(w2>=-1)&&(w2<=1)/*&&(w1+w2>=0)&&(w1+w2<=1)*/ )  //not-bad triangle
		{
		   //dist[PoleNum]= (x- P1_x)*(x- P1_x)+ (y- P1_y)*(y- P1_y) + (x- P2_x)*(x- P2_x)+ (y- P2_y)*(y- P2_y)+ (x- P3_x)*(x- P3_x)+ (y- P3_y)*(y- P3_y);   //neareast triangle
		   dist[PoleNum]= (x- (P1_x+ P2_x + P3_x)/3)*(x- (P1_x+ P2_x + P3_x)/3)+ (y- (P1_y+ P2_y + P3_y)/3)*(y- (P1_y+ P2_y + P3_y)/3) ;                     //true nearest triangle
		   //printf("AT LEAST ONE!\n");
		}
		else
		{
			dist[PoleNum]= 10000000000;
		}
	 }

		min = 1000000000;
		minPtr = 0;
		for(PoleNum=0; PoleNum< numTriangleVertices/3; PoleNum++)
		{
			if(dist[PoleNum] < min)
			{
				min = dist[PoleNum];
				minPtr = PoleNum;
			}
		}

		k1= 3*minPtr;
	
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
		P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
		P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);

		*xx=  0 + (P_XX);
        *yy=  0 + (P_YY);
        *zz=  0 + (P_ZZ);
#ifdef SHOWLINES
		//show the triangle that covers the dot in thick lines
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );  
#endif
   }//end of "else"
   //free ( triangleIndexList );
   return ct;
}

int Cal_Coordinates_Delaunay_improved_3rdVersion(/*char *txt_name,*/ int x, int y, float *xx, float *yy, float *zz )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	 //printf("numTriangleVertices is:%d\n", numTriangleVertices);
	 //printf("num of poles in the list is:%d\n", count-1);




   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

#ifdef SHOWLINES
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
#endif
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
       // cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			 //printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	         //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	        // printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
#ifdef SHOWLINES					 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
   //printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //when the point is not in a delaunay triangle, find close good-ones from all possible triangles(can be non-delaunay ones)
   {
	 float u[100],v[100],dist[40][40][40],temp[40][40][40];  //ning
	 float min;
	 int minPtr1, minPtr2, minPtr3, k, i;
	 int best_index[3];
	 int PoleNum= 0;

	  int N= count-1;  //loop through all poles
	  min = 1000000000;

	  int k1, k2, k3;
	 
      minPtr1= 0;
	  minPtr2=  N/2;
      minPtr3=  N;//If none good found, use this initial three points 

   for(k1=0; k1<N; k1++)
	  for(k2= k1+1; k2<N-1; k2++)     
        for(k3=k2+1; k3<N; k3++)
   {
	   if ((k1==k2)||(k2==k3)||(k1==k3)) continue;
	 
		P1_x= inputNum[5*k1+1];
		P1_y= inputNum[5*k1+2];
		P1_XX= inputNum[5*k1+3];
		P1_YY= inputNum[5*k1+4];
		P1_ZZ= inputNum[5*k1+5];
		P2_x= inputNum[5*k2+1];
		P2_y= inputNum[5*k2+2];
		P2_XX= inputNum[5*k2+3];
		P2_YY= inputNum[5*k2+4];
		P2_ZZ= inputNum[5*k2+5];
		P3_x= inputNum[5*k3+1];
		P3_y= inputNum[5*k3+2];
        P3_XX= inputNum[5*k3+3];
		P3_YY= inputNum[5*k3+4];
		P3_ZZ= inputNum[5*k3+5];

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		int restraint= 1;
		if ( (w1>=(-1)*restraint)&&(w1<= restraint)&&(w2>= (-1)*restraint)&&(w2<= restraint)/*&&(w1+w2>=0)&&(w1+w2<=1)*/ )  //not-bad triangle
		{
		   //dist[PoleNum]= (x- P1_x)*(x- P1_x)+ (y- P1_y)*(y- P1_y) + (x- P2_x)*(x- P2_x)+ (y- P2_y)*(y- P2_y)+ (x- P3_x)*(x- P3_x)+ (y- P3_y)*(y- P3_y);   //neareast triangle
		   dist[k1][k2][k3]= (x- (P1_x+ P2_x + P3_x)/3)*(x- (P1_x+ P2_x + P3_x)/3)+ (y- (P1_y+ P2_y + P3_y)/3)*(y- (P1_y+ P2_y + P3_y)/3) ;                     //true nearest triangle
		   //printf("AT LEAST ONE! (k1, k2, k3)= (%d, %d, %d)\n", k1, k2, k3 );
		}
		else
		{
			dist[k1][k2][k3]= 10000000000;
		}

		if(dist[k1][k2][k3] < min)
			{
				min = dist[k1][k2][k3];
				minPtr1 = k1;
				minPtr2= k2;
				minPtr3= k3;
			}
	 }
	
		P1_x= inputNum[5*minPtr1+1];   
		P1_y= inputNum[5*minPtr1+2];
		P1_XX= inputNum[5*minPtr1+3];
		P1_YY= inputNum[5*minPtr1+4];
		P1_ZZ= inputNum[5*minPtr1+5];
		P2_x= inputNum[5*minPtr2+1];
		P2_y= inputNum[5*minPtr2+2];
		P2_XX= inputNum[5*minPtr2+3];
		P2_YY= inputNum[5*minPtr2+4];
		P2_ZZ= inputNum[5*minPtr2+5];
		P3_x= inputNum[5*minPtr3+1];
		P3_y= inputNum[5*minPtr3+2];
        P3_XX= inputNum[5*minPtr3+3];
		P3_YY= inputNum[5*minPtr3+4];
		P3_ZZ= inputNum[5*minPtr3+5];	

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
		P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
		P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);

		*xx=  0 + (P_XX);
        *yy=  0 + (P_YY);
        *zz=  0 + (P_ZZ);

#ifdef SHOWLINES
		//show the triangle that covers the dot in thick lines
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );  
#endif
   }//end of "else"
   //free ( triangleIndexList );
   return ct;
}


int Cal_Coordinates_Delaunay_improved_lastVersion(int x, int y, float *xx, float *yy, float *zz )
{
   int AT_LEAST_ONE= 0;
   int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	// printf("numTriangleVertices is:%d\n", numTriangleVertices);
	// printf("num of poles in the list is:%d\n", count-1);




   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

#ifdef SHOWLINES
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
#endif
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
       // cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			// printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	        // printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	        // printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
#ifdef SHOWLINES					 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
   //printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //when the point is not in a delaunay triangle, find close good-ones from all possible triangles(can be non-delaunay ones)
   {
	 float u[100],v[100],dist[40][40][40],temp[40][40][40];  //ning
	 float min;
	 int minPtr1, minPtr2, minPtr3, k, i;
	 int best_index[3];
	 int PoleNum= 0;

	  int N= count-1;  //loop through all poles
	  min = 1000000000;

	  int k1, k2, k3;
	 
      minPtr1= 0;
	  minPtr2=  N/2;
      minPtr3=  N;//If none good found, use this initial three points 

   for(k1=0; k1<N; k1++)
	  for(k2= k1+1; k2<N-1; k2++)     
        for(k3=k2+1; k3<N; k3++)
   {
	   if ((k1==k2)||(k2==k3)||(k1==k3)) continue;
	 
		P1_x= inputNum[5*k1+1];
		P1_y= inputNum[5*k1+2];
		P1_XX= inputNum[5*k1+3];
		P1_YY= inputNum[5*k1+4];
		P1_ZZ= inputNum[5*k1+5];
		P2_x= inputNum[5*k2+1];
		P2_y= inputNum[5*k2+2];
		P2_XX= inputNum[5*k2+3];
		P2_YY= inputNum[5*k2+4];
		P2_ZZ= inputNum[5*k2+5];
		P3_x= inputNum[5*k3+1];
		P3_y= inputNum[5*k3+2];
        P3_XX= inputNum[5*k3+3];
		P3_YY= inputNum[5*k3+4];
		P3_ZZ= inputNum[5*k3+5];

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		int restraint= 1;
		if ( (w1>=(-1)*restraint)&&(w1<= restraint)&&(w2>= (-1)*restraint)&&(w2<= restraint)/*&&(w1+w2>=0)&&(w1+w2<=1)*/ )  //not-bad triangle
		{
		   //dist[PoleNum]= (x- P1_x)*(x- P1_x)+ (y- P1_y)*(y- P1_y) + (x- P2_x)*(x- P2_x)+ (y- P2_y)*(y- P2_y)+ (x- P3_x)*(x- P3_x)+ (y- P3_y)*(y- P3_y);   //neareast triangle
		   dist[k1][k2][k3]= (x- (P1_x+ P2_x + P3_x)/3)*(x- (P1_x+ P2_x + P3_x)/3)+ (y- (P1_y+ P2_y + P3_y)/3)*(y- (P1_y+ P2_y + P3_y)/3) ;                     //true nearest triangle
		   AT_LEAST_ONE= 1; //at least one good triangle meet the requirements
		   //printf("AT LEAST ONE! (k1, k2, k3)= (%d, %d, %d)\n", k1, k2, k3 );
		}
		else
		{
			dist[k1][k2][k3]= 10000000000;
		}

		if(dist[k1][k2][k3] < min)
			{
				min = dist[k1][k2][k3];
				minPtr1 = k1;
				minPtr2= k2;
				minPtr3= k3;
			}
	 }
	
		if(AT_LEAST_ONE==1) {
		P1_x= inputNum[5*minPtr1+1];   
		P1_y= inputNum[5*minPtr1+2];
		P1_XX= inputNum[5*minPtr1+3];
		P1_YY= inputNum[5*minPtr1+4];
		P1_ZZ= inputNum[5*minPtr1+5];
		P2_x= inputNum[5*minPtr2+1];
		P2_y= inputNum[5*minPtr2+2];
		P2_XX= inputNum[5*minPtr2+3];
		P2_YY= inputNum[5*minPtr2+4];
		P2_ZZ= inputNum[5*minPtr2+5];
		P3_x= inputNum[5*minPtr3+1];
		P3_y= inputNum[5*minPtr3+2];
        P3_XX= inputNum[5*minPtr3+3];
		P3_YY= inputNum[5*minPtr3+4];
		P3_ZZ= inputNum[5*minPtr3+5];	

		w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
		w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );

		P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
		P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
		P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);

		*xx=  0 + (P_XX);
        *yy=  0 + (P_YY);
        *zz=  0 + (P_ZZ);

#ifdef SHOWLINES
		//show the triangle that covers the dot in thick lines
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(125),
		 3, 8, 0 );  
#endif
		}
		else {
			ct= Cal_Coordinates4(x,y,xx,yy,zz);
		}
   }//end of "else"
   //free ( triangleIndexList );
   return ct;
}

int Cal_Coordinates_Delaunay_planeFitting(int x, int y, float *xx, float *yy, float *zz )
{
   int AT_LEAST_ONE= 0;
   int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	 //printf("numTriangleVertices is:%d\n", numTriangleVertices);
	// printf("num of poles in the list is:%d\n", count-1);


   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

#ifdef SHOWLINES
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
#endif

#ifdef COMPENSATE
		P1_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
	    P1_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
		P2_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
	    P2_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
		P3_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
	    P3_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
#endif
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
       // cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			 //printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	         //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	         //printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
#ifdef SHOWLINES					 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
  // printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //when the point is not in a delaunay triangle, use planeFitting method to calculate (XX,YY,ZZ)
   {
	   Cal_Coordinates_planeFitting(x, y, xx, yy, zz );
   }
   return ct;
}

int Cal_Coordinates_Delaunay_curveFitting(int x, int y, float *xx, float *yy, float *zz )
{
   int AT_LEAST_ONE= 0;
   int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	 //printf("numTriangleVertices is:%d\n", numTriangleVertices);
	// printf("num of poles in the list is:%d\n", count-1);


   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

#ifdef SHOWLINES
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
#endif

#ifdef COMPENSATE
		P1_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
	    P1_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
		P2_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
	    P2_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
		P3_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
	    P3_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
#endif
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
       // cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			 //printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	         //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	         //printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
#ifdef SHOWLINES					 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
   //printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //when the point is not in a delaunay triangle, use curveFitting method to calculate (XX,YY,ZZ)
   {
	   Cal_Coordinates_curveFitting(x, y, xx, yy, zz );
   }
   return ct;
}


int Cal_Coordinates_Delaunay_curveFitting3D(int x, int y, float *xx, float *yy, float *zz )
{
   int AT_LEAST_ONE= 0;
   int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 200 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);
	 //printf("numTriangleVertices is:%d\n", numTriangleVertices);
	 //printf("num of poles in the list is:%d\n", count-1);


   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   //printf("triangleIndexList[%d]= %d\n",k1, triangleIndexList[k1]);
	   //printf("triangleIndexList[%d]= %d\n",k1+1,triangleIndexList[k1+1]);
	   //printf("triangleIndexList[%d]= %d\n\n",k1+2,triangleIndexList[k1+2]);
		P1_x= inputNum[5*(triangleIndexList[k1])+1];   
		P1_y= inputNum[5*(triangleIndexList[k1])+2];
		P1_XX= inputNum[5*(triangleIndexList[k1])+3];
		P1_YY= inputNum[5*(triangleIndexList[k1])+4];
		P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
		P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
		P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
		P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
		P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
		P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
		P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
		P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
        P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
		P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
		P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

#ifdef SHOWLINES
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
					 1, 8, 0 );
		cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
		cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
		 1, 8, 0 );
#endif

#ifdef COMPENSATE
		P1_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
	    P1_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1])+2]), int(inputNum[5*(triangleIndexList[k1])+1]));
		P2_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
	    P2_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+1])+2]), int(inputNum[5*(triangleIndexList[k1+1])+1]));
		P3_x=  CV_MAT_ELEM( *remapX, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
	    P3_y=  CV_MAT_ELEM( *remapY, float, int(inputNum[5*(triangleIndexList[k1+2])+2]), int(inputNum[5*(triangleIndexList[k1+2])+1]));
#endif
		//cvCircle(groundImg, cvPoint(inputNum[5*(triangleIndexList[82])+1]/x_scale + offset_x, inputNum[5*(triangleIndexList[82])+2]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
        //cvCircle(groundImg, cvPoint(inputNum[6]/x_scale + offset_x, inputNum[7]/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
       // cvCircle(groundImg, cvPoint(843.366/x_scale + offset_x,  995.726/x_scale + offset_y),4,cvScalarAll(0),4,8,0);
		
			w1= ((x-P1_x)*(P3_y - P1_y)-(y-P1_y)*(P3_x- P1_x))/( (P2_x-P1_x)*(P3_y-P1_y)-(P2_y-P1_y)*(P3_x-P1_x) );
			w2= ((x-P1_x)*(P2_y - P1_y)-(y-P1_y)*(P2_x- P1_x))/( (P3_x-P1_x)*(P2_y-P1_y)-(P3_y-P1_y)*(P2_x-P1_x) );
			//printf("w1 and w2 are: %f %f \n", w1, w2 );

				if ( (w1>=0)&&(w1<=1)&&(w2>=0)&&(w2<=1)&&(w1+w2>=0)&&(w1+w2<=1) )  //this should make it unique
			{
	        //printf("w1 and w2 are: %f %f \n", w1, w2 );
			P_XX= P1_XX + w1*(P2_XX- P1_XX) + w2*(P3_XX- P1_XX);
			P_YY= P1_YY + w1*(P2_YY- P1_YY) + w2*(P3_YY- P1_YY);
			P_ZZ= P1_ZZ + w1*(P2_ZZ- P1_ZZ) + w2*(P3_ZZ- P1_ZZ);
			
			 //printf("(w1,w2) is:(%f, %f)\n", w1, w2);
			 //printf("triangleIndexList[k1]= %d\n",triangleIndexList[k1]);
	         //printf("triangleIndexList[k1+1]= %d\n",triangleIndexList[k1+1]);
	         //printf("triangleIndexList[k1+2]= %d\n",triangleIndexList[k1+2]);

			 if(ct== 0)
				 {
					 sXX = 0 + (P_XX);
			         sYY = 0 + (P_YY);
			         sZZ = 0 + (P_ZZ);
#ifdef SHOWLINES					 
					 //show the triangle that covers the dot in thick lines
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );
					cvLine( groundImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(0),
					 3, 8, 0 );  
#endif
			 }
			ct++;
			}
   }
   //printf("Delaunay count is:%d\n", ct);
   if(ct !=0)
   {
	   *xx= sXX;
       *yy= sYY;
       *zz= sZZ;
   }
   else //when the point is not in a delaunay triangle, use curveFitting method to calculate (XX,YY,ZZ)
   {
	   Cal_Coordinates_curveFitting3D(x, y, xx, yy, zz );
   }
   return ct;
}

int calibration()
{
	board_w = 8; // Board width in squares
	board_h = 8; // Board height 
	n_boards = 10; // Number of boards
	board_n = board_w * board_h;
	board_sz = cvSize( board_w, board_h );
	//CvCapture* capture = cvCreateCameraCapture( 0 );
	//assert( capture );

	//cvNamedWindow( "Calibration" );
	// Allocate Sotrage
	/*CvMat**/ image_points		    = cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	/*CvMat**/ object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	/*CvMat**/ point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	/*CvMat**/ intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	/*CvMat**/ distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );
	/*CvMat**/ rotation_vectors		= cvCreateMat( n_boards, 3, CV_32FC1 );  //new
	/*CvMat**/ translation_vectors	= cvCreateMat( n_boards, 3, CV_32FC1 );   //new

	corners = new CvPoint2D32f[ board_n ];

	//IplImage *image = cvQueryFrame( capture );
	IplImage *image_calibration = 0;
		sprintf( inputName_calibration, "%s%d%s", "..\\data\\", img_num_calibration, ".JPG"); 
        if( (image_calibration = cvLoadImage( inputName_calibration, 1)) == 0 )
		{
			printf("Can't find images in directory Calibration Tool v1.0\data\1\  .");
		    return -1;
		}
	IplImage *gray_image = cvCreateImage( cvGetSize( image_calibration ), 8, 1 );
	//printf("Create gray_image!\n");

    //fitin();     //put the picture in the background image

	// Capture Corner views loop until we've got n_boards
	// succesful captures (all corners on the board are found)

	while( successes < n_boards ){
		// Skp every board_dt frames to allow user to move chessboard
		//if( frame++ % board_dt == 0 ){
			// Find chessboard corners:
			int found = cvFindChessboardCorners( image_calibration, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
            //printf("Doing findcorners\n");
			// Get subpixel accuracy on those corners
			cvCvtColor( image_calibration, gray_image, CV_BGR2GRAY );
			cvFindCornerSubPix( gray_image, corners, corner_count, cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			// Draw it
			cvDrawChessboardCorners( image_calibration, board_sz, corners, corner_count, found );
			//cvShowImage( "Calibration", image_calibration );
			cvShowImage( "background", image_calibration );

			//printf("Corner count is: %d\n", corner_count);

			// If we got a good board, add it to our data
			if( corner_count == board_n ){
				step = successes*board_n;
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
               // printf("successes is: %d\n", successes);
				successes++;      //if 8 successes, finish loop
			}
		//} 

		// Handle pause/unpause and ESC
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 )
			return 0;
		
		img_num_calibration++;
		sprintf( inputName_calibration, "%s%d%s", "..\\data\\", img_num_calibration, ".JPG"); 
        if( (image_calibration = cvLoadImage( inputName_calibration, 1)) == 0 ) 
		{
		printf("That was the last image.");
		return -1;  // Get next image
		}
		cvShowImage( "background", image_calibration );
		
	} // End collection while loop

	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
	CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
	CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );
	
	// Transfer the points into the correct size matrices
	for( int i = 0; i < successes*board_n; ++i ){
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for( int i=0; i < successes; ++i ){
		CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat( &object_points );
	cvReleaseMat( &image_points );
	cvReleaseMat( &point_counts );

	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	// Calibrate the camera
	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image_calibration ), 
	//	intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO );
	intrinsic_matrix, distortion_coeffs, rotation_vectors, translation_vectors, CV_CALIB_FIX_ASPECT_RATIO );
	
     
	//printf("point_counts2 is %d\n", object_points2->data.fl[1 * object_points2->cols + 2]);

	
	// Save the intrinsics and distortions
	/*
	cvSave( "Intrinsics.xml", intrinsic_matrix );
	cvSave( "Distortion.xml", distortion_coeffs );
	cvSave( "Rotation.xml", rotation_vectors );
	cvSave( "Translation.xml", translation_vectors);
	*/
   /*
	printf("intrinsic saved to Intrinsics.xml\n");
	printf("distortion saved to Distortion.xml\n");
	printf("rotation  saved to Rotation.xml\n");
	printf("translation saved to Translation.xml\n");
   */
	// Example of loading these matrices back in
	intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	distortion = (CvMat*)cvLoad( "Distortion.xml" );
	//rotation = (CvMat*)cvLoad( "Rotation.xml" );
	//translation = (CvMat*)cvLoad( "Translation.xml" );
	/*
	printf("...");
	printf("...");
	printf("....\n");
	printf("Loading Camera's intrinsic parameters...\n");
	printf("intrinsic loaded from Intrinsics.xml\n");
	printf("distortion loaded from Distortion.xml\n");
    */

	// Build the undistort map that we will use for all subsequent frames
	/*
	IplImage* mapx = cvCreateImage( cvGetSize( image_calibration ), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize( image_calibration ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );
    */
	// Run the camera to the screen, now showing the raw and undistorted image
	//cvNamedWindow( "Undistort" );
    //cvShowImage( "Calibration", image_calibration ); // Show raw image  
   
	//while( image ){
		//IplImage *t = cvCloneImage( image_calibration );	
		//cvRemap( t, image_calibration, mapx, mapy ); // undistort image
		//cvReleaseImage( &t );
		cvShowImage( "background", image_calibration ); // Show corrected image
        cvWaitKey( 200 );
		// Handle pause/unpause and esc
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
	cvReleaseImage(&gray_image);
	//printf("Release gray_image!\n");
    
	return 0;
}

void fitin()
{

   groundImg= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
   printf("Create groundImg!\n");
   cvCopy(groundImg_org, groundImg);
   
   image= cvCreateImage( cvSize(image_org->width, image_org->height), IPL_DEPTH_8U, 3);
   printf("Create image!\n");
   cvCopy(image_org, image);
   width_ground= groundImg->width;
   height_ground= groundImg->height; 
   scale= float(image->width)/float(image->height) ;
   img_size.height=  (0.7*height_ground);
   img_size.width=   scale*(img_size.height);   //so the width/height of image does not change
   standard= cvCreateImage( img_size, IPL_DEPTH_8U, 3);
   printf("Create standard!\n");
   cvResize(image,  standard,  CV_INTER_LINEAR );
}
void buttonDark( float Array[2][2], IplImage* img )
{
   for (xx= Array[0][1]; xx< Array[1][1]; xx++)
	{
      for (yy= Array[0][0]; yy< Array[1][0]; yy++)
	  {
		     ((uchar *)(img->imageData + yy*step1))[xx*channels1 + 0]= 0;
			 ((uchar *)(img->imageData + yy*step1))[xx*channels1 + 1]= 0;
			 ((uchar *)(img->imageData + yy*step1))[xx*channels1 + 2]= 0;
	  }
	}
}
double drawlines( int pointarray[1024][2], int n1, int n2)
{
	int x01, x02;
	int y01, y02;
	int ct1, ct2;

    x01= int(( pointarray[n1][0]- offset_x)*x_scale);
	y01= int((pointarray[n1][1]- offset_y)*x_scale);
	x02= int(( pointarray[n2][0]- offset_x)*x_scale);
	y02= int((pointarray[n2][1]- offset_y)*x_scale);
	//printf("The first point you selected in original image's coordinate is:(%d, %d)\n", x01,y01);
	//printf("The second point you selected in original image's coordinate is:(%d, %d)\n", x02,y02);

#ifdef COMPENSATE
	int tmp_x01, tmp_x02, tmp_y01, tmp_y02;
	
	tmp_x01=  CV_MAT_ELEM( *remapX, float, y01, x01);
	tmp_y01=  CV_MAT_ELEM( *remapY, float, y01, x01);
	tmp_x02=  CV_MAT_ELEM( *remapX, float, y02, x02);
	tmp_y02=  CV_MAT_ELEM( *remapY, float, y02, x02);
/*
	tmp_x01=  CV_MAT_ELEM( *mapX, float, y01, x01);
	tmp_y01=  CV_MAT_ELEM( *mapY, float, y01, x01);
	tmp_x02=  CV_MAT_ELEM( *mapX, float, y02, x02);
	tmp_y02=  CV_MAT_ELEM( *mapY, float, y02, x02);
*/
	if(((tmp_x01==0)&&(tmp_y01== 0))||((tmp_x02==0)&&(tmp_y02==0)))  ;// printf("Still use planeFittting without compensation\n");
	else {   //use compensation
	x01= tmp_x01;
	y01= tmp_y01;
	x02= tmp_x02;
	y02= tmp_y02;
	}
#endif
	//Cal_Coordinates3( x01, y01, xx1, yy1, zz1);
	//Cal_Coordinates3( x02, y02, xx2, yy2, zz2);
	//ct1= Cal_Coordinates4( x01, y01, xx1, yy1, zz1);
	//ct2= Cal_Coordinates4( x02, y02, xx2, yy2, zz2);
	//ct1= Cal_Coordinates_Delaunay_improved( x01, y01, xx1, yy1, zz1);
	//ct2= Cal_Coordinates_Delaunay_improved( x02, y02, xx2, yy2, zz2);
	//ct1= Cal_Coordinates_Delaunay_improved_2ndVersion( x01, y01, xx1, yy1, zz1);
	//ct2= Cal_Coordinates_Delaunay_improved_2ndVersion( x02, y02, xx2, yy2, zz2);
	//ct1= Cal_Coordinates_Delaunay_improved_3rdVersion( x01, y01, xx1, yy1, zz1);
	//ct2= Cal_Coordinates_Delaunay_improved_3rdVersion( x02, y02, xx2, yy2, zz2);
	//ct1= Cal_Coordinates_Delaunay_improved_lastVersion( x01, y01, xx1, yy1, zz1);
	//ct2= Cal_Coordinates_Delaunay_improved_lastVersion( x02, y02, xx2, yy2, zz2);
#ifdef PLANE_FITTING
	ct1= Cal_Coordinates_Delaunay_planeFitting( x01, y01, xx1, yy1, zz1);
	ct2= Cal_Coordinates_Delaunay_planeFitting( x02, y02, xx2, yy2, zz2);
	if(flag_planeFitting== 1){ //if either of the two dots are not in a delaunay triangle, both of them use planeFitting
		Cal_Coordinates_planeFitting( x01, y01, xx1, yy1, zz1);
		Cal_Coordinates_planeFitting( x02, y02, xx2, yy2, zz2);
	}
	flag_planeFitting= 0;
#endif
#ifdef CURVE_FITTING
	ct1= Cal_Coordinates_Delaunay_curveFitting( x01, y01, xx1, yy1, zz1);
	ct2= Cal_Coordinates_Delaunay_curveFitting( x02, y02, xx2, yy2, zz2);
	if(flag_curveFitting== 1){ //if either of the two dots are not in a delaunay triangle, both of them use curveFitting
		Cal_Coordinates_curveFitting( x01, y01, xx1, yy1, zz1);
		Cal_Coordinates_curveFitting( x02, y02, xx2, yy2, zz2);
	}
	flag_curveFitting= 0;
#endif
#ifdef CURVE_FITTING3D
	ct1= Cal_Coordinates_Delaunay_curveFitting3D( x01, y01, xx1, yy1, zz1);
	ct2= Cal_Coordinates_Delaunay_curveFitting3D( x02, y02, xx2, yy2, zz2);
	if(flag_curveFitting3D== 1){ //if either of the two dots are not in a delaunay triangle, both of them use curveFitting
		Cal_Coordinates_curveFitting3D( x01, y01, xx1, yy1, zz1);
		Cal_Coordinates_curveFitting3D( x02, y02, xx2, yy2, zz2);
	}
	flag_curveFitting3D= 0;
#endif

	double temp=  (*xx1-*xx2)*(*xx1-*xx2)+ (*yy1-*yy2)*(*yy1-*yy2)+(*zz1-*zz2)*(*zz1-*zz2);	            
    double dist= sqrt( temp );
	dist= 0.1*dist;  //make the unit to be m

    CvPoint a= { pointarray[n1][0] ,pointarray[n1][1] };
    CvPoint b= { pointarray[n2][0] ,pointarray[n2][1] };
    cvLine( groundImg, a, b, cvScalarAll(255),                         //Draw the line between two points selected.
             1, 8, 0 );
   
    CvPoint c= { (pointarray[n1][0]+ pointarray[n2][0])/2, (pointarray[n1][1] + pointarray[n2][1])/2};
    //sprintf( text, "%d", int(dist));

  sprintf( text, "%.2f", dist); 
  CvFont font;
  cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
  cvPutText( groundImg, text, c, &font, cvScalar(0, 255, 255, 255));    //write the distance in the middle of the line

  //write the number at the bottom black board too  
  buttonDark(BlackBoard, groundImg);
  CvPoint d = { 55, 656 };
  sprintf( text, "Distance: %.2f", dist);
  cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0)); 

  d.y += 22;
  sprintf( text, "Current Folder: %s   Current Frame: %s", folder, PicName[img_num]);
  cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0));
  
   if((ct1<1)||(ct2<1))  //used to be 50
  {
	  flag_inaccurate= 1;
	  //CvPoint d = { 50, 648 };
      //sprintf( text, "Inaccurate distance!");
      //cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255)); 
  }
   else
   {
	  flag_inaccurate= 1;
	  //CvPoint d = { 50, 648 };
      //sprintf( text, "Inaccurate distance!");  //written in black, make the sentence disappear
      //cvPutText( groundImg, text, d, &font, cvScalar(0, 0, 0, 0)); 
   }
  //exhibit ct1 and ct2 to show if this distance is accurate
  /*
   CvPoint d = { 50, 648 };
   sprintf( text, "ct1= %d; ct= %d", ct1, ct2);
   cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255)); 
   */
 // printf("count_current_dot= %d;     count_previous_dot= %d\n", ct1, ct2);
 

  return dist;
}

void writeNumber (int times, int pointarray[1024][2])
{

  dotx= pointarray[times][0];
  doty= pointarray[times][1];
  sprintf( text, "%d", times+1);
 /*
 cvInitFont( CvFont* font, int font_face,
                         double hscale, double vscale,
                         double shear CV_DEFAULT(0),
                         int thickness CV_DEFAULT(1),
                         int line_type CV_DEFAULT(8));
						 */
 CvFont font;
 cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
 cvPutText( groundImg, text, cvPoint(dotx, doty), &font, cvScalar(228, 26, 3, 255));
 cvShowImage( "background", groundImg );
 
}

void paintNumber ( int times, int pointarray[1024][2] )
{
  dotx= pointarray[times][0];
  doty= pointarray[times][1];

  if (times == 1)
  {
	  //paint 1
      //number = one;
	  //cvReleaseImage(&number);
	  number= cvCreateImage( cvSize(one->width, one->height), IPL_DEPTH_8U, 3);printf("Create number!\n");
      cvCopy(one, number);
  }
  else if (times == 2)
  {
	  //paint 2
	  //number = two;
	  //cvReleaseImage(&number);
	  number= cvCreateImage( cvSize(two->width, two->height), IPL_DEPTH_8U, 3);printf("Create number!\n");
      cvCopy(two, number);
  }
  else if (times == 0)
  {
	  //paint 3
	  //number = three;
	  //cvReleaseImage(&number);
	  number= cvCreateImage( cvSize(three->width, three->height), IPL_DEPTH_8U, 3);printf("Create number!\n");
      cvCopy(three, number);
  }
  else //if (times!=4)
  {
	  //printf("times = 0");
  }
   

   step1= groundImg->widthStep;
   step3= number->widthStep;
   data1    = (uchar *)groundImg->imageData;
   channels1 = groundImg ->nChannels;
   data3    = (uchar *)number->imageData;
   channels3 = number->nChannels;
   if (times==0)
   {
   offsetNumber_x= pointarray[2][0];
   offsetNumber_y= pointarray[2][1];
   //printf("offsetNumber_x= %d\n",pointarray[2][0]);
   //printf("offsetNumber_y= %d\n",pointarray[2][1]);
   }
   else
   {
   offsetNumber_x= pointarray[times-1][0];
   offsetNumber_y= pointarray[times-1][1];
   //printf("offsetNumber_x= %d\n",pointarray[times-1][0]);
   //printf("offsetNumber_y= %d\n",pointarray[times-1][1]);
   }
  
   for (y= 0; y< number->height; y++)
   {
	   for (x= 0; x< number->width; x++)
	   {
            //data1[ (offset_x+x) + (offset_y+y)*step1 + channels1*z ]=   data2[x+ y*step2+ z*channels2]; 
			((uchar *)(groundImg->imageData + (offsetNumber_y+ y)*step1 ))[(offsetNumber_x+ x)*channels1 + 0]= ((uchar *)(number->imageData + y*step3))[x*channels3 + 0];
			((uchar *)(groundImg->imageData + (offsetNumber_y+ y)*step1 ))[(offsetNumber_x+ x)*channels1 + 1]= ((uchar *)(number->imageData + y*step3))[x*channels3 + 1];
			((uchar *)(groundImg->imageData + (offsetNumber_y+ y)*step1 ))[(offsetNumber_x+ x)*channels1 + 2]= ((uchar *)(number->imageData + y*step3))[x*channels3 + 2];
	   }
   }
  cvShowImage( "background", groundImg );
  cvWaitKey(0);
}

void buttonLight ( float Array[2][2] )
{
   for (xx= Array[0][1]; xx< Array[1][1]; xx++)
	{
      for (yy= Array[0][0]; yy< Array[1][0]; yy++)
	  {
		     ((uchar *)(groundImg->imageData + yy*step1))[xx*channels1 + 0]= 255;
	  }
	}

}
void buttonLight_unlimited ( float Array[2][2], IplImage* img )
{
   for (xx= Array[0][1]; xx< Array[1][1]; xx++)
	{
      for (yy= Array[0][0]; yy< Array[1][0]; yy++)
	  {
		     ((uchar *)(img->imageData + yy*step1))[xx*channels1 + 0]= 255;
	  }
	}
}

int DispImage(int img_num)
{
   //=============================================
   //release image before creating them again
	cvReleaseImage( &t );         // printf("Release t\n");
	cvReleaseImage(&image_org);    //printf("Release image_org\n");
	cvReleaseImage(&groundImg);    //printf("Release groundImg!\n");
	cvReleaseImage(&image);        //printf("Release image!\n");
	cvReleaseImage(&standard);     //printf("Release standard!\n");
	cvReleaseImage(&groundImg_copy);  //printf("Release groundImg_copy!\n");
   // cvReleaseImage(&foreImg);     printf("Release foreImg!\n");
   //============================================


	sprintf( inputName, "%s\\%s", folder, PicName[img_num]);    
	//sprintf( inputName, "%s%d%s", folder, img_num, ".JPG"); 
   //free(image_org);
   image_org = cvLoadImage( inputName, 1);
   //printf("Create  image_org!\n");

   //display undistorted image instead
#ifdef SHOW_UNDISTORTED   
    t = cvCloneImage(image_org );	
	cvRemap( t, image_org, mapx, mapy ); // undistort image_org undistorted image
	cvReleaseImage( &t );
	//cvNamedWindow("undistorted image");
	//cvShowImage("undistorted image", image_org);
#endif

#ifdef SHOW_UNDISTORTED2
	t = cvCloneImage(image_org );	
	cvRemap(image_org, t, mapx, mapy ); //make t undistorted image
    cvNamedWindow("undistorted image");
	cvShowImage("undistorted image", t);
#endif

   //Make them gray   //cancel this
   groundImg= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
   //printf("Create groundImg!\n");
   cvCopy(groundImg_org, groundImg);//in, out
   
   image= cvCreateImage( cvSize(image_org->width, image_org->height), IPL_DEPTH_8U, 3);
   //printf("Create image!\n");
   cvCopy(image_org, image);

   //Resize image based on the window/backgroundimage
   width_ground= groundImg->width;
   height_ground= groundImg->height; 
   scale= float(image->width)/float(image->height) ;
   img_size.height=  (0.7*height_ground);
   img_size.width=   scale*(img_size.height);   //so the width/height of image does not change
   standard= cvCreateImage( img_size, IPL_DEPTH_8U, 3);
   //printf("Create standard!\n");
   cvResize(image,  standard,  CV_INTER_LINEAR );
   /////////////////////////////////////////
    x_scale= (image->height)/(0.7*height_ground);
	////////////////////////////////////////////
   foreImg= standard;  //printf("create foreImg\n");
   //Set image to the right position on top of the ground image
   // write the fore ground image onto the ground image
   step1= groundImg->widthStep;
   step2= foreImg->widthStep;
   data1    = (uchar *)groundImg->imageData;
   data2    = (uchar *)foreImg->imageData;
   channels1 = groundImg ->nChannels;
   channels2 = foreImg->nChannels;
  
   for (y= 0; y< img_size.height; y++)
   {
	   for (x= 0; x< img_size.width; x++)
	   {
            //data1[ (offset_x+x) + (offset_y+y)*step1 + channels1*z ]=   data2[x+ y*step2+ z*channels2]; 
			((uchar *)(groundImg->imageData + (offset_y+y)*step1 ))[(offset_x+x)*channels1 + 0]= ((uchar *)(foreImg->imageData + y*step2))[x*channels2 + 0];
			((uchar *)(groundImg->imageData + (offset_y+y)*step1 ))[(offset_x+x)*channels1 + 1]= ((uchar *)(foreImg->imageData + y*step2))[x*channels2 + 1];
			((uchar *)(groundImg->imageData + (offset_y+y)*step1 ))[(offset_x+x)*channels1 + 2]= ((uchar *)(foreImg->imageData + y*step2))[x*channels2 + 2];
	   }
   }
   //get a copy for re-paint
   groundImg_copy= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
   //printf("Create groundImg_copy!\n");
   cvCopy(groundImg, groundImg_copy);
   /**********************************************************/
   cvShowImage( "background", groundImg );


   
   //update Image region size
   Image[1][1]= img_size.width + offset_x; 
   Image[1][0]= img_size.height + offset_y; 

   return 1;
}
void InitStruct()
{
	gdata			= (data_Struct*)malloc(sizeof(data_Struct));
	//gdata->cursor_x_position	= (int*)malloc(1024 * sizeof(int));
	//gdata->cursor_y_position	= (int*)malloc(1024 * sizeof(int));
	//gdata->distance_from_camera	= (double*)malloc(1024 * sizeof(double));
    //gdata->distance_from_previous_point	= (double*)malloc(1024 * sizeof(double));
    gdata->sum_distance_sequence = 0;
}
/*
void InitFolderPicture()
{
	system("dir ..\\data\\ /B /A:D >..\\data\\folderList.txt");      //generate folderList, having all folder's names
    FILE* fp_folderList = fopen("..\\data\\folderList.txt", "r" );   //read this txt
	fscanf(fp_folderList, "%s", &folder_name);                          //derive first folder's name

	sprintf( folder, "..\\data\\%s", folder_name);                      //derivbe first folder's directory
	sprintf( cmd_arg, "dir %s\\ /B /O:D >%s\\PictureList.txt", folder, folder);   //modify command argument
    system(cmd_arg);                                                      //generate Picture list in that folder

    sprintf( PictureList_dir, "..\\data\\%s\\PictureList.txt", folder);   //derive 
	FILE* fp_PictureList = fopen( PictureList_dir, "r" );
	fscanf( fp_PictureList, "%s", &PicName);  
	
	printf("Fist folder is: %s\n", folder);
    printf("Fist Pic is: %s\n", PicName);

}
*/
void InitFolderPicture2()
{
	/*
    int i, flag_folder, flag_image;
    int Num_folder= 0;
    int Num_image= 0;	
    char **folder_name;
	char **PicName;
	*/
	int i;

	folder_name = (char **)malloc(256 * sizeof(char *));
	for(n=0; n<256; n++)
		folder_name[n] = (char *)malloc(256 * sizeof(char));
		
	PicName = (char **)malloc(256 * sizeof(char *));
	for(n=0;n<256; n++)
		PicName[n] = (char *)malloc(256 * sizeof(char));

	temp = (char **)malloc(256 * sizeof(char *));
	for(n=0;n<256; n++)
		temp[n] = (char *)malloc(256 * sizeof(char));
	
	system("dir ..\\data\\ /B /A:D >..\\data\\folderList.txt");      //generate folderList, having all folder's names
    FILE* fp_folderList = fopen("..\\data\\folderList.txt", "r" );   //read this txt
	
	for(i= 0; i<256; i++)
	{
	  
	  flag_folder= fscanf(fp_folderList, "%s", folder_name[i]);                          //derive first folder's name
	  if (flag_folder==1)
	  {
	     Num_folder++;
		 printf("folder name is: %s\n", folder_name[i]);
	  }
	  
		/*
	  flag_folder= fscanf(fp_folderList, "%s", temp[i]);                          //derive first folder's name
	  char* space_or_nl;
      fgets(space_or_nl, sizeof(char), fp_folderList-1);
	  if (flag_folder==1)
	  {
		 //sprintf( folder_name[Num_folder], "%s", temp[i]);
	     if(space_or_nl== "\n") 
		 {
			 Num_folder++;
		 }
		 else
		 {
             sprintf( folder_name[Num_folder], "%s", temp[i]);
		 }
	  }
	  */
	  /*
	  if ((fgets(folder_name[i], sizeof(folder_name[i]), fp_folderList))!=NULL)  //derive first folder's name
	  {
		  Num_folder++;
		  printf("folder name is: %s", folder_name[i]);
	  }
	  */
	  
	}
	fclose(fp_folderList );
	//fscanf(fp_folderList, "%s", &folder_name);                          //derive first folder's name
	sprintf( folder, "..\\data\\%s", folder_name[0]);                      //derivbe first folder's directory
	
	sprintf( cmd_arg, "dir %s\\ /B /O:N >%s\\PictureList.txt", folder, folder);   //modify command argument
	//sprintf( cmd_arg, "dir %s\\ /B /O:D >%s\\PictureList.txt", folder, folder);   //modify command argument
    system(cmd_arg);                                                      //generate Picture list in that folder

    sprintf( PictureList_dir, "..\\data\\%s\\PictureList.txt", folder);   //derive 
	FILE* fp_PictureList = fopen( PictureList_dir, "r" );
	
	for(i= 0; i<256; i++)
	{
	  flag_image= fscanf(fp_PictureList, "%s", PicName[i]);                          //derive first image's name
	  if (flag_image==1)
	  {
	     Num_image++;
	  }
	  //printf("flag_image is: %d\n",flag_image);
	}
    fclose(fp_PictureList );
	 Num_image--;  //exclude the txt file
	//fscanf( fp_PictureList, "%s", &PicName[1]);  
	
	printf("\n First folder is: %s\n", folder);
    printf("First Pic is: %s\n", PicName[0]);
	printf("Num_folder is: %d\n", Num_folder);
	printf("Num_image is: %d\n", Num_image);
	

}
//The mouse handler for the main menu and measuring
void onMouse(int event,int x,int y,int flags,void* param) 
{ 
	switch(event)
	{
      case CV_EVENT_LBUTTONDOWN:
        if(flags & CV_EVENT_FLAG_CTRLKEY) 
          printf("Left button down with CTRL pressed\n");
		  
		  //Allow Drag and Drop again: ning	
	      flag_DG= 1;
        break; 
		
 
      case CV_EVENT_LBUTTONUP:

		if( (x> NextFolder[0][1])&& (x< NextFolder[1][1]) &&(y> NextFolder[0][0]) &&(y< NextFolder[1][0]) )  
		{
			if (flag_NF)
			{
			 flag_NI= 1;
             flag_NF= 1;
             flag_SV= 0;
             
#ifdef SAVE_TXT
            fprintf( fp_DATA, "\n");
#endif
#ifdef SAVE_XLS
			which_row++;
#endif 
	        buttonLight ( NextFolder );
	        cvShowImage( "background", groundImg );
	        cvWaitKey(3);
             
            /*
			folder_num++;
			//sprintf( folder, "%s%d%s", "..\\data\\", folder_num, "\\"); 
			//sprintf( inputName, "%s%d%s", folder, 1, ".JPG");
            sprintf( folder, "%s%d%s", "..\\data\\", folder_name, "\\"); 
			sprintf( inputName, "%s%d%s", folder, PicName[1], ".JPG");
            */
			/*
                folder_num++; 
				sprintf( folder, "..\\data\\%s", folder_name[folder_num]);                      //derivbe first folder's directory
				sprintf( cmd_arg, "dir %s\\ /B /O:D >%s\\PictureList.txt", folder, folder);   //modify command argument
				system(cmd_arg);                                                      //generate Picture list in that folder

				sprintf( PictureList_dir, "..\\data\\%s\\PictureList.txt", folder);   //derive 
				FILE* fp_PictureList = fopen( PictureList_dir, "r" );
				
				for(int i= 0; i<256; i++)
				{
				  flag_image= fscanf(fp_PictureList, "%s", PicName[i]);                          //derive first folder's name
				  if (flag_image==1)
				  {
					 Num_image++;
				  }
				}
				
				fclose(fp_PictureList );
				Num_image--;  //exclude the txt file. The total number of images in this folder
				sprintf( inputName, "%s\\%s", folder, PicName[0]);
				printf("Current folder is: %s\n", folder);
				printf("Fist Pic is: %s\n", PicName[0]);
             */
			  if (folder_num < Num_folder- 1)
			  {
				  printf("folder num is: %d\n", folder_num);
                  ChangeFolderFunc(1);  //1 means next folder
			  }
			
               //if  ( (image_org = cvLoadImage( inputName, 1))==0)
			   if (folder_num >= Num_folder- 1)
	          {
				  
	         flag_NI= 1;
             flag_NF= 0;
             flag_SV= 0;
             //flag_EXIT= 1;

             //sprintf( folder, "%s%d%s", "..\\data\\", folder_num-1, "\\"); 
			 //sprintf( inputName, "%s%d%s", folder, img_num, ".JPG");
			  //ChangeFolderFunc(0);  //0 means previous folder

            
			        if  ( (image_org = cvLoadImage( inputName, 1))==0)    {
						cvReleaseImage(&image_org);
						
						ChangeFolderFunc(0);  //0 means previous folder. If the next folder contains none available image, go back to the precious folder
						img_num= temp_img_num; //recover the img_num

						cvReleaseImage(&image_org);    //printf("Release image_org in if\n");
						DispImage(img_num);
						CvPoint d = { 50, 656 };
						sprintf( text, "No new image in the next folder. ");
						CvFont font;
						cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
						cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

						d.y += 22;
						sprintf( text, "Please delete empty folders in ../CalibrationTool/data/.");
						cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

						cvShowImage("background",groundImg);
						cvSetMouseCallback("background",onMouse,&mouseParam);

						//flag_NI= 0;
						flag_SV= 0;
						//flag_EXIT= 1; 
					}
					else{
					 cvReleaseImage(&image_org);
					 DispImage(img_num);
					 CvPoint d = { 50, 648 };
					 sprintf( text, "This is the last folder and sequence ");
					 CvFont font;    
					 cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f, 0,1,CV_AA);
					 cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));
					
					 cvShowImage("background",groundImg);
					 cvSetMouseCallback("background",onMouse,&mouseParam);
					 //cvWaitKey(0);
					}
	         
	          }
			   else
			   {
				temp_img_num= img_num;
                img_num= 0;
			    times= 0;

				     //----------------added 2/10/2014 ning
				    sprintf( inputName, "%s\\%s", folder, PicName[img_num]); //see the current img_num, if there is an available image
					//printf("create image_org!\n");
					if  ( (image_org = cvLoadImage( inputName, 1))==0)    
					{
						ChangeFolderFunc(0);  //0 means previous folder. If the next folder contains none available image, go back to the precious folder
						img_num= temp_img_num; //recover the img_num

						cvReleaseImage(&image_org);    //printf("Release image_org in if\n");
						DispImage(img_num);
						CvPoint d = { 50, 656 };
						sprintf( text, "No new image in the next folder. ");
						CvFont font;
						cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
						cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

						d.y += 22;
						sprintf( text, "Please delete empty folders in ../CalibrationTool/data/.");
						cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

						cvShowImage("background",groundImg);
						cvSetMouseCallback("background",onMouse,&mouseParam);

						//flag_NI= 0;
						flag_SV= 0;
						//flag_EXIT= 1; 
					}
					else
					{
					   cvReleaseImage(&image_org);   // printf("Release image_org in else\n");
					   DispImage(img_num);

					   //Blackboard display
					  buttonDark(BlackBoard, groundImg);
					  CvPoint d = { 55, 656 };
					  CvFont font;
                      cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
					  //sprintf( text, "Distance: %.2f", dist);
					  //cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0)); 
					  d.y += 22;
					  sprintf( text, "Current Folder: %s   Current Frame: %s", folder, PicName[img_num]);
					  cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0));
					  cvShowImage( "background", groundImg );

					}

				//----------------
	            //DispImage(img_num);  //previously like this. without above code between //-----  and  ----//
			   }
			}
			else
			{}

		}
/*
if( (x> Open[0][1])&& (x< Open[1][1]) &&(y> Open[0][0]) &&(y< Open[1][0]) )
{
   //doOpen();
	printf("Open\n");
	buttonLight ( Open );
	cvShowImage( "background", groundImg );
	cvWaitKey(3);
	//show the distances
	distance (  pointarray, img_num );
}*/
else if( (x> Next[0][1])&& (x< Next[1][1]) &&(y> Next[0][0]) &&(y< Next[1][0]) )
{
	
   //doNext();
	//printf("Next\n");
	if(flag_NI)
	{
    flag_NI= 1;
    flag_SV= 0;
    //flag_EXIT= 1;
	flag_PI= 1;


	buttonLight ( Next );
	cvShowImage( "background", groundImg );
	cvWaitKey(3);
	



	img_num++;
	//sprintf( inputName, "%s%d%s", folder, img_num+1, ".JPG"); 
	sprintf( inputName, "%s\\%s", folder, PicName[img_num+1]);



    //printf("create image_org!\n");
    if  ( (image_org = cvLoadImage( inputName, 1))==0)    
	{
		cvReleaseImage(&image_org);    //printf("Release image_org in if\n");
        DispImage(img_num);
	    CvPoint d = { 50, 656 };
        sprintf( text, "No new image in this folder. ");
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
        cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

		d.y += 22;
		sprintf( text, "Please press Next Folder for a new sequence.");
		cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

		cvShowImage("background",groundImg);
	    cvSetMouseCallback("background",onMouse,&mouseParam);
	
	flag_NI= 0;
    flag_SV= 0;
    //flag_EXIT= 1;
	 
	}
	else
	{
	cvReleaseImage(&image_org);    //printf("Release image_org in else\n");
	DispImage(img_num);
	 
	  //Blackboard display
	  buttonDark(BlackBoard, groundImg);
	  CvPoint d = { 55, 656 };
	  CvFont font;
	  cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
	  //sprintf( text, "Distance: %.2f", dist);
	  //cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0)); 
	  d.y += 22;
	  sprintf( text, "Current Folder: %s   Current Frame: %s", folder, PicName[img_num]);
	  cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 0));
	  cvShowImage( "background", groundImg );
	}
	
	//printf("times is: %d\n", times);
	int t;
    for ( t= 1; t< times; t++)
	{
	drawlines(pointarray, t, t-1);
	}
	for ( t= 0; t< times; t++)
	{
	writeNumber(t, pointarray);
	}
    
	cvSetMouseCallback("background",onMouse,&mouseParam);
    cvWaitKey(0);
	}
	else
	{}
}
	 else if( (x> Exit[0][1])&& (x< Exit[1][1]) &&(y> Exit[0][0]) &&(y< Exit[1][0]) )
{
   //doExit();
	if(flag_EXIT)
	{
#ifdef SAVE_TXT
		fclose(fp_DATA);
#endif
#ifdef SAVE_XLS
		//if(xlBookSave(book,"../data.xls")) printf("data.xls saved!\n");
		if (xlBookSave(book, "../data.xls"))  printf("xml Saved!\n");
		xlBookRelease(book);
#endif

		cvReleaseImage(&mapx); 
	    cvReleaseImage(&mapy);
		cvDestroyWindow( "background" );
	}
	else
	{
		cvDestroyWindow( "background" );
	}
}	


else if( (x> Prev[0][1])&& (x< Prev[1][1]) &&(y> Prev[0][0]) &&(y< Prev[1][0]) )
{

	if(flag_PI)
	{
    flag_NI= 1;
    flag_SV= 0;
    //flag_EXIT= 1;

	buttonLight ( Prev );
	cvShowImage( "background", groundImg );
	cvWaitKey(3);
	
	img_num--;
	DispImage(img_num);

	//sprintf( inputName, "%s%d%s", folder, img_num- 1, ".JPG"); 
	//sprintf( inputName, "%s\\%s", folder, PicName[img_num- 1]);

    //if  ( (image_org = cvLoadImage( inputName, 1))==0)    
	  if  ( (img_num <1) )   
	{
        DispImage(img_num);
	    CvPoint d = { 50, 656 };
        sprintf( text, "No previous image in this folder. ");
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
        cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

		d.y += 22;
		sprintf( text, "Please press Next Folder or Next Image");
		cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

		cvShowImage("background",groundImg);
	    cvSetMouseCallback("background",onMouse,&mouseParam);
	
	flag_PI= 0;
    flag_SV= 0;
    
	 
	}
	else
	{
	DispImage(img_num);
	}
	
	//printf("times is: %d\n", times);
	int t;
    for ( t= 1; t< times; t++)
	{
	drawlines(pointarray, t, t-1);
	}
	for ( t= 0; t< times; t++)
	{
	writeNumber(t, pointarray);
	}
    
	cvSetMouseCallback("background", onMouse, &mouseParam);
    cvWaitKey(0);
	 }
	else
	{}
}

else if( (x> NewPath[0][1])&& (x< NewPath[1][1]) &&(y> NewPath[0][0]) &&(y< NewPath[1][0]) )
{
	if(flag_NP)
	{
    //flag_NI= 1;
    flag_SV= 0;
    
    /*
	cvCopy(groundImg_copy, groundImg);
	buttonLight ( NewPath );
	cvShowImage( "background", groundImg );
	cvWaitKey(5);

	cvCopy(groundImg_copy, groundImg);
	cvShowImage( "background", groundImg );
	*/

	cvCopy( groundImg, groundImg_copy);
	buttonLight ( NewPath );
	cvShowImage( "background", groundImg );
	cvWaitKey(15);

	cvCopy(groundImg_copy, groundImg);
	//cvShowImage( "background", groundImg );

    times= 0;
#ifdef SAVE_TXT
    fprintf( fp_DATA, "\n"); 
#endif
#ifdef SAVE_XLS
	which_row++;
#endif
	}
	else{}

}

else if( (x> Save[0][1])&& (x< Save[1][1]) &&(y> Save[0][0]) &&(y< Save[1][0]) )
{
	if(flag_SV)
	{
    //flag_NI= 1;
    flag_SV= 0;
    flag_EXIT= 1;

	//ning :  1.5.4 
	FILE* fplist = fopen(filelist, "w" );
	for (int i = 0; i<= count-1; i++)
	{
		fprintf(fplist, "%f %f %f %f %f\n", inputNum[5*i +1], inputNum[5*i +2], inputNum[5*i +3], inputNum[5*i +4], inputNum[5*i +5]);  
	}
    fclose(fplist);

	cvCopy(groundImg, groundImg_copy);
	buttonLight ( Save );
	cvShowImage( "background", groundImg );
	cvWaitKey(3);

	cvCopy(groundImg_copy, groundImg);
	cvShowImage( "background", groundImg );

	}
	else{}
}

else if ( (x> Drag[0][1])&& (x< Drag[1][1]) &&(y> Drag[0][0]) &&(y< Drag[1][0]) )
{
	if(flag_DG)
	{
		cvReleaseImage(&selectedImg);
		selectedImg = cvCloneImage(groundImg_copy);
		drawlines_drag(0, selectedImg);
		buttonLight_unlimited ( Drag, selectedImg );

		//Display
		CvPoint d = { 50, 656 };
	    sprintf( text, "Drag red dots to adjust the ground surface.");
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
        cvPutText( selectedImg, text, d, &font, cvScalar(255, 255, 255, 255));

        d.y += 22;
	    sprintf( text, "1. Clicking Save will update the positions in file 2. Click Drag again to return to measurement.");
	    cvPutText( selectedImg, text, d, &font, cvScalar(255, 255, 255, 255));


		//allow Save
		flag_SV= 1;

		cvShowImage("background", selectedImg);
		cvSetMouseCallback("background",onMouseDrag,&mouseParam);
		cvWaitKey(0);

	
	}
	else{}
}

else if( (x> Image[0][1])&& (x< Image[1][1]) &&(y> Image[0][0]) &&(y< Image[1][0]) )  
 {        
	 if(flag_else)
	 {
			   //*****************************************************//
               //get a red dot where clicked by mouse
               cvCircle(  groundImg, cvPoint(x,y), 3, CvColorRed, 1, 8, 0 );
               cvShowImage( "background", groundImg );
               pointarray[times][0]=x;
               pointarray[times][1]=y;
               //printf("The %d dot chosen is:(%d, %d)\n ",times+1,x,y); 

#ifdef SHOW_UNDISTORTED2
			   int xWholeImg, yWholeImg;
			    xWholeImg= int(( x- offset_x)*x_scale);
	            yWholeImg= int(( y- offset_y)*x_scale);
				printf("xwhole, ywhole= (%d, %d)\n", CV_MAT_ELEM( *remapX, float, yWholeImg, xWholeImg), CV_MAT_ELEM( *remapY, float, yWholeImg, xWholeImg));
			   cvCircle( t, cvPoint(CV_MAT_ELEM( *remapX, float, yWholeImg, xWholeImg), CV_MAT_ELEM( *remapY, float, yWholeImg, xWholeImg)), 3, CvColorRed, 1, 8, 0 );
			   cvShowImage("undistorted image", t);
#endif
			   //printf("(xx2, yy2) :(%f, %f)\n ",*xx2,*yy2);
               if (times> 0)
               {
				   
				  gdata->distance_from_previous_point[times]= drawlines(pointarray, times-1, times); printf("\n Distance: %.2f \n", gdata->distance_from_previous_point[times]); 
				  gdata->cursor_x_position[times]= x;
			      gdata->cursor_y_position[times]= y;
			      gdata->distance_from_camera[times]=   0.1* sqrt((*xx2)*(*xx2)+ (*yy2)*(*yy2)+ (*zz2)*(*zz2)) ;
			      gdata->sum_distance_sequence += gdata->distance_from_previous_point[times];   
				  gdata->angle_animal_camera= atan2((*xx2),(-1)*(*yy2)) * 180/3.14;
				  gdata->angle_animal_animal= acos((-1)* (      (gdata->distance_from_previous_point[times])*(gdata->distance_from_previous_point[times])
					                                      -(gdata->distance_from_camera[times])*(gdata->distance_from_camera[times])
													      -(gdata->distance_from_camera[times-1])*(gdata->distance_from_camera[times-1])   )/ (2*(gdata->distance_from_camera[times])*(gdata->distance_from_camera[times-1]))) *180/3.14;
										  

#ifdef SAVE_TXT                    
				  fprintf( fp_DATA, "          %s                   %s                     %d                 %d                   %f                   %f                   %f               %f               %f\n", 
					                folder_name[folder_num],
									PicName[img_num],
					                gdata->cursor_x_position[times],
									gdata->cursor_y_position[times],
									gdata->distance_from_camera[times],
									gdata->distance_from_previous_point[times],
									gdata->sum_distance_sequence,
									gdata->angle_animal_camera,
									gdata->angle_animal_animal);
#endif
#ifdef SAVE_XLS
				  if(sheet){
					  //printf("HIHIHIHI\n");
					  which_row++;
					  xlSheetWriteStr(sheet, which_row, 0, folder_name[folder_num], 0);
					  xlSheetWriteStr(sheet, which_row, 1, PicName[img_num], 0);
					  xlSheetWriteNum(sheet, which_row, 2,  gdata->cursor_x_position[times], 0);
					  xlSheetWriteNum(sheet, which_row, 3,  gdata->cursor_y_position[times], 0);
					  xlSheetWriteNum(sheet, which_row, 4,  gdata->distance_from_camera[times], 0);
					  xlSheetWriteNum(sheet, which_row, 5,  gdata->distance_from_previous_point[times], 0);
					  xlSheetWriteNum(sheet, which_row, 6,  gdata->sum_distance_sequence, 0);
					  xlSheetWriteNum(sheet, which_row, 7,  gdata->angle_animal_camera, 0);
					  xlSheetWriteNum(sheet, which_row, 8,  gdata->angle_animal_animal, 0);
					  xlSheetWriteStr(sheet, 0, 8, "angle_animal_animal", 0); //in case this software says buyme here.
					  if (xlBookSave(book, "../data.xls"))  ;//printf("xml Saved!\n");
				  }
#endif
               }
               times++;
               writeNumber(times-1, pointarray);
               break;
	    }
	else{}
 }//end else

}//end switch
}//end onMouse



//Another mouse handler for the drag and drop of the dots, and its sub-functions
int drawlines_drag( int point, IplImage* selectedImg )
{
   //int *best_index;
	int best_index[10];

   float	P1_x, P1_y, P1_XX, P1_YY, P1_ZZ,
			P2_x, P2_y, P2_XX, P2_YY, P2_ZZ,
			P3_x, P3_y, P3_XX, P3_YY, P3_ZZ;
   int P_x= x,
	   P_y= y;
   double		P_XX,
	   P_YY,
	   P_ZZ;  
   double w1,w2, cosTheta;
   int ct;
   float sXX, sYY, sZZ;
   int k0, k1;//, k2, k3;

   double dx21, dy21, dx31, dy31;

   //allocate PolesXY for function "BuildTriangleIndexList"
   //static float PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   static int PolesXY[ 64 * 2 ] ;   // to hold a set of random points
   for(k0=0; k0< count-1; k0++)
   {
	   PolesXY[2*k0+0]=  int(inputNum[5*k0+1]);
	   PolesXY[2*k0+1]=  int(inputNum[5*k0+2]);
   }

    // create an list of the triangles that connect those test points
	// the input points must be integers, and output list is WORD
	//WORD *triangleIndexList;   // this does not need initialization
	static int numTriangleVertices;                                     // this does not need initialization
	triangleIndexList =  BuildTriangleIndexList(
							  (void*)PolesXY,          // An array of random integers between 0 and MAX_RAND= 32000
							  0,//(float)3200,//(float)32000,
							  count-1,       // the number of points/poles in the list
							  2,        // 2, because the list is X and Y points, no Z values
							  1,
							  &numTriangleVertices);

   ct = 0;
   sXX= sYY = sZZ = 0;

   for(k1=0; k1< numTriangleVertices; k1+=3)
   {
	   if ((( triangleIndexList[k1] != point )&& ( triangleIndexList[k1+1] != point )&& ( triangleIndexList[k1+2] != point ))|| (point==0))
	   {
			P1_x= inputNum[5*(triangleIndexList[k1])+1];   
			P1_y= inputNum[5*(triangleIndexList[k1])+2];
			P1_XX= inputNum[5*(triangleIndexList[k1])+3];
			P1_YY= inputNum[5*(triangleIndexList[k1])+4];
			P1_ZZ= inputNum[5*(triangleIndexList[k1])+5];
			P2_x= inputNum[5*(triangleIndexList[k1+1])+1];
			P2_y= inputNum[5*(triangleIndexList[k1+1])+2];
			P2_XX= inputNum[5*(triangleIndexList[k1+1])+3];
			P2_YY= inputNum[5*(triangleIndexList[k1+1])+4];
			P2_ZZ= inputNum[5*(triangleIndexList[k1+1])+5];
			P3_x= inputNum[5*(triangleIndexList[k1+2])+1];
			P3_y= inputNum[5*(triangleIndexList[k1+2])+2];
			P3_XX= inputNum[5*(triangleIndexList[k1+2])+3];
			P3_YY= inputNum[5*(triangleIndexList[k1+2])+4];
			P3_ZZ= inputNum[5*(triangleIndexList[k1+2])+5];	

			cvCircle(selectedImg, 
				     cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y),
					 4,
					 cvScalar(0, 0,  255,0), 2, 8, 0);
			cvCircle(selectedImg, 
				     cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y),
					 4,
					 cvScalar(0, 0,  255,0), 2, 8, 0);
			cvCircle(selectedImg, 
				     cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y),
					 4,
					 cvScalar(0, 0,  255,0), 2, 8, 0);


			cvLine( selectedImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
						 1, 8, 0 );
			cvLine( selectedImg, cvPoint(P1_x/x_scale + offset_x ,P1_y/x_scale + offset_y), cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvScalarAll(255),
			 1, 8, 0 );
			cvLine( selectedImg, cvPoint(P3_x/x_scale + offset_x ,P3_y/x_scale + offset_y), cvPoint(P2_x/x_scale + offset_x ,P2_y/x_scale + offset_y), cvScalarAll(255),
			 1, 8, 0 );
	   }

   }
 
   return ct;
}

IplImage* findImg(int x, int y)
{
	int xx, yy;   //xx is the corresponding x of groundImg, in frontImg

	cvReleaseImage(&selectedImg);
	selectedImg = cvCloneImage(groundImg_copy);

	buttonLight_unlimited ( Drag, selectedImg );

	xx= int(( x- offset_x)*x_scale);
	yy= int(( y- offset_y)*x_scale);

	point = -1;
	for (int i= 0; i< count; i++) {
		if  (   ( (xx<inputNum[5*i+1]+10)&&(xx>inputNum[5*i+1]-10) ) && ( (yy<inputNum[5*i+2]+10)&&(yy>inputNum[5*i+2]-10) )  )
		{
			point= i; //point selected. It is the i_th point. 
		}
	}

	//draw latest-updated lines
    drawlines_drag(point, selectedImg);

	//show selectedImg
	//cvShowImage("Drag & Drop", selectedImg);
	cvShowImage("background", selectedImg);

	return selectedImg;
}
void releaseImg(IplImage* selectedImg, int x, int y)
{
	//save and update coordinates
	int xx, yy;   //xx is the corresponding x of groundImg, in frontImg
	xx= int(( x- offset_x)*x_scale);
	yy= int(( y- offset_y)*x_scale);

    inputNum[5*point+1]= xx;
	inputNum[5*point+2]= yy;

	//set point to -1. Point unselected.
	point= -1;

	//update
	drawlines_drag(0, selectedImg);
	//cvShowImage("Drag & Drop", selectedImg);
	cvShowImage("background", selectedImg);
	
	//release selectedImg
    cvReleaseImage(&selectedImg);

    
}

void onMouseDrag(int event,int x,int y,int flags,void* param)
{
    
	switch(event) {
	case CV_EVENT_LBUTTONDOWN:		//left button press
		
		if( (x> Drag[0][1])&& (x< Drag[1][1]) &&(y> Drag[0][0]) &&(y< Drag[1][0]) )  //If click again, Finish Drag and Drop mode
		{
			img_num= 0; //ning 1.5.0   //display from the beginning frame
			cvReleaseImage(&image_org);    //printf("Release image_org in else\n");
			DispImage(img_num);

			//ning 1.5.3  : Do the fitting again with updated inputNum[1280]
			#ifdef PLANE_FITTING
				//Read Points in and do plane fitting.
				readPoints(data_xyXX, data_xyYY, data_xyZZ, count); //ning 0.9.0
				planeFitting(); 
			#endif

			#ifdef CURVE_FITTING
				 curveFitting(inputNum, count);  //the output is Ce, which is a global
			#endif

			#ifdef CURVE_FITTING3D
				 curveFitting(inputNum, count); 
				 curveFitting3D(inputNum, count);  //the output is Ce, which is a global
			#endif

		    //ning 1.5.4 : update 2D_3D.txt with updated inputNum
			flag_SV= 0;

            //Finish Drag and Drop mode
			flag_DG= 0;
			cvShowImage("background",groundImg);
	        cvSetMouseCallback("background",onMouse,&mouseParam); 
			cvWaitKey(0);
		}
		//------------ning 1.5.0
		else if( (x> Next[0][1])&& (x< Next[1][1]) &&(y> Next[0][0]) &&(y< Next[1][0]) )
		{
		   //doNext();
			//printf("Next\n");
			if(flag_NI)
			{
			flag_NI= 1;
			flag_SV= 0;
			//flag_EXIT= 1;
			flag_PI= 1;

			buttonLight ( Next );
			cvShowImage( "background", groundImg );
			cvWaitKey(3);
			
			img_num++;
			//sprintf( inputName, "%s%d%s", folder, img_num+1, ".JPG"); 
			sprintf( inputName, "%s\\%s", folder, PicName[img_num+1]);

			//printf("create image_org!\n");
			if  ( (image_org = cvLoadImage( inputName, 1))==0)    
			{
				cvReleaseImage(&image_org);    //printf("Release image_org in if\n");
				DispImage(img_num);
				CvPoint d = { 50, 656 };
				sprintf( text, "No new image in this folder. ");
				CvFont font;
				cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
				cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

				d.y += 22;
				sprintf( text, "Please press Next Folder for a new sequence.");
				cvPutText( groundImg, text, d, &font, cvScalar(0, 255, 0, 255));

				cvShowImage("background",groundImg);
				cvSetMouseCallback("background",onMouseDrag,&mouseParam);
			
			flag_NI= 0;
			flag_SV= 0;
			//flag_EXIT= 1;
			 
			}
			else
			{
			cvReleaseImage(&image_org);    //printf("Release image_org in else\n");
			DispImage(img_num);

			 //drawlines_drag(0, groundImg);   //ning 1.5.0
			 //cvShowImage( "background", groundImg ); //ning 1.5.0
                //ning 1.5.0
			 	cvReleaseImage(&selectedImg);
				selectedImg = cvCloneImage(groundImg_copy);
				buttonLight_unlimited ( Drag, selectedImg );
				//draw latest-updated lines
				drawlines_drag(point, selectedImg);
				//show selectedImg
				cvShowImage("background", selectedImg);
			}
		    
			cvSetMouseCallback("background",onMouseDrag,&mouseParam);
			cvWaitKey(0);
			}
			else
			{}
		}
         
		else if( (x> Save[0][1])&& (x< Save[1][1]) &&(y> Save[0][0]) &&(y< Save[1][0]) )
		{
			if(flag_SV)
			{
			//flag_SV= 0;
			flag_EXIT= 1;

			FILE* fplist = fopen(filelist, "w" );
			for (int i = 0; i<= count-1; i++)
			{
				fprintf(fplist, "%f %f %f %f %f\n", inputNum[5*i +1], inputNum[5*i +2], inputNum[5*i +3], inputNum[5*i +4], inputNum[5*i +5]);  
			}
			fclose(fplist);
			

			cvCopy(selectedImg, groundImg_copy);
			buttonLight_unlimited(Save, selectedImg);
			cvShowImage("background", selectedImg);
			cvWaitKey(35);
			
			cvCopy(groundImg_copy, selectedImg);
			buttonLight_unlimited(Drag, selectedImg);
			cvShowImage("background", selectedImg);
		    }
			else{}
		}
		//-----------
		else   selectedImg=findImg( x, y);
		break;

	case CV_EVENT_LBUTTONUP:	//left mouse button release
		if((selectedImg!=NULL)&&point!=-1){
			releaseImg(selectedImg,x,y); //save and update coordinates
			selectedImg=NULL;
		}
		break;

	case CV_EVENT_MOUSEMOVE:
		/* draw a rectangle*/
		if(point!=-1){
			if(selectedImg!=NULL){
				tempImg = cvCloneImage(selectedImg);
				cvRectangle(tempImg,
					cvPoint(x - 1, y - 1),
					cvPoint(x + 1, y + 1),
					cvScalar(0, 0,  255,0), 2, 8, 0);

				//adjust the lines
				for(int i= 0;i< count-1;i++){
					if(i!=point){
						
						cvLine(tempImg,
							cvPoint(x,y ),
							cvPoint( inputNum[5*i+1]/x_scale + offset_x  , inputNum[5*i+2]/x_scale + offset_y ),
							cvScalar(0, 0,  255,0), 1,8,0);
					}
				}
				//cvShowImage("Drag & Drop", tempImg);
				cvShowImage("background", tempImg);
			}
			break;
		}
		cvReleaseImage(&tempImg);
	}
}

//Main function
int main( int argc, char** argv )
{
    InitStruct();
	//InitFolderPicture();
	InitFolderPicture2();

    // Load camera's intrinsic parameters
	/*CvMat * */intrinsic = (CvMat*)cvLoad( "..//Intrinsics.xml" );
	/*CvMat * */distortion = (CvMat*)cvLoad( "..//Distortion.xml" );

   //create a window
   success= cvNamedWindow( "background", CV_WINDOW_AUTOSIZE );
   if (success == 0)
   {
	   printf("Can't create window, please contact us.");
       cvWaitKey( delay1 ); //Time to see this warning
	   return -1;
   }
   else
   {
     //Load ground image                        
	 if( (groundImg_org = cvLoadImage( "..\\img\\background.jpg", 1)) == 0 ) 
	 {
		 printf("Can't find file background.jpg in directory Calibration Tool v1.0\img\  .");
		 return -1;
	 }
   }

   //Load number image
   if( (one = cvLoadImage( "..\\img\\one.JPG", 1)) == 0 ) 
   {
	    printf("Can't find file one.jpg in directory Calibration Tool v1.0\img\  ");
		return -1;
   }
   if( (two = cvLoadImage( "..\\img\\two.JPG", 1)) == 0 )   
   {
	    printf("Can't find file two.jpg in directory Calibration Tool v1.0\img\  ");
		return -1;
   }
   if( (three = cvLoadImage( "..\\img\\three.JPG", 1)) == 0 ) 
   {
	    printf("Can't find file three.jpg in directory Calibration Tool v1.0\img\  ");
		return -1;
   }

   //start from the first image of the first folder 
   //folder_num= 1;    //!!!
   //img_num= 1;      //!!!
   //sprintf( folder, "%s%d%s", "..\\data\\", folder_num, "\\"); //!!!
   
    //DISPLAY SOME INSTRUCTIONS
   /*
    groundImg_org_text= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
    cvCopy(groundImg_org, groundImg_org_text);
    CvPoint d = { 45, 648 };
    sprintf( text,  "Please click Calibration first to get the intrinsic parameters of the camera.");
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
    cvPutText( groundImg_org_text, text, d, &font, cvScalar(0, 0, 255, 255));
    cvShowImage( "background", groundImg_org_text );
    */

    xx1= &xxx1;
	yy1= &yyy1;
	zz1= &zzz1;
    xx2= &xxx2;
	yy2= &yyy2;
	zz2= &zzz2;
//////////////////////////////////////////////////
	FILE* fpl;    //stream to read stick2D_camera3D.txt
	if ((fpl= fopen(txt_name, "r" ))==0)
   {
	flag_NI= 0;
    flag_NF= 0;
    flag_SV= 0;
    flag_EXIT= 0;
	flag_else= 0;
	flag_NP = 0;
	flag_PI = 0;

    groundImg_org_text= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
	printf("Create groundImg_org_text!\n");
    cvCopy(groundImg_org, groundImg_org_text);
    CvPoint d = { 65, 670 };
    sprintf( text,  "No Calibration info available. Please run Calibrate.exe first.");
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f, 0, 1, CV_AA);
    cvPutText( groundImg_org_text, text, d, &font, cvScalar(255, 255, 255, 255));
    cvShowImage( "background", groundImg_org_text );
	mouseParam= 5;
    cvSetMouseCallback("background",onMouse,&mouseParam);
    cvWaitKey(0);
   }
   else
   {
	//Read in pole calibration information 
    FILE* fplist = fopen(filelist, "r" );
	for (int i = 1; i<= Max; i++)
	{
		if(  (fscanf(fplist, "%f", &inputNum[i]))==1)
		{
			count++;
		}
	}
    fclose(fplist);
	count= count/5;
	printf("Number of avalable poles is: %d\n", count-1);
    /*  //This are re-put some lines below. ignore this
	readPoints(data_xyXX, data_xyYY, data_xyZZ, count); //ning 0.9.0
    planeFitting();  */
    //***********************************************************************************************
    //sprintf( inputName, "%s%d%s", folder, img_num, ".JPG");    //!!!
    //sprintf( inputName, "%s\\%s", folder, PicName);
	 sprintf( inputName, "%s\\%s", folder, PicName[0]);
	 //printf("PicName[0] is: %s\n", PicName[0]);
	 //printf("inputName is: %s\n", inputName);

    //printf("create image_org!\n");
	if (image_org = cvLoadImage( inputName, 1))  
	{
	//Calculate mapx and mapy for compensation use  v0.9.3
	cvReleaseImage(&mapx); 
	cvReleaseImage(&mapy);
    mapx = cvCreateImage( cvGetSize(image_org), IPL_DEPTH_32F, 1 );
	mapy = cvCreateImage( cvGetSize(image_org), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );
	t = cvCloneImage(image_org );	
	//cvRemap( t, image_org, mapx, mapy ); // undistort image
	//cvReleaseImage( &t );
	//cvNamedWindow("undistorted image");
	//cvShowImage("undistorted image", image_org);
	//cvWaitKey(0);
		
	// Create matrix of mapx and mapy
	mapX = cvCreateMat( mapx->height, mapx->width, CV_32FC1);
	mapY = cvCreateMat( mapy->height, mapy->width, CV_32FC1);
    int new_x, new_y;
	remapX = cvCreateMat( mapx->height, mapx->width, CV_32FC1);
	remapY = cvCreateMat( mapy->height, mapy->width, CV_32FC1);

	cvConvert(mapx, mapX);
	cvConvert(mapy, mapY);

	for (int y0= 0; y0< mapx->height; y0++)
   {
	   for (int x0= 0; x0< mapx->width; x0++)
	   {
		    new_x= CV_MAT_ELEM( *mapX, float, y0, x0 );
		    new_y= CV_MAT_ELEM( *mapY, float, y0, x0 );

		   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 0]= 0;
		   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 1]= 0;
		   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 2]= 0;
		   
		   if((new_x>=0)&&(new_x< mapx->width)&&(new_y>=0)&&(new_y< mapx->height)) {
			   //printf("(newx, newy)= (%d, %d)\n", new_x, new_y);
			   /*  //should be reverse as below. Online statement is wrong.
			   ((uchar*)image_org->imageData + new_y* image_org->widthStep)[new_x*image_org->nChannels + 0]=  ((uchar*)t->imageData + y0* t->widthStep)[x0*t->nChannels + 0];
			   ((uchar*)image_org->imageData + new_y* image_org->widthStep)[new_x*image_org->nChannels + 1]=  ((uchar*)t->imageData + y0* t->widthStep)[x0*t->nChannels + 1];
			   ((uchar*)image_org->imageData + new_y* image_org->widthStep)[new_x*image_org->nChannels + 2]=  ((uchar*)t->imageData + y0* t->widthStep)[x0*t->nChannels + 2];
               */
			   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 0]=  ((uchar*)t->imageData + new_y* t->widthStep)[new_x*t->nChannels + 0];
			   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 1]=  ((uchar*)t->imageData + new_y* t->widthStep)[new_x*t->nChannels + 1];
			   ((uchar*)image_org->imageData + y0* image_org->widthStep)[x0*image_org->nChannels + 2]=  ((uchar*)t->imageData + new_y* t->widthStep)[new_x*t->nChannels + 2];
			   
			   CV_MAT_ELEM( *remapX, float, new_y, new_x)= x0;
		       CV_MAT_ELEM( *remapY, float, new_y, new_x)= y0;
		   }
	   }
   }
	//cvNamedWindow("undistorted image");
	//cvShowImage("undistorted image", image_org);
	//cvWaitKey(0);
	cvReleaseImage( &t );

#ifdef PLANE_FITTING
	//Read Points in and do plane fitting.
	readPoints(data_xyXX, data_xyYY, data_xyZZ, count); //ning 0.9.0
    planeFitting(); 
#endif

#ifdef CURVE_FITTING
     curveFitting(inputNum, count);  //the output is Ce, which is a global
#endif

#ifdef CURVE_FITTING3D
	 curveFitting(inputNum, count); 
     curveFitting3D(inputNum, count);  //the output is Ce, which is a global
#endif
	//Display image
    DispImage(img_num);  
	//*************************************************************************************************
#ifdef SAVE_TXT 
	fp_DATA= fopen("..\\data.txt", "w");
	fprintf( fp_DATA, "     Sequence_ID_Number|| Pict_number_in_sequence || cursor_x_position || cursor_y_Position || distance_from_camera || distance_from_previous_point || sum_distance_sequence || angle_animal_camera || angle_animal_animal\n");
#endif
#ifdef SAVE_XLS
	book= xlCreateBook();
	if(book){
		sheet= xlBookAddSheet(book, "Sheet1", 0);
		if(sheet){
		    //FormatHandle dataFormat;
			xlSheetWriteStr(sheet, 0, 0, "Sequence_ID_Number", 0);
			xlSheetWriteStr(sheet, 0, 1, "Pict_number_in_sequence", 0);
			xlSheetWriteStr(sheet, 0, 2, "cursor_x_position", 0);
			xlSheetWriteStr(sheet, 0, 3, "cursor_y_Position", 0);
			xlSheetWriteStr(sheet, 0, 4, "distance_from_camera", 0);
			xlSheetWriteStr(sheet, 0, 5, "distance_from_previous_point", 0);
			xlSheetWriteStr(sheet, 0, 6, "sum_distance_sequence", 0);
			xlSheetWriteStr(sheet, 0, 7, "angle_animal_camera", 0);
			xlSheetWriteStr(sheet, 0, 8, "angle_animal_animal", 0);

			xlSheetSetCol(sheet, 0, 0, 19, 0, 0);
			xlSheetSetCol(sheet, 1, 1, 22, 0, 0);
			xlSheetSetCol(sheet, 2, 3, 15, 0, 0);
			xlSheetSetCol(sheet, 4, 4, 20, 0, 0);
			xlSheetSetCol(sheet, 5, 5, 25, 0, 0);
			xlSheetSetCol(sheet, 6, 6, 21, 0, 0);
			xlSheetSetCol(sheet, 7, 8, 18, 0, 0);

			which_row= 1;  //going to write numbers on row 1(it is the second row).
		}
		//if (xlBookSave(book, "../data.xls"))  printf("xml Saved!\n");
	}
#endif

	CvPoint d = { 50, 656 };
    //sprintf( text,  "Loading 10 sticks points that are on the ground...");
	sprintf( text, "Ready to measure distance.");
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.5f, 0.5f,0,1,CV_AA);
    cvPutText( groundImg, text, d, &font, cvScalar(255, 255, 255, 255));

    d.y += 22;
	//sprintf( text, "Parameters for calculating distances are set.");
	sprintf( text, "You can also click Drag to tune pole position first (optional)");
	cvPutText( groundImg, text, d, &font, cvScalar(255, 255, 255, 255));

	cvShowImage("background",groundImg);
	cvSetMouseCallback("background",onMouse,&mouseParam); //ning
    /*
	cvReleaseImage(&selectedImg);
	selectedImg = cvCloneImage(groundImg_copy);
	drawlines_drag(0, selectedImg);
	cvShowImage("Drag & Drop", selectedImg);
    cvSetMouseCallback("Drag & Drop",onMouseDrag,&mouseParam);
    cvWaitKey(0);
	*/
	cvWaitKey(0);
	//*************************************************************************************************
	}
	else
	{
	flag_NI= 0;
    flag_NF= 0;
    flag_SV= 0;
    flag_EXIT= 0;
	flag_else= 0;
	groundImg_org_text= cvCreateImage( cvSize(groundImg_org->width, groundImg_org->height), IPL_DEPTH_8U, 3);
	//printf("Create groundImg_org_text!\n");
    cvCopy(groundImg_org, groundImg_org_text);
    CvPoint d = { 50, 656 };
    sprintf( text,  "No images available. Or first image folder empty." );
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,0.7f, 0.7f,0,1,CV_AA);
    cvPutText( groundImg_org_text, text, d, &font, cvScalar(255, 255, 255, 255));

	sprintf( text, "Please put in data images or delete empty folders.");
    d.y += 22;
    cvPutText( groundImg_org_text, text, d, &font, cvScalar(255, 255, 255, 255));

    cvShowImage( "background", groundImg_org_text );
	mouseParam= 5;
    cvSetMouseCallback("background",onMouse,&mouseParam);
    cvWaitKey(0);
	}
	}
}