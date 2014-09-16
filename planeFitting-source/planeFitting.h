#define PI 3.1415926

struct tagPoint
{
	double x;// As Double
	double y;// As Double
	double z;// As Double
};

struct tagLine2D
{
	double k;
	double b;
	double Angle;
	double Straightness;
	double RSQ;
};

/*
3D line's formula is showing as following.
'1)|Ax +By +Z+D =0
'  |A1x+B1y+z+D1=0
'2)(x-x0)/m=(y-y0)/n=(z-z0)/p----(x-x0)/m=(y-y0)/n=z/1
'3)x=mt+x0,y=nt+y0,z=pt+z0
'Only point's coordinate is (a+b*Z, c+d*Z,Z),so the line's vector is {b,d,1}
*/

struct tagLine3D
{
	double x;// As Double
	double y;// As Double
	double z;// As Double
	double x0;// As Double
	double y0;// As Double
	double z0;// As Double
	double xAn;// As Double
	double yAn;// As Double
	double zAn;// As Double
	double m,n,p,Angle;
	double Straightness;
	double MinSt,MaxSt;

};

struct tagVector
{
	double a,b,c;
};

struct tagPlane
{
	double x,y,z;     //The distance from the origin to the centroid, as measured along the x-axis.
	double ax,by,cz,d;//Z + A*x + B*y + C =0  z's coefficient is just 1
	double Angle;
	double xAn,yAn,zAn;
	double Flat,MinFlat,MaxFlat;
};
/*
'*************************************************************

'  ????Intersect

'  ???  ?????????

'  ??? v1  - tagVector

'         v2  - tagVector

'         LinePlane  -long   0:??????????,???:?????????,??????????(0~90)

'  ????Double?,??:?.

'*************************************************************
*/
double Intersect(tagVector v1,tagVector v2, long LinePlane = 0)
{
	//LinePlane 0 :line -line ,1:line --Plane
	double tmp,tmpSqr1,tmpSqr2;
	tmp = (v1.a * v2.a + v1.b * v2.b + v1.c * v2.c);
	//'MsgBox tm
	tmpSqr1 = sqrt(v1.a * v1.a + v1.b * v1.b  + v1.c *v1.c);
	tmpSqr2 = sqrt(v2.a * v2.a + v2.b * v2.b  + v2.c *v2.c);
	if (tmpSqr1 != 0)
	{
		if (tmpSqr2 != 0)
		{
			tmp = tmp/tmpSqr1/tmpSqr2;
		}
		else
		{
			tmp = tmp / tmpSqr1;
		}
	}
	else
	{
		if (tmpSqr2 != 0)
			tmp = tmp / tmpSqr2;
		else
			tmp = 0;
	}
	if (LinePlane != 0)
	{
		tmp = abs(tmp);
	}
	if (-tmp * tmp + 1 != 0)
	{
		tmp = atan(-tmp / sqrt(-tmp * tmp + 1)) + 2 * atan(1.0);
		tmp = tmp / PI * 180;
	}
	else
	{
		tmp = 90;
	}
	return tmp;
}

/*************************************************************

'  ????PointToPlane

'  ???  ????????

'  ???  dataRaw  - tagPoint?  ??????(x,y,z)

'          Plane  - tagPlane  Double

'          RtnDistance -Double    ??????????.

'          AbsDist     -Long?????????????????????????????????

'  ????Long?????0????-1

'*************************************************************/
double PointToPlane(tagPoint dataRaw,tagPlane Plane,double& rtnDistance, bool AbsDist = false)
{
	long i,lb,ub;
	double tmp = (Plane.ax * dataRaw.x + Plane.by * dataRaw.y + Plane.cz * dataRaw.z + Plane.d)\
			/sqrt(Plane.ax*Plane.ax + Plane.by*Plane.by + Plane.cz*Plane.cz);
	if (AbsDist != false)
		tmp = abs(tmp);
	rtnDistance = tmp;
	return rtnDistance;
};

long PlaneSet(tagPoint dataRaw[],tagPlane& Plane,int AMT) // z+Ax+BY+C=0
{
	double MaxF,MinF,tmp;
	if (AMT < 3)  //AMT= amount
		return 0;
	long i,n;
	n = AMT;
	double x,y,z,XY,XZ,YZ;
	double X2,Y2;
	double a,b,c,d;
	double a1,b1,z1;
	double a2,b2,z2;
	tagVector n1;     //{.ax,by,1}  s1
	tagVector n2;     //{0,0,N} XY plane  s2
	tagVector n3;     //line projected plane
	tagVector xLine,yLine,zLine,SLine;
	tagVector VectorPlane;
	xLine.a = 1; 
	xLine.b = 0; 
	xLine.c = 0;

	yLine.a = 0;
	yLine.b = 1;
	yLine.c = 0;

	zLine.a = 0;
	zLine.b = 0;
	zLine.c = 1;

	x = y = z =0;
	XY = XZ = YZ = 0;
	X2 = Y2 = 0;

	for (int i = 0;i < AMT;i++)
	{
		x += dataRaw[i].x;
		y += dataRaw[i].y;
		z += dataRaw[i].z;

		XY += dataRaw[i].x * dataRaw[i].y;
		XZ += dataRaw[i].x * dataRaw[i].z;
		YZ += dataRaw[i].y * dataRaw[i].z;
		X2 += dataRaw[i].x * dataRaw[i].x;
		Y2 += dataRaw[i].y * dataRaw[i].y;
	}
	z1 = n * XZ - x * z;//              'e=z-Ax-By-C  z=Ax+By+D
	a1 = n * X2 - x * x;//
	b1 = n * XY - x * y;
	z2 = n * YZ - y * z;
	a2 = n * XY - x * y;
	b2 = n * Y2 - y * y;
	a = (z1 * b2 - z2 * b1) / (a1 * b2 - a2 * b1);  
	b = (a1 * z2 - a2 * z1) / (a1 * b2 - a2 * b1);
	c = 1;
	d = (z - a * x - b * y) / n;

//printf("HELLO!!!!!!!!   a= %f\n", a);
	Plane.x = x / AMT;
	Plane.y = y / AMT;
	Plane.z = z / AMT;
	//'sum(Mi *Ri)/sum(Mi) ,Mi is mass . here  Mi is seted to be 1 and .z is just the average of z
	Plane.ax = -a;
	Plane.by = -b;
	Plane.cz = 1;
	Plane.d = -d; //z=Ax+By+D-----Ax+By+Z+D=0

	VectorPlane.a = Plane.ax;
	VectorPlane.b = Plane.by;
	VectorPlane.c = 1;

	Plane.xAn = Intersect(VectorPlane, xLine);
	Plane.yAn = Intersect(VectorPlane, yLine);
	Plane.zAn = Intersect(VectorPlane, zLine);

	n1.a = Plane.ax; 
	n1.b = Plane.by;
	n1.c = 1;

	SLine.a = Plane.ax;
	SLine.b = Plane.by;
	SLine.c = 0;

	Plane.Angle = Intersect(xLine, SLine);// (xLine.A * SLine.A + xLine.A * SLine.B + xLine.C * SLine.C)
	//if (SLine.b < 0)
	{
		Plane.Angle = 360 - Plane.Angle;
		double MaxF,MinF,rDist;
		for (int i = 0;i < AMT;i++)
		{
			PointToPlane(dataRaw[i],Plane,rDist,false);
			if (i == 0)
			{
				MaxF = MinF = rDist;
			}
			else
			{
				if (MaxF < rDist)
					MaxF = rDist;
				if (MinF > rDist)
					MinF = rDist;
			}
		}
		Plane.MaxFlat = MaxF;
		Plane.MinFlat = MinF;
		Plane.Flat = MaxF - MinF;
	}
} 



long PlaneSet2(tagPoint dataRaw[],tagPlane& Plane,int AMT) // z+Ax+BY+C=0
{
	long double MaxF,MinF,tmp;
	if (AMT < 3)  //AMT= amount
		return 0;
	long double i,n;
	n = AMT;
	long double x,y,z,XY,XZ,YZ;
	long double X2,Y2;
	long double a,b,c,d;
	long double a1,b1,z1;
	long double a2,b2,z2;
	tagVector n1;     //{.ax,by,1}  s1
	tagVector n2;     //{0,0,N} XY plane  s2
	tagVector n3;     //line projected plane
	tagVector xLine,yLine,zLine,SLine;
	tagVector VectorPlane;
	xLine.a = 1; 
	xLine.b = 0; 
	xLine.c = 0;

	yLine.a = 0;
	yLine.b = 1;
	yLine.c = 0;

	zLine.a = 0;
	zLine.b = 0;
	zLine.c = 1;

	x = y = z =0;
	XY = XZ = YZ = 0;
	X2 = Y2 = 0;

	for (int i = 0;i < AMT;i++)
	{
		x += dataRaw[i].x;
		y += dataRaw[i].y;
		z += dataRaw[i].z;

		XY += dataRaw[i].x * dataRaw[i].y;
		XZ += dataRaw[i].x * dataRaw[i].z;
		YZ += dataRaw[i].y * dataRaw[i].z;
		X2 += dataRaw[i].x * dataRaw[i].x;
		Y2 += dataRaw[i].y * dataRaw[i].y;
	}
	z1 = n * XZ - x * z;//              'e=z-Ax-By-C  z=Ax+By+D
	a1 = n * X2 - x * x;//
	b1 = n * XY - x * y;
	z2 = n * YZ - y * z;
	a2 = n * XY - x * y;
	b2 = n * Y2 - y * y;
	a = (z1 * b2 - z2 * b1) / (a1 * b2 - a2 * b1);  
	b = (a1 * z2 - a2 * z1) / (a1 * b2 - a2 * b1);
	c = 1;
	d = (z - a * x - b * y) / n;

//printf("HELLO!!!!!!!!   a= %f\n", a);
	Plane.x = x / AMT;
	Plane.y = y / AMT;
	Plane.z = z / AMT;
	//'sum(Mi *Ri)/sum(Mi) ,Mi is mass . here  Mi is seted to be 1 and .z is just the average of z
	Plane.ax = -a;
	Plane.by = -b;
	Plane.cz = 1;
	Plane.d = -d; //z=Ax+By+D-----Ax+By+Z+D=0

	VectorPlane.a = Plane.ax;
	VectorPlane.b = Plane.by;
	VectorPlane.c = 1;

	Plane.xAn = Intersect(VectorPlane, xLine);
	Plane.yAn = Intersect(VectorPlane, yLine);
	Plane.zAn = Intersect(VectorPlane, zLine);

	n1.a = Plane.ax; 
	n1.b = Plane.by;
	n1.c = 1;

	SLine.a = Plane.ax;
	SLine.b = Plane.by;
	SLine.c = 0;

	Plane.Angle = Intersect(xLine, SLine);// (xLine.A * SLine.A + xLine.A * SLine.B + xLine.C * SLine.C)
	//if (SLine.b < 0)
	{
		Plane.Angle = 360 - Plane.Angle;
		double MaxF,MinF,rDist;
		for (int i = 0;i < AMT;i++)
		{
			PointToPlane(dataRaw[i],Plane,rDist,false);
			if (i == 0)
			{
				MaxF = MinF = rDist;
			}
			else
			{
				if (MaxF < rDist)
					MaxF = rDist;
				if (MinF > rDist)
					MinF = rDist;
			}
		}
		Plane.MaxFlat = MaxF;
		Plane.MinFlat = MinF;
		Plane.Flat = MaxF - MinF;
	}
} 


//*****************************************************************************//
//These will be put into the windows.cpp
/*
//These variables should be global.
tagPoint data_xyXX[256];   //256*5= 1280, the number of inputNum[]
tagPoint data_xyYY[256];   //256*5= 1280, the number of inputNum[]
tagPoint data_xyZZ[256];   //256*5= 1280, the number of inputNum[]
tagPlane Plane1, Plane2, Plane3;

//Read in points for plane fitting
void readPoints(tagPoint data_xyXX[], tagPoint data_xyYY[], tagPoint data_xyYY[], int count){
	for(int i= 0; i< count; i++){
		data_xyXX[i].x= inputNum[5*i + 1];
		data_xyYY[i].x= inputNum[5*i + 1];
		data_xyZZ[i].x= inputNum[5*i + 1];
		data_xyXX[i].y= inputNum[5*i + 2];
		data_xyYY[i].y= inputNum[5*i + 2];
		data_xyZZ[i].y= inputNum[5*i + 2];

		data_xyXX[i].z= inputNum[5*i + 3];
		data_xyYY[i].z= inputNum[5*i + 4];
		data_xyZZ[i].z= inputNum[5*i + 5];
	}
}

//Plane fitting
void planeFitting(){
	PlaneSet(data_xyXX, &Plane1, count);
	PlaneSet(data_xyYY, &Plane2, count);
	PlaneSet(data_xyZZ, &Plane3, count);
}

//Calculate Coordinates using plane fitting
void Cal_Coordinates_planeFitting(int x, int y, float *xx, float *yy, float *zz )
{
	float XX, YY, ZZ;
	XX= (-1)*( plane1.d + plane1.ax * x + plane1.by * y ) / plane1.cz;
	YY= (-1)*( plane2.d + plane2.ax * x + plane2.by * y ) / plane2.cz;
	ZZ= (-1)*( plane3.d + plane3.ax * x + plane3.by * y ) / plane3.cz;
    *xx= XX;
	*yy= YY;
	*zz= ZZ;
}
*/