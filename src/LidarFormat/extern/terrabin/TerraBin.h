

//////////////////////////////////////////////////////////////////////////
//     Structures et fonctions de lecture du format BIN de TerraSolid
//////////////////////////////////////////////////////////////////////////

#ifndef TERRABIN_H
#define TERRABIN_H


#include <stdio.h>

typedef unsigned char BYTE ;
typedef unsigned short USHORT ;
typedef unsigned int UINT ;

//	Undefined time stamp
#define SEC_UNDEFINED	0xFFFFFFFF

//	Internal/UOR point
typedef struct
{
	int	x ;
	int	y ;
	int	z ;
} TerraPoint3d ;

//	Master unit/meter point
typedef struct
{
	double	x ;
	double	y ;
	double	z ;
} TerraDp3d ;

//	TerraScan binary file header
typedef struct
{
	int HdrSize ;		// sizeof(ScanHdr)
	int HdrVersion ;	// Version 20020715, 20010712, 20010129 or 970404
	int Tunniste ;		// Always 970401
	char Magic[4];		// CXYZ
	int PntCnt;		// Number of points stored
	int Units ;			// Units per meter = subpermast * uorpersub
	double OrgX ;		// Coordinate system origin
	double OrgY ;
	double OrgZ ;
	int Time ;			// 32 bit integer time stamps appended to points
	int Color ;			// Color values appended to points
}	TerraScanHeader;

//	TerraScan binary file point record (ver 20010712, 20010129, 970404)
typedef struct
{
	BYTE Code ;			// Classification code 0-255
	BYTE Line ;			// Flightline number 0-255
	USHORT EchoInt ;	// Intensity bits 0-13, echo bits 14-15
	int X ;			// Easting
	int Y ;			// Northing
	int Z ;			// Elevation
} TerraScanRow;

//	TerraScan binary file point record ver 20020715
typedef struct
{
	TerraPoint3d Pnt ;		// Coordinates
	BYTE Code ;			// Classification code
	BYTE Echo ;			// Echo information
	BYTE Flag ;			// Runtime flag (view visibility)
	BYTE Mark ;			// Runtime flag
	USHORT Line ;		// Flightline number
	USHORT Intensity ;	// Intensity value
} TerraScanPnt ;

//	RGB color
typedef struct
{
	BYTE	red ;
	BYTE	green ;
	BYTE	blue ;
} TerraRgbClr ;



/*-------------------------------------------------------------------
	Attemp reading TerraScan binary file header information
	from file Name.

	Return  1 on success.
	Return  0 if could not open file.
	Return -1 if file is of wrong format.
*/
int		ScanGetHeader( TerraScanHeader *Hdr, const char *Name);

/*-------------------------------------------------------------------
	Read binary scan points from file Path into table Tbl.
	Table has been allocated for Cnt points.

	Return number of points read.
	Return  0 if not a valid binary file.
	Return -1 if could not open file.
*/
int     ScanReadBinary( TerraScanPnt *Tbl, UINT *Sec, TerraRgbClr *Clr, int Cnt, const char *Path);

/*-------------------------------------------------------------------
	Read TerraScan binary file header and check validity.

	Return 1 if valid header was found.
	Return 0 if not TerraScan binary file.
*/
int		ScanReadHdr( TerraScanHeader *Hp, FILE *File);

/*-------------------------------------------------------------------
	Check if H is a valid TerraScan binary file header.

	Return 5 if valid header newer than 15.07.2002.
	Return 4 if valid header version 15.07.2002.
	Return 3 if valid header version 12.07.2001.
	Return 2 if valid header version 29.01.2001.
	Return 1 if valid header version 04.04.1997.
	Return 0 if not valid.
*/
int		SrvScanHeaderValid( TerraScanHeader *H);


#endif //TERRABIN_H
