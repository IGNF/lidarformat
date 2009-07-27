
#include "TerraBin.h"
#include <memory.h>


/*-------------------------------------------------------------------
	Attemp reading TerraScan binary file header information
	from file Name.
	
	Return  1 on success.
	Return  0 if could not open file.
	Return -1 if file is of wrong format.
*/

int		ScanGetHeader( TerraScanHeader *Hdr, const char *Name)
{
	FILE	*File ;
	int		Ok ;

	File = fopen( Name, "rb") ;
	if (!File)
		return (0) ;

	Ok = ScanReadHdr( Hdr, File) ;
	fclose( File) ;
	if (!Ok)
		return (-1) ;

	return (1) ;
}

/*-------------------------------------------------------------------
	Read binary scan points from file Path into table Tbl.
	Table has been allocated for Cnt points.

	Return number of points read.
	Return  0 if not a valid binary file.
	Return -1 if could not open file.
*/

int     ScanReadBinary( TerraScanPnt *Tbl, UINT *Sec, TerraRgbClr *Clr, int Cnt, const char *Path)
{
	FILE	*File ;
	TerraScanHeader	Hdr ;
	TerraScanRow	Mit ;
	TerraScanPnt	*P ;
	BYTE	Rgb[4] ;
	UINT	Time = SEC_UNDEFINED ;
	int		Ret  = 0 ;
	int		Ok ;

	//	Open file for reading

	File = fopen( Path, "rb") ;
	if (!File)									return (-1) ;

	if (!ScanReadHdr( &Hdr, File))
		Hdr.PntCnt = 0 ;

	//	Use white if no color in file

	Rgb[0] = 255 ;
	Rgb[1] = 255 ;
	Rgb[2] = 255 ;

	while ((Ret < Hdr.PntCnt) && (Ret < Cnt)) {
		P = Tbl + Ret ;
		if (Hdr.HdrVersion == 20020715) {
			Ok = fread( P, sizeof(*P), 1, File) ;
			if (Ok != 1)							break ;
		}
		else {
			Ok = fread( &Mit, sizeof(Mit), 1, File) ;
			if (Ok != 1)							break ;

			//	Convert point information

			P->Pnt.x     = Mit.X ;
			P->Pnt.y     = Mit.Y ;
			P->Pnt.z     = Mit.Z ;
			P->Code      = Mit.Code ;
			P->Line      = Mit.Line ;
			P->Intensity = Mit.EchoInt & 0x3FFF ;
			P->Echo      = (Mit.EchoInt >> 14) ;
		}

		//	Read time stamp if given

		if (Hdr.Time)
			fread( &Time, sizeof(UINT), 1, File) ;
		if (Sec)
			Sec[Ret] = Time ;

		//	Read color value if given

		if (Hdr.Color)
			fread( Rgb, 4, 1, File) ;
		if (Clr) {
			Clr[Ret].red   = Rgb[0] ;
			Clr[Ret].blue  = Rgb[1] ;
			Clr[Ret].green = Rgb[2] ;
		}
		Ret++ ;
	}
	fclose( File) ;

	return (Ret) ;
}

/*-------------------------------------------------------------------
	Read TerraScan binary file header and check validity.

	Return 1 if valid header was found.
	Return 0 if not TerraScan binary file.
*/

int		ScanReadHdr( TerraScanHeader *Hp, FILE *File)
{
	TerraScanHeader	Hdr ;
	int		CopySize ;
	int		Val ;
	int		Ok ;

	Ok  = fread( &Hdr, sizeof(TerraScanHeader), 1, File) ;
	if (Ok != 1)								return (0) ;

	Val = SrvScanHeaderValid( &Hdr) ;
	if (!Val)									return (0) ;

	if (Hdr.HdrSize != sizeof(TerraScanHeader)) {
		CopySize = sizeof(TerraScanHeader) ;
		if (Hdr.HdrSize < CopySize)
			CopySize = Hdr.HdrSize ;
		memset( Hp, 0, sizeof(TerraScanHeader)) ;
		memcpy( Hp, &Hdr, CopySize) ;
		fseek( File, Hdr.HdrSize, SEEK_SET) ;
	}
	else {
		*Hp = Hdr ;
	}
	return (1) ;
}

/*-------------------------------------------------------------------
	Check if H is a valid TerraScan binary file header.

	Return 5 if valid header newer than 15.07.2002.
	Return 4 if valid header version 15.07.2002.
	Return 3 if valid header version 12.07.2001.
	Return 2 if valid header version 29.01.2001.
	Return 1 if valid header version 04.04.1997.
	Return 0 if not valid.
*/

int		SrvScanHeaderValid( TerraScanHeader *H)
{
	int		V ;

	if (H->Tunniste != 970401)					return (0) ;
	if (memcmp(H->Magic,"CXYZ",4))				return (0) ;

	V = H->HdrVersion ;
	if (V == 970404)							return (1) ;
	if (V == 20010129)							return (2) ;
	if (V == 20010712)							return (3) ;
	if (V == 20020715)							return (4) ;
	if ((V > 20020715) && (V < 20051231))		return (5) ;

	return (0) ;
}
