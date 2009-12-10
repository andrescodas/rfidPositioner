#ifndef RFID_POSITIONER_STRUCT_H
#define RFID_POSITIONER_STRUCT_H

#define B21R


typedef struct POSITION
{
	double xRob;
	double yRob;
	double theta;
} POSITION;



/* Set new variances on (x,y,theta)
   With : v0 = vxx;  v1 = vxy;  v2 = vyy;  v3 = vxt; v4 = vyt;  v5 = vtt */
typedef struct POSITION_ERROR
{
	double position_cov[6];
} POSITION_ERROR;

#endif /* RFID_POSITIONER_STRUCT_H */
