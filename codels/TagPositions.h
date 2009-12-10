#ifndef _TAG_POSITIONS_H_
#define _TAG_POSITIONS_H_


#include "server/rfidPositionerHeader.h"

#include <map>
#include <vector>
#include <math.h>


const int TAG_ID_MAX_SIZE = 16;  //the maximum length of the tag id string
const double PI = 3.14159265358979323846;
const double PROBABILITY_OUTSIDE_MODEL = 0;


//ANTENNA POSITIONS PARAMETERS
//  - NUMBER_OF_ANTENNAS -> the number of antennas present on the robot
//  - the only other important structure is the ANTENNA_POSITIONS array that describes the positions of the antenna centers relative to the center of the robot (the angles are relative to the forward direction)
//  - in our case, we use 2 auxiliary structures to describe these positions, since the robot has radial symmetry:
//      - ANTENNA_POSITION_RADIUS - the radius of the circle on which all antenna centers lie
//      - ANTENNA_POSITION_ANGLES - the angles that each center is viewed on from the center of the robot relative to the forward direction

const int NUMBER_OF_ANTENNAS = 8;


// TODO --> Determine antenna Positions take care with orientations
double const ANTENNA_POSITION_RADIUS = 0.35;
double const ANTENNA_POSITION_ANGLES[] = {4*PI/4,
                                          3*PI/4,
                                          2*PI/4,
                                          1*PI/4,
                                          0*PI/4,
                                          7*PI/4,
                                          6*PI/4,
                                          5*PI/4
                                         };




//a structure describing a point (x,y) in 2D
typedef struct Point2D {
  double x;
  double y;

  Point2D(double _x, double _y){
	x=_x;
	y=_y;
	}

  Point2D(){
	x=0;
	y=0;
	}

} Point2D;

//a structure describing a tag (x,y,id)
typedef struct Tag2D {
	double x;
	double y;
	char tagid[TAG_ID_MAX_SIZE+1];

	Tag2D (double _x, double _y, const char* _tagid){
		x=_x;
		y=_y;
		for(unsigned int i=0;i<strlen(_tagid);++i) {
			tagid[i] = _tagid[i];
		}
		tagid[TAG_ID_MAX_SIZE] = 0;
	}
	Tag2D(){x=0;
		y=0;
		tagid[0] = 0;
	}
} Tag2D;


//a structure describing a possible tag detection (antenna,probability of detection,tag_id)
typedef struct TagDetection {
	int antenna;
	char tagid[TAG_ID_MAX_SIZE+1];
	TagDetection* next;

	TagDetection(int _antenna, const char* _tagid ,TagDetection* _next) {
		antenna = _antenna;
		next = _next;

		for (unsigned int i=0;i<strlen(_tagid);++i) {
			tagid[i] = _tagid[i];
		}
		tagid[TAG_ID_MAX_SIZE] = 0;
	}

	TagDetection() {
		antenna = -1;
		tagid[0]=0;
		next = NULL;
	}

} TagDetection;

//a structure describing a tag expectation (antenna,probability of detection,tag_id)
typedef struct TagExpectation {
	int antenna;
	double probability;
	char tagid[TAG_ID_MAX_SIZE+1];
	TagExpectation* next;

	TagExpectation(int _antenna, double _probability, const char* _tagid,TagExpectation* _next) {
		antenna = _antenna;
		probability = _probability;
		next = _next;

		for (unsigned int i=0;i<strlen(_tagid);++i) {
			tagid[i] = _tagid[i];
		}
		tagid[TAG_ID_MAX_SIZE] = 0;
	}

	TagExpectation() {
		antenna = -1;
		probability = 0;
		tagid[0]=0;
		next = NULL;
	}

} TagExpectation;

int tagNumber(const char* tagName);

struct strCmp {
	bool operator()( const char* s1, const char* s2 ) const {
	return strcmp( s1, s2 ) < 0;
	}
};

//a map that allows us to recover a tag (position) by id
typedef std::map<const char*, Point2D, strCmp> TagMap;
//a vector that will index the tags in ascending order according to position
typedef std::vector<Tag2D> Tag2DVector;

//all function descriptions are in the .cc file
TagMap initTagMap();

Tag2DVector initTag2DVector(TagMap);

// force angle representation to (-PI,PI]
double angleWrap(double angle);


void getTagDetections(TagDetection** tagDetections, RFID_TAGLIST_POSTER_STR* rfidPoster);

void freeTagDetection(TagDetection* tagDetections);

void freeTagExpectation(TagExpectation* tagsExpected);

//#define BOGDANMODEL


#ifdef BOGDANMODEL

//SENSOR MODEL PARAMETERS
//  - SENSOR_SIZE -> describes the size of the matrix describing the sensor model (the matrix is square of (2*SENSOR_SIZE-1) size)
//  - SENSOR_SIZE_LENGTH -> the length (in meters) corresponding to SENSOR_SIZE cells of the sensor model matrix
//  - PROBABILITY_OUTSIDE_MODEL -> the probability of detecting a tag outside the sensor model
const int SENSOR_SIZE = 100;
const double SENSOR_SIZE_LENGTH = 4;

//ANTENNA POSITIONS PARAMETERS
//  - NUMBER_OF_ANTENNAS -> the number of antennas present on the robot
//  - the only other important structure is the ANTENNA_POSITIONS array that describes the positions of the antenna centers relative to the center of the robot (the angles are relative to the forward direction)
//  - in our case, we use 2 auxiliary structures to describe these positions, since the robot has radial symmetry:
//      - ANTENNA_POSITION_RADIUS - the radius of the circle on which all antenna centers lie
//      - ANTENNA_POSITION_ANGLES - the angles that each center is viewed on from the center of the robot relative to the forward direction


double const ANTENNA_POSITIONS[][3] = {
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[0]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[0]),
                                                                    ANTENNA_POSITION_ANGLES[0]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[1]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[1]),
                                                                    ANTENNA_POSITION_ANGLES[1]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[2]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[2]),
                                                                    ANTENNA_POSITION_ANGLES[2]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[3]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[3]),
                                                                    ANTENNA_POSITION_ANGLES[3]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[4]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[4]),
                                                                    ANTENNA_POSITION_ANGLES[4]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[5]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[5]),
                                                                    ANTENNA_POSITION_ANGLES[5]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[6]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[6]),
                                                                    ANTENNA_POSITION_ANGLES[6]
                                       },
                                       {ANTENNA_POSITION_RADIUS*cos(ANTENNA_POSITION_ANGLES[7]),
                                        ANTENNA_POSITION_RADIUS*sin(ANTENNA_POSITION_ANGLES[7]),
                                                                    ANTENNA_POSITION_ANGLES[7]
                                       }
                                      };


void initSensorModel(double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1],const char* basicModelFile);

int getExpectedTagDetections(TagExpectation** tagExpectation,
                             double x, double y, double theta,
                             double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1],
                             Tag2DVector tags
                            );




#else // #ifdef BOGDANMODEL


// CHOOSE ONE AND ONLY ONE OF THE THREE!!

//#define GEOMETRICALMODEL
//#define FILEMODELCENTERED
#define FILEMODELPOLAR

#ifdef GEOMETRICALMODEL

//MODELCONSIDERATIONS  international measurements system
// TODO :  APROXIMATE BETTER!
#define MINOR_RADIUS 1
#define ARC_ANGLE 1.65806278939461  // 95 degrees
#define ARC_RADIUS  5
#define MAJOR_RADIUS 10

#define PROB_INSIDE_LITTLE_CIRCLE 0.7
#define PROB_INSIDE_ARC 0.9
#define PROB_INSIDE_BIG_CIRCLE 0.5


#elif defined FILEMODELCENTERED

const char SENSOR_BASE_MODEL[] = "~/sensor_base_model_C.in";

void initSensorModel();

#elif defined FILEMODELPOLAR

const char SENSOR_BASE_MODEL[] = "~/simulation/sensor_base_model_P.in";
void initSensorModel();


#else

#error "NO TYPE OF MODEL DEFINED"

#endif



int getExpectedTagDetections(TagExpectation** tagsExpected,
							double x, double y, double theta,
							Tag2DVector tags
                            );


#endif // #ifdef BOGDANMODEL


#endif // _TAG_POSITIONS_H_
