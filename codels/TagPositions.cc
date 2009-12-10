#include "TagPositions.h"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <string>



//initTagMap
//  - description
//      - the function allows the construction of the list of tags (all tags in the environment should be inputed by modifying this function)
//  - input: none
//  - output: the tag map
TagMap initTagMap() {
  TagMap tagmap;
  tagmap.clear();

  tagmap["e0040000c1b2fd01\0"] = Point2D(	 -4.07,	  -1); 	//tag: 0
  tagmap["e00400007cd7fc01\0"] = Point2D(	 -4.07,	  1.84); 	//tag: 1
  tagmap["e0040000079efd01\0"] = Point2D(	  3,	  1.5 ); 	//tag: 2
  tagmap["e004000076defc01\0"] = Point2D(	 00.50,	  3.55);	//tag: 3
  tagmap["e0040000dab1fd01\0"] = Point2D(	 01.50, +00.50); 	//tag: 4
  tagmap["e0040000fa9afd01\0"] = Point2D(	+02.50, -03.00); 	//tag: 5
  tagmap["e004000080ddfc01\0"] = Point2D(	-0.84,	-03.27); 	//tag: 6


  return tagmap;
}

int tagNumber(const char* tagName){


	if( strcmp(tagName,"e0040000c1b2fd01\0") == 0){
		return 0;
	}else if( strcmp(tagName,"e00400007cd7fc01\0") == 0){
		return 1;
	}else if( strcmp(tagName,"e0040000079efd01\0") == 0){
		return 2;
	}else if( strcmp(tagName,"e004000076defc01\0")== 0){
		return 3;
	}else if( strcmp(tagName,"e0040000dab1fd01\0")== 0){
		return 4;
	}else if( strcmp(tagName,"e0040000fa9afd01\0")== 0){
		return 5;
	}else if( strcmp(tagName,"e004000080ddfc01\0")== 0){
		return 6;
	}else{
		return -1;
	}


}

//Tag2Dcmp
//  - description
//      - the function introduces an order relation over the tags, from left to right and down to up (position-wise)
//  - input: the two tags to be compared
//  - output: true if t1<t2, false otherwise
bool Tag2Dcmp( Tag2D t1, Tag2D t2 ) {
	if(t1.x != t2.x){
		return t1.x < t2.x;
	}else{
		return t1.y < t2.y;
	}
}

//inits the vector containing tags sorted by x, and in case of equal x by y
Tag2DVector initTag2DVector(TagMap tm) {
	Tag2DVector result(tm.size());
	int k=0;
	for(TagMap::const_iterator it = tm.begin(); it != tm.end(); ++it) {
		result[k++] = Tag2D(it->second.x,it->second.y,it->first);
	}
	sort(result.begin(), result.end(), Tag2Dcmp);
	return result;
}


//returns de minimum of a and b
double min(double a, double b) {
  if (a<b) return a;
  else return b;
}

//returns de maximum of a and b
double max(double a, double b) {
  if (a>b) return a;
  else return b;
}


//this function requires a take on the poster before calling it and a give after the call
//it stores in the tagDetections array the detections the robot receives in reality
void getTagDetections(TagDetection** tagDetections, RFID_TAGLIST_POSTER_STR* rfidPoster) {
	//printf("[getTagDetections] nbtags = %d\n",rfidPoster->list.nbTags);
	TagDetection* before = NULL;
	TagDetection* detected = NULL;

	for (int i=0;i<rfidPoster->list.nbTags;++i) {
		for (int j=0;j<NUMBER_OF_ANTENNAS;++j) {
			if (rfidPoster->list.tags[i].antennas[j] == 1) {

				if(*tagDetections == NULL ){  // there is not before

					detected  = (TagDetection*)malloc(sizeof(TagDetection));
					*tagDetections = detected;
				}else{
					before = detected;
					detected = (TagDetection*)malloc(sizeof(TagDetection));
					before->next = detected;
				}

				*detected = TagDetection(j,rfidPoster->list.tags[i].tagId,NULL);
			}
		}
	}
}

void freeTagDetection(TagDetection* tagDetections){
	TagDetection* tagDetectionAux = tagDetections;
	TagDetection* before = 0;

		while(tagDetectionAux != NULL){

			before = tagDetectionAux;
			tagDetectionAux = tagDetectionAux->next;
			free(before);
		}
}

void freeTagExpectation(TagExpectation* tagsExpected){
	TagExpectation* tagsExpectedAux = tagsExpected;
	TagExpectation* before = 0;

		while(tagsExpectedAux != NULL){

			before = tagsExpectedAux;
			tagsExpectedAux = tagsExpectedAux->next;
			free(before);
		}
}


// force angle representation to ( -PI,PI]
double angleWrap(double angle){
	double angleAux = angle;

	while(angleAux <= -PI){
		angleAux = angleAux + 2 * PI;
	}

	while(angleAux > PI){
		angleAux = angleAux - 2 * PI;
	}

	return angleAux;
}




#ifdef BOGDANMODEL




//generates a sensor model from a basic model file
void generateSensorModel(double sensorModel[2*SENSOR_SIZE+1][2*SENSOR_SIZE+1], const char* basicModelFile, int antenna_number) {
	FILE* inputFile;
	inputFile = fopen(basicModelFile, "r");

	if (inputFile == NULL) {
		fprintf(stderr, "[generateSensorModel] ERROR: Could not open the file containing the base model (%s)\n",basicModelFile);
		int n = 2*SENSOR_SIZE+1;
		for (int i=0;i<n;++i) {
			for (int j=0;j<n;++j) {
				sensorModel[i][j] = PROBABILITY_OUTSIDE_MODEL;
			}
		}
		return;
 	}

	int n;
	double sensor_size_length;

	fscanf(inputFile,"%d",&n);
	fscanf(inputFile,"%lf",&sensor_size_length);

	double** auxSensorModel = (double**)malloc(n*sizeof(double*));
	for (int i=0;i<n;++i) {
		auxSensorModel[i] = (double*)malloc(n*sizeof(double));
	}

	for (int i=0;i<n;++i) {
		for (int j=0;j<n;++j) {
			fscanf(inputFile,"%lf",&(auxSensorModel[i][j]));
		}
	}

	fclose(inputFile);

	double xa = ANTENNA_POSITIONS[antenna_number][0];
	double ya = ANTENNA_POSITIONS[antenna_number][1];
	double ta = ANTENNA_POSITIONS[antenna_number][2];

	for (int i=0;i<2*SENSOR_SIZE+1;++i) {
		for (int j=0;j<2*SENSOR_SIZE+1;++j) {

			//first we pass from i,j in destination model to x,y in destination model
			double x_destmodel_initial, y_destmodel_initial;
			x_destmodel_initial = ( j - (SENSOR_SIZE))*SENSOR_SIZE_LENGTH/(double)SENSOR_SIZE;
			y_destmodel_initial = (-i + (SENSOR_SIZE))*SENSOR_SIZE_LENGTH/(double)SENSOR_SIZE;

			//next we account for center displacement in destination model
			double x_destmodel_centered,y_destmodel_centered;
			x_destmodel_centered = x_destmodel_initial - xa;
			y_destmodel_centered = y_destmodel_initial - ya;

			//now we are centered in the destination model; next we account for rotation
			double x_destmodel_centered_and_rotated,y_destmodel_centered_and_rotated;
			x_destmodel_centered_and_rotated = x_destmodel_centered * cos(-ta) - y_destmodel_centered * sin(-ta);
			y_destmodel_centered_and_rotated = x_destmodel_centered * sin(-ta) + y_destmodel_centered * cos(-ta);

			//now we pass to the source model without forgeting to account for scale variation
			double x_sourcemodel,y_sourcemodel;
			x_sourcemodel = x_destmodel_centered_and_rotated * sensor_size_length/(double)SENSOR_SIZE_LENGTH ;
			y_sourcemodel = y_destmodel_centered_and_rotated * sensor_size_length/(double)SENSOR_SIZE_LENGTH ;

			//finally we pass from x,y to i,j in the source
			int i_s, j_s;
			j_s = n/2 + (int)floor(x_sourcemodel*0.5*n/sensor_size_length);
			i_s = n/2 - (int)floor(y_sourcemodel*0.5*n/sensor_size_length);
			if (i_s >= n || i_s < 0 || j_s >= n || j_s < 0) {
					sensorModel[i][j] = PROBABILITY_OUTSIDE_MODEL;
			} else {
				sensorModel[i][j] = auxSensorModel[i_s][j_s];
			}
      /*
      printf("GSM[%d] (%2d,%2d)->(%5.3lf,%5.3lf)->(%5.3lf,%5.3lf)->(%5.3lf,%5.3lf)->(%5.3lf,%5.3lf)->(%2d,%2d)[%3.1lf]\n",
             antenna_number,
             i,j,
             x_destmodel_initial,y_destmodel_initial,
             x_destmodel_centered,y_destmodel_centered,
             x_destmodel_centered_and_rotated,y_destmodel_centered_and_rotated,
             x_sourcemodel,y_sourcemodel,
             i_s,j_s,
             (i_s >= n || i_s < 0 || j_s >= n || j_s < 0)?(9.9):(auxSensorModel[i_s][j_s]));
      */
		}
	}

	for (int i=0;i<n;++i) {
		free(auxSensorModel[i]);
	}
	free(auxSensorModel);

	return;
}

//generates the model for all the antennas
void initSensorModel(double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1], const char* basicModelFile) {

	printf("Starting generation of antenna models\n");

	char buffer[256];

	for(int i=0;i<NUMBER_OF_ANTENNAS;++i){

		generateSensorModel(sensorModel[i], basicModelFile, i);

		sprintf(buffer,"%s%d.in",basicModelFile,i);
		FILE* auxf = fopen(buffer,"w");
		fprintf(auxf,"%d %lf\n",2*SENSOR_SIZE-1,SENSOR_SIZE_LENGTH);
		for (int l=0;l<2*SENSOR_SIZE+1;++l) {
			for (int k=0;k<2*SENSOR_SIZE+1;++k) {
				fprintf(auxf,"%3.1lf ",sensorModel[i][l][k]);
			}
			fprintf(auxf,"\n");
		}
		fclose(auxf);
	}
  printf("Finished generation of antenna models\n");
}

//reduces real coordinates to line,column values in the sensor model; if outside model, i=j=-1
void reduceToRobotSensorSystem(	double robot_position_x, double robot_position_y, double robot_position_theta,
								double real_position_x, double real_position_y,int* i, int* j){

	double c = cos(robot_position_theta);
	double s = sin(robot_position_theta);
	double relative_position_x = real_position_x - robot_position_x;
	double relative_position_y = real_position_y - robot_position_y;
	double perceived_position_x = floor(( c * relative_position_x + s * relative_position_y)*SENSOR_SIZE / SENSOR_SIZE_LENGTH);
	double perceived_position_y = floor((-s * relative_position_x + c * relative_position_y)*SENSOR_SIZE / SENSOR_SIZE_LENGTH);

	*j = SENSOR_SIZE + (int)perceived_position_x;
	*i = SENSOR_SIZE - (int)perceived_position_y; //matrix line numbering is upsidedown

	if (	*i >  2*SENSOR_SIZE ||
			*i <              0 ||
			*j >  2*SENSOR_SIZE ||
			*j <              0
	) {
	*i = -1; //positions outside the sensor model
	*j = -1;
	}
}


//generates in the tagExpectations array the detections the robot receives in theory (the expected detections)
int getExpectedTagDetections(	TagExpectation** tagExpectations,
								double x, double y, double theta,
								double sensorModel[NUMBER_OF_ANTENNAS][2*SENSOR_SIZE+1][2*SENSOR_SIZE+1],
								Tag2DVector tags
							) {
	TagExpectation* before = NULL;
	TagExpectation* newTagExpectation = NULL;

	Tag2D currentPosition;
	currentPosition.x = x +
						SENSOR_SIZE*sqrt(2)*
											min( cos(theta+ M_PI/4),
											min( cos(theta+ M_PI/4+		M_PI/2),
											min( cos(theta+ M_PI/4+		M_PI),
							                     cos(theta+ M_PI/4+		3*M_PI/2))));

	currentPosition.y = y +
						SENSOR_SIZE*sqrt(2)*min(	sin(theta+	M_PI/4),
											min(	sin(theta+	M_PI/4+M_PI/2),
											min(	sin(theta+	M_PI/4+M_PI),
													sin(theta+	M_PI/4+3*M_PI/2))));

	int k = 0;
	int max_k = tags.size();
	int detectionCount = 0;

  /* This part is meant to not include in the testing the tags that are too far away, for efficiency; not tested yet so still commented
  while (k<max_k && Tag2Dcmp( tags[k], currentPosition )) ++k;

  currentPosition.x = x + SENSOR_SIZE*sqrt(2)*max(cos(theta+M_PI/4),max(cos(theta+M_PI/4+M_PI/2),max(cos(theta+M_PI/4+M_PI), cos(theta+M_PI/4+3*M_PI/2))));
  currentPosition.y = y + SENSOR_SIZE*sqrt(2)*max(sin(theta+M_PI/4),max(sin(theta+M_PI/4+M_PI/2),max(sin(theta+M_PI/4+M_PI), sin(theta+M_PI/4+3*M_PI/2))));
  while (k<max_k && Tag2Dcmp( tags[k], currentPosition )) {
  */
	while (k<max_k) {
		int i,j;
		reduceToRobotSensorSystem(x,y,theta,tags[k].x,tags[k].y,&i,&j);
		if (i != -1) {
			for (int antenna_number=0;antenna_number<NUMBER_OF_ANTENNAS;++antenna_number) {
				//TODO REMOVE IN FINAL VERSION (this is due to antenna 1 malfunctioning)
				if (antenna_number != 1) {

					if (sensorModel[antenna_number][i][j] > 0) {


						if(*tagExpectations == NULL){
							newTagExpectation = (TagExpectation*)malloc(sizeof(TagExpectation));
							*tagExpectations = newTagExpectation;
						}else{
							before = newTagExpectation;
							newTagExpectation = (TagExpectation*)malloc(sizeof(TagExpectation));
							before->next = newTagExpectation;
						}

						*newTagExpectation = TagExpectation(antenna_number,sensorModel[antenna_number][i][j],tags[k].tagid,NULL);

					}
				}else {
					//malfunction so do nothing
				}
			}
		}
		++k;
	}
	return detectionCount;
}




#else // #ifdef BOGDANMODEL




#ifdef GEOMETRICALMODEL


double probabilityModel(double distance,double angle){

	double p;

	if(distance < MINOR_RADIUS){
		p = PROB_INSIDE_LITTLE_CIRCLE;
	}else if((distance <= ARC_RADIUS) && (2*fabs(angle) <= ARC_ANGLE ) ){
		p = PROB_INSIDE_ARC;
	}else if(distance <= MAJOR_RADIUS){
		p = PROB_INSIDE_BIG_CIRCLE;
	}else{
		p = PROBABILITY_OUTSIDE_MODEL;
	}

	return p;
}



#elif defined FILEMODELCENTERED



int modelSize[2];
double* sensorModel = NULL;
int centerIndex;

// discretization(in meters) of the matrix representation
double cellStep;


//generates the model for all the antennas
void initSensorModel() {

	FILE* inputFile;
	inputFile = fopen(SENSOR_BASE_MODEL, "r");

	if (inputFile == NULL) {
		fprintf(stderr, "[generateSensorModel] ERROR: Could not open the file containing the base model (%s)\n",SENSOR_BASE_MODEL);
 	}else{


 		fscanf(inputFile,"%u",&modelSize[0]);
 		fscanf(inputFile,"%u",&modelSize[1]);
 		fscanf(inputFile,"%lf",&cellStep);

 		centerIndex = static_cast<int>(modelSize[1]*floor(modelSize[0]/2)+floor(modelSize[1]/2));

 		sensorModel = (double*)malloc(modelSize[0]*modelSize[1]*sizeof(double));

 		for(int i = 0; i < modelSize[0];i++){
 			for(int j = 0; j < modelSize[1];j++){
 				fscanf(inputFile,"%lf",&sensorModel[modelSize[1]*i+j]);
 			}
 		}
 	}

}


double probabilityModel(double distance,double angle){

	int x = (int)floor(distance*cos(angle)/cellStep);  // must be round instead of floor
	int y = (int)floor(distance*sin(angle)/cellStep);

	if((2*abs(x)>modelSize[1]) || (2*abs(y)>modelSize[0])){  // if outsideModel]
		return PROBABILITY_OUTSIDE_MODEL;
	}else{
		return sensorModel[centerIndex + y*modelSize[1]+x];
	}


}


#elif defined FILEMODELPOLAR


int modelSize[2];
double* sensorModel = NULL;  // this is really a matrix but represent in a lineal vector
// discretization(in meters) of the matrix representation
double stepDistance;
double stepRadius;


//generates the model for all the antennas
void initSensorModel() {

	FILE* inputFile;
	int scanReturn;
	char * pHome;
	char location[128];


	pHome = getenv("HOME");
	strcpy(location,pHome);
	strcat(location,SENSOR_BASE_MODEL);
	inputFile = fopen(location, "r");


	if (inputFile == NULL) {
		fprintf(stderr, "[generateSensorModel] ERROR: Could not open the file containing the base model (%s)\n",SENSOR_BASE_MODEL);
 	}else{

 		scanReturn = fscanf(inputFile,"%u",&modelSize[0]);
 		scanReturn = fscanf(inputFile,"%u",&modelSize[1]);
 		scanReturn = fscanf(inputFile,"%lf",&stepDistance);
 		scanReturn = fscanf(inputFile,"%lf",&stepRadius);

 		sensorModel = (double*)malloc(modelSize[0]*modelSize[1]*sizeof(double));

 		for(int i = 0; i < modelSize[0];i++){
 			for(int j = 0; j < modelSize[1];j++){
 				scanReturn = fscanf(inputFile,"%lf",&sensorModel[modelSize[1]*i+j]);
 			}
 		}
 	}

}



double probabilityModel(double distance,double angle){

	int d = (int)floor(distance/stepDistance);
	int a = (int)floor(fabs(angle)/stepRadius);

	// in case angle a == PI it will be always outside the model!
	if (fabs(angle) == PI){
		a = a - 1;
	}

	if(( (d+1)>modelSize[0]) || ((a+1)>modelSize[1]) ){  // if outsideModel]
		return PROBABILITY_OUTSIDE_MODEL;
	}else{
		return sensorModel[d*modelSize[1]+a];
	}


}





#else

#error "NO TYPE OF MODEL DEFINED"

#endif



//Represent the position of the tag relative to the robot in polar coordinates
void reduceToRobotSensorSystem(	double robot_position_x, double robot_position_y, double robot_position_theta,
								double tag_position_x, double tag_position_y,int antenna_number,double* distance, double* angle){


	// Antenna direction relative to -->  center:  robot position , direction: global x axis direction
	double thetaAntenna = ANTENNA_POSITION_ANGLES[antenna_number]+ robot_position_theta;


	//antenna absolute position
	double xa;
	double ya;

	xa = robot_position_x + ANTENNA_POSITION_RADIUS * cos(thetaAntenna);
	ya = robot_position_y + ANTENNA_POSITION_RADIUS * sin(thetaAntenna);

	*distance = sqrt( (tag_position_x-xa)*(tag_position_x-xa) + (tag_position_y-ya)*(tag_position_y-ya) );

	// Tag angle relative to --> center: antenna position, direction: global x axis direction
	double thetaTag = atan2((tag_position_y-ya),(tag_position_x-xa));

	*angle = angleWrap(thetaTag - thetaAntenna);

}


//Return every detectable tag with its detection likelihood for each antenna
int getExpectedTagDetections(	TagExpectation** tagsExpected,
								double x, double y, double theta,
								Tag2DVector tags
							){

	int tagSize = tags.size();
	double p;
	TagExpectation* newTagExpectation;
	TagExpectation* before;
	int numberCreated = 0;
	for(int t=0;t<tagSize;t++){
		for(int antenna_number = 0; antenna_number < NUMBER_OF_ANTENNAS;antenna_number++){

			double distance = 0;
			double angle = 0;

			reduceToRobotSensorSystem(x,y,theta,tags[t].x,tags[t].y,antenna_number,&distance,&angle);

			p = probabilityModel(distance,angle);

			if(p>0){

				if(*tagsExpected == NULL){
					newTagExpectation = (TagExpectation*)malloc(sizeof(TagExpectation));
					*tagsExpected = newTagExpectation;
				}else{
					before = newTagExpectation;
					newTagExpectation = (TagExpectation*)malloc(sizeof(TagExpectation));
					before->next = newTagExpectation;
				}

				*newTagExpectation = TagExpectation(antenna_number,p,tags[t].tagid,NULL);
				numberCreated = numberCreated+1;
			}
		}
	}
	return numberCreated;
}




#endif // #ifdef BOGDANMODEL


