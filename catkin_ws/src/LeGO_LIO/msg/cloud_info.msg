Header header 

int32[] startRingIndex
int32[] endRingIndex

float32 startOrientation
float32 endOrientation
float32 orientationDiff

bool[]    segmentedCloudGroundFlag # true - ground point, false - other points
uint32[]  segmentedCloudColInd # point column index in range image
float32[] segmentedCloudRange # point range 
# Robosense: absolute timestamp for each point in segmentedCloud, aligned by index.
# For other lidars this flag is false and featureAssociation uses the legacy angle-based time.
bool hasPointTimestamp
float64[] segmentedCloudTimestamp
