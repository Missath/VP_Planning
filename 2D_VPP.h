#include "3D_VPP.cpp"


typedef struct {
	MatrixXd InputWall;           // Walls read from the input file
	MatrixXd ToScanWalls;         // ToScanWalls (extend 2 columns with starting and ending patch id of each ToScanWall)
	MatrixXd WallPatches;         // info for descritized wall patches

	int n_MainWalls;               // number of main building exterior walls
	int n_Obstacles;              // number of obstacle walls
	int n_RestrictedArea;       // number of restricted area walls
	int n_IndependentWalls;          // number of independent walls
	int n_Cylindar;                // number of cylindar columns
	int n_Rectangle;                // number of rectangular columns

	MatrixXd Polygon_main;         //polygon made by the main building walls
	MatrixXd id_Polygon_Obstacles;         //start and end wall ids for each obstacle (used for polygon construction later)
	MatrixXd id_Polygon_RA;         //start and end wall ids for each restricted area (used for polygon construction later)


} WallInfo;

typedef struct {

	double x;
	double y;
	RowVectorXd ScoreTab;
	double IncAngSum;

} iVPInfo;

typedef struct {

	MatrixXd BstVPs;
	MatrixXd Groups;
	MatrixXd BstScoreTable;

} Solution;

typedef struct {

	BOOL On;
	BOOL In;

} PolygonCheck;


MatrixXd ReadFile(string FileName);
WallInfo getWallInfo(MatrixXd InputWall, int WallTypes);
WallInfo getWalls(MatrixXd InputWall);
WallInfo getWallPatches(MatrixXd wall, WallInfo WALLINFO);
MatrixXd getGrid(WallInfo WALLINFO, int ScanArea, double iStep);
MatrixXd getCandidatePoints(WallInfo WALLINFO, double iStep, int ScanArea);
MatrixXd cleanVPs(MatrixXd VP_, WallInfo WALLINFO);
MatrixXd getScoreTab(MatrixXd VP, WallInfo WALLINFO);
iVPInfo getVisibility(Vector2d VPi, WallInfo WALLINFO);
Solution GreedyAlg(MatrixXd ScoreTab, double CurrentStep, int GAType, double GroupDis, MatrixXd VP, double col_counter);
void CoverageTest(MatrixXd ST);
Solution goHierarchical(Solution GA, WallInfo WALLINFO, MatrixXd ScoreTab, MatrixXd VP, double iStep, int ScanArea, double Multiplier, int GAType, int Col_counter);
MatrixXd getGrid_Local(MatrixXd wall, MatrixXd Boundary, MatrixXd Polygon, int ScanArea, double iStep, double GroupDis);

RowVectorXd getVisibility_t(Vector2d VPi, MatrixXd All_Wall, MatrixXd Targets_candidates);

PolygonCheck PtInPolygon(Vector2d p, MatrixXd ptPolygon);
MatrixXd square2wall(MatrixXd square);
MatrixXd pillar2wall(MatrixXd pillar);
Vector3d LnFcn(VectorXd lnBeg, VectorXd lnEnd);
BOOL IfCross(Vector3d LnCoeff1, Vector3d LnCoeff2, Vector2d lnBeg1, Vector2d lnEnd1, Vector2d lnBeg2, Vector2d lnEnd2);  // used here
BOOL IfCrossWithCircle(Vector2d lnBeg, Vector2d lnEnd, Vector3d lnCoeff, Vector2d CircleCenter, double radius);

/* --- Alternative functions for line section cross check --- */
