#include "Main.h"


int main()
{

	/* ------ read wall file ------ */
	string FileName1 = "InputWall.txt";
	MatrixXd InputWall = ReadFile(FileName1);


	cout << endl << endl;
	/* ----- 1. set scanner location ----- */
	int ScanArea;
	do
	{
		cout << ">>> Please choose scanner location (1 or 2):" << endl << "    1.INDOOR 2.OUTDOOR" << "      ";
		cin >> ScanArea;
	} while (ScanArea != 1 && ScanArea != 2);
	cout << endl;


	/* ----- 2. get wall info: polygons, patches ----- */
	int WallTypes;
	do
	{
		cout << ">>> Please choose the type of walls to be scanned (1 or 2):" << endl << "    1.Exterior walls only   2.All walls (obstacles included)" << "      ";  // if or not scan the obstacle and independent walls and obstacles
		cin >> WallTypes;
	} while (WallTypes != 1 && WallTypes != 2);

	WallInfo WALLINFO = getWallInfo(InputWall, WallTypes);


	/* ----- 3. choose VP candidates creating strategy ----- */
	int HIER;
	do
	{
		cout << ">>> Please choose the strategy you want to use (1. Non-hierarchical 2. Hierarchical): ";
		cin >> HIER;
	} while (HIER != 1 && HIER != 2);



	/* ----- 4. get VP candidates  ----- */
	double iStep;
	cout << ">>> Please set an initial step length (m): ";
	cin >> iStep;

	MatrixXd VP = getCandidatePoints(WALLINFO, iStep, ScanArea);


	/* ----- 5. get score table  ----- */
	MatrixXd ScoreTab = getScoreTab(VP, WALLINFO);



	int GAType;
	do
	{
		cout << ">>> Please choose Greedy Algorithm you want to use (1. Original 2. Weighted): ";
		cin >> GAType;
	} while (GAType != 1 && GAType != 2);



	/* ----- 6. get non-hierarchical solution  ----- */
	clock_t t_0;
	t_0 = clock();

	double col_counter = WALLINFO.n_Cylindar + WALLINFO.n_Rectangle;   // the number of columns(pillars) edges in the walls segments, used for 3-edge scanning constraints
	Solution GA = GreedyAlg(ScoreTab, iStep, GAType, 0, VP, col_counter);
	MatrixXd BstVPs = GA.BstVPs;
	MatrixXd BstScoreTable = GA.BstScoreTable;
	MatrixXd VPgroups = GA.Groups;

	clock_t t_1;
	t_1 = clock();

	double dur;
	dur = (double)(t_1 - t_0);
	cout << "    Runtime_creating_GA_Solution (s): " << (dur / CLOCKS_PER_SEC) << endl << endl;




	/* ----- 7. get hierarchical solution  ----- */
	if (HIER == 2)
	{
		double Multiplier;
		cout << ">>> Please set a distance multiplier to group neighbouring VPs: ";
		cin >> Multiplier;

		cout << "    Step length (m): " << iStep << "   ";
		GA = GreedyAlg(ScoreTab, iStep, GAType, Multiplier, VP, col_counter);
		cout << GA.BstVPs.rows() << " optimal VPs were selected. " << endl;
		GA = goHierarchical(GA, WALLINFO, ScoreTab, VP, iStep, ScanArea, Multiplier, GAType, col_counter);
	}



	/* ----- wrtie results to files  ----- */
	//ofstream outfile_1("out_ScoreTable_initial.txt");
	//   outfile_1 << ScoreTab;

	ofstream outfile_2("out_Patches.txt");
	outfile_2 << WALLINFO.WallPatches;

	ofstream outfile_3("out_VP_Solution.txt");
	outfile_3 << GA.BstVPs;

	ofstream outfile_4("out_VPCandidates.txt");
	outfile_4 << VP;
	//outfile_444 << OldVP;

	ofstream outfile_5("out_ScoreTable_final.txt");
	outfile_5 << GA.BstScoreTable;




	/* ----- 8. get target candidates and visibility score table to the optimal VPs from last step ----- */
	string FileName2 = "out_VP_Solution.txt";
	BstVPs = ReadFile(FileName2);


	cout << ">>> Please choose step length (m) for targets candidates: ";
	int iStep_t;
	cin >> iStep_t;

	MatrixXd Targets_candidates = getCandidatePoints(WALLINFO, iStep_t, ScanArea);
	MatrixXd ScoreTab_t = MatrixXd::Zero(GA.BstVPs.rows(), Targets_candidates.rows());

	for (int i = 0; i < GA.BstVPs.rows(); i++)
	{
		Vector2d iVP = { GA.BstVPs(i,2), GA.BstVPs(i,3) };
		ScoreTab_t.row(i) = getVisibility_t(iVP, WALLINFO.ToScanWalls, Targets_candidates);
	}

	/* ----- wrtie results to files  ----- */
	ofstream outfile_6("out_ScoreTable_VPtoTargets.txt");
	outfile_6 << ScoreTab_t;
	ofstream outfile_7("out_TargetCandidates.txt");
	outfile_7 << Targets_candidates;






	system("pause");
	return 0;
}

/* ----- end of main ----- */
