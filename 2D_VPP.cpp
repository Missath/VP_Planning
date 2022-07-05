#include "2D_VPP.h"

/* ---------- Functions --------------- */

MatrixXd ReadFile(string FileName)
{


	ifstream infile(FileName);

	if (!infile)
	{
		cout << "Unable to open infile! " << endl << endl;
		system("pause");
	}


	char c;
	char before_c = ' ';
	int nRow = 0;
	int nCol = 0;

	while (infile.get(c))
	{

		if (c == '\n')
			nRow++;

		if (nRow == 0 && c == ' ' && before_c != ' ')
			nCol++;

		before_c = c;

	}

	nRow += 1;
	nCol += 1;

	infile.clear();
	infile.seekg(0, ios::beg); // back to the file beginning



	/* txt to matrix */
	MatrixXd file(nRow, nCol);

	for (int i = 0; i < nRow; i++)
	{
		for (int j = 0; j < nCol; j++)
		{
			infile >> file(i, j);
		}
	}

	return file;
}

WallInfo getWallInfo(MatrixXd InputWall, int WallTypes)
{
	cout << endl;
	WallInfo WALLINFO = getWalls(InputWall);
	//All_Walls All_WallInfo = getWalls(InputWall);  
	MatrixXd All_Wall = WALLINFO.ToScanWalls;

	int Counter_OutWalls = WALLINFO.n_MainWalls;   // Counter_OutWalls: the main building walls to be scanned
	int Counter_Obstacles = WALLINFO.n_Obstacles;
		
	int Counter_RAWalls = WALLINFO.n_RestrictedArea;   // Counter_OtherWalls_not2scan: indoor/outdoor restricted area that can not place VP but not obstacles the visibilities 

	//cout << Counter_OutWalls << "  " << Counter_Obstacles << endl;

	//Plolygon: main building walls
	MatrixXd outerLoopPolygon(Counter_OutWalls, 2);
	outerLoopPolygon = All_Wall.block(0, 1, Counter_OutWalls, 2);


	// number of obstacles/restricted areas
	int nObstaclePolygon = All_Wall.block(Counter_OutWalls, 0, Counter_Obstacles, 1).sum() / (1 + 2);  // number of obstacles
	int nRAPolygon = InputWall.block(Counter_OutWalls + Counter_Obstacles, 0, Counter_RAWalls, 1).sum() / (1 + 2);  // number of restricted areas


	// start and end wall id for each obstacles/restricted areas
	MatrixXd id_ObstaclePolygons(nObstaclePolygon, 2);
	MatrixXd id_RAPolygons(nRAPolygon, 2);

	int m = 0;
	for (int i = 0; i < Counter_Obstacles; i++)
	{
		if (InputWall(i, 0) == 1)
			id_ObstaclePolygons(m, 0) = i;
		if (InputWall(i, 0) == 2)
		{
			id_ObstaclePolygons(m, 1) = i;
			m++;
		}
	}

	m = 0;
	for (int i = Counter_OutWalls + Counter_Obstacles; i < Counter_OutWalls + Counter_Obstacles + Counter_RAWalls; i++)
	{
		if (InputWall(i, 0) == 1)
			id_RAPolygons(m, 0) = i;
		if (InputWall(i, 0) == 2)
		{
			id_RAPolygons(m, 1) = i;
			m++;
		}
	}

	WALLINFO.Polygon_main = outerLoopPolygon;
	WALLINFO.id_Polygon_Obstacles = id_ObstaclePolygons;
	WALLINFO.id_Polygon_RA = id_RAPolygons;




	/* ----- get discretized wall patches ----- */
	if (WallTypes == 1)
		WALLINFO = getWallPatches(All_Wall.topRows(Counter_OutWalls), WALLINFO);
	else if (WallTypes == 2)
		WALLINFO = getWallPatches(All_Wall, WALLINFO);
	// includes all the toScan wall segments




	return WALLINFO;
}

// Reorder the wall file to the walls to be scanned: Building wall - obstacles walls - independent walls - cylindar columns (4 edges) - rectangel columns(4 edges)
WallInfo getWalls(MatrixXd InputWall)
{
	int nWall = InputWall.rows();
	int Counter_Obstacles = 0, Counter_OutWalls = 0, Counter_InWalls_not2scan = 0, Counter_SeperateWalls = 0, Counter_Pillars = 0, Counter_Squares = 0;

	for (int i = 0; i < nWall; i++)
	{
		if (InputWall(i, 10) == 0)
			Counter_Obstacles++;  // obstacles

		else if (InputWall(i, 10) == 1)
			Counter_OutWalls++;   // building exterior walls (main walls)

		else if (InputWall(i, 10) == 2)
			Counter_InWalls_not2scan++;   // restricted area "walls"

		else if (InputWall(i, 10) == 3)
			Counter_SeperateWalls++;     // independent walls

		else if (InputWall(i, 10) == 4)
			Counter_Pillars++;           // cylindar columns

		else if (InputWall(i, 10) == 5)
			Counter_Squares++;           // rectangle columns

	}

	
	MatrixXd All_Wall(Counter_Obstacles + Counter_OutWalls + Counter_SeperateWalls + 4 * Counter_Pillars + 4 * Counter_Squares, InputWall.cols());
	int n = 0;

	for (int i = Counter_Obstacles; i < Counter_Obstacles + Counter_OutWalls; i++)
	{
		All_Wall.row(n) = InputWall.row(i);
		n++;
	}

	for (int i = 0; i < Counter_Obstacles; i++)
	{
		All_Wall.row(n) = InputWall.row(i);
		n++;
	}

	for (int i = Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan; i < Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan + Counter_SeperateWalls; i++)
	{
		All_Wall.row(n) = InputWall.row(i);
		n++;
	}

	for (int i = Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan + Counter_SeperateWalls; i < Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan + Counter_SeperateWalls + Counter_Pillars; i++)
	{
		All_Wall.middleRows(n, 4) = pillar2wall(InputWall.row(i));
		n = n + 4;
	}

	for (int i = Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan + Counter_SeperateWalls + Counter_Pillars; i < Counter_Obstacles + Counter_OutWalls + Counter_InWalls_not2scan + Counter_SeperateWalls + Counter_Pillars + Counter_Squares; i++)
	{
		All_Wall.middleRows(n, 4) = square2wall(InputWall.row(i));
		n = n + 4;
	}


	

	VectorXd typeCounters(6);
	typeCounters << Counter_OutWalls, Counter_Obstacles, Counter_InWalls_not2scan, Counter_SeperateWalls, Counter_Pillars, Counter_Squares;
	
	WallInfo WALLINFO;
	WALLINFO.InputWall = InputWall;
	WALLINFO.ToScanWalls = All_Wall;

	WALLINFO.n_MainWalls = Counter_OutWalls;
	WALLINFO.n_Obstacles = Counter_Obstacles;
	WALLINFO.n_RestrictedArea = Counter_InWalls_not2scan;
	WALLINFO.n_IndependentWalls = Counter_SeperateWalls;
	WALLINFO.n_Cylindar = Counter_Pillars;
	WALLINFO.n_Rectangle = Counter_Squares;

	return WALLINFO;
}

WallInfo getWallPatches(MatrixXd wall, WallInfo WALLINFO)
{

	cout << ">>> Please enter patch size (size of each wall patch): ";
	double Patch;
	cin >> Patch;

	int nWall = wall.rows();
	MatrixXd New_wall(wall.rows(), wall.cols() + 2);
	VectorXi nPatch(nWall, 1);

	for (int i = 0; i < nWall; i++)
	{
		New_wall.row(i).head(wall.cols()) = wall.row(i);
		Vector2d iwall = { New_wall(i, 4) - New_wall(i, 1), New_wall(i, 5) - New_wall(i, 2) };
		nPatch(i) = ceil(iwall.norm() / Patch);  //nPatch in each wall

	}

	MatrixXd WallPatches(nPatch.sum(), 6);


	int id = 0;

	for (int i = 0; i < nWall; i++)
	{

		VectorXd xPatch = VectorXd::LinSpaced(nPatch(i) + 1, New_wall(i, 1), New_wall(i, 4));
		VectorXd yPatch = VectorXd::LinSpaced(nPatch(i) + 1, New_wall(i, 2), New_wall(i, 5));
		//cout << xPatch2 << endl << yPatch2;

		New_wall(i, 11) = id; // start id for patches in wall i

		for (int j = 0; j < nPatch(i); j++)
		{

			Vector2d PatchBeg = { xPatch(j), yPatch(j) };
			Vector2d PatchEnd = { xPatch(j + 1), yPatch(j + 1) };;


			WallPatches(id, 0) = i;  // mother wall
			WallPatches(id, 1) = id;
			WallPatches(id, 2) = xPatch(j);
			WallPatches(id, 3) = yPatch(j);  // x,y for one vertex
			WallPatches(id, 4) = xPatch(j + 1);
			WallPatches(id, 5) = yPatch(j + 1);  // x,y for another vertex

			id++;
		}

		New_wall(i, 12) = id - 1; // end id for patches in wall i
	}

	WALLINFO.ToScanWalls = New_wall;  // extend input wall file with starting and ending patch ids
	WALLINFO.WallPatches = WallPatches;

	cout << "    " << id << " wall patches created." << endl << endl;

	return WALLINFO;

}

MatrixXd getCandidatePoints(WallInfo WALLINFO, double iStep, int ScanArea)
{
	clock_t t0;
	t0 = clock();
	double dur;


	// get discretized VPs within/out of the building wall
	MatrixXd VP_ = getGrid(WALLINFO, ScanArea, iStep);
	MatrixXd VP__ = cleanVPs(VP_, WALLINFO);   // remove VPs not in the free space (obstacles, restricted areas)
	cout << "    " << VP__.rows() << " candidate points are saved in the free space." << endl;
	
	clock_t t1;
	t1 = clock();
	dur = (double)(t1 - t0);
	cout << "    Runtime_creating_candidate_points (s): " << (dur / CLOCKS_PER_SEC) << endl << endl;

	return VP__;
}

MatrixXd getGrid(WallInfo WALLINFO, int ScanArea, double Step)
{
	MatrixXd wall = WALLINFO.ToScanWalls;
	MatrixXd Polygon = WALLINFO.Polygon_main;
	double dis_para = 1;   //distance threshold from VP to wall (no VPs if too close to the walls)


	switch (ScanArea)
	{

	case 1:
	{
		double minX = (wall.col(1)).minCoeff();
		double maxX = (wall.col(1)).maxCoeff();
		double minY = (wall.col(2)).minCoeff();
		double maxY = (wall.col(2)).maxCoeff();

		MatrixXd Boundary(2, 2);
		Boundary << minX, minY,
			maxX, maxY;

		int nX = int(floor((Boundary(1, 0) - Boundary(0, 0)) / Step));
		int nY = int(floor((Boundary(1, 1) - Boundary(0, 1)) / Step));

		MatrixXd iVP(nX * nY, 3); // descritized VPs

		int nOutOn = 0; // number of point out of and on the polygon
		int nClose = 0; // number of point close to the polygon
		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				// remove point out of and on the polygon
				iVP(j * nX + i, 0) = Boundary(0, 0) + (2 * i + 1) * Step / 2;
				iVP(j * nX + i, 1) = Boundary(0, 1) + (2 * j + 1) * Step / 2;
				iVP(j * nX + i, 2) = Step;


				// if PtInPolygon
				Vector2d p;
				p = { iVP(j * nX + i, 0) ,iVP(j * nX + i, 1) };

				PolygonCheck polygoncheck = PtInPolygon(p, Polygon);
				if (!polygoncheck.In)
				{
					iVP(j * nX + i, 0) = 52525;
					nOutOn++;
				}
				else
				{
					// remove points close to the walls
					bool isClose = 0;
					for (int j = 0; j < wall.rows(); j++)
					{
						if (wall(j, 10) == 0 || wall(j, 10) == 1 || wall(j, 10) == 3 || wall(j, 10) == 4 || wall(j, 10) == 5)
						{
							Vector2d jWallBeg = { wall(j,1),wall(j,2) };
							Vector2d jWallEnd = { wall(j,4),wall(j,5) };

							Vector3d jLnCoeff = LnFcn(jWallBeg, jWallEnd);

							BOOL close2wall = IfCrossWithCircle(jWallBeg, jWallEnd, jLnCoeff, p, dis_para); 
							if (close2wall)
							{
								isClose = 1;
								break;
							}
						}
					}
					if (isClose)
					{
						iVP(j * nX + i, 0) = 52525;
						nClose++;
					}
				}
			}
		}

		int nSaved = nX * nY - nOutOn - nClose;


		/* ----- save inpolygon and far to wall VPs ----- */
		MatrixXd VP(nSaved, 4);
		int m = 0;

		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				if (iVP(j * nX + i, 0) != 52525 && m < nSaved)
				{
					VP(m, 0) = iVP(j * nX + i, 0);
					VP(m, 1) = iVP(j * nX + i, 1);
					VP(m, 2) = iVP(j * nX + i, 2);
					VP(m, 3) = 0;  // Summation of incidence angle

					m++;
				}
			}
		}

		cout << "    " << nSaved << " candidate points are created in the chosen area." << endl;

		return VP;
	}


	case 2:
	{
		cout << ">>> Please decide the boundary for scanner placement (distance away from the building): ";
		double OuterBoundary;
		cin >> OuterBoundary;

		double minX = (wall.col(1)).minCoeff();
		double maxX = (wall.col(1)).maxCoeff();
		double minY = (wall.col(2)).minCoeff();
		double maxY = (wall.col(2)).maxCoeff();

		MatrixXd Boundary(2, 2);
		Boundary << minX, minY,
			maxX, maxY;

		int nX = int(floor(((Boundary(1, 0) + OuterBoundary) - (Boundary(0, 0) - OuterBoundary)) / Step));
		int nY = int(floor(((Boundary(1, 1) + OuterBoundary) - (Boundary(0, 1) - OuterBoundary)) / Step));

		// LPPOINT iVP = new POINT[nX * nY]; 
		MatrixXd iVP(nX * nY, 3);// descritized VPs


		int nInOn = 0; // number of point in and on polygon
		int nClose = 0; // number of point close to the polygon
		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				// remove point out of and on the polygon
				iVP(j * nX + i, 0) = Boundary(0, 0) - OuterBoundary + (2 * i + 1) * Step / 2;
				iVP(j * nX + i, 1) = Boundary(0, 1) - OuterBoundary + (2 * j + 1) * Step / 2;
				iVP(j * nX + i, 2) = Step;

				// if PtInPolygon
				Vector2d p;
				p = { iVP(j * nX + i, 0) ,iVP(j * nX + i, 1) };

				PolygonCheck polygoncheck = PtInPolygon(p, Polygon);

				if (polygoncheck.In || polygoncheck.On)
				{
					iVP(j * nX + i, 0) = 52525;
					nInOn++;
				}
				else
				{
					// remove points close to the polygon
					bool isClose = 0;
					for (int j = 0; j < wall.rows(); j++)
					{
						if (wall(j, 10) == 0 || wall(j, 10) == 1 || wall(j, 10) == 3 || wall(j, 10) == 4 || wall(j, 10) == 5)
						{
							Vector2d jWallBeg = { wall(j,1),wall(j,2) };
							Vector2d jWallEnd = { wall(j,4),wall(j,5) };

							Vector3d jLnCoeff = LnFcn(jWallBeg, jWallEnd);
							BOOL close2wall = IfCrossWithCircle(jWallBeg, jWallEnd, jLnCoeff, p, dis_para);
							if (close2wall)
							{
								isClose = 1;
								break;
							}
						}
					}
					if (isClose)
					{
						iVP(j * nX + i, 0) = 52525;
						nClose++;
					}
				}
			}
		}



		/* ----- save out of polygon VPs ----- */

		int nSaved = nX * nY - nInOn - nClose;

		MatrixXd VP(nSaved, 4);
		int m = 0;

		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				if (iVP(j * nX + i, 0) != 52525 && m < nSaved)
				{
					VP(m, 0) = iVP(j * nX + i, 0);
					VP(m, 1) = iVP(j * nX + i, 1);
					VP(m, 2) = iVP(j * nX + i, 2);
					VP(m, 3) = 0; // Summation of incidence angle

					m++;
				}
			}
		}


		cout << "    " << nSaved << " candidate points are created in the chosen area." << endl;

		return VP;
	}

	}

}

MatrixXd cleanVPs(MatrixXd VP_, WallInfo WALLINFO)
{
	int nInOn = 0;
	// remove VPs in the obstacles
	for (int j = 0; j < WALLINFO.n_Obstacles; j++)
	{
		int size_polygon = WALLINFO.id_Polygon_Obstacles(j, 1) - WALLINFO.id_Polygon_Obstacles(j, 0) + 1;  // number of walls for this obstacle polygon
		MatrixXd ObstaclePolygon(size_polygon, 2);

		// save VPs out of the obstacle polygon
		int m = 0;
		ObstaclePolygon = WALLINFO.InputWall.block(WALLINFO.id_Polygon_Obstacles(j, 0), 1, size_polygon, 2);


		for (int i = 0; i < VP_.rows(); i++)
		{
			Vector2d p;
			p = { VP_(i, 0) ,VP_(i, 1) };
			PolygonCheck polygoncheck = PtInPolygon(p, ObstaclePolygon);
			if (polygoncheck.In || polygoncheck.On)
			{
				VP_(i, 0) = 52525;
				nInOn++;
			}

		}
	}

	// remove VPs in the restricted area (RA)
	for (int j = 0; j < WALLINFO.n_RestrictedArea; j++)
	{
		int size_polygon = WALLINFO.id_Polygon_RA(j, 1) - WALLINFO.id_Polygon_RA(j, 0) + 1;   // number of walls for this RA polygon
		MatrixXd RA_Polygon(size_polygon, 2);

		/// save VPs out of restricted area polygon
		int m = 0;
		RA_Polygon = WALLINFO.InputWall.block(WALLINFO.id_Polygon_RA(j, 0), 1, size_polygon, 2);


		for (int i = 0; i < VP_.rows(); i++)
		{
			Vector2d p;
			p = { VP_(i, 0) ,VP_(i, 1) };
			PolygonCheck polygoncheck = PtInPolygon(p, RA_Polygon);
			if (polygoncheck.In || polygoncheck.On)
			{
				VP_(i, 0) = 52525;
				nInOn++;
			}

		}
	}


	/* ----- save all VPs in the free space ----- */
	int nSave = VP_.rows() - nInOn;
	MatrixXd VP__(nSave, 4);
	int m = 0;
	for (int i = 0; i < VP_.rows(); i++)
	{
		if (VP_(i, 0) != 52525 && m < nSave)
		{
			VP__.row(m) = VP_.row(i);
			m++;
		}
	}

	return VP__;
}

MatrixXd getScoreTab(MatrixXd VP, WallInfo WALLINFO)
{
	clock_t t0;
	t0 = clock();

	MatrixXd ScoreTab = MatrixXd::Zero(VP.rows(), WALLINFO.WallPatches.rows());

	iVPInfo IVP;
	for (int i = 0; i < VP.rows(); i++)
	{
		Vector2d iVP = { VP(i,0),VP(i,1) };

		IVP = getVisibility(iVP, WALLINFO);
		VP(i, 3) = IVP.IncAngSum;  // currently not used
		ScoreTab.row(i) = IVP.ScoreTab;
	}

	cout << "    A " << VP.rows() << "*" << WALLINFO.WallPatches.rows() << " Score Table was created. ";

	clock_t t1;
	t1 = clock();
	double dur = (double)(t1 - t0);
	cout << endl << "    Runtime_creating_Score_Table (s): " << (dur / CLOCKS_PER_SEC) << endl << endl;

	CoverageTest(ScoreTab);

	return ScoreTab;
}

iVPInfo getVisibility(Vector2d VPi, WallInfo WALLINFO)
{

	MatrixXd WallPatches = WALLINFO.WallPatches;
	MatrixXd wall = WALLINFO.ToScanWalls;
	Vector2d VP__ = VPi;

	RowVectorXd nScoreTab = RowVectorXd::Zero(WallPatches.rows()); // Score table for VP n

	// default parameters
	double MaxInc = 65;   // incidence angle
	double MaxRng = 100;  // range
	double MinRng = 1;

	double IncAngSum = 0;
	int nWall = wall.rows();


	for (int j = 0; j < WallPatches.rows(); j++)
	{

		///* Condition 1: Visibility */  // Most time-consuming part! 

		Vector2d jWallPatchesBeg = { WallPatches(j,2),WallPatches(j,3) };
		Vector2d jWallPatchesEnd = { WallPatches(j,4),WallPatches(j,5) };

		Vector3d jLnBegCoeff = LnFcn(VP__, jWallPatchesBeg);
		Vector3d jLnEndCoeff = LnFcn(VP__, jWallPatchesEnd);

		bool inVisible = 0;  // if a patch is visible form VPn
		for (int m = 0; m < nWall; m++)
		{
			if (wall(m, 10) == 0 || wall(m, 10) == 1 || wall(m, 10) == 3 || wall(m, 10) == 4 || wall(m, 10) == 5)
			{
				Vector2d mWallBeg = { wall(m,1),wall(m,2) };
				Vector2d mWallEnd = { wall(m,4),wall(m,5) };

				Vector3d mLnCoeff = LnFcn(mWallBeg, mWallEnd);

				BOOL crossBeg = IfCross(jLnBegCoeff, mLnCoeff, VP__, jWallPatchesBeg, mWallBeg, mWallEnd);
				BOOL crossEnd = IfCross(jLnEndCoeff, mLnCoeff, VP__, jWallPatchesEnd, mWallBeg, mWallEnd);
				//cout << "crossBeg: " << m << "  " << crossBeg << endl;

				if (crossBeg == 1 || crossEnd == 1)
				{
					inVisible = 1;
					//break;   // If incidence ray cross with any wall, this patch is unvisible, break all other walls
				}
			}
			else if (wall(m, 10) == 4)
			{
				Vector2d CircleCenter = { wall(m, 1),wall(m, 2) };
				BOOL CirclecrossBeg = IfCrossWithCircle(VP__, jWallPatchesBeg, jLnBegCoeff, CircleCenter, wall(m, 4));
				BOOL CirclecrossEnd = IfCrossWithCircle(VP__, jWallPatchesEnd, jLnEndCoeff, CircleCenter, wall(m, 4));

				if (CirclecrossBeg == 1 || CirclecrossEnd == 1)
				{
					inVisible = 1;
					break;   // If incidence ray cross with any wall, this patch is unvisible, break all other walls
				}

			}
		}
		if (inVisible)
			continue;   // If incidence ray cross with any wall, this patch is unvisible, continue to the next patch


		/* Condition 2: Range */
		Vector2d jIncRayBeg = VP__ - jWallPatchesBeg;  // Incidence ray for patch j
		//Vector2d jIncRayEnd = VP__ - jWallPatchesEnd;

		double jRangeBeg = jIncRayBeg.norm();
		//double jRangeEnd = jIncRayEnd.norm();


		if (jRangeBeg > MaxRng || jRangeBeg < MinRng)
			//if (jRangeBeg > MaxRng || jRangeBeg < MinRng || jRangeEnd > MaxRng || jRangeEnd < MinRng)
			continue;



		/* Condition 3: Incidence Angle */
		Vector2d nn = jWallPatchesEnd - jWallPatchesBeg;
		Vector2d NormVec = { nn(0), nn(1) };
		double jIncAngBeg = acos(jIncRayBeg.dot(NormVec) / (jIncRayBeg.norm() * NormVec.norm())) * 180 / pi;  // jIncAng = [0,180]
		//double jIncAngEnd = abs(acos(jIncRayEnd.dot(NormVec) / (jIncRayEnd.norm() * NormVec.norm())) * 180 / pi);

		double jInc = abs(jIncAngBeg - 90);


		if (jInc > MaxInc)
			continue;



		/* --- All pass, visible ! --- */

		nScoreTab(j) = 1;
	}


	iVPInfo IVP;
	IVP.x = VP__(0);
	IVP.y = VP__(1);
	IVP.ScoreTab = nScoreTab;
	IVP.IncAngSum = IncAngSum;

	return IVP;

}

// Get optimal VPs and group the selected VPs based on the distance
Solution GreedyAlg(MatrixXd ScoreTab, double CurrentStep, int GAType, double Buffer, MatrixXd VP, double col_counter)
{
	double OverlapRate = 0;

	switch (GAType)
	{
	case 1:
	{

		// Count the number of VPs in solution
		MatrixXd ST_toCheck = ScoreTab;
		MatrixXd ST_toSum = ScoreTab;
		MatrixXd WallCoverage_ST;
		MatrixXd ColCoverage_ST;
		bool WallCoverageCheck = 0;  //fail
		bool ColCoverageCheck = 1;   //fail

		int counter = 0;  // saved VPs in GA solutions
		while (!WallCoverageCheck || !ColCoverageCheck)
		{
			if (ST_toSum.sum() == 0)
				ST_toSum = ST_toCheck;

			VectorXd Score = ST_toSum.rowwise().sum();
			Index maxIndex;
			double maxVal = Score.maxCoeff(&maxIndex);

			ST_toCheck.row(maxIndex) = MatrixXd::Zero(1, ST_toCheck.cols());

			MatrixXd rep_ST_toSum(ST_toSum.rows(), ST_toSum.cols());
			for (int i = 0; i < ST_toSum.rows(); i++)
				rep_ST_toSum.row(i) = ST_toSum.row(maxIndex);

			//MatrixXd ST_toCheck_Filtered = rep_ST_toSum.cwiseProduct(ST_toCheck);
			double FilterThreshold = OverlapRate * ScoreTab.row(maxIndex).sum();
			//VectorXd Score_Filtered = ST_toCheck_Filtered.rowwise().sum();
			VectorXd Score_Filtered = ST_toCheck * ST_toSum.row(maxIndex).transpose();

			for (int i = 0; i < ST_toCheck.cols(); i++)
			{
				if (ST_toSum(maxIndex, i) == 1)
					ST_toCheck.col(i) = MatrixXd::Zero(ST_toCheck.rows(), 1);
			}
			counter++;

			MatrixXd ST_toCheck_Sum = ST_toCheck.colwise().sum();
			WallCoverage_ST = ST_toCheck_Sum.leftCols(ST_toCheck.cols() - 4 * col_counter);


			if (WallCoverage_ST.sum() == 0)
				WallCoverageCheck = 1;

			// // Check if at least 3-edges of each col are scanned
			ColCoverage_ST = ST_toCheck_Sum.rightCols(4 * col_counter);
			ColCoverageCheck = 1;
			for (int i = 0; i < 4 * col_counter; i = i + 4)
			{
				MatrixXd i_ColCoverage = ColCoverage_ST.middleCols(i, 4);
				//cout << i_ColCoverage << endl;
				int i_counter = 0;
				for (int j = 0; j < 4; j++)
				{
					if (i_ColCoverage(0, j) > 0)
						i_counter++;
				}
				if (i_counter < 2)  // less than 2 edges
				{
					ColCoverageCheck = 0;
					break;
				}
			}


			///// ***  process for overlap constraints
			int counterTmp = 0;
			double OverlapRate_tmp = OverlapRate;
			while (counterTmp == 0)
			{
				for (int i = 0; i < Score_Filtered.rows(); i++)
					if (Score_Filtered(i) >= FilterThreshold)
					{
						counterTmp++;
						break;
					}

				if (counterTmp == 0)
				{
					OverlapRate_tmp = OverlapRate_tmp - 0.05;
					if (OverlapRate_tmp < 0)
						OverlapRate_tmp = 0;
					FilterThreshold = OverlapRate_tmp * ScoreTab.row(maxIndex).sum();
				}

			}

			ST_toSum = 0 * ST_toSum;
			for (int i = 0; i < Score_Filtered.rows(); i++)
				if (Score_Filtered(i) >= FilterThreshold)
					ST_toSum.row(i) = ST_toCheck.row(i);
		}

		// Get solution
		ST_toCheck = ScoreTab;
		ST_toSum = ScoreTab;
		MatrixXd BstVPs(counter, 7);  //id-MaxIndex-x-y-densify_or_not-groupID-step
		MatrixXd BstScoreTable(counter, ScoreTab.cols());
		int group = 1;

		for (int j = 0; j < counter; j++)
		{
			if (ST_toSum.sum() == 0)
				ST_toSum = ST_toCheck;

			VectorXd Score = ST_toSum.rowwise().sum();
			Index maxIndex;
			double maxVal = Score.maxCoeff(&maxIndex);


			ST_toCheck.row(maxIndex) = MatrixXd::Zero(1, ST_toCheck.cols());


			//MatrixXd ST_toCheck_Filtered = rep_ST_toSum.cwiseProduct(ST_toCheck);
			double FilterThreshold = OverlapRate * ScoreTab.row(maxIndex).sum();
			//VectorXd Score_Filtered = ST_toCheck_Filtered.rowwise().sum();
			VectorXd Score_Filtered = ST_toCheck * ST_toSum.row(maxIndex).transpose();


			for (int i = 0; i < ST_toCheck.cols(); i++)
			{
				if (ST_toSum(maxIndex, i) == 1)
					ST_toCheck.col(i) = MatrixXd::Zero(ST_toCheck.rows(), 1);
			}



			int counterTmp = 0;
			double OverlapRate_tmp = OverlapRate;
			while (counterTmp == 0)
			{
				for (int i = 0; i < Score_Filtered.rows(); i++)
					if (Score_Filtered(i) >= FilterThreshold)
					{
						counterTmp++;
						break;
					}

				if (counterTmp == 0)
				{
					OverlapRate_tmp = OverlapRate_tmp - 0.05;
					if (OverlapRate_tmp < 0)
						OverlapRate_tmp = 0;
					FilterThreshold = OverlapRate_tmp * ScoreTab.row(maxIndex).sum();
				}

			}

			ST_toSum = 0 * ST_toSum;
			for (int i = 0; i < Score_Filtered.rows(); i++)
				if (Score_Filtered(i) >= FilterThreshold)
					ST_toSum.row(i) = ST_toCheck.row(i);


			BstScoreTable.row(j) = ScoreTab.row(maxIndex);
			BstVPs(j, 0) = j;
			BstVPs(j, 1) = maxIndex;
			BstVPs(j, 2) = VP(maxIndex, 0);
			BstVPs(j, 3) = VP(maxIndex, 1);
			BstVPs(j, 6) = VP(maxIndex, 2);

			// get VPgroup: group VPs by distance
			if (j == 0)
			{
				BstVPs(j, 4) = 0;
				BstVPs(j, 5) = group;
			}
			else
			{
				Vector2d p_j = { BstVPs(j, 2), BstVPs(j, 3) };
				bool isbreak = 0;

				for (int i = 0; i < j; i++)
				{

					Vector2d p_i = { BstVPs(i, 2), BstVPs(i, 3) };
					Vector2d v_ij = p_i - p_j;

					if (v_ij.norm() < Buffer * CurrentStep)
					{
						BstVPs(j, 4) = 1;
						BstVPs(i, 4) = 1;
						BstVPs(j, 5) = BstVPs(i, 5);

						isbreak = 1;
						break;
					}

				}

				if (!isbreak)
				{
					BstVPs(j, 4) = 0;
					BstVPs(j, 5) = ++group;

				}
			}

		}


		MatrixXd VPgroups = 52525 * MatrixXd::Ones(group, 9); //id - number of points in group - minX - minY - maxX - maxY - if_to_be_densified - idx - resolution

		for (int j = 0; j < group; j++)
		{
			VPgroups(j, 0) = j + 1;
			VPgroups(j, 1) = 0;


			for (int i = 0; i < counter; i++)
			{
				if (BstVPs(i, 5) == j + 1)
				{

					VPgroups(j, 1) = VPgroups(j, 1) + 1;

					if (VPgroups(j, 2) == 52525)
					{
						VPgroups(j, 2) = BstVPs(i, 2);
						VPgroups(j, 3) = BstVPs(i, 3);
						VPgroups(j, 4) = BstVPs(i, 2);
						VPgroups(j, 5) = BstVPs(i, 3);
					}
					else
					{
						if (BstVPs(i, 2) < VPgroups(j, 2))
							VPgroups(j, 2) = BstVPs(i, 2);

						if (BstVPs(i, 3) < VPgroups(j, 3))
							VPgroups(j, 3) = BstVPs(i, 3);

						if (BstVPs(i, 2) > VPgroups(j, 4))
							VPgroups(j, 4) = BstVPs(i, 2);

						if (BstVPs(i, 3) > VPgroups(j, 5))
							VPgroups(j, 5) = BstVPs(i, 3);
					}

					VPgroups(j, 7) = BstVPs(i, 1);
				}
			}

			if (VPgroups(j, 1) > 1)
				VPgroups(j, 6) = 1;  // more than 1 point in group, this group needs to be densified

			else
				VPgroups(j, 6) = 0;

			VPgroups(j, 8) = VP(VPgroups(j, 7), 2);   //current resolution (step length)
		}

		Solution OGA;
		OGA.BstVPs = BstVPs;
		OGA.Groups = VPgroups;
		OGA.BstScoreTable = BstScoreTable;
		return OGA;
	}

	case 2:
	{

		MatrixXd temp_ST(ScoreTab.rows(), ScoreTab.cols());
		for (int i = 0; i < ScoreTab.rows(); i++)
		{
			temp_ST.row(i) = ScoreTab.colwise().sum();
		}
		MatrixXd WST_toSum0 = ScoreTab.cwiseQuotient(temp_ST);

		MatrixXd WST_toSum = WST_toSum0;
		MatrixXd ST_toCheck = ScoreTab;
		VectorXd ScannedTimes = ST_toCheck.colwise().sum();
		MatrixXd WallCoverage_ST;
		MatrixXd ColCoverage_ST;
		bool WallCoverageCheck = 0;  //fail
		bool ColCoverageCheck = 0;   //fail

		int counter = 0;
		while (!WallCoverageCheck || !ColCoverageCheck)
		{
			if (WST_toSum.sum() == 0)
			{
				// To get weighted score table: WST_toSum
				ScannedTimes = ST_toCheck.colwise().sum();
				for (int i = 0; i < ST_toCheck.rows(); i++)
				{
					for (int j = 0; j < ST_toCheck.cols(); j++)
					{
						if (ST_toCheck(i, j) == 1)
							WST_toSum(i, j) = ST_toCheck(i, j) / ScannedTimes(j);
						else
							WST_toSum(i, j) = ST_toCheck(i, j);
					}
				}
			}

			VectorXd BinaryScore = ScoreTab.rowwise().sum();
			VectorXd WeightedScore = WST_toSum.rowwise().sum();
			Index maxIndex;
			double maxVal = WeightedScore.maxCoeff(&maxIndex);

			MatrixXd rep_ST_toCheck(ST_toCheck.rows(), ST_toCheck.cols());
			for (int i = 0; i < ST_toCheck.rows(); i++)
				rep_ST_toCheck.row(i) = ST_toCheck.row(maxIndex);

			MatrixXd ST_toCheck_Filtered = rep_ST_toCheck.cwiseProduct(ST_toCheck);
			double FilterThreshold = OverlapRate * ScoreTab.row(maxIndex).sum();
			VectorXd Score_Filtered = ST_toCheck_Filtered.rowwise().sum();
			Score_Filtered(maxIndex) = 0;

			for (int i = 0; i < ST_toCheck.cols(); i++)
			{
				if (ST_toCheck(maxIndex, i) == 1)
					ST_toCheck.col(i) = MatrixXd::Zero(ST_toCheck.rows(), 1);
			}
			counter++;


			MatrixXd ST_toCheck_Sum = ST_toCheck.colwise().sum();
			// Check if cols are at least 3-edge scanned
			WallCoverage_ST = ST_toCheck_Sum.leftCols(ST_toCheck.cols() - 4 * col_counter);
			if (WallCoverage_ST.sum() == 0)
				WallCoverageCheck = 1;

			ColCoverage_ST = ST_toCheck_Sum.rightCols(4 * col_counter);
			ColCoverageCheck = 1;
			for (int i = 0; i < 4 * col_counter; i = i + 4)
			{
				MatrixXd i_ColCoverage = ColCoverage_ST.middleCols(i, 4);
				//cout << i_ColCoverage << endl;
				int i_counter = 0;
				for (int j = 0; j < 4; j++)
				{
					if (i_ColCoverage(0, j) > 0)
						i_counter++;
				}
				if (i_counter > 1)
				{
					ColCoverageCheck = 0;
					break;
				}
			}

			int counterTmp = 0;
			double OverlapRate_tmp = OverlapRate;
			while (counterTmp == 0)
			{
				for (int i = 0; i < Score_Filtered.rows(); i++)
					if (Score_Filtered(i) >= FilterThreshold)
					{
						counterTmp++;
						break;
					}

				if (counterTmp == 0)
				{
					OverlapRate_tmp = OverlapRate_tmp - 0.05;
					if (OverlapRate_tmp < 0)
						OverlapRate_tmp = 0;
					FilterThreshold = OverlapRate_tmp * ScoreTab.row(maxIndex).sum();
				}

			}

			WST_toSum = 0 * WST_toSum;
			for (int i = 0; i < Score_Filtered.rows(); i++)
				if (Score_Filtered(i) >= FilterThreshold)
					WST_toSum.row(i) = ST_toCheck.row(i);

			// To get weighted score table: WST_toSum
			ScannedTimes = WST_toSum.colwise().sum(); // two ways to calculate scannedtimes (using filtered VPs)
													  //ScannedTimes = ST_toCheck.colwise().sum();   // (using all VPs)
			for (int i = 0; i < ST_toCheck.rows(); i++)
			{
				for (int j = 0; j < ST_toCheck.cols(); j++)
				{
					if (WST_toSum(i, j) == 1)
						WST_toSum(i, j) = ST_toCheck(i, j) / ScannedTimes(j);

				}
			}

		}

		// get WGA solution
		WST_toSum = WST_toSum0;
		ST_toCheck = ScoreTab;
		ScannedTimes = ST_toCheck.colwise().sum();
		MatrixXd BstVPs(counter, 7); //id-MaxIndex-x-y-zoomornot-groupID-step
		MatrixXd BstScoreTable(counter, ScoreTab.cols());
		int group = 1;

		for (int m = 0; m < counter; m++)
		{
			if (WST_toSum.sum() == 0)
			{
				// To get weighted score table: WST_toSum
				ScannedTimes = ST_toCheck.colwise().sum();
				for (int i = 0; i < ST_toCheck.rows(); i++)
				{
					for (int j = 0; j < ST_toCheck.cols(); j++)
					{
						if (ST_toCheck(i, j) == 1)
							WST_toSum(i, j) = ST_toCheck(i, j) / ScannedTimes(j);
						else
							WST_toSum(i, j) = ST_toCheck(i, j);
					}
				}
			}

			VectorXd BinaryScore = ScoreTab.rowwise().sum();
			VectorXd WeightedScore = WST_toSum.rowwise().sum();
			Index maxIndex;
			double maxVal = WeightedScore.maxCoeff(&maxIndex);

			MatrixXd rep_ST_toCheck(ST_toCheck.rows(), ST_toCheck.cols());
			for (int i = 0; i < ST_toCheck.rows(); i++)
				rep_ST_toCheck.row(i) = ST_toCheck.row(maxIndex);

			MatrixXd ST_toCheck_Filtered = rep_ST_toCheck.cwiseProduct(ST_toCheck);
			double FilterThreshold = OverlapRate * ScoreTab.row(maxIndex).sum();
			VectorXd Score_Filtered = ST_toCheck_Filtered.rowwise().sum();
			Score_Filtered(maxIndex) = 0;

			for (int i = 0; i < ST_toCheck.cols(); i++)
			{
				if (ST_toCheck(maxIndex, i) == 1)
					ST_toCheck.col(i) = MatrixXd::Zero(ST_toCheck.rows(), 1);
			}


			int counterTmp = 0;
			double OverlapRate_tmp = OverlapRate;
			while (counterTmp == 0)
			{
				for (int i = 0; i < Score_Filtered.rows(); i++)
					if (Score_Filtered(i) >= FilterThreshold)
					{
						counterTmp++;
						break;
					}

				if (counterTmp == 0)
				{
					OverlapRate_tmp = OverlapRate_tmp - 0.05;
					if (OverlapRate_tmp < 0)
						OverlapRate_tmp = 0;
					FilterThreshold = OverlapRate_tmp * ScoreTab.row(maxIndex).sum();
				}

			}

			WST_toSum = 0 * WST_toSum;
			for (int i = 0; i < Score_Filtered.rows(); i++)
				if (Score_Filtered(i) >= FilterThreshold)
					WST_toSum.row(i) = ST_toCheck.row(i);

			// To get weighted score table: WST_toSum
			ScannedTimes = WST_toSum.colwise().sum(); // two ways to calculate scannedtimes (using filtered VPs)
													  //ScannedTimes = ST_toCheck.colwise().sum();   // (using all VPs)
			for (int i = 0; i < ST_toCheck.rows(); i++)
			{
				for (int j = 0; j < ST_toCheck.cols(); j++)
				{
					if (WST_toSum(i, j) == 1)
						WST_toSum(i, j) = ST_toCheck(i, j) / ScannedTimes(j);

				}
			}



			BstScoreTable.row(m) = ScoreTab.row(maxIndex);
			BstVPs(m, 0) = m;
			BstVPs(m, 1) = maxIndex;
			BstVPs(m, 2) = VP(maxIndex, 0);
			BstVPs(m, 3) = VP(maxIndex, 1);
			BstVPs(m, 6) = VP(maxIndex, 2);

			if (m == 0)
			{
				BstVPs(m, 4) = 0;
				BstVPs(m, 5) = group;
			}
			else
			{
				Vector2d p_m = { BstVPs(m, 2), BstVPs(m, 3) };
				bool isbreak = 0;

				for (int n = 0; n < m; n++)
				{

					Vector2d p_n = { BstVPs(n, 2), BstVPs(n, 3) };
					Vector2d v_mn = p_m - p_n;

					if (v_mn.norm() < Buffer * CurrentStep)
					{
						BstVPs(m, 4) = 1;
						BstVPs(n, 4) = 1;
						BstVPs(m, 5) = BstVPs(n, 5);


						isbreak = 1;
						break;
					}

				}

				if (!isbreak)
				{
					BstVPs(m, 4) = 0;
					BstVPs(m, 5) = ++group;

				}
			}
		}

		MatrixXd VPgroups = 52525 * MatrixXd::Ones(group, 9); //id - number of points in group - minX - minY - maxX - maxY - zoom - idx - resolution

		for (int j = 0; j < group; j++)
		{
			VPgroups(j, 0) = j + 1;
			VPgroups(j, 1) = 0;


			for (int i = 0; i < counter; i++)
			{
				if (BstVPs(i, 5) == j + 1)
				{

					VPgroups(j, 1) = VPgroups(j, 1) + 1;

					if (VPgroups(j, 2) == 52525)
					{
						VPgroups(j, 2) = BstVPs(i, 2);
						VPgroups(j, 3) = BstVPs(i, 3);
						VPgroups(j, 4) = BstVPs(i, 2);
						VPgroups(j, 5) = BstVPs(i, 3);
					}
					else
					{
						if (BstVPs(i, 2) < VPgroups(j, 2))
							VPgroups(j, 2) = BstVPs(i, 2);

						if (BstVPs(i, 3) < VPgroups(j, 3))
							VPgroups(j, 3) = BstVPs(i, 3);

						if (BstVPs(i, 2) > VPgroups(j, 4))
							VPgroups(j, 4) = BstVPs(i, 2);

						if (BstVPs(i, 3) > VPgroups(j, 5))
							VPgroups(j, 5) = BstVPs(i, 3);
					}

					VPgroups(j, 7) = BstVPs(i, 1);
				}
			}

			if (VPgroups(j, 1) > 1)
				VPgroups(j, 6) = 1;

			else
				VPgroups(j, 6) = 0;

			VPgroups(j, 8) = VP(VPgroups(j, 7), 2);

		}

		Solution WGA;
		WGA.BstVPs = BstVPs;
		WGA.Groups = VPgroups;
		WGA.BstScoreTable = BstScoreTable;
		return WGA;
	}
	}

}

Solution goHierarchical(Solution GA, WallInfo WALLINFO, MatrixXd ScoreTab, MatrixXd VP, double iStep, int ScanArea, double Multiplier, int GAType, int col_counter)
{
	clock_t t0;
	t0 = clock();


	MatrixXd OldScoreTab = ScoreTab;
	MatrixXd OldVP = VP;
	MatrixXd Boundary(2, 2);

	MatrixXd BstVPs = GA.BstVPs;
	MatrixXd BstScoreTable = GA.BstScoreTable;
	MatrixXd VPgroups = GA.Groups;

	while (VPgroups.col(6).sum() != 0)
	{
		iStep = iStep * 0.5;
		cout << "    Step length (m): " << iStep << "   ";
		int nGroup = VPgroups.rows();

		// Undensified VPs (saved from old VPs)
		int nOld = 0;
		for (int j = 0; j < nGroup; j++)
		{
			if (VPgroups(j, 6) == 0)
				nOld++;
		}

		VectorXd Nnew(nGroup - nOld);
		MatrixXd VP_;

		int i = 0;
		for (int j = 0; j < nGroup; j++)
		{
			if (VPgroups(j, 6) == 1)
			{
				Boundary << VPgroups(j, 2), VPgroups(j, 3),
					VPgroups(j, 4), VPgroups(j, 5);

				VP_ = getGrid_Local(WALLINFO.ToScanWalls, Boundary, WALLINFO.Polygon_main, ScanArea, iStep, Multiplier);
				VP = cleanVPs(VP_, WALLINFO);


				Nnew(i) = VP.rows();
				i++;
			}
		}
		int nNew = Nnew.sum();



		MatrixXd DensifiedScoreTab = MatrixXd::Zero(nOld + nNew, WALLINFO.WallPatches.rows());
		MatrixXd DensifiedVP = MatrixXd::Zero(nOld + nNew, 4);

		int m = 0;
		for (int j = 0; j < nGroup; j++)
		{
			if (VPgroups(j, 6) == 0)
			{
				DensifiedScoreTab.row(m) = OldScoreTab.row(VPgroups(j, 7));
				DensifiedVP.row(m) = OldVP.row(VPgroups(j, 7));

				m++;
			}
		}


		// get score table
		int counter = nOld;
		for (int j = 0; j < nGroup; j++)
		{
			if (VPgroups(j, 6) == 1)
			{
				Boundary << VPgroups(j, 2), VPgroups(j, 3),
					VPgroups(j, 4), VPgroups(j, 5);

				VP_ = getGrid_Local(WALLINFO.ToScanWalls, Boundary, WALLINFO.Polygon_main, ScanArea, iStep, Multiplier);
				VP = cleanVPs(VP_, WALLINFO);

				for (int nn = 0; nn < VP.rows(); nn++)
				{

					Vector2d iVP = { VP(nn,0),VP(nn,1) };

					iVPInfo iDensifiedVP = getVisibility(iVP, WALLINFO);
					DensifiedScoreTab.row(counter + nn) = iDensifiedVP.ScoreTab;
					DensifiedVP.row(counter + nn) << VP(nn, 0), VP(nn, 1), iStep, iDensifiedVP.IncAngSum;

				}

				counter = counter + VP.rows();
			}
		}
		
		
		GA = GreedyAlg(DensifiedScoreTab, iStep, GAType, Multiplier, DensifiedVP, col_counter);
		BstVPs = GA.BstVPs;
		BstScoreTable = GA.BstScoreTable;
		VPgroups = GA.Groups;

		cout << BstVPs.rows() << " optimal VPs were selected. " << endl;

		OldScoreTab = DensifiedScoreTab;
		OldVP = DensifiedVP;
	}
	
	double dur;
	clock_t t1 = clock();;
	dur = (double)(t1 - t0);
	cout << "    Runtime_creating_solution (s): " << (dur / CLOCKS_PER_SEC) << endl << endl;

	CoverageTest(BstScoreTable);

	return GA;
}

// Function: if point is in polygon
// Methods: cross between level line passing level line of p and all edges
// Output: point in polygon if odd cross (cross at the same side of p)
// 
// POINT p
// LPPOINT ptPolygon: coordinate of polygon nodes (closure or misclosure)
// int nCount: number of polygon nodes

PolygonCheck PtInPolygon(Vector2d p, MatrixXd Polygon)

{
	int nCount = Polygon.rows();
	int nCross = 0;
	bool OnPolygon = 0;
	PolygonCheck polygoncheck = { 0,0 };

	for (int i = 0; i < nCount; i++)

	{

		Vector2d p1;
		p1 = { Polygon(i, 0) ,Polygon(i, 1) };
		Vector2d p2;
		p2 = { Polygon((i + 1) % nCount, 0) ,Polygon((i + 1) % nCount, 1) };



		// no cross point between line y=p.y and line p1p2

		if (p1(1) == p2(1)) // (p1p2 // y=p0.y or overlap)
		{
			if (p1(1) == p(1) && p(0) > min(p1(0), p2(0)) && p(0) < max(p1(0), p2(0)))

				OnPolygon = 1;  // overlap, cross point on horizontal polygon edge

			continue;

		}

		else if (p(1) < min(p1(1), p2(1))) // cross point at extended p1p2

			continue;

		else if (p(1) >= max(p1(1), p2(1))) // cross point at extended p1p2

			continue;


		else // cross 
		{
			// crosspoint.x--------------------------------------------------------------
			double x = (double)(p(1) - p1(1)) * (double)(p2(0) - p1(0)) / (double)(p2(1) - p1(1)) + p1(0);

			if (x == p(0))
			{
				OnPolygon = 1;   //cross point on vertical (or tilt) polygon edge
				break;
			}

			else if (x > p(0))

				nCross++; // nCross from the same side (all > or all <)
		}


	}

	// odd nCross, point falls in polygon ---

	if (OnPolygon)
		polygoncheck.On = 1;

	else if (!OnPolygon && nCross % 2 == 1)
	{
		polygoncheck.In = 1;
	}

	return polygoncheck;

}


// Discretize the wall into wall patches & update Matrix wall with patch ids



MatrixXd pillar2wall(MatrixXd pillar)
{

	MatrixXd PillarEdges(4, pillar.cols());

	Vector3d A = { pillar(1) - pillar(4) / 2,  pillar(2) - pillar(4) / 2,  pillar(3) };
	Vector3d B = { pillar(1) + pillar(4) / 2,  pillar(2) - pillar(4) / 2,  pillar(3) };
	Vector3d C = { pillar(1) + pillar(4) / 2,  pillar(2) + pillar(4) / 2,  pillar(3) };
	Vector3d D = { pillar(1) - pillar(4) / 2,  pillar(2) + pillar(4) / 2,  pillar(3) };

	PillarEdges << 1, A(0), A(1), A(2), B(0), B(1), B(2), 0, -1, 0, 5,
		2, B(0), B(1), B(2), C(0), C(1), C(2), 1, 0, 0, 5,
		3, C(0), C(1), C(2), D(0), D(1), D(2), 0, 1, 0, 5,
		4, D(0), D(1), D(2), A(0), A(1), A(2), -1, 0, 0, 5;

	return PillarEdges;
}

MatrixXd square2wall(MatrixXd square)
{
	double SqrRotation = asin(square(8) / square(7));
	double SqrDiagAng = atan(square(5) / square(4));
	double SqrDiagDis = sqrt(square(5) * square(5) + square(4) * square(4));

	MatrixXd SquareEdges(4, square.cols());

	Vector3d A = { square(1),  square(2),  square(3) };
	Vector3d B = { square(1) + square(4) * cos(SqrRotation),  square(2) + square(4) * sin(SqrRotation),  square(3) };
	Vector3d C = { square(1) + SqrDiagDis * cos(SqrRotation + SqrDiagAng),  square(2) + SqrDiagDis * sin(SqrRotation + SqrDiagAng),  square(3) };
	Vector3d D = { square(1) + square(5) * sin(-SqrRotation),  square(2) + square(5) * cos(-SqrRotation),  square(3) };

	SquareEdges << 1, A(0), A(1), A(2), B(0), B(1), B(2), -square(8), square(7), square(9), 5,
		2, B(0), B(1), B(2), C(0), C(1), C(2), -square(7), -square(8), square(9), 5,
		3, C(0), C(1), C(2), D(0), D(1), D(2), square(8), -square(7), square(9), 5,
		4, D(0), D(1), D(2), A(0), A(1), A(2), square(7), square(8), square(9), 5;

	return SquareEdges;
}


MatrixXd getGrid_Local(MatrixXd wall, MatrixXd Boundary, MatrixXd Polygon, int ScanArea, double Step, double GroupDis)
{
	double OuterBoundary = GroupDis * 3;
	double dis_para = 1;   //distance threshold from VP to wall (no VPs if too close to the walls)

	switch (ScanArea)
	{

	case 1:
	{

		int nX = int(floor(((Boundary(1, 0) + OuterBoundary) - (Boundary(0, 0) - OuterBoundary)) / Step));
		int nY = int(floor(((Boundary(1, 1) + OuterBoundary) - (Boundary(0, 1) - OuterBoundary)) / Step));

		MatrixXd iVP(nX * nY, 3); // descritized VPs

		int nOutOn = 0; // number of point out of and on the polygon
		int nClose = 0; // number of point close to the polygon
		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				// remove point out of and on the polygon
				iVP(j * nX + i, 0) = Boundary(0, 0) - OuterBoundary + (2 * i + 1) * Step / 2;
				iVP(j * nX + i, 1) = Boundary(0, 1) - OuterBoundary + (2 * j + 1) * Step / 2;
				iVP(j * nX + i, 2) = Step;


				// if PtInPolygon
				Vector2d p;
				p = { iVP(j * nX + i, 0) ,iVP(j * nX + i, 1) };

				PolygonCheck polygoncheck = PtInPolygon(p, Polygon);

				if (!polygoncheck.In && !polygoncheck.On)
				{
					iVP(j * nX + i, 0) = 52525;
					nOutOn++;
				}
				else
				{
					// remove points close to the polygon
					bool isClose = 0;
					for (int j = 0; j < wall.rows(); j++)
					{
						if (wall(j, 10) == 1 || wall(j, 10) == 3 || wall(j, 10) == 4 || wall(j, 10) == 5)
						{
							Vector2d jWallBeg = { wall(j,1),wall(j,2) };
							Vector2d jWallEnd = { wall(j,4),wall(j,5) };

							Vector3d jLnCoeff = LnFcn(jWallBeg, jWallEnd);
							BOOL close2wall = IfCrossWithCircle(jWallBeg, jWallEnd, jLnCoeff, p, dis_para);
							if (close2wall)
							{
								isClose = 1;
								break;
							}
						}
					}
					if (isClose)
					{
						iVP(j * nX + i, 0) = 52525;
						nClose++;
					}
				}
			}
		}

		int nSaved = nX * nY - nOutOn - nClose;

		/* ----- save inpolygon and far to wall VPs ----- */
		MatrixXd VP(nSaved, 4);
		int m = 0;

		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				if (iVP(j * nX + i, 0) != 52525 && m < nSaved)
				{
					VP(m, 0) = iVP(j * nX + i, 0);
					VP(m, 1) = iVP(j * nX + i, 1);
					VP(m, 2) = iVP(j * nX + i, 2);
					VP(m, 3) = 0;  // Summation of incidence angle

					m++;
				}
			}
		}

		return VP;
	}


	case 2:
	{

		int nX = int(floor(((Boundary(1, 0) + OuterBoundary) - (Boundary(0, 0) - OuterBoundary)) / Step));
		int nY = int(floor(((Boundary(1, 1) + OuterBoundary) - (Boundary(0, 1) - OuterBoundary)) / Step));

		// LPPOINT iVP = new POINT[nX * nY]; 
		MatrixXd iVP(nX * nY, 3);// descritized VPs


		int nInOn = 0; // number of point in and on polygon
		int nClose = 0; // number of point close to the polygon
		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				// remove point out of and on the polygon
				iVP(j * nX + i, 0) = Boundary(0, 0) - OuterBoundary + (2 * i + 1) * Step / 2;
				iVP(j * nX + i, 1) = Boundary(0, 1) - OuterBoundary + (2 * j + 1) * Step / 2;
				iVP(j * nX + i, 2) = Step;

				// if PtInPolygon
				Vector2d p;
				p = { iVP(j * nX + i, 0) ,iVP(j * nX + i, 1) };

				PolygonCheck polygoncheck = PtInPolygon(p, Polygon);
				if (polygoncheck.In || polygoncheck.On)
				{
					iVP(j * nX + i, 0) = 52525;
					nInOn++;
				}
				else
				{
					// remove points close to the polygon
					bool isClose = 0;
					for (int j = 0; j < wall.rows(); j++)
					{
						if (wall(j, 10) == 1 || wall(j, 10) == 3 || wall(j, 10) == 4 || wall(j, 10) == 5)
						{
							Vector2d jWallBeg = { wall(j,1),wall(j,2) };
							Vector2d jWallEnd = { wall(j,4),wall(j,5) };

							Vector3d jLnCoeff = LnFcn(jWallBeg, jWallEnd);
							BOOL close2wall = IfCrossWithCircle(jWallBeg, jWallEnd, jLnCoeff, p, dis_para);
							if (close2wall)
							{
								isClose = 1;
								break;
							}
						}
					}
					if (isClose)
					{
						iVP(j * nX + i, 0) = 52525;
						nClose++;
					}
				}
			}
		}

		/* ----- save out of polygon VPs ----- */

		int nSaved = nX * nY - nInOn - nClose;

		MatrixXd VP(nSaved, 4);
		int m = 0;

		for (int j = 0; j < nY; j++)
		{
			for (int i = 0; i < nX; i++)
			{
				if (iVP(j * nX + i, 0) != 52525 && m < nSaved)
				{
					VP(m, 0) = iVP(j * nX + i, 0);
					VP(m, 1) = iVP(j * nX + i, 1);
					VP(m, 2) = iVP(j * nX + i, 2);
					VP(m, 3) = 0; // Summation of incidence angle

					m++;
				}
			}
		}

		return VP;
	}

	}

}

RowVectorXd getVisibility_t(Vector2d VPi, MatrixXd All_Wall, MatrixXd Targets_candidates)
{

	MatrixXd wall = All_Wall;
	Vector2d VP = VPi;

	RowVectorXd nScoreTab = RowVectorXd::Zero(Targets_candidates.rows()); // Score table for VP n

																		  // default parameters
																		  //double MaxInc = 65;
	double MaxRng = 100;
	double MinRng = 0;

	int nWall = wall.rows();
	int nTargets = Targets_candidates.rows();

	for (int i = 0; i < nTargets; i++)
	{

		Vector2d iTarget = { Targets_candidates(i,0), Targets_candidates(i,1) };
		Vector3d iLnCoeff = LnFcn(VP, iTarget);

		///* Condition 1: Visibility */
		bool inVisible = 0;  // if a target is visible form VP
		for (int j = 0; j < nWall; j++)
		{
			if (wall(j, 10) == 0 || wall(j, 10) == 1 || wall(j, 10) == 3 || wall(j, 10) == 4 || wall(j, 10) == 5)
			{
				Vector2d jWallBeg = { wall(j,1),wall(j,2) };
				Vector2d jWallEnd = { wall(j,4),wall(j,5) };

				Vector3d jLnCoeff = LnFcn(jWallBeg, jWallEnd);
				BOOL cross = IfCross(iLnCoeff, jLnCoeff, VP, iTarget, jWallBeg, jWallEnd);

				if (cross == 1)
				{
					inVisible = 1;
					break;   // If incidence ray cross with any wall, this patch is invisible, break all other walls
				}
			}
			else if (wall(j, 10) == 4)
			{
				Vector2d CircleCenter = { wall(j, 1),wall(j, 2) };
				BOOL Circlecross = IfCrossWithCircle(VP, iTarget, iLnCoeff, CircleCenter, wall(j, 4));

				if (Circlecross == 1)
				{
					//cout << j << " section cross with pillar " << m << endl;
					inVisible = 1;
					break;   // If incidence ray cross with any wall, this patch is unvisible, break all other walls
				}

			}

		}

		if (inVisible)
			continue;   // If incidence ray cross with any wall, this target is unvisible, continue to the next target


						/* Condition 2: Range */
		Vector2d iIncRay = VP - iTarget;  // Incidence ray for target i

		double iRange = iIncRay.norm();

		if (iRange > MaxRng || iRange < MinRng)
			continue;


		/* --- All pass, visible ! --- */
		nScoreTab(i) = 1;
	}

	return nScoreTab;
}


Vector3d LnFcn(VectorXd lnBeg, VectorXd lnEnd)
{
	Vector3d LnCoeff;
	if (lnBeg(0) - lnEnd(0) != 0)
	{
		LnCoeff(0) = (lnBeg(1) - lnEnd(1)) / (lnEnd(0) - lnBeg(0));
		LnCoeff(1) = 1;
		LnCoeff(2) = -lnBeg(1) - LnCoeff(0) * lnBeg(0);
	}
	else
	{
		LnCoeff(0) = 1;
		LnCoeff(1) = 0;
		LnCoeff(2) = -lnBeg(0);
	}

	return LnCoeff;
}


BOOL IfCross(Vector3d LnCoeff1, Vector3d LnCoeff2, Vector2d lnBeg1, Vector2d lnEnd1, Vector2d lnBeg2, Vector2d lnEnd2)
{
	double idxA = LnCoeff1(0) * lnBeg2(0) + LnCoeff1(1) * lnBeg2(1) + LnCoeff1(2);
	if (abs(idxA) < pow(10, -10))      // In case of truncations errors
		idxA = 0;

	double idxB = LnCoeff1(0) * lnEnd2(0) + LnCoeff1(1) * lnEnd2(1) + LnCoeff1(2);
	if (abs(idxB) < pow(10, -10))
		idxB = 0;

	double idxC = LnCoeff2(0) * lnBeg1(0) + LnCoeff2(1) * lnBeg1(1) + LnCoeff2(2);
	if (abs(idxC) < pow(10, -10))
		idxC = 0;

	double idxD = LnCoeff2(0) * lnEnd1(0) + LnCoeff2(1) * lnEnd1(1) + LnCoeff2(2);
	if (abs(idxD) < pow(10, -10))
		idxD = 0;
	// In case of truncations errors

	// cout << idxA << " " << idxB << " " << idxC << " " << idxD << " " << endl;

	BOOL cross = 0;
	if (idxA * idxB < 0 && idxC * idxD < 0)   //consider touched lines as cross to aovid the edge uncertainty in laser beam
		cross = 1;

	return cross;
}


// http://blog.csdn.net/swordsman___ddz/article/details/52659060
BOOL IfCrossWithCircle(Vector2d lnBeg, Vector2d lnEnd, Vector3d lnCoeff, Vector2d CircleCenter, double radius)
{
	double IO1 = (lnBeg(0) - CircleCenter(0)) * (lnBeg(0) - CircleCenter(0)) + (lnBeg(1) - CircleCenter(1)) * (lnBeg(1) - CircleCenter(1));
	double IO2 = (lnEnd(0) - CircleCenter(0)) * (lnEnd(0) - CircleCenter(0)) + (lnEnd(1) - CircleCenter(1)) * (lnEnd(1) - CircleCenter(1));
	if (IO1 <= radius * radius || IO2 <= radius * radius)  // any vertex falls into circle: cross
		return 1;
	else
	{
		double dist1, dist2, angle1, angle2; // ax + by + c = 0;  
											 //Vector3d lnCoeff = LnFcn(lnBeg, lnEnd);

		dist1 = abs(lnCoeff(0) * CircleCenter(0) + lnCoeff(1) * CircleCenter(1) + lnCoeff(2)) / sqrt(lnCoeff(0) * lnCoeff(0) + lnCoeff(1) * lnCoeff(1));
		/*dist1 *= dist1;
		dist2 = (lnCoeff(0) * lnCoeff(0) + lnCoeff(1) * lnCoeff(1) * radius * radius;*/
		if (dist1 >= radius)    //circle center to line distance > r: not cross  
			return 0;
		else
		{

			angle1 = (CircleCenter(0) - lnBeg(0)) * (lnEnd(0) - lnBeg(0)) + (CircleCenter(1) - lnBeg(1)) * (lnEnd(1) - lnBeg(1)); //cos(angle1)
			angle2 = (CircleCenter(0) - lnEnd(0)) * (lnBeg(0) - lnEnd(0)) + (CircleCenter(1) - lnEnd(1)) * (lnBeg(1) - lnEnd(1)); //cos(angle1)
			if (angle1 > 0 && angle2 > 0) // two sharp angles: cross
				return 1;
			else
				return 0;
		}

	}

}



void CoverageTest(MatrixXd ST)
{
	VectorXd rowSum = ST.colwise().sum();
	double CoverageRate;

	if (rowSum.prod() == 0)
	{
		cout << "    Wall coverage: not fully covered ";
		int zeroSum = 0;
		for (int i = 0; i < ST.cols(); i++)
		{
			if (rowSum(i) == 0)
				zeroSum++;

		}
		//cout << endl;
		CoverageRate = double(100 * (ST.cols() - zeroSum)) / ST.cols();
		cout << "(" << fixed << setprecision(4) << CoverageRate << ")" << endl << endl;
	}

	else
		cout << "    Wall coverage: Fully covered (100%)" << endl << endl;

}

