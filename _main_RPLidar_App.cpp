/***************************************************************
 * Name:      white_bot_2App.cpp
 * Purpose:   Code for Application Class
 * Author:    Khemarat Boonyapaluk (korla.march@gmail.com)
 * Created:   2018-07-16
 * Copyright: Khemarat Boonyapaluk (korlamarch.com)
 * License:
 **************************************************************/

#include "wx_pch.h"
#include "white_bot_2App.h"

 //(*AppHeaders
#include "white_bot_2Main.h"
#include <wx/image.h>
//*)

#include "iostream"

IMPLEMENT_APP(white_bot_2App);

using namespace mrpt;

bool white_bot_2App::OnInit()
{
	BotModel* commonModel;
	try
	{
		ios_base::sync_with_stdio(true);
		freopen("log.txt", "w", stdout);
		freopen("err.txt", "w", stderr);

		// Starting in wxWidgets 2.9.0, we must reset numerics locale to "C",
		//  if we want numbers to use "." in all countries. The App::OnInit() is a
		//  perfect place to undo
		//  the default wxWidgets settings. (JL @ Sep-2009)
		wxSetlocale(LC_NUMERIC, wxString(wxT("C")));

		// Create a common BotModel
		commonModel = new BotModel();
	}
	catch (std::exception &e)
	{
		cerr << "[Init Function]  Closing due to exception:\n" << e.what() << endl;
	}
	catch (...)
	{
		cerr << "[Init Function] Untyped exception! Closing." << endl;
	}

	//(*AppInitialize
	bool wxsOK = true;
	wxInitAllImageHandlers();

	if (wxsOK)
	{
		white_bot_2Frame* Frame = new white_bot_2Frame(0, -1, commonModel);
		Frame->Show();
		SetTopWindow(Frame);
	}
	//*)

	try
	{
		// ------ Start All Threads ------ 

		// Manual control thread
		mrpt::system::TThreadHandle controlThread;
		{
			std::cout << "\n\n==== Launching Robot Control thread ==== \n";
			controlThread = mrpt::system::createThread(white_bot_2App::ControlThread, commonModel);
		}
		// Sensor Grabber
		mrpt::system::TThreadHandle hSensorThread;
		{
			std::cout << "\n\n==== Launching LIDAR grabbing thread ==== \n";
			hSensorThread = mrpt::system::createThread(white_bot_2App::SensorThread, commonModel);
		}
		// Action/SF Generator
		mrpt::system::TThreadHandle ASFGenThread;
		{
			std::cout << "\n\n==== Launching Action/SF Generator thread ==== \n";
			ASFGenThread = mrpt::system::createThread(white_bot_2App::ASFGeneratorThread, commonModel);
		}
		// PF Localizer
		mrpt::system::TThreadHandle PFLocalizerThread;
		{
			std::cout << "\n\n==== Launching PF Localizer thread ==== \n";
			PFLocalizerThread = mrpt::system::createThread(white_bot_2App::PFLocalizerThread, commonModel);
		}
		// Reactive Navigation
		mrpt::system::TThreadHandle ReactiveNavThread;
		{
			std::cout << "\n\n==== Launching Reactive Navigation thread ==== \n";
			ReactiveNavThread = mrpt::system::createThread(white_bot_2App::ReactiveNavThread, commonModel);
		}

		// -------------------------------

		// Wait and check if the robot is ready:
		mrpt::system::sleep(2000);
		if (commonModel->allThreadsMustExit)
		{
			throw std::runtime_error("\n\n==== Rebot Initiation Error : Program will exit ==== \n");
		}

	}
	catch (std::exception &e)
	{
		cerr << "[ThreadStater Function]  Closing due to exception:\n" << e.what() << endl;
	}
	catch (...)
	{
		cerr << "[ThreadStarter Function] Untyped exception! Closing." << endl;
	}

	return wxsOK;

}

// ------------------------------------------------------
//				ControlThread
// ------------------------------------------------------
void white_bot_2App::ControlThread(BotModel* botData)
{
	try
	{
		std::string		Create_Comport;
		double			SPEED_X;
		double			SPEED_Y;
		double			WheelDistance;
		int				sleepTime_control;
		RobotConnector* robot = &(botData->robot);
		utils::CTicTac	readData_timeout;

		// Read Config
		Create_Comport = botData->cfgFile->read_string("ICREATE", "COM_port", "", true);
		SPEED_X = botData->cfgFile->read_double("ICREATE", "speed_x", 1.0, false);
		SPEED_Y = botData->cfgFile->read_double("ICREATE", "speed_y", 1.0, false);
		WheelDistance = botData->cfgFile->read_double("ICREATE", "wheelDistance", 1.0, true);
		sleepTime_control = botData->cfgFile->read_int("GLOBAL", "sleepTime_Control", 30, true);

		// Connect to the Robot
		if (!robot->Connect(Create_Comport.c_str()))
		{
			throw std::runtime_error(string("***Error***: Can't connect to robot @") + string(Create_Comport));
		}

		robot->DriveDirect(100, 100);

		// Reset Encoder
		botData->encoder_left = 0;
		botData->encoder_right = 0;

		double preVL, preVR;
		double vl, vr;
		int x = 0;
		int state = 0;
		double prevDist = 0;
		int tick = 0;
		double botX, botY;
		double rs;
		int turnRand;

		double points[5][2];
		points[0][0] = 3.129; points[0][1] = -7.726;
		points[1][0] = 1.590; points[1][1] = -5.736;
		points[2][0] = -0.132; points[2][1] = -2.751;
		points[3][0] = -2.540; points[3][1] = 0.414;
		points[4][0] = -5.040; points[4][1] = 4.256;
		int pathLen = 0;
		double path[5][2];
		int curPoint = 0;

		while (!botData->allThreadsMustExit)
		{
			double vx, vz;
			vx = vz = 0.0;
			if (botData->isStart)
			{

				if (botData->isManual) {
					/*
					string s = "";
					for (int i = 0; i < pathLen; i++) {
						s += to_string(path[i][0]) + "," + to_string(path[i][1]) + "\n";
					}
					throw std::runtime_error(string(s));
					*/

					double robotX = botData->locationEstimation.x();
					double robotY = botData->locationEstimation.y();
					double x = path[curPoint][0];
					double y = path[curPoint][1];
					double dist = (std::pow(std::pow(robotX - x, 2) + std::pow(robotY - y, 2), 0.5));

					double vx;
					double vz;
					vx = vz = 0;

					if (state == 0) {
						if (tick == 0) {
							botX = robotX;
							botY = robotY;
						}
						vx = 0.25;
						tick++;
						if (tick > 60) {
							vx = 0;
						}
						if (tick > 150) {
							tick = 0;
							state = 1;
						}
					}
					else if (state == 1) {
						double angle1 = std::atan2(y - botY, x - botX);
						double angle2 = std::atan2(robotY - botY, robotX - botX);
						double res = angle1 - angle2;
						rs = res * 180 / 3.141592;
						if (rs > 180) {
							rs = rs - 360;
						}
						if (rs < -180) {
							rs = rs + 360;
						}
						//throw std::runtime_error(string(to_string(rs)));

						if (std::abs(rs) < 16.5) {
							state = 4;
							tick = 0;
						}
						else {
							state = 2;
						}
					}
					else if (state == 2) {
						vx = -0.25;
						tick++;
						if (tick > 60) {
							tick = 0;
							state = 3;
						}
					}
					else if (state == 3) {
						if (rs > 0) {
							vz = 0.155;
						}
						else {
							vz = -0.155;
						}

						tick++;
						if (tick > std::abs(rs)) {
							vz = 0;
						}
						if (tick > std::abs(rs)+40) {
							tick = 0;
							state = 0;
						}
					}
					else if (state == 4) {
						vx = 0.17;
						if (dist < 0.65) {
							state = 5;
							tick = 0;
						}
						if (prevDist < dist) {
							tick++;
						}
						else {
							tick--;
						}
						if (tick > 10) {
							state = 0;
							tick = 0;
						}
					}
					else if (state == 5) {
						vx = vz = 0;
						tick++;
						if (tick > 10) {
							if (curPoint < pathLen) {
								curPoint++;
								state = 0;
							}
						}
					}
					else if (state == 7) {
						tick++;
						vx = -0.25;
						if (tick > 200) {
							vx = 0;
							vz = -0.25;
							if (turnRand) vz = 0.25;
						}
						if (tick > 245) {
							vx = 0.25;
							vz = 0;
						}
						if (tick > 445) {
							vx = 0;
						}
						if (tick > 465) {
							tick = 0;
							state = 0;
						}
					}

					if (botData->robotData.bumper[0] == 1 || botData->robotData.bumper[1] == 1) {
						state = 7;
						tick = 0;
						turnRand = rand() & 1;
					}

					prevDist = dist;

					vl = vx - vz;
					vr = vx + vz;

					//if (vl == preVL && vr == preVR) continue;

					preVL = vl;
					preVR = vr;

					int velL = (int)(vl*Create_MaxVel);
					int velR = (int)(vr*Create_MaxVel);

					// Drive the Robot
					if (!robot->DriveDirect(velL, velR))
					{
						cerr << "SetControl Fail" << endl;
						throw std::runtime_error(string("***Error***: Can't drive the robot @") + string(Create_Comport));
					}
				}
				else {

					state = 0;
					while (1) {
						if (!robot->DriveDirect(0, 0))
						{
							cerr << "SetControl Fail" << endl;
							throw std::runtime_error(string("***Error***: Can't drive the robot @") + string(Create_Comport));
						}
					}

				}
			}
			else {
				// Stop the robot movement
				if (!robot->DriveDirect(0, 0))
				{
					cerr << "SetControl Fail" << endl;
					throw std::runtime_error(string("***Error***: Can't drive the robot @") + string(Create_Comport));
				}

				// ---------------------------------------------

				while (1) {
					state = 0;
					curPoint = 0;
					if (botData->isStart) break;
					double robotX = botData->locationEstimation.x();
					double robotY = botData->locationEstimation.y();
					double x = botData->targetPose.x;
					double y = botData->targetPose.y;

					int robotIndex, targetIndex;
					double robotDist = 9999, targetDist = 9999;
					for (int i = 0; i < 5; i++) {
						double cRDist = (std::pow(std::pow(robotX - points[i][0], 2) + std::pow(robotY - points[i][1], 2), 0.5));
						if (cRDist < robotDist) {
							robotDist = cRDist;
							robotIndex = i;
						}
						double cTDist = (std::pow(std::pow(x - points[i][0], 2) + std::pow(y - points[i][1], 2), 0.5));
						if (cTDist < targetDist) {
							targetDist = cTDist;
							targetIndex = i;
						}
					}
					if (robotIndex == targetIndex || (robotIndex >= 1 && robotIndex <= 3 && targetIndex >= 1 && targetIndex <= 3)) {
						pathLen = 1;
						path[0][0] = x;
						path[0][1] = y;
					}
					else {
						if (robotIndex < targetIndex) {
							int p = 0;
							for (int i = robotIndex; i < targetIndex + 2; i++) {
								if (i == targetIndex + 1) {
									path[p][0] = x;
									path[p][1] = y;
								}
								else {
									path[p][0] = points[i][0];
									path[p][1] = points[i][1];
								}
								p++;
							}
							pathLen = targetIndex - robotIndex + 2;
						}
						else {
							int p = 0;
							for (int i = robotIndex; i > targetIndex - 2; i--) {
								if (i == targetIndex - 1) {
									path[p][0] = x;
									path[p][1] = y;
								}
								else {
									path[p][0] = points[i][0];
									path[p][1] = points[i][1];
								}
								p++;
							}
							pathLen = robotIndex - targetIndex + 2;
						}
					}
				}
			}

			{
				synch::CCriticalSectionLocker	lock(&botData->cs_robotData);
				if (robot->ReadData(botData->robotData))
				{
					//printf("Get Data: %d %d\n", botData->robotData.distance, botData->robotData.angle);
					if (abs(botData->robotData.distance) < 500 && abs(botData->robotData.angle) < 200)
					{
						botData->encoder_right += 2.4 * (botData->robotData.distance +
							botData->robotData.angle*WheelDistance * 1000 * M_PI / 360.0);
						botData->encoder_left += 2.4 * (botData->robotData.distance -
							botData->robotData.angle*WheelDistance * 1000 * M_PI / 360.0);
					}

					readData_timeout.Tic();
				}
				else if (readData_timeout.Tac() > 1.0)
				{
					cout << "Robot : Read Data failed" << endl;
					readData_timeout.Tic();
				}
			}

			mrpt::system::sleep(sleepTime_control);
		}

		robot->Disconnect();
	}
	catch (std::exception &e)
	{
		cerr << "[ControlThread]  Closing due to exception:\n" << e.what() << endl;
		botData->allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[ControlThread] Untyped exception! Closing." << endl;
		botData->allThreadsMustExit = true;
	}
}

// ------------------------------------------------------
//				SensorThread
// ------------------------------------------------------
void white_bot_2App::SensorThread(BotModel* botData)
{
	using namespace mrpt::system;
	using namespace mrpt::hwdrivers;
	try
	{
		const string sensor_label = "LIDAR_SENSOR";
		string driver_name = botData->cfgFile->read_string(sensor_label, "driver", "", true);

		CGenericSensorPtr	sensor = CGenericSensor::createSensorPtr(driver_name);
		if (!sensor)
		{
			throw std::runtime_error(string("***ERROR***: Class name not recognized: ") + driver_name);
		}

		// Load common & sensor specific parameters:
		sensor->loadConfig(*botData->cfgFile, sensor_label);

		cout << format("[thread_%s] Starting...", sensor_label.c_str()) << " at " << sensor->getProcessRate() << " Hz" << endl;

		ASSERTMSG_(sensor->getProcessRate() > 0, "process_rate must be set to a valid value (>0 Hz).");
		int		process_period_ms = round(1000.0 / sensor->getProcessRate());

		// Init device:
		sensor->initialize();


		while (!botData->allThreadsMustExit)
		{
			TTimeStamp t0 = now();

			// Process
			sensor->doProcess();

			// Get new observations
			CGenericSensor::TListObservations	lstObjs;
			sensor->getObservations(lstObjs);

			{
				synch::CCriticalSectionLocker	lock(&botData->cs_global_list_obs);
				botData->global_list_obs.insert(lstObjs.begin(), lstObjs.end());
			}

			lstObjs.clear();

			// wait until the process period:
			TTimeStamp t1 = now();
			double	At = timeDifference(t0, t1);
			int At_rem_ms = process_period_ms - At * 1000;
			if (At_rem_ms > 0)
				sleep(At_rem_ms);
		}

		sensor.clear();
		cout << format("[thread_%s] Closing...", sensor_label.c_str()) << endl;
	}
	catch (std::exception &e)
	{
		cerr << "[SensorThread]  Closing due to exception:\n" << e.what() << endl;
		botData->allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[SensorThread] Untyped exception! Closing." << endl;
		botData->allThreadsMustExit = true;
	}
}

// ------------------------------------------------------
//				Action/SF Generator Thread
// ------------------------------------------------------
void white_bot_2App::ASFGeneratorThread(BotModel* botData)
{
	using namespace mrpt::obs;
	using namespace mrpt::hwdrivers;
	try
	{
		int sleeptime;
		double WheelDistance;
		utils::CTicTac timeout_read_scans;

		// Read config
		sleeptime = botData->cfgFile->read_int("GLOBAL", "sleeptime_ASF", 30, true);
		WheelDistance = botData->cfgFile->read_double("ICREATE", "wheelDistance", 1.0, true);

		// parameter for the odometry (from encoder)
		CActionRobotMovement2D::TMotionModelOptions odom_params;
		odom_params.modelSelection = CActionRobotMovement2D::mmGaussian;
		odom_params.gaussianModel.minStdXY = botData->cfgFile->read_double("ODOMETRY_PARAMS", "minStdXY", 0.04);
		odom_params.gaussianModel.minStdPHI = DEG2RAD(botData->cfgFile->read_double("ODOMETRY_PARAMS", "minStdPhi", 2.0));

		// previous encoder (used to compute absolute value)
		int pre_encoder_left = botData->encoder_left;
		int pre_encoder_right = botData->encoder_right;

		while (!botData->allThreadsMustExit)
		{
			CObservation2DRangeScanPtr observation;
			{
				mrpt::synch::CCriticalSectionLocker obsLock(&botData->cs_global_list_obs);
				if (!botData->global_list_obs.empty())
				{
					// Get the lastest one
					for (CGenericSensor::TListObservations::reverse_iterator it = botData->global_list_obs.rbegin();
						!observation && it != botData->global_list_obs.rend(); ++it)
					{
						if (it->second.present() && IS_CLASS(it->second, CObservation2DRangeScan))
						{
							observation = CObservation2DRangeScanPtr(it->second);
						}
					}
					botData->global_list_obs.clear();
				}
			}

			// Wait for a laser scan, wait for it:
			if (!observation) {
				if (timeout_read_scans.Tac() > 1.0) {
					timeout_read_scans.Tic();
					cout << "[ASFGenerator] Warning: *** Waiting for laser scans from the Device ***\n";
				}
				mrpt::system::sleep(5);
				continue;
			}
			else {
				timeout_read_scans.Tic(); // Reset timeout
			}

			// Compute action from the encoder value
			CActionRobotMovement2DPtr act = CActionRobotMovement2D::Create();
			act->timestamp = mrpt::system::now();
			act->hasEncodersInfo = true;
			act->encoderLeftTicks = botData->encoder_left - pre_encoder_left;
			act->encoderRightTicks = botData->encoder_right - pre_encoder_right;

			act->motionModelConfiguration = odom_params;

			act->computeFromEncoders(0.001, 0.001, WheelDistance);

			pre_encoder_left = botData->encoder_left;
			pre_encoder_right = botData->encoder_right;

			// calculate new velocity
			int dt = (act->timestamp - botData->ASFTime) / 10000.0;
			poses::CPose2D actpos = act->poseChange->getMeanVal();
			TTwist2D newVelocity(actpos.m_coords[0] / dt, actpos.m_coords[1] / dt, actpos.phi() / dt);
			newVelocity.rotate(botData->odometry.phi());

			{
				synch::CCriticalSectionLocker	lock(&botData->cs_ASF);

				// Insert observation and action into Action/SF
				botData->ASFTime = act->timestamp;
				botData->sf.clear();
				botData->sf.insert(observation);


				botData->actions.clear();
				botData->actions.insert(*act);
				botData->odometry += actpos;
				botData->estVelocity = newVelocity;

				act.clear_unique();
			}

			// Build the obstacles points map for navigation:
			{
				synch::CCriticalSectionLocker obstacles_lock(&botData->cs_obstacles);

				botData->latest_obstacles.insertionOptions.minDistBetweenLaserPoints = 0.005f;
				botData->latest_obstacles.insertionOptions.also_interpolate = false;

				botData->latest_obstacles.clear(); // erase old points
				botData->latest_obstacles.insertObservation(observation.pointer());
			}

			system::sleep(sleeptime);
		}

	}
	catch (std::exception &e)
	{
		cerr << "[ASFGenerator]  Closing due to exception:\n" << e.what() << endl;
		botData->allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[ASFGenerator] Untyped exception! Closing." << endl;
		botData->allThreadsMustExit = true;
	}
}

// ------------------------------------------------------
//				PF Localizer Thread
// ------------------------------------------------------
void white_bot_2App::PFLocalizerThread(BotModel* botData) {
	using namespace mrpt::bayes;
	using namespace mrpt::math;
	using namespace mrpt::obs;
	using namespace mrpt::maps;
	using namespace mrpt::poses;
	using namespace mrpt::utils;

	try
	{
		const string sect("PFLOCALIZER");

		int			particles_count;	// Number of initial particles (if size>1)
		int			PF_sleeptime;
		bayes::CParticleFilter::TParticleFilterOptions	pfOptions; // PF-algorithm Options
		slam::TMonteCarloLocalizationParams		pdfPredictionOptions; // PDF Options
		CParticleFilter::TParticleFilterStats	PF_stats;
		slam::CMonteCarloLocalization2D  pdf;
		CParticleFilter		PF;
		utils::CTicTac		tictac, timeout_read_ASF;

		// Read Config
		particles_count = botData->cfgFile->read_int(sect, "particles_count", 0, true);
		pfOptions.loadFromConfigFile(*botData->cfgFile, "PF_options");
		pdfPredictionOptions.KLD_params.loadFromConfigFile(*botData->cfgFile, "KLD_options");
		PF_sleeptime = botData->cfgFile->read_int("GLOBAL", "sleeptime_PF", 30, true);

		float init_PDF_min_x = 0, init_PDF_min_y = 0, init_PDF_max_x = 0, init_PDF_max_y = 0;
		MRPT_LOAD_CONFIG_VAR(init_PDF_min_x, float, (*botData->cfgFile), sect)
			MRPT_LOAD_CONFIG_VAR(init_PDF_min_y, float, (*botData->cfgFile), sect)
			MRPT_LOAD_CONFIG_VAR(init_PDF_max_x, float, (*botData->cfgFile), sect)
			MRPT_LOAD_CONFIG_VAR(init_PDF_max_y, float, (*botData->cfgFile), sect)

			// Randomize the generator
			mrpt::random::randomGenerator.randomize();

		// Gridmap / area of initial uncertainty:
		COccupancyGridMap2D::TEntropyInfo	gridInfo;
		botData->metricMap.m_gridMaps[0]->computeEntropy(gridInfo);
		printf("The gridmap has %.04fm2 observed area, %u observed cells\n", gridInfo.effectiveMappedArea, (unsigned)gridInfo.effectiveMappedCells);

		// PDF Options:
		pdf.options = pdfPredictionOptions;
		pdf.options.metricMap = &botData->metricMap;

		// Set the PF object:
		PF.m_options = pfOptions;

		// Initialize the PDF:
		// -----------------------------
		tictac.Tic();
		if (!botData->cfgFile->read_bool(sect, "init_PDF_mode", false, /*Fail if not found*/true))
		{
			pdf.resetUniformFreeSpace(
				botData->metricMap.m_gridMaps[0].pointer(),
				0.7f,
				particles_count,
				init_PDF_min_x, init_PDF_max_x,
				init_PDF_min_y, init_PDF_max_y,
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_min_phi_deg", -180)),
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_max_phi_deg", 180))
			);
		}
		else {
			pdf.resetUniform(
				init_PDF_min_x, init_PDF_max_x,
				init_PDF_min_y, init_PDF_max_y,
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_min_phi_deg", -180)),
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_max_phi_deg", 180)),
				particles_count
			);
		}


		printf("PDF of %u particles initialized in %.03fms\n", particles_count, 1000 * tictac.Tac());

		botData->pdf = &pdf;

		// -----------------------------
		//		Particle filter
		// -----------------------------
		CPose2D					pdfEstimation;
		system::TTimeStamp		ASF_last_timestamp = 0;

		while (!botData->allThreadsMustExit)
		{
			// ----------------------------------------
			// RUN ONE STEP OF THE PARTICLE FILTER:
			// ----------------------------------------
			tictac.Tic();
			obs::CActionCollection		action;
			obs::CSensoryFrame			sf;
			{
				synch::CCriticalSectionLocker	lock(&botData->cs_ASF);
				if (ASF_last_timestamp != botData->ASFTime)
				{
					// if there is a new ASF, read it
					action = botData->actions;
					sf = botData->sf;
					ASF_last_timestamp = botData->ASFTime;
				}
			}

			if (!action.size() || !sf.size())
			{
				if (timeout_read_ASF.Tac() > 1.0)
				{
					timeout_read_ASF.Tic();
					cout << "[PFLocalizer] Warning: *** Waiting for Action/SF from the generator ***\n";
				}
				// no new ASF found, wait for it
				system::sleep(5);
				continue;
			}
			else {
				// reset ASF
				timeout_read_ASF.Tic();
			}

			// if there is no movement, skip the execution
			/* CPose2D pose_change;
			action.getBestMovementEstimation()->poseChange->getMean(pose_change);
			if (pose_change.x() == 0.0 && pose_change.y() == 0.0)
			{
				continue;
			} */

			// Execute the algorithm
			{
				synch::CCriticalSectionLocker pdfLocker(&botData->cs_pdf);
				PF.executeOn(
					pdf,
					&action,			// Action
					&sf,	// Obs.
					&PF_stats		// Output statistics
				);
			}

			// Update the location in BotModel
			{
				synch::CCriticalSectionLocker lock(&botData->cs_location);
				synch::CCriticalSectionLocker pdfLocker(&botData->cs_pdf);
				pdf.getCovarianceAndMean(botData->locationCov, botData->locationEstimation);
			}

			system::sleep(PF_sleeptime);
		}


	}
	catch (std::exception &e)
	{
		cerr << "[PFLocalizer]  Closing due to exception:\n" << e.what() << endl;
		botData->allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[PFLocalizer] Untyped exception! Closing." << endl;
		botData->allThreadsMustExit = true;
	}
}


// ------------------------------------------------------
//				Reactive Navigation Thread
// ------------------------------------------------------
void white_bot_2App::ReactiveNavThread(BotModel* botData)
{
	using namespace mrpt::nav;
	try
	{
		RobotNavInterface nav_interface(botData);

		// read confid
		int sleepTime_react = botData->cfgFile->read_int("GLOBAL", "sleeptime_React", 30, true);

		// Build a new navigator:
		botData->react_nav = new mrpt::nav::CReactiveNavigationSystem(nav_interface);
		CReactiveNavigationSystem* react = botData->react_nav;

		react->enableKeepLogRecords();
		react->enableLogFile(true);

		ASSERT_(react);

		// Load params:
		react->loadConfigFile(*botData->cfgFile);
		react->initialize();

		while (!botData->allThreadsMustExit)
		{
			// Navigate:
			if (botData->isStart && !botData->isManual)
			{
				react->navigationStep();
			}

			sleep(sleepTime_react);
		}
	}
	catch (std::exception &e)
	{
		cerr << "[ReactiveNav]  Closing due to exception:\n" << e.what() << endl;
		botData->allThreadsMustExit = true;
	}
	catch (...)
	{
		cerr << "[ReactiveNav] Untyped exception! Closing." << endl;
		botData->allThreadsMustExit = true;
	}
}