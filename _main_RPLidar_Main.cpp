/***************************************************************
 * Name:      white_bot_2Main.cpp
 * Purpose:   Code for Application Frame
 * Author:    Khemarat Boonyapaluk (korla.march@gmail.com)
 * Created:   2018-07-16
 * Copyright: Khemarat Boonyapaluk (korlamarch.com)
 * License:
 **************************************************************/

#include "wx_pch.h"
#include "white_bot_2Main.h"
#include <wx/msgdlg.h>

//(*InternalHeaders(white_bot_2Frame)
#include <wx/intl.h>
#include <wx/settings.h>
#include <wx/string.h>
//*)

//helper functions
enum wxbuildinfoformat {
    short_f, long_f };

wxString wxbuildinfo(wxbuildinfoformat format)
{
    wxString wxbuild(wxVERSION_STRING);

    if (format == long_f )
    {
#if defined(__WXMSW__)
        wxbuild << _T("-Windows");
#elif defined(__UNIX__)
        wxbuild << _T("-Linux");
#endif

#if wxUSE_UNICODE
        wxbuild << _T("-Unicode build");
#else
        wxbuild << _T("-ANSI build");
#endif // wxUSE_UNICODE
    }

    return wxbuild;
}

//(*IdInit(white_bot_2Frame)
const long white_bot_2Frame::ID_CUSTOM1 = wxNewId();
const long white_bot_2Frame::ID_CUSTOM2 = wxNewId();
const long white_bot_2Frame::ID_TEXTCTRL1 = wxNewId();
const long white_bot_2Frame::ID_BUTTON1 = wxNewId();
const long white_bot_2Frame::ID_BUTTON2 = wxNewId();
const long white_bot_2Frame::ID_BUTTON3 = wxNewId();
const long white_bot_2Frame::ID_TOGGLEBUTTON1 = wxNewId();
const long white_bot_2Frame::ID_BUTTON8 = wxNewId();
const long white_bot_2Frame::ID_BUTTON4 = wxNewId();
const long white_bot_2Frame::ID_BUTTON5 = wxNewId();
const long white_bot_2Frame::ID_BUTTON6 = wxNewId();
const long white_bot_2Frame::ID_TOGGLEBUTTON2 = wxNewId();
const long white_bot_2Frame::ID_BUTTON7 = wxNewId();
const long white_bot_2Frame::ID_MENUITEM1 = wxNewId();
const long white_bot_2Frame::idMenuAbout = wxNewId();
const long white_bot_2Frame::ID_STATUSBAR1 = wxNewId();
const long white_bot_2Frame::ID_TIMER1 = wxNewId();
//*)

BEGIN_EVENT_TABLE(white_bot_2Frame,wxFrame)
    //(*EventTable(white_bot_2Frame)
    //*)
END_EVENT_TABLE()

white_bot_2Frame::white_bot_2Frame(wxWindow* parent, wxWindowID id, BotModel* _botData)
	: botData(_botData)
{
    //(*Initialize(white_bot_2Frame)
    wxBoxSizer* BoxSizer1;
    wxBoxSizer* BoxSizer2;
    wxBoxSizer* BoxSizer3;
    wxFlexGridSizer* FlexGridSizer1;
    wxFlexGridSizer* FlexGridSizer2;
    wxMenu* Menu1;
    wxMenu* Menu2;
    wxMenuBar* MenuBar1;
    wxMenuItem* MenuItem1;
    wxMenuItem* MenuItem2;
    wxStaticBoxSizer* StaticBoxSizer1;
    wxStaticBoxSizer* StaticBoxSizer2;
    wxStaticBoxSizer* StaticBoxSizer3;

    Create(parent, wxID_ANY, _("White Bot Control"), wxDefaultPosition, wxDefaultSize, wxDEFAULT_FRAME_STYLE, _T("wxID_ANY"));
    SetClientSize(wxSize(306,352));
    SetMinSize(wxSize(750,500));
    SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));
    FlexGridSizer1 = new wxFlexGridSizer(2, 1, 0, 0);
    FlexGridSizer1->AddGrowableCol(0);
    FlexGridSizer1->AddGrowableRow(0);
    BoxSizer2 = new wxBoxSizer(wxHORIZONTAL);
    StaticBoxSizer3 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("3D View"));
    m_plot3D = new CMyGLCanvas(this,ID_CUSTOM1,wxDefaultPosition,wxSize(400,250),wxTAB_TRAVERSAL,_T("ID_CUSTOM1"));
    StaticBoxSizer3->Add(m_plot3D, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE, 5);
    BoxSizer2->Add(StaticBoxSizer3, 2, wxALL|wxEXPAND, 5);
    BoxSizer3 = new wxBoxSizer(wxVERTICAL);
    StaticBoxSizer1 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Sensor View"));
    m_plotLaser = new CMyGLCanvas(this,ID_CUSTOM2,wxDefaultPosition,wxSize(200,100),wxTAB_TRAVERSAL,_T("ID_CUSTOM2"));
    StaticBoxSizer1->Add(m_plotLaser, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE, 5);
    BoxSizer3->Add(StaticBoxSizer1, 3, wxALL|wxEXPAND, 5);
    StaticBoxSizer2 = new wxStaticBoxSizer(wxHORIZONTAL, this, _("Robot Status"));
    textStatus = new wxTextCtrl(this, ID_TEXTCTRL1, wxEmptyString, wxDefaultPosition, wxSize(150,72), wxTE_MULTILINE|wxTE_READONLY, wxDefaultValidator, _T("ID_TEXTCTRL1"));
    StaticBoxSizer2->Add(textStatus, 1, wxALL|wxEXPAND, 5);
    BoxSizer3->Add(StaticBoxSizer2, 2, wxALL|wxEXPAND, 5);
    BoxSizer2->Add(BoxSizer3, 1, wxEXPAND, 5);
    FlexGridSizer1->Add(BoxSizer2, 1, wxALL|wxEXPAND|wxFIXED_MINSIZE, 5);
    BoxSizer1 = new wxBoxSizer(wxHORIZONTAL);
    FlexGridSizer2 = new wxFlexGridSizer(2, 5, 0, 0);
    btnStart = new wxButton(this, ID_BUTTON1, _("Start"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON1"));
    FlexGridSizer2->Add(btnStart, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnManual = new wxButton(this, ID_BUTTON2, _("Manual"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON2"));
    FlexGridSizer2->Add(btnManual, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnReset = new wxButton(this, ID_BUTTON3, _("Reset Nav"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON3"));
    FlexGridSizer2->Add(btnReset, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSetTarget = new wxToggleButton(this, ID_TOGGLEBUTTON1, _("Set Target"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TOGGLEBUTTON1"));
    FlexGridSizer2->Add(btnSetTarget, 1, wxALL|wxEXPAND, 5);
    btnFindPath = new wxButton(this, ID_BUTTON8, _("Find Path"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON8"));
    FlexGridSizer2->Add(btnFindPath, 1, wxALL, 5);
    btnStop = new wxButton(this, ID_BUTTON4, _("Stop"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON4"));
    FlexGridSizer2->Add(btnStop, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnAuto = new wxButton(this, ID_BUTTON5, _("Auto"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON5"));
    FlexGridSizer2->Add(btnAuto, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnLoadMap = new wxButton(this, ID_BUTTON6, _("Load Map"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON6"));
    FlexGridSizer2->Add(btnLoadMap, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnSetWaypoints = new wxToggleButton(this, ID_TOGGLEBUTTON2, _("Set Waypoints"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_TOGGLEBUTTON2"));
    FlexGridSizer2->Add(btnSetWaypoints, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    btnResetPDF = new wxButton(this, ID_BUTTON7, _("Reset PDF"), wxDefaultPosition, wxDefaultSize, 0, wxDefaultValidator, _T("ID_BUTTON7"));
    FlexGridSizer2->Add(btnResetPDF, 1, wxALL|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 5);
    BoxSizer1->Add(FlexGridSizer2, 1, wxBOTTOM|wxLEFT|wxALIGN_CENTER_HORIZONTAL|wxALIGN_CENTER_VERTICAL, 10);
    FlexGridSizer1->Add(BoxSizer1, 1, wxALL|wxALIGN_LEFT|wxALIGN_CENTER_VERTICAL, 5);
    SetSizer(FlexGridSizer1);
    MenuBar1 = new wxMenuBar();
    Menu1 = new wxMenu();
    MenuItem1 = new wxMenuItem(Menu1, ID_MENUITEM1, _("Quit\tAlt-F4"), _("Quit the application"), wxITEM_NORMAL);
    Menu1->Append(MenuItem1);
    MenuBar1->Append(Menu1, _("&File"));
    Menu2 = new wxMenu();
    MenuItem2 = new wxMenuItem(Menu2, idMenuAbout, _("About\tF1"), _("Show info about this application"), wxITEM_NORMAL);
    Menu2->Append(MenuItem2);
    MenuBar1->Append(Menu2, _("Help"));
    SetMenuBar(MenuBar1);
    StatusBar1 = new wxStatusBar(this, ID_STATUSBAR1, 0, _T("ID_STATUSBAR1"));
    int __wxStatusBarWidths_1[1] = { -1 };
    int __wxStatusBarStyles_1[1] = { wxSB_NORMAL };
    StatusBar1->SetFieldsCount(1,__wxStatusBarWidths_1);
    StatusBar1->SetStatusStyles(1,__wxStatusBarStyles_1);
    SetStatusBar(StatusBar1);
    timUpdate3D.SetOwner(this, ID_TIMER1);
    timUpdate3D.Start(100, false);
    SetSizer(FlexGridSizer1);
    Layout();

    Connect(ID_BUTTON1,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnStartClick);
    Connect(ID_BUTTON2,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnManualClick);
    Connect(ID_BUTTON3,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnResetClick);
    Connect(ID_BUTTON8,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnSetWaypointsToggle);
    Connect(ID_BUTTON4,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnStopClick);
    Connect(ID_BUTTON5,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnAutoClick);
    Connect(ID_BUTTON6,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnLoadMapClick);
	Connect(ID_TOGGLEBUTTON1,wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnSetTargetToggle);
    Connect(ID_TOGGLEBUTTON2,wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnSetWaypointsToggle);
    Connect(ID_BUTTON7,wxEVT_COMMAND_BUTTON_CLICKED,(wxObjectEventFunction)&white_bot_2Frame::OnbtnResetPDFClick);
    Connect(ID_MENUITEM1,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&white_bot_2Frame::OnQuit);
    Connect(idMenuAbout,wxEVT_COMMAND_MENU_SELECTED,(wxObjectEventFunction)&white_bot_2Frame::OnAbout);
    Connect(ID_TIMER1,wxEVT_TIMER,(wxObjectEventFunction)&white_bot_2Frame::OntimUpdate3DTrigger);
    //*)

	// load config
	float SENSOR_RADIUS = botData->cfgFile->read_float("LIDAR_SENSOR", "max_length", 1.0, false);

	// Set Auto/Manual Button
	if (botData->isManual)
	{
		btnManual->Disable();
		btnAuto->Enable();
	}
	else {
		btnAuto->Disable();
		btnManual->Enable();
	}

	// Set Start/Stop Button
	if (botData->isStart)
	{
		btnStart->Disable();
		btnStop->Enable();
	}
	else {
		btnStart->Enable();
		btnStop->Disable();
	}

	// Connect 3D plot to the event listener
	m_plot3D->Connect(wxEVT_MOTION, (wxObjectEventFunction)&white_bot_2Frame::Onplot3DMouseMove, 0, this);
	m_plot3D->Connect(wxEVT_LEFT_DOWN, (wxObjectEventFunction)&white_bot_2Frame::Onplot3DMouseClick, 0, this);
	m_plot3D->Connect(wxEVT_RIGHT_DOWN, (wxObjectEventFunction)&white_bot_2Frame::Onplot3DMouseClick, 0, this);

	// Redirect Console Output to Textbox
	wxStreamToTextRedirector redirect(textStatus);
	//cout.rdbuf(textStatus);
	//cout << "Begin of cout redirect" << endl;

	WX_START_TRY

	// Populate 3D views:
	// -------------------------------
	{
		mrpt::opengl::CGridPlaneXYPtr obj = mrpt::opengl::CGridPlaneXY::Create(-50, 50, -50, 50, 0, 1);
		obj->setColor_u8(mrpt::utils::TColor(30, 30, 30, 50));
		m_plot3D->m_openGLScene->insert(obj);
	}

	gl_grid = mrpt::opengl::CSetOfObjects::Create();
	m_plot3D->m_openGLScene->insert(gl_grid);
	this->updateMap3DView();

	// Create Robot
	gl_robot_local = mrpt::opengl::CSetOfObjects::Create();
	gl_robot = mrpt::opengl::CSetOfObjects::Create();
	{
		mrpt::opengl::CSetOfObjectsPtr gl_robot_render = mrpt::opengl::CSetOfObjects::Create();
		gl_robot_render->setName("robot_render");
		gl_robot_render->insert(mrpt::opengl::stock_objects::RobotPioneer());
		gl_robot->insert(gl_robot_render);
	}

	m_plot3D->m_openGLScene->insert(gl_robot);

	gl_scan3D = mrpt::opengl::CPlanarLaserScan::Create();
	gl_scan3D->enableLine(false);
	gl_scan3D->enableSurface(true);
	gl_scan3D->setPointsWidth(3.0);
	gl_scan3D->setSurfaceColor(0.0f, 0.0f, 1.0f, 0.3f);
	gl_robot->insert(gl_scan3D);

	gl_robot_sensor_range = mrpt::opengl::CDisk::Create(0, 0);
	gl_robot_sensor_range->setColor_u8(TColor(0, 255, 0, 90));
	gl_robot_sensor_range->setLocation(0, 0, 0.01);
	gl_robot_sensor_range->setDiskRadius(SENSOR_RADIUS*1.01, SENSOR_RADIUS*0.99);
	gl_robot->insert(gl_robot_sensor_range);

	gl_robot_path = mrpt::opengl::CSetOfLines::Create();
	gl_robot_path->setLineWidth(1);
	gl_robot_path->setColor_u8(TColor(40, 40, 40, 200));
	m_plot3D->m_openGLScene->insert(gl_robot_path);

	// The pf locatizer's cov:
	gl_loc_cov = mrpt::opengl::CEllipsoid::Create();
	gl_loc_cov->setColor(1, 0, 0, 0.6);
	gl_loc_cov->setLineWidth(4);
	gl_loc_cov->setQuantiles(3);
	gl_loc_cov->set2DsegmentsCount(60);
	m_plot3D->m_openGLScene->insert(gl_loc_cov);


	gl_target = mrpt::opengl::CSetOfObjects::Create();
	gl_target->setVisibility(false);
	{	// Sign of "target"
		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create(1, 0, 0, 0.2f, 0, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1, 0, 0, -0.2f, 0, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0, 1, 0, 0, 0.2f, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); gl_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0, -1, 0, 0, -0.2f, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); gl_target->insert(obj);
		m_plot3D->m_openGLScene->insert(gl_target);
	}

	{
		gl_waypoints_clicking = opengl::CSetOfObjects::Create();
		m_plot3D->m_openGLScene->insert(gl_waypoints_clicking);

		gl_waypoints_status = opengl::CSetOfObjects::Create();
		m_plot3D->m_openGLScene->insert(gl_waypoints_status);
	}

	{	// Sign of "picking a navigation target":
		m_gl_placing_nav_target = opengl::CSetOfObjects::Create();

		mrpt::opengl::CArrowPtr obj;
		obj = mrpt::opengl::CArrow::Create(1, 0, 0, 0.2f, 0, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(-1, 0, 0, -0.2f, 0, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0, 1, 0, 0, 0.2f, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); m_gl_placing_nav_target->insert(obj);
		obj = mrpt::opengl::CArrow::Create(0, -1, 0, 0, -0.2f, 0, 0.4f, 0.05f, 0.15f); obj->setColor_u8(TColor(0, 0, 255)); m_gl_placing_nav_target->insert(obj);
		m_gl_placing_nav_target->setVisibility(false); // Start invisible.
		m_plot3D->m_openGLScene->insert(m_gl_placing_nav_target);
	}

	m_plot3D->m_openGLScene->insert(mrpt::opengl::stock_objects::CornerXYZ(1));

	gl_robot_ptg_prediction = mrpt::opengl::CSetOfLines::Create();
	gl_robot_ptg_prediction->setName("ptg_prediction");
	gl_robot_ptg_prediction->setLineWidth(2.0);
	gl_robot_ptg_prediction->setColor_u8(mrpt::utils::TColor(0x00, 0x00, 0xff));
	gl_robot->insert(gl_robot_ptg_prediction);

	// *** gl_robot_ptg_prediction->clear();

	// Set camera:
	m_plot3D->cameraPointingX = 0;
	m_plot3D->cameraPointingY = 0;
	m_plot3D->cameraPointingZ = 0;
	m_plot3D->cameraZoomDistance = 40;
	m_plot3D->cameraElevationDeg = 70;
	m_plot3D->cameraAzimuthDeg = -100;
	m_plot3D->cameraIsProjective = true;

	// Update positions of stuff:
	this->updateOpenglObjects();

	WX_END_TRY
}

white_bot_2Frame::~white_bot_2Frame()
{
    //(*Destroy(white_bot_2Frame)
    //*)
}

void white_bot_2Frame::updateMap3DView()
{
	gl_grid->clear();
	botData->metricMap.getAs3DObject(gl_grid);
}

void white_bot_2Frame::updateOpenglObjects()
{
	poses::CPose2D loc;
	math::CMatrixDouble33 cov;
	{
		mrpt::synch::CCriticalSectionLocker lock(&botData->cs_location);
		loc = botData->locationEstimation;
		cov = botData->locationCov;
	}

	gl_robot->setPose(loc);
	// animate target:
	{
		const double TARGET_BOUNCE_MIN = 0.7;
		const double TARGET_BOUNCE_MAX = 1;

		const double TARGET_BOUNCE_PERIOD = 1.0;
		const double t = fmod(m_runtime.Tac(), TARGET_BOUNCE_PERIOD) / TARGET_BOUNCE_PERIOD;

		// Parabolic path
		const double s = 4 * t*(TARGET_BOUNCE_MAX - TARGET_BOUNCE_MIN)*(1 - t) + TARGET_BOUNCE_MIN;

		gl_target->setLocation(botData->targetPose.x, botData->targetPose.y, 0);
		gl_target->setScale(s);
	}

	// update the status
	textStatus->Clear();
	*textStatus << "odometry: X" << botData->odometry.x() << " Y" << botData->odometry.y() << " A" << botData->odometry.phi() << "\n";
	*textStatus << "Encoder L:" << botData->encoder_left << " R:" << botData->encoder_right << "\n";
	*textStatus << "Start:" << botData->isStart << " Manual:" << botData->isManual << "\n";
	*textStatus << "Velocity L:" << botData->robotData.requestVelLeft << " R:" << botData->robotData.requestVelRight << "\n";
	*textStatus << "Battery " << botData->robotData.battCharge << "/" << botData->robotData.battCap << "\n";
	*textStatus << "Request Lin:" << botData->request_lin_vel << " Ang:" << botData->request_ang_vel << "\n";

	// update scan
	obs::CObservation2DRangeScanPtr obs;
	{
		synch::CCriticalSectionLocker lock(&botData->cs_ASF);
		obs = botData->sf.getObservationByClass<obs::CObservation2DRangeScan>();
		obs.make_unique();
	}
	if (obs)
	{
		gl_scan3D->setScan(*obs);
	}

	// Update path graph:
	const TPoint3D  cur_pt(loc.x(), loc.y(), 0.01);

	static int decim_path = 0;
	if (gl_robot_path->empty() || ++decim_path>10) {
		gl_robot_path->appendLine(cur_pt, cur_pt);
	}
	else {
		gl_robot_path->appendLineStrip(cur_pt);
		decim_path = 0;
	}

	// The pf locatizer's cov:
	gl_loc_cov->setLocation(loc.x(), loc.y(), 0);
	gl_loc_cov->setCovMatrix(cov, 2);

	// Add particles to 3D scene (for debugging propose)
	if(botData->pdf)
	{
		mrpt::opengl::CRenderizablePtr parts = m_plot3D->m_openGLScene->getByName("particles");
		if (parts) m_plot3D->m_openGLScene->removeObject(parts);

		mrpt::opengl::CSetOfObjectsPtr p;
		{
			synch::CCriticalSectionLocker pdflock(&botData->cs_pdf);
			p = botData->pdf->getAs3DObject<mrpt::opengl::CSetOfObjectsPtr>();
		}
		p->setName("particles");
		m_plot3D->m_openGLScene->insert(p);
	}

	// Draw a predicted path
	if (botData->react_nav)
	{
		nav::CAbstractPTGBasedReactive *ptg_nav = dynamic_cast<nav::CAbstractPTGBasedReactive *>(botData->react_nav);

		nav::CLogFileRecord lfr;
		if (ptg_nav) ptg_nav->getLastLogRecord(lfr);

		if (lfr.infoPerPTG.size() > 0 && ptg_nav)
		{
			// Selected PTG path:
			if (lfr.nSelectedPTG <= (int)ptg_nav->getPTG_count())  // the == case is for "NOP motion cmd"
			{
				const bool is_NOP_op = (lfr.nSelectedPTG == (int)ptg_nav->getPTG_count());
				const int idx_ptg = is_NOP_op ? lfr.ptg_index_NOP : lfr.nSelectedPTG;

				mrpt::nav::CParameterizedTrajectoryGenerator* ptg = idx_ptg >= 0 ? ptg_nav->getPTG(idx_ptg) : nullptr;
				if (ptg)
				{
					// Draw path:
					const int selected_k = is_NOP_op ? lfr.ptg_last_k_NOP : ptg->alpha2index(lfr.infoPerPTG[lfr.nSelectedPTG].desiredDirection);
					float max_dist = ptg->getRefDistance();
					gl_robot_ptg_prediction->clear();

					ptg->updateNavDynamicState(is_NOP_op ? lfr.ptg_last_navDynState : lfr.navDynState);

					ptg->renderPathAsSimpleLine(selected_k, *gl_robot_ptg_prediction, 0.10, max_dist);
					gl_robot_ptg_prediction->setColor_u8(mrpt::utils::TColor(0xff, 0x00, 0x00));

					// Place it:
					if (is_NOP_op) {
						gl_robot_ptg_prediction->setPose(lfr.rel_pose_PTG_origin_wrt_sense_NOP);
					}
					else {
						gl_robot_ptg_prediction->setPose(poses::CPose3D());
					}

					double min_shape_dists = 1.0;
					for (double d = min_shape_dists; d < max_dist; d += min_shape_dists)
					{
						uint32_t step;
						if (!ptg->getPathStepForDist(selected_k, d, step))
							continue;
						mrpt::math::TPose2D p;
						ptg->getPathPose(selected_k, step, p);
						ptg->add_robotShape_to_setOfLines(*gl_robot_ptg_prediction, mrpt::poses::CPose2D(p));
					}
				}
			}
		}
	}

	CWaypointsNavigator *wp_nav = dynamic_cast<CWaypointsNavigator *>(botData->react_nav);

	if (wp_nav)
	{
	static wxFrame *wxFrWpInfo = nullptr;
	static wxTextCtrl * edWpLog = nullptr;
	if (!wxFrWpInfo)
	{
	wxFrWpInfo = new wxFrame(this, -1, wxT("Waypoints info"), wxDefaultPosition, wxSize(400, 150), wxRESIZE_BORDER | wxMINIMIZE_BOX | wxMAXIMIZE_BOX | wxCAPTION | wxCLIP_CHILDREN | wxSTAY_ON_TOP);

	edWpLog = new wxTextCtrl(wxFrWpInfo, wxNewId(), wxEmptyString, wxDefaultPosition, wxSize(400, 150), wxTE_MULTILINE | wxTE_READONLY | wxTE_DONTWRAP | wxALWAYS_SHOW_SB, wxDefaultValidator, _T("ID_TEXTCTRL_WP"));
	edWpLog->SetMinSize(wxSize(190, 60));
	wxFont edLogFont(8, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxNORMAL, false, wxEmptyString, wxFONTENCODING_DEFAULT);
	edWpLog->SetFont(edLogFont);
	}

	TWaypointStatusSequence wp_status;
	wp_nav->getWaypointNavStatus(wp_status);
	const std::string sWpLog = wp_status.getAsText();

	if (!wp_status.waypoints.empty())
	if (!wxFrWpInfo->IsShown())
	wxFrWpInfo->Show();

	if (wxFrWpInfo->IsShown())
	edWpLog->SetValue(_U(sWpLog.c_str()));

	// Plot waypoints being clicked by the user graphically:
	m_waypoints_clicked.getAsOpenglVisualization(*gl_waypoints_clicking);

	// Plot firmly set waypoints and their status:
	wp_status.getAsOpenglVisualization(*gl_waypoints_status);
	}

	// Refresh:
	m_plot3D->Refresh();
}


void white_bot_2Frame::Onplot3DMouseMove(wxMouseEvent& event)
{
	int X, Y;
	event.GetPosition(&X, &Y);
	bool skip_normal_process = false;

	// Intersection of 3D ray with ground plane ====================
	TLine3D ray;
	m_plot3D->m_openGLScene->getViewport("main")->get3DRayForPixelCoord(X, Y, ray);
	// Create a 3D plane, e.g. Z=0
	const TPlane ground_plane(TPoint3D(0, 0, 0), TPoint3D(1, 0, 0), TPoint3D(0, 1, 0));
	// Intersection of the line with the plane:
	TObject3D inters;
	intersect(ray, ground_plane, inters);
	// Interpret the intersection as a point, if there is an intersection:
	TPoint3D inters_pt;
	if (inters.getPoint(inters_pt))
	{
		m_curCursorPos.x = inters_pt.x;
		m_curCursorPos.y = inters_pt.y;

		switch (m_cursorPickState)
		{
		case cpsPickTarget:
		case cpsPickWaypoints:
		{
			m_gl_placing_nav_target->setVisibility(true);
			m_gl_placing_nav_target->setLocation(m_curCursorPos.x, m_curCursorPos.y, 0.01);
		}
		break;
		default:
			break;
		};

		StatusBar1->SetStatusText(wxString::Format(wxT("X=%.03f Y=%.04f Z=0"), m_curCursorPos.x, m_curCursorPos.y), 2);
	}

	if (!skip_normal_process)
	{
		// Do normal process in that class:
		m_plot3D->OnMouseMove(event);
	}
}

void white_bot_2Frame::Onplot3DMouseClick(wxMouseEvent& event)
{
	using namespace mrpt::nav;

	WX_START_TRY

	bool skip_normal_process = false;

	switch (m_cursorPickState)
	{
	case cpsPickWaypoints:
	{
		if (event.ButtonIsDown(wxMOUSE_BTN_LEFT))
		{
			const bool allow_skip_wps = !event.ShiftDown();
			double heading = TWaypoint::INVALID_NUM;
			m_waypoints_clicked.waypoints.push_back(TWaypoint(m_curCursorPos.x, m_curCursorPos.y, 0.2 /* allowed dist */, allow_skip_wps, heading));
		}
		if (event.ButtonIsDown(wxMOUSE_BTN_RIGHT))
		{
			btnSetTarget->SetValue(false); btnSetTarget->Refresh();
			m_gl_placing_nav_target->setVisibility(false);

			CWaypointsNavigator *wp_nav = dynamic_cast<CWaypointsNavigator *>(botData->react_nav);
			if (wp_nav)
				wp_nav->navigateWaypoints(m_waypoints_clicked);

			m_waypoints_clicked.clear();

			m_plot3D->SetCursor(*wxSTANDARD_CURSOR); // End of cross cursor
			m_cursorPickState = cpsNone; // end of mode
			
			// Reset Buttons
			btnSetWaypoints->SetValue(m_cursorPickState == cpsPickWaypoints); btnSetWaypoints->Refresh();
			m_gl_placing_nav_target->setVisibility(m_cursorPickState == cpsPickWaypoints);
		}
	}
	break;
	case cpsPickTarget:
		if (event.ButtonIsDown(wxMOUSE_BTN_LEFT))
		{
			botData->targetPose = m_curCursorPos;

			btnSetTarget->SetValue(false);
			btnSetTarget->Refresh();
			m_gl_placing_nav_target->setVisibility(false);

			// Issue a new navigation cmd:
			CAbstractPTGBasedReactive::TNavigationParamsPTG   navParams;
			navParams.target.target_coords.x = botData->targetPose.x;
			navParams.target.target_coords.y = botData->targetPose.y;
			navParams.target.targetAllowedDistance = 0.40f;
			navParams.target.targetIsRelative = false;

			// Optional: restrict the PTGs to use
			//navParams.restrict_PTG_indices.push_back(1);
			ASSERT_(botData->react_nav != nullptr);

			botData->react_nav->navigate(&navParams);
			gl_target->setVisibility(true);

			m_plot3D->SetCursor(*wxSTANDARD_CURSOR); // End of cross cursor
			m_cursorPickState = cpsNone; // end of mode
		}
		break;
	default:
		break;
	}


	if (!skip_normal_process) {
		// Do normal process in that class:
		m_plot3D->OnMouseDown(event);
	}
	WX_END_TRY
}

void white_bot_2Frame::OnQuit(wxCommandEvent& event)
{
    Close();
}

void white_bot_2Frame::OnAbout(wxCommandEvent& event)
{
    wxString msg = wxbuildinfo(long_f);
    wxMessageBox(msg, _("Welcome to..."));
}

void white_bot_2Frame::OnbtnStartClick(wxCommandEvent& event)
{
	botData->isStart = true;

	btnStart->Disable();
	btnStop->Enable();
}

void white_bot_2Frame::OnbtnStopClick(wxCommandEvent& event)
{
	botData->isStart = false;

	btnStart->Enable();
	btnStop->Disable();
}

void white_bot_2Frame::OnbtnManualClick(wxCommandEvent& event)
{
	botData->isManual = true;

	btnManual->Disable();
	btnAuto->Enable();
}

void white_bot_2Frame::OnbtnAutoClick(wxCommandEvent& event)
{
	botData->isManual = false;

	btnAuto->Disable();
	btnManual->Enable();
}

void white_bot_2Frame::OnbtnLoadMapClick(wxCommandEvent& event)
{
}

void white_bot_2Frame::OnbtnSetTargetToggle(wxCommandEvent& event)
{
	if (m_cursorPickState != cpsPickTarget)
	{
		m_cursorPickState = cpsPickTarget;
		cout << "m_plot" << endl;
		m_plot3D->SetCursor(*wxCROSS_CURSOR);
	}
	else
	{	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor(*wxSTANDARD_CURSOR);
		m_gl_placing_nav_target->setVisibility(false);
	}
	btnSetTarget->SetValue(m_cursorPickState == cpsPickTarget);
	btnSetTarget->Refresh();
}


void white_bot_2Frame::OntimUpdate3DTrigger(wxTimerEvent& event)
{
	this->updateOpenglObjects();

}

void white_bot_2Frame::OnbtnResetClick(wxCommandEvent& event)
{
	m_gl_placing_nav_target->setVisibility(false);
	gl_target->setVisibility(false);
	m_waypoints_clicked.clear();
	gl_robot_path->clear();
	//botData->react_nav->cancel();
}

void white_bot_2Frame::OnbtnSetWaypointsToggle(wxCommandEvent& event)
{
	nav::CWaypointsNavigator *wp_nav = dynamic_cast<nav::CWaypointsNavigator *>(botData->react_nav);
	if (!wp_nav)
	{
		wxMessageBox(wxT("Navigator class does not support waypoints sequences!"));
		return;
	}

	if (m_cursorPickState != cpsPickWaypoints) {
		m_cursorPickState = cpsPickWaypoints;
		m_plot3D->SetCursor(*wxCROSS_CURSOR);
		m_waypoints_clicked.clear();
	}
	else {	// Cancel:
		m_cursorPickState = cpsNone;
		m_plot3D->SetCursor(*wxSTANDARD_CURSOR);
	}
	btnSetWaypoints->SetValue(m_cursorPickState == cpsPickWaypoints); btnSetWaypoints->Refresh();
	m_gl_placing_nav_target->setVisibility(m_cursorPickState == cpsPickWaypoints);
	m_plot3D->Refresh();
}

void white_bot_2Frame::OnbtnFindPathClick(wxCommandEvent& event)
{
}

void white_bot_2Frame::OnbtnResetPDFClick(wxCommandEvent& event)
{
	WX_START_TRY
	
	const string sect("PFLOCALIZER");
	int			particles_count;	// Number of initial particles (if size>1)
	slam::CMonteCarloLocalization2D  pdf;

	// Read Config
	particles_count = botData->cfgFile->read_int(sect, "particles_count", 0, true);

	float init_PDF_min_x = 0, init_PDF_min_y = 0, init_PDF_max_x = 0, init_PDF_max_y = 0;
	MRPT_LOAD_CONFIG_VAR(init_PDF_min_x, float, (*botData->cfgFile), sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_min_y, float, (*botData->cfgFile), sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_max_x, float, (*botData->cfgFile), sect)
	MRPT_LOAD_CONFIG_VAR(init_PDF_max_y, float, (*botData->cfgFile), sect)

	// Initialize the PDF:
	// -----------------------------
	{
		synch::CCriticalSectionLocker pdflocker(&botData->cs_pdf);
		if (!botData->cfgFile->read_bool(sect, "init_PDF_mode", false, /*Fail if not found*/true))
		{
			botData->pdf->resetUniformFreeSpace(
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
			botData->pdf->resetUniform(
				init_PDF_min_x, init_PDF_max_x,
				init_PDF_min_y, init_PDF_max_y,
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_min_phi_deg", -180)),
				DEG2RAD(botData->cfgFile->read_float(sect, "init_PDF_max_phi_deg", 180)),
				particles_count
			);
		}
	}


	printf("PDF of %u particles initialized\n", particles_count);

	WX_END_TRY
}
