#include<iostream>
#include<vector>
#include<string>
#include<sstream>

#include "MassSpringSystemSimulator.h"
#include "TemplateSimulator.h"

struct MassPoint {
	Vec3 position;
	Vec3 v;
	bool isFixed;
	Vec3 force;
	double mass;
};

struct Spring {
	int point1;
	int point2;
	double init_L;
	double stiffness;
};

std::vector<MassPoint> massPoints;
std::vector<Spring> springs;
// Vec3 gravity = (0, 0, -10);
int16_t gravity = -10;
float radius = 0.02;

// the bounding volume
int16_t x_upBound = 2;
int16_t x_lowBound = -2;
int16_t y_upBound = 2;
int16_t y_lowBound = 0;
int16_t z_upBound = 2;
int16_t z_lowBound = -2;


//constructor
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	m_fMass = 0;
	m_fStiffness = 0;
	m_iIntegrator = 0;
	m_fDamping = 0;
	m_externalForce = Vec3();
}



void new_Force(std::vector<Spring>& springs, std::vector<MassPoint>& massPoints, double damping)
{
	//Spring Forces
	for (auto& spring : springs) {
		MassPoint& point1 = massPoints[spring.point1];
		MassPoint& point2 = massPoints[spring.point2];
		//double lengthDif = sqrt(point1.position.squaredDistanceTo(point2.position)) - spring.init_L;
		double diff_length = norm(point1.position.operator-(point2.position)) - spring.init_L;
		double spring_f = spring.stiffness * diff_length;
		Vec3 direction = point2.position.operator-(point1.position);
		normalize(direction);
		point1.force.operator+=(direction * spring_f);
		point2.force.operator-=(direction * spring_f);
	}
	// damping & gravity
	for (auto& point : massPoints) {
		point.force.operator+=(- damping * point.v);
		point.force[1] += gravity;
		/*cout << " point2 force x: " << massPoints[1].force[0] << " " <<
			" point2 force y: " << massPoints[1].force[1] << " " <<
			" point2 force z: " << massPoints[1].force[2] << " " << endl;

		cout << " point1 force x: " << massPoints[0].force[0] << " " <<
			" point1 force y: " << massPoints[0].force[1] << " " <<
			" point1 force z: " << massPoints[0].force[2] << " " << endl;*/
		if (point.position[0] + radius >= x_upBound || point.position[0] - radius <= x_lowBound) {
			point.force = 0;
			point.v[0] = -point.v[0];
		}
		if (point.position[1] + radius >= y_upBound || point.position[1] - radius <= y_lowBound) {
			point.force = 0;
			point.v[1] = -point.v[1];
		}
		if (point.position[2] + radius >= z_upBound || point.position[2] - radius <= z_lowBound) {
			point.force = 0;
			point.v[2] = -point.v[2];
		}
	}

}

void euler(std::vector<Spring>& springs, std::vector<MassPoint>& points, float timestep, double damping) {
	
	for (auto& point : points) {
		
		if (!point.isFixed) {
			point.position.operator+=(timestep * point.v);
			point.v.operator+=(timestep * (point.force / point.mass));
			// cout << point.force << endl;
		}
		point.force = 0;
	}
	new_Force(springs, points, damping);
	/*cout << " point1 force x: " << points[0].force[0] << " " <<
		 " point1 force y: " << points[0].force[1] << " " <<
		 " point1 force z: " << points[0].force[2] << " " << endl;

	cout << " point2 force x: " << points[1].force[0] << " " <<
		" point2 force y: " << points[1].force[1] << " " <<
		" point2 force z: " << points[1].force[2] << " " << endl;*/
}

void midpoint(std::vector<Spring>& springs, std::vector<MassPoint>& points,
	float timestep, double damping) {
	std::vector<MassPoint> mid_points = points;

	euler(springs, mid_points, timestep * 0.5,damping);
	//int len = this->points.size();
	//springForce(springs, mid_points, damping);
	/*for (auto& point : points) {
		if (!point.isFixed)
		{
			point.position += timestep
			point.v += timestep
		}

	}*/
	for (size_t i = 0; i < points.size(); i++) {
		if (!points[i].isFixed)
		{
			points[i].position.operator+=(timestep * mid_points[i].v);
			points[i].v.operator+=(timestep * mid_points[i].force / points[i].mass);
		}
	}

	
	////return 0;
}


// UI functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler, Leap Frog, MidPoint, Complex";
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = 0;
	m_mouse.y = 0;
	m_trackmouse.x = 0;
	m_trackmouse.y = 0;
	m_oldtrackmouse.x = 0;
	m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;

	springs.clear();
	massPoints.clear();
	m_fDamping = 1;

	setMass(10);
	setStiffness(40);

	// collision = false;
	if (m_iTestCase < 3) {
		int p0 = addMassPoint(Vec3(0.1, 0.1, 0.1), Vec3(0, -1, 0), false);
		int p1 = addMassPoint(Vec3(0.1, 1, 0.1), Vec3(1, 0, 0.3), false);
		int p2 = addMassPoint(Vec3(1, 0.1, 0.1), Vec3(1, 0, -0.51), false);
		int p3 = addMassPoint(Vec3(0.1, 1, 0.1), Vec3(0, 1, -0.61), false);
		int p4 = addMassPoint(Vec3(1, 1, 1.5), Vec3(0.3, 0.82, -1), false);
		int p5 = addMassPoint(Vec3(0.1, 1, 0.1), Vec3(0.7, 0, -1), false);
		int p6 = addMassPoint(Vec3(0.3, 1, 0.4), Vec3(1, 0.8, -1), false);
		int p7 = addMassPoint(Vec3(1, 0.8, 0.1), Vec3(-0.8, -0.5, -1), false);
		int p8 = addMassPoint(Vec3(0, 0.4, 1), Vec3(1, 0, -1), false);
		int p9 = addMassPoint(Vec3(1.3, 0.52, 0.1), Vec3(1, -0.6, -1), false);
		int p10 = addMassPoint(Vec3(1, 1, 01.5), Vec3(0, 1.4, -1), false);
		addSpring(p0, p1, 1);
		addSpring(p9, p3, 0.6);
		addSpring(p0, p4, 0.9);
		addSpring(p1, p6, 2);
		addSpring(p5, p7, 1);
		addSpring(p7, p10, 1.5);
		addSpring(p2, p6, 1);
		addSpring(p7, p9, 1.2);
		addSpring(p8, p7, 0.9);
		addSpring(p8, p1, 1.1);

		/*int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		int p1 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, -1), false);
		addSpring(m0, m1, 1);*/
	}

	else {
		
	}


	/*switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}*/
}


void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Euler !\n";
		break;
	case 1:
		cout << "Leap Frog!\n";

		break;
	case 2:
		cout << "Midpoint !\n";
	
		break;

	default:
		cout << "Empty Test!\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse data to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		//todo
		//m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	}
	else {
		// TemplateSimulator::m_vfMovableObjectFinalPos = TemplateSimulator::m_vfMovableObjectPos;
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	if (!massPoints.empty()) {
		// FIXME

		switch (m_iTestCase)
		{// handling different cases
		case 0:
		{
			euler(springs, massPoints, timeStep, m_fDamping);
			break;
		}

		case 1: {
			//leapFrog(springs, massPoints, timeStep, m_fDamping);
			break;
		}

		case 2: {
			midpoint(springs, massPoints, timeStep, m_fDamping);
			break;
		}
		default:
			break;
		}

	}
	
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{

		for (int i = 0; i < massPoints.size(); i++) {
			DUC->drawSphere(massPoints[i].position, Vec3(radius, radius, radius));
		}
		
		for (int i = 0; i < springs.size(); i++) {
			DUC->beginLine();
			DUC->drawLine(massPoints[springs[i].point1].position, Vec3(0.1, 1.0, 1.0),
						  massPoints[springs[i].point2].position, Vec3(0.1, 1.0, 1.0));
			DUC->endLine();
		}
		
	

}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}



//simulation


void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
	for (auto& point : massPoints)
		point.mass = m_fMass;
}


void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
	for (auto& spring : springs)
		spring.stiffness = m_fStiffness;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}

int  MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	MassPoint point_add;
	point_add = { position,Velocity,isFixed,Vec3(),m_fMass };
	massPoints.push_back(point_add);
	return massPoints.size() - 1;
}

void  MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	if (massPoints.size() >= 2 && masspoint1 < massPoints.size() && masspoint2 < massPoints.size()) {
		Spring spring_add;
		spring_add.point1 = masspoint1;
		spring_add.point2 = masspoint2;
		spring_add.init_L = initialLength;
		spring_add.stiffness = m_fStiffness;
		springs.push_back(spring_add);
	}

}

int  MassSpringSystemSimulator::getNumberOfMassPoints() {
	return massPoints.size();
}

int  MassSpringSystemSimulator::getNumberOfSprings() {
	return springs.size();
}

Vec3  MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return massPoints[index].position;
}


Vec3  MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return massPoints[index].v;
}


//void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
//	// external_force = force; FIXME
//
//}
