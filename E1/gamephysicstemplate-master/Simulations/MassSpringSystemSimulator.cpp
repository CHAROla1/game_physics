#include "MassSpringSystemSimulator.h"

/*
UI执行顺序
notifyCaseChanged
initUI
simulateTimestep
drawFrame
*/
#define sphere_size 0.025

// ====== MassSpringSystemSimulator begin ======
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	cout << "Simulator constructor " << endl;
	vector<Point> points;
	this->m_points = points;

	vector<Spring> springs;
	this->m_springs = springs;
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler Method, Leap_Frog, Midpoint Method";
}

// ÿ��һ��test�ͻ�ִ��һ��
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;

	switch (m_iTestCase)
	{
	case 0:
		cout << "initUI Euler " << endl;
		break;
	case 1:
		cout << "initUI Leap-Frog " << endl;
		break;
	case 2:
		cout << "initUI Midpoint " << endl;
		break;
	}

	this->m_points.clear();
	this->m_springs.clear();

	Point p0(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
	p0.mass = 10;
	Point p1(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
	p1.mass = 10;
	this->m_points.push_back(p0);
	this->m_points.push_back(p1);

	Spring s0_1(0, 1, 1);
	s0_1.setStiffness(40);
	this->m_springs.push_back(s0_1);
}
void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);

	for (int i = 0; i < this->m_springs.size(); i++) {
		this->DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		this->DUC->drawSphere(this->m_points.at(this->m_springs.at(i).point1).position, Vec3(sphere_size, sphere_size, sphere_size));
		this->DUC->drawSphere(this->m_points.at(this->m_springs.at(i).point2).position, Vec3(sphere_size, sphere_size, sphere_size));
		this->DUC->beginLine();
		this->DUC->drawLine(this->m_points.at(this->m_springs.at(i).point1).position, Vec3(0, 1, 0), this->m_points.at(this->m_springs.at(i).point2).position, Vec3(0, 1, 0));
		this->DUC->endLine();
	}
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Changed to Euler Method !\n";
		break;
	case 1:
		cout << "Changed to Leap_Frog !\n";
		break;
	case 2:
		cout << "Changed to Midpoint Method !\n";
		break;
	}
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
	switch (this->m_iTestCase) {
	case 0: //euler method
		EulerMethod(timeStep);
		break;
	case 1:
		LeapFrogMethod(timeStep);
		break;
	case 2:
		MidpointMethod(timeStep);
		break;
	}
}

void MassSpringSystemSimulator::onClick(int x, int y) {}
void MassSpringSystemSimulator::onMouse(int x, int y) {}

void MassSpringSystemSimulator::setMass(float mass) {
	this->m_fMass = mass;
}
void MassSpringSystemSimulator::setStiffness(float stiffness) {
	this->m_fStiffness = stiffness;
}
void MassSpringSystemSimulator::setDampingFactor(float damping) {
	this->m_fDamping = damping;
}
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
	Point p(position, Velocity, isFixed);
	this->m_points.push_back(p);
	return 0;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	Spring sp(masspoint1, masspoint2, initialLength);
	this->m_springs.push_back(sp);
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return this->m_points.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return this->m_springs.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return this->m_points.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return this->m_points.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	this->m_externalForce = force;
}

// wufei yang
void MassSpringSystemSimulator::HeunMethod(float timeStep) {
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring s = this->m_springs.at(i);
		Point p1 = this->m_points.at(s.point1);
		Point p2 = this->m_points.at(s.point2);

		float dis = norm(p1.position - p2.position);
		Vec3 force_p1 = -s.stiffness * (dis - s.initialLength) * (p1.position - p2.position) / dis;
		Vec3 old_p1_acc = force_p1 / p1.mass;
		Vec3 old_p2_acc = -force_p1 / p2.mass;

		// perform euler step as helper;
		Vec3 euler_p1_vel = p1.velocity + timeStep * old_p1_acc;
		Vec3 euler_p2_vel = p2.velocity + timeStep * old_p2_acc;

		Vec3 euler_p1_pos = p1.position + timeStep * p1.velocity;
		Vec3 euler_p2_pos = p2.position + timeStep * p2.velocity;

		// calculate new position;
		p1.position = p1.position + timeStep / 2 * (euler_p1_vel + p1.velocity);
		p2.position = p2.position + timeStep / 2 * (euler_p2_vel + p2.velocity);

		// calculate acceleration at euler step position;
		dis = norm(p1.position - p2.position);
		force_p1 = -s.stiffness * (dis - s.initialLength) * (p1.position - p2.position) / dis;
		Vec3 euler_p1_acc = force_p1 / p1.mass;
		Vec3 euler_p2_acc = -force_p1 / p2.mass;

		// calculate new velocity;
		p1.velocity = p1.velocity + timeStep / 2 * (euler_p1_acc + old_p1_acc);
		p2.velocity = p2.velocity + timeStep / 2 * (euler_p2_acc + old_p2_acc);
	}
}

// wufe yang
void MassSpringSystemSimulator::EulerMethod(float timeStep) {
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		cout << "current position p1: " << p1->position << endl;
		cout << "current position p2: " << p2->position << endl;
		cout << "==========================================" << endl;

		// calculate new position;
		p1->position = p1->position + timeStep * p1->velocity;
		p2->position = p2->position + timeStep * p2->velocity;

		// calculte new velocity;
		float dis = norm(p1->position - p2->position);
		Vec3 force_p1 = -s->stiffness * (dis - s->initialLength) * (p1->position - p2->position) / dis;
		Vec3 p1_acc = force_p1 / p1->mass;
		Vec3 p2_acc = -force_p1 / p2->mass;

		p1->velocity = p1->velocity + timeStep * p1_acc;
		p2->velocity = p2->velocity + timeStep * p2_acc;
	}
}

void MassSpringSystemSimulator::MidpointMethod(float timeStep) {
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		cout << "current position p1: " << p1->position << endl;
		cout << "current position p2: " << p2->position << endl;

		float dis = norm(p1->position - p2->position);
		Vec3 force_p1 = -s->stiffness * (dis - s->initialLength) * (p1->position - p2->position) / dis;
		Vec3 old_p1_acc = force_p1 / p1->mass;
		Vec3 old_p2_acc = -force_p1 / p2->mass;

		// perform midpoint step;
		Vec3 mid_p1_pos = p1->position + timeStep / 2 * p1->velocity;
		Vec3 mid_p2_pos = p2->position + timeStep / 2 * p2->velocity;

		Vec3 mid_p1_vel = p1->velocity + timeStep / 2 * old_p1_acc;
		Vec3 mid_p2_vel = p2->velocity + timeStep / 2 * old_p2_acc;

		// calculate acceleration at mid point;
		dis = norm(mid_p1_pos - mid_p2_pos);
		force_p1 = -s->stiffness * (dis - s->initialLength) * (mid_p1_pos - mid_p2_pos) / dis;
		Vec3 mid_p1_acc = force_p1 / p1->mass;
		Vec3 mid_p2_acc = -force_p1 / p2->mass;

		// use midstep values to perform full step;
		p1->position = p1->position + timeStep * mid_p1_vel;
		p2->position = p2->position + timeStep * mid_p2_vel;

		p1->velocity = p1->velocity + timeStep * mid_p1_acc;
		p2->velocity = p2->velocity + timeStep * mid_p2_acc;
	}
}

void MassSpringSystemSimulator::LeapFrogMethod(float timeStep) {
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		cout << "current position p1: " << p1->position << endl;
		cout << "current position p2: " << p2->position << endl;

		float dis = norm(p1->position - p2->position);
		Vec3 force_p1 = -s->stiffness * (dis - s->initialLength) * (p1->position - p2->position) / dis;
		Vec3 old_p1_acc = force_p1 / p1->mass;
		Vec3 old_p2_acc = -force_p1 / p2->mass;

		Vec3 new_p1_vel = p1->velocity + timeStep * old_p1_acc;
		Vec3 new_p2_vel = p2->velocity + timeStep * old_p2_acc;

		p1->position = p1->position + timeStep * new_p1_vel;
		p2->position = p2->position + timeStep * new_p2_vel;
		p1->velocity = new_p1_vel;
		p2->velocity = new_p2_vel;
	}
}

// ====== MassSpringSystemSimulator end ======


// ====== Spring begin ======
void Spring::setStiffness(float stiffness) {
	this->stiffness = stiffness;
}
// ====== Spring end ======