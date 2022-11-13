#include "MassSpringSystemSimulator.h"
/*
UI执行顺序
notifyCaseChanged
initUI
simulateTimestep
drawFrame
*/
#define sphere_size 0.025
#define world_upper_bound 1
#define world_lower_bound -1
#define GRAVITY Vec3(0, -0.5, 0)

// ====== MassSpringSystemSimulator begin ======
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_iTestCase = 0;
	cout << "Simulator constructor " << endl;
	this->addGravity = false;
	this->collision = 0;
	vector<Point> points;
	this->m_points = points;

	vector<Spring> springs;
	this->m_springs = springs;
}

// UI Functions
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler Method, Leap_Frog, Midpoint Method";
}

// 每换一个test就会执行一次
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Gravity", TW_TYPE_BOOL8, &this->addGravity, "");
	TwType TW_TYPE_TESTCASE = TwDefineEnumFromString("Collision", "No, Ground floor, Walls");
	TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_TESTCASE, &this->collision, "");
	//TwAddVarRW(DUC->g_pTweakBar, "Collision", TW_TYPE_BOOL8, &this->collision, "");

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

	Point p0(Vec3(-0.5, -0.5, 0), Vec3(-0.1, 0, 0), false);
	Point p1(Vec3(-0.5, 0, 0), Vec3(-0.1, 0.1, 0), false);
	Point p2(Vec3(-0.5,0.5,0), Vec3(0, 0.1, 0), false);
	Point p3(Vec3(0, 0.5, 0), Vec3(0, -0.1, 0), false);
	Point p4(Vec3(0.5, 0.5, 0), Vec3(-0.1, 0, 0), false);
	Point p5(Vec3(0.5, 0, 0), Vec3(0.1, 0, 0), false);
	Point p6(Vec3(0.5, -0.5, 0), Vec3(0, 0.2, 0), false);
	Point p7(Vec3(0, -0.5, 0), Vec3(0, 0.1, 0), false);
	Point p8(Vec3(-0.25, 0, 0), Vec3(-0.1, 0, 0), false);
	Point p9(Vec3(0.25, 0, 0), Vec3(0.1, 0.1, 0), false);
	this->m_points.push_back(p0);
	this->m_points.push_back(p1);
	this->m_points.push_back(p2);
	this->m_points.push_back(p3);
	this->m_points.push_back(p4);
	this->m_points.push_back(p5);
	this->m_points.push_back(p6);
	this->m_points.push_back(p7);
	this->m_points.push_back(p8);
	this->m_points.push_back(p9);

	for (int i = 0; i < 7; i++) {
		this->m_springs.push_back(Spring(i, i + 1, 1));
	}
	this->m_springs.push_back(Spring(7, 0, 1));
	this->m_springs.push_back(Spring(3, 8, 1));
	this->m_springs.push_back(Spring(3, 9, 1));
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

	for (int i = 0; i < this->m_points.size(); i++) {
		this->DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		this->DUC->drawSphere(this->m_points.at(i).position, Vec3(sphere_size, sphere_size, sphere_size));
	}

	for (int i = 0; i < this->m_springs.size(); i++) {
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

void MassSpringSystemSimulator::onClick(int x, int y) {
	this->m_trackmouse.x = x;
	this->m_trackmouse.y = y;
}
void MassSpringSystemSimulator::onMouse(int x, int y) {
	this->m_oldtrackmouse.x = x;
	this->m_oldtrackmouse.y = y;
	this->m_trackmouse.x = x;
	this->m_trackmouse.y = y;
}

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

void MassSpringSystemSimulator::addForce() {
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		float dis = norm(p1->position - p2->position);
		Vec3 force_p1 = -s->stiffness * (dis - s->initialLength) * (p1->position - p2->position) / dis;

		p1->force += force_p1;
		p2->force += -force_p1;

		if (p1->isFixed) {
			p1->clearForce();
		}

		if (p2->isFixed) {
			p2->clearForce();
		}
	}
}

void MassSpringSystemSimulator::clearForce() {
	for (int i = 0; i < this->m_points.size(); i++) {
		this->m_points.at(i).clearForce();
	}
}

Vec3 MassSpringSystemSimulator::updateVelocity(const Point &p, const Vec3 &vel, const Vec3 &acc, float timeStep) {
	Vec3 result = vel + timeStep * acc;
	switch (this->collision) {
	case 0: // no collision
		break;

	case 1: // collision with ground floor
		if (p.position[1] - sphere_size <= world_lower_bound) {
			result[1] *= -1;
			//result[1] = this->addGravity ? -1 * (result[1] - GRAVITY[1]) : -1 * result[1];
		}
		break;

	case 2: // collision with walls
		// world x axis
		if (p.position[0] + sphere_size >= world_upper_bound || p.position[0] - sphere_size <= world_lower_bound) {
			result[0] *= -1;
		}
		// world y axis
		if (p.position[1] + sphere_size >= world_upper_bound || p.position[1] - sphere_size <= world_lower_bound) {
			result[1] *= -1;
			//result[1] = this->addGravity ? -1 * (result[1] - GRAVITY[1]) : -1 * result[1];
		}
		// world z axis
		if (p.position[2] + sphere_size >= world_upper_bound || p.position[2] - sphere_size <= world_lower_bound) {
			result[2] *= -1;
		}
		break;
	}
	return result;
}

Vec3 MassSpringSystemSimulator::updatePosition(const Vec3& position, const Vec3& vel, float timeStep) {
	Vec3 result = position + timeStep * vel;
	switch (this->collision)
	{
	case 0: // no collision
		break;
	case 1: // collision with ground floor
		if (result[1] - sphere_size < world_lower_bound) {
			result[1] = world_lower_bound + sphere_size;
		}
		break;
	case 2: // collision with walls
		// world x axis
		if (result[0] - sphere_size < world_lower_bound) {
			result[0] = world_lower_bound + sphere_size;
		}
		else if (result[0] + sphere_size > world_upper_bound) {
			result[0] = world_upper_bound - sphere_size;
		}

		// world y axis
		if (result[1] - sphere_size < world_lower_bound) {
			result[1] = world_lower_bound + sphere_size;
		}
		else if (result[1] + sphere_size > world_upper_bound) {
			result[1] = world_upper_bound - sphere_size;
		}

		// world z axis
		if (result[2] - sphere_size < world_lower_bound) {
			result[2] = world_lower_bound + sphere_size;
		}
		else if (result[2] + sphere_size > world_upper_bound) {
			result[2] = world_upper_bound - sphere_size;
		}
	}
	return result;
}

// wufe yang
void MassSpringSystemSimulator::EulerMethod(float timeStep) {
	this->addForce();
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		//cout << "current position p1" << p1->position << endl;
		//cout << "current position p2" << p2->position << endl;

		// calculate new position;
		p1->position = updatePosition(p1->position, p1->velocity, timeStep);
		p2->position = updatePosition(p2->position, p2->velocity, timeStep);

		// calculte new velocity;
		Vec3 p1_acc = p1->force / p1->mass;
		Vec3 p2_acc = p2->force / p2->mass;
		if (this->addGravity) {
			p1_acc = p1_acc + GRAVITY;
			p2_acc = p2_acc + GRAVITY;
		}

		p1->velocity = updateVelocity(*p1, p1->velocity, p1_acc, timeStep);
		p2->velocity = updateVelocity(*p2, p2->velocity, p2_acc, timeStep);
	}
	this->clearForce();
}

void MassSpringSystemSimulator::MidpointMethod(float timeStep) {
	vector<Point> old_points(this->m_points);

	this->addForce();
	for (int i = 0; i < this->m_springs.size(); i++) {
		// after for loop, all data in m_points and m_springs are of Midstep
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		cout << "current position p1" << p1->position << endl;
		cout << "current position p2" << p2->position << endl;

		Vec3 old_p1_acc = p1->force / p1->mass;
		Vec3 old_p2_acc = p2->force / p2->mass;
		if (this->addGravity) {
			old_p1_acc = old_p1_acc + GRAVITY;
			old_p2_acc = old_p2_acc + GRAVITY;
		}

		// perform midpoint step;
		//p1->position = p1->position + timeStep / 2 * p1->velocity;
		//p2->position = p2->position + timeStep / 2 * p2->velocity;
		p1->position = updatePosition(p1->position, p1->velocity, timeStep / 2);
		p2->position = updatePosition(p2->position, p2->velocity, timeStep / 2);

		//p1->velocity = p1->velocity + timeStep / 2 * old_p1_acc;
		//p2->velocity = p2->velocity + timeStep / 2 * old_p2_acc;
		p1->velocity = updateVelocity(*p1, p1->velocity, old_p1_acc, timeStep / 2);
		p2->velocity = updateVelocity(*p2, p2->velocity, old_p2_acc, timeStep / 2);
	}
	this->clearForce();

	this->addForce();
	for (int i = 0; i < this->m_springs.size(); i++) {
		// calculate acceleration at mid point;
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		Vec3 mid_p1_acc = p1->force / p1->mass;
		Vec3 mid_p2_acc = p2->force / p2->mass;
		if (this->addGravity) {
			mid_p1_acc = mid_p1_acc + GRAVITY;
			mid_p2_acc = mid_p2_acc + GRAVITY;
		}

		// use midstep values to perform full step;
		//p1->position = old_points.at(s->point1).position + timeStep * p1->velocity;
		//p2->position = old_points.at(s->point2).position + timeStep * p2->velocity;
		p1->position = updatePosition(old_points.at(s->point1).position, p1->velocity, timeStep);
		p2->position = updatePosition(old_points.at(s->point2).position, p2->velocity, timeStep);

		//p1->velocity = old_points.at(s->point1).velocity + timeStep * mid_p1_acc;
		//p2->velocity = old_points.at(s->point2).velocity + timeStep * mid_p2_acc;
		p1->velocity = updateVelocity(*p1, old_points.at(s->point1).velocity, mid_p1_acc, timeStep);
		p2->velocity = updateVelocity(*p2, old_points.at(s->point2).velocity, mid_p2_acc, timeStep);
	}
	this->clearForce();
}

void MassSpringSystemSimulator::LeapFrogMethod(float timeStep) {
	this->addForce();
	for (int i = 0; i < this->m_springs.size(); i++) {
		Spring* s = &this->m_springs.at(i);
		Point* p1 = &this->m_points.at(s->point1);
		Point* p2 = &this->m_points.at(s->point2);

		cout << "current position p1" << p1->position << endl;
		cout << "current position p2" << p2->position << endl;

		Vec3 old_p1_acc = p1->force / p1->mass;
		Vec3 old_p2_acc = p2->force / p2->mass;
		if (this->addGravity) {
			old_p1_acc = old_p1_acc + GRAVITY;
			old_p2_acc = old_p2_acc + GRAVITY;
		}

		//Vec3 new_p1_vel = p1->velocity + timeStep * old_p1_acc;
		//Vec3 new_p2_vel = p2->velocity + timeStep * old_p2_acc;
		Vec3 new_p1_vel = updateVelocity(*p1, p1->velocity, old_p1_acc, timeStep);
		Vec3 new_p2_vel = updateVelocity(*p2, p2->velocity, old_p2_acc, timeStep);

		//p1->position = p1->position + timeStep * new_p1_vel;
		//p2->position = p2->position + timeStep * new_p2_vel;
		p1->position = updatePosition(p1->position, new_p1_vel, timeStep);
		p2->position = updatePosition(p2->position, new_p2_vel, timeStep);

		p1->velocity = new_p1_vel;
		p2->velocity = new_p2_vel;
	}
	this->clearForce();
}

// ====== MassSpringSystemSimulator end ======