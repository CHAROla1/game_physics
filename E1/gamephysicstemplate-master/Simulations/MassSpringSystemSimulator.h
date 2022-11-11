#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

/* hang */
struct Spring {
	massPoint* masspoint1;
	massPoint* masspoint2;
	float initialLength;
	Spring(massPoint* p1, massPoint* p2, float l, float s): masspoint1(p1), masspoint2(p2), initialLength(l) {};
}

struct massPoint {
	Vec3 position;
	Vec3 velocity;
	bool isFixed;
	Vec3 force;
	massPoint(Vec3 p, Vec3 v, bool f, Vec3 force = Vec3()): position(p), velocity(v), isFixed(f) {};
}
/* hang */

class MassSpringSystemSimulator:public Simulator{
public:
	// Construtors
	MassSpringSystemSimulator();

	// decstructors
	~MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	// integrator
	void eulerIntegrator(float timeStep);
	void midpointIntergrator(float timeStep);
	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	/** hang */
	vector<massPoint*> massPoints;
	vector<Spring*> springs;
	void addForce(massPoint* point);
	void updateLength(Spring* spring);
	pair<Vec3, Vec3> eulerHelper(float timeStep, massPoint* point);
	

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	float m_scale;
};
#endif