#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    setMass(10.0);
    setStiffness(40.0);
    setDampingFactor(0.0);
    m_externalForce = Vec3();
    m_iIntegrator = EULER;
    setScale(0.01, 0.01, 0.01);
}

MassSpringSystemSimulator::~MassSpringSystemSimulator() { 
    if (!m_massPoints.empty()) {
        for each (auto * point in m_massPoints) delete point;
        m_massPoints.clear();
    }
    if (!m_springs.empty()) {
        for each (auto * spring in m_springs) delete spring;
        m_springs.clear();
    }
}

float bound_x = 2.0f;
float bound_y = 2.0f;
float bound_z = 2.0f;

// --------------------------------- Specific Functions -------------------------------
void MassSpringSystemSimulator::setMass(float mass) {
    m_fMass = mass;
}

void MassSpringSystemSimulator::setDampingFactor(float dampingFactor) {
    m_fDamping = dampingFactor;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
    m_fStiffness = stiffness;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed) {
    m_massPoints.push_back(new massPoint(position, Velocity, isFixed));
    return m_massPoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
    m_springs.push_back(new Spring(masspoint1, masspoint2, initialLength));
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
    m_externalForce = force;
}

int MassSpringSystemSimulator::getNumberOfSprings() { return m_springs.size(); }

int MassSpringSystemSimulator::getNumberOfMassPoints() { return m_massPoints.size(); }

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return m_massPoints.at(index)->position; }

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return m_massPoints.at(index)->velocity; }

// ------------ integrator implementations -------------

void MassSpringSystemSimulator::collision(massPoint* massPoint) {
    if (massPoint->position.x - m_scale <= -bound_x || massPoint->position.x + m_scale >= bound_x)
        massPoint->velocity.x = -massPoint->velocity.x;
    if (massPoint->position.y - m_scale <= -bound_y || massPoint->position.y + m_scale >= bound_y)
        massPoint->velocity.y = -massPoint->velocity.y;
    if (massPoint->position.z - m_scale <= -bound_z || massPoint->position.z + m_scale >= bound_z)
        massPoint->velocity.z = -massPoint->velocity.z;
}

void MassSpringSystemSimulator::updateForce(vector<massPoint*> massPoints) {
    Vec3 forceInternal_p1 = Vec3();
    Vec3 forceInternal_p2 = Vec3();
    for each(auto* spring in m_springs) {
        massPoint* p1 = massPoints[spring->masspoint1];
        massPoint* p2 = massPoints[spring->masspoint2];
        
        // F_int = -k * (l - L) * dirction
        if (p1 != NULL && p2 != NULL) { 
            Vec3 l = p1->position - p2->position;
            Vec3 damping_1 = m_fDamping * p1->velocity;
            Vec3 damping_2 = m_fDamping * p2->velocity;
            forceInternal_p1 = -( m_fStiffness * ((float) norm(l) - spring->initialLength) * getNormalized(l));
            forceInternal_p2 = ( m_fStiffness * ((float) norm(l) - spring->initialLength) * getNormalized(l));
            // update the force to p1 and p2
            p1->force += (forceInternal_p1 + m_externalForce - damping_1);
            p2->force += (forceInternal_p2 + m_externalForce - damping_2);
        }
    }
}

/** calculate the position and velocity of the masspoint with euler and update the force within a unique timeStep. */
void MassSpringSystemSimulator::eulerHelper(float timeStep, vector<massPoint*> massPoints) {
    for each (auto * masspoint in m_massPoints) {
        if (!masspoint->isFixed){
            Vec3 acc = masspoint->force / m_fMass;
            masspoint->position += (masspoint->velocity * timeStep);
            masspoint->velocity += timeStep * acc;
            collision(masspoint);
            masspoint->force = 0.0f;
        }
    }
    updateForce(m_massPoints);
}

void MassSpringSystemSimulator::eulerIntegrator(float timeStep) {
    vector<pair<Vec3, Vec3> > temp;
    eulerHelper(timeStep, m_massPoints);
}

void MassSpringSystemSimulator::midpointIntergrator(float timeStep) { 
    // store the midpoints (in order to calculate the force with midpoint positions)
    vector<massPoint*> massPoints = vector<massPoint*>(m_massPoints);
    eulerHelper(timeStep / 2.0f, massPoints);
    updateForce(massPoints);
    // update the position and velocityof each mass point
    for (int i=0; i<getNumberOfMassPoints(); i++) {
        Vec3 accMidPoint = massPoints.at(i)->force / m_fMass;
        m_massPoints.at(i)->position += (timeStep * massPoints.at(i)->velocity);
        m_massPoints.at(i)->velocity += (timeStep * accMidPoint);
    }
}


// ------------------------------ UI Functions ------------------------------

void MassSpringSystemSimulator::setScale(float x, float y, float z) {
    m_scale = Vec3(x, y, z);
}

const char * MassSpringSystemSimulator::getTestCasesStr(){
	return "Euler, Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
    this->DUC = DUC;

    reset();

    int p1 = addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false);
    int p2 = addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false);
    int p3 = addMassPoint(Vec3(0.0f, 1.0f, 4.0f), Vec3(-1.0f, 0.0f, 3.0f), false);
    int p4 = addMassPoint(Vec3(2.1f, 2.0f, 0.0f), Vec3(1.0f, 2.0f, 0.0f), false);
    int p5 = addMassPoint(Vec3(3.0f, 2.0f, 1.0f), Vec3(0.0f, 0.0f, 2.0f), false);
    int p6 = addMassPoint(Vec3(2.0f, 2.0f, 4.0f), Vec3(1.0f, 1.0f, 0.0f), true);
    int p7 = addMassPoint(Vec3(4.0f, 0.0f, 1.0f), Vec3(2.0f, 0.0f, 1.0f), false);
    int p8 = addMassPoint(Vec3(2.0f, 4.0f, 3.0f), Vec3(1.0f, 0.0f, 1.0f), false);
    int p9 = addMassPoint(Vec3(1.0f, 1.0f, 1.0f), Vec3(-1.0f, 1.0f, 0.0f), false);
    int p10 = addMassPoint(Vec3(2.0f, 2.0f, 1.0f), Vec3(0.0f, 0.0f, 0.0f), false);

    addSpring(p1, p2, 1.0f);
    addSpring(p2, p3, 1.0f);
    addSpring(p1, p4, 1.0f);
    addSpring(p3, p5, 1.0f);
    addSpring(p5, p6, 1.0f);
    addSpring(p2, p7, 1.0f);
    addSpring(p6, p8, 1.0f);
    addSpring(p7, p9, 1.0f);
    addSpring(p8, p10, 1.0f);
    addSpring(p3, p6, 1.0f);
    addSpring(p9, p2, 1.0f);
    addSpring(p10, p1, 1.0f);
    updateForce(m_massPoints);

    switch (m_iIntegrator){
        case EULER:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        case MIDPOINT:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        // case LEAPFROG:
        // TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        // TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        // TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        // TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        // break;
        default: break;
    }
}

void MassSpringSystemSimulator::reset() { 
    m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
    if (!m_massPoints.empty()) {
        for each (auto * point in m_massPoints) delete point;
        m_massPoints.clear();
    }
    if (!m_springs.empty()) {
        for each (auto * spring in m_springs) delete spring;
        m_springs.clear();
    }
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    for each(auto* point in m_massPoints) {
        DUC->drawSphere(point->position, m_scale);
    }
    for each(auto* spring in m_springs) {
        DUC->beginLine();
        DUC->drawLine(m_massPoints[spring->masspoint1]->position, Vec3(3,5,7),
                        m_massPoints[spring->masspoint2]->position, Vec3(3,5,7));
        DUC->endLine();
    }
}

void MassSpringSystemSimulator::notifyCaseChanged(int integrator) { 
    m_iIntegrator = integrator;
	switch (m_iIntegrator)
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

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {
    Vec3 gravity = Vec3(0.0f, 0.0f, -10.0f);
    applyExternalForce(gravity);
}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {
    switch (m_iIntegrator) {
        case EULER:
        eulerIntegrator(timeStep);
        break;
        case LEAPFROG:
        break;
        case MIDPOINT:
        midpointIntergrator(timeStep);
        break;
        default:break;
    }
}

void MassSpringSystemSimulator::onClick(int x, int y) {

}

void MassSpringSystemSimulator::onMouse(int x, int y) {

}