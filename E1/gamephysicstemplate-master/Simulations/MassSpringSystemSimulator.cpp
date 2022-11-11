#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    setMass(10.0);
    setStiffness(40.0);
    setDampingFactor(0.0);
    m_externalForce = Vec3();
    m_iIntegrator = EULER;
    setScale(0.05, 0.05, 0.05);
}

MassSpringSystemSimulator::~MassSpringSystemSimulator() { 
    if (!m_massPoints.empty()) {
        for each(auto* point in m_massPoints) delete point;
    }
    if (!m_springs.empty()) {
        for each(auto* spring in m_springs) delete spring;
    }
}

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

void MassSpringSystemSimulator::updateForce(vector<massPoint*> massPoints) {
    Vec3 forceInternal_p1 = Vec3();
    Vec3 forceInternal_p2 = Vec3();
    for each(auto* spring in m_springs) {
        massPoint* p1 = massPoints[spring->masspoint1];
        massPoint* p2 = massPoints[spring->masspoint2];
        
        // F_int = -k * (l - L) * dirction
        if (p1 != NULL && p2 != NULL) { 
            Vec3 l = p1->position - p2->position;
            forceInternal_p1 = ( m_fStiffness * ((float) norm(l) - spring->initialLength) * getNormalized(l));
            forceInternal_p2 = ( m_fStiffness * ((float) norm(l) - spring->initialLength) * getNormalized(l));
            // update the force to p1 and p2
            p1->force += forceInternal_p1;
            p2->force -= forceInternal_p2;
        }
    }
}

/** calculate the position and velocity of the masspoint with euler within a unique timeStep.
 * ! Not the final vectors! Just a variant!
 * @return a pair of vector (velocity, position)
 */
pair<Vec3, Vec3> MassSpringSystemSimulator::eulerHelper(float timeStep, massPoint* point) {
    Vec3 acc = point->force / m_fMass;
    Vec3 velocity = timeStep * acc;
    Vec3 position = (point->velocity * timeStep);
    // clear force
    point->force = 0.0f;
    return make_pair(velocity, position);
}

void MassSpringSystemSimulator::eulerIntegrator(float timeStep) {
    // store the updated position and velocity (in order of a simultaneous start-up)
    vector<pair<Vec3, Vec3> > temp;
    for each(auto* masspoint in m_massPoints) {
        pair<Vec3, Vec3> p = eulerHelper(timeStep, masspoint);
        temp.push_back(p);
    }
    updateForce(m_massPoints);
    // update the position and velocity of each mass point
    for (int i=0; i<getNumberOfMassPoints(); i++) {
        pair<Vec3, Vec3> p = temp.at(i);
        m_massPoints.at(i)->velocity += p.first;
        m_massPoints.at(i)->position += p.second;
    }
}

void MassSpringSystemSimulator::midpointIntergrator(float timeStep) { 
    // store the midpoint (in order to calculate the force with midpoint positions)
    vector<massPoint*> massPoints;
    for each(auto* masspoint in m_massPoints) {
        pair<Vec3, Vec3> p = eulerHelper(timeStep / 2.0f, masspoint);
        Vec3 velocityMidPoint = p.first;
        Vec3 positionMidPoint = p.second;
        massPoint* temp = new massPoint(masspoint->velocity+velocityMidPoint, masspoint->position+positionMidPoint, 
                                        masspoint->isFixed, masspoint->force);
        massPoints.push_back(temp);
    }
    updateForce(massPoints);
    // update the position, velocity and force of each mass point
    for (int i=0; i<getNumberOfMassPoints(); i++) {
        Vec3 accMidPoint = massPoints.at(i)->force / m_fMass;
        m_massPoints.at(i)->position += (timeStep * massPoints.at(i)->position);
        m_massPoints.at(i)->velocity += (timeStep * accMidPoint);
    }
}


// ------------------------------ UI Functions ------------------------------

void MassSpringSystemSimulator::setScale(float x, float y, float z) {
    m_scale = Vec3(x, y, z);
}

const char * MassSpringSystemSimulator::getTestCasesStr(){
	return "Euler, LeapFrog, Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
    this->DUC = DUC;

    reset();

    addSpring(addMassPoint(Vec3(0.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f), false), 
                addMassPoint(Vec3(0.0f, 2.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), false),
                    1.0f);

    switch (m_iIntegrator){
        case EULER:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        // TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        case MIDPOINT:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        // TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        case LEAPFROG:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        // TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        default: break;
    }
}

void MassSpringSystemSimulator::reset() { 
    m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
    if (!m_massPoints.empty()) {
        for each (auto * point in m_massPoints) delete point;
    }
    if (!m_springs.empty()) {
        for each (auto * spring in m_springs) delete spring;
    }
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    for each(auto* point in m_massPoints) {
        DUC->drawSphere(point->position, m_scale);
    }
    for each(auto* spring in m_springs) {
        DUC->drawLine(m_massPoints[spring->masspoint1]->position, Vec3(3,5,7),
                        m_massPoints[spring->masspoint2]->position, Vec3(3,5,7));
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