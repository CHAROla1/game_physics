#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator() {
    setMass(10.0);
    setStiffness(40.0);
    setDampingFactor(0.0);
    m_externalForce = Vec3();
    m_iIntegrator = EULER;
}

MassSpringSystemSimulator::~MassSpringSystemSimulator() { 
    if (massPoints != nullptr) {
        for each(auto* point in massPoints) delete point;
    }
    if (springs != nullptr) {
        for each(auto* spring in springs) delete spring;
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
    masspoints.push_back(new MassPoint(position, Velocity, isFixed));
    return masspoints.size() - 1;
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
    springs.push_back(new Spring(masspoints[masspoint1], masspoints[masspoint2], initialLength));
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
    m_externalForce = force;
}

int MassSpringSystemSimulator::getNumberOfSprings() { return springs.size(); }

int MassSpringSystemSimulator::getNumberOfMassPoints() { return masspoints.size(); }

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) { return masspoints.at(index)->position; }

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) { return masspoints.at(index)->velocity; }

// ------------ integrator implementations -------------

void MassSpringSystemSimulator::addForce(massPoint* p1) {
    Vec3 forceInternal = Vec3();
    for each(auto* spring : springs) {
        massPoint* p2 = NULL;
        // find the other masspoints which share the same springs with p1
        if (spring->masspoint1 == p1) p2 = spring->masspoint2;
        else if (spring->masspoint2 == p1) p2 = spring->masspoint1;
        
        // F_int = -k * (l - L) * dirction
        if (p2 != NULL) { 
            Vec3 l = p1->position - p2->position;
            forceInternal -= ( m_fStiffness * ((float) norm(l) - spring->initialLength) * getNormalized(l));
        }
    }
    // return the force to p1
    p1->force = (forceInternal + m_externalForce);
}

/** calculate the position and velocity of the masspoint with euler within a unique timeStep.
 * ! Not the final vectors! Just a variant!
 * @return a pair of vector (velocity, position)
 */
pair<Vec3, Vec3> MassSpringSystemSimulator::eulerHelper(float timeStep, massPoint* point) {
    Vec3 acc = point->force / m_fMass;
    Vec3 velocity = timeStep * acc;
    Vec3 position = (point->velocity * timeStep);
    return make_pair(velocity, position);
}

void MassSpringSystemSimulator::eulerIntegrator(float timeStep) {
    // store the updated position and velocity (in order of a simultaneous start-up)
    vector<pair<Vec3, Vec3> > temp;
    for each(auto* masspoint : massPoints) {
        pair<Vec3, Vec3> p = eulerHelper(timeStep, masspoint);
        temp.push_back(p);
        addForce(masspoint);
    }
    // update the position and velocity of each mass point
    for (int i=0; i<getNumberOfMassPoints(); i++) {
        pair<Vec3, Vec3> p = temp.at(i);
        massPoints.at(i)->velocity += p.first;
        massPoints.at(i)->position += p.second;
    }
}

void MassSpringSystemSimulator::midpointIntergrator(float timeStep) { 
    // store the midpoint position, velocity and force (in order of a simultaneous start-up)
    vector<Vec3> velocities;
    vector<Vec3> positions;
    vector<Vec3> forces;
    for each(auto* masspoint : massPoints) {
        pair<Vec3, Vec3> p = eulerHelper(timeStep / 2.0, masspoint);
        Vec3 velocityMidPoint = p.first;
        Vec3 positionMidPoint = p.second;
        massPoint* temp = new massPoint(masspoint->velocity+velocityMidPoint, masspoint->position+positionMidPoint, 
                                        masspoint->isFixed, masspoint->force);
        addForce(temp);
        velocity.push_back(velocityMidPoint);
        positions.push_back(positionMidPoint);
        forces.push_back(temp->force);
    }
    // update the position, velocity and force of each mass point
    for (int i=0; i<getNumberOfMassPoints(); i++) {
        Vec3 accMidPoint = forces.at(i) / m_fMass;
        massPoints.at(i)->position += (timeStep * velocities.at(i));
        massPoints.at(i)->velocity += (timeStep * accMidPoint);
        massPoints.at(i)->force += forces.at(i);
    }
}


// ------------------------------ UI Functions ------------------------------

const char * MassSpringSystemSimulator::getTestCasesStr(){
	return "Euler, Midpoint";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC) {
    this->DUC = DUC;
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
        case LEAPFROG:
        TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Mass", TW_TYPE_FLOAT, &m_fMass, "min=1");
        TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0");
        TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_scale, "min=0.01 step=0.01");
        break;
        default: break;
    }
}

void MassSpringSystemSimulator::reset() { 
    m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) { 

}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed) {

}

void MassSpringSystemSimulator::simulateTimestep(float timeStep) {

}

void MassSpringSystemSimulator::onClick(int x, int y) {

}

void MassSpringSystemSimulator::onMouse(int x, int y) {

}