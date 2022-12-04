#include "RigidBodySystemSimulator.h"

RigidBodySystemSimulator::RigidBodySystemSimulator() {
    m_iTestCase = 0;
}


// ---------------------------------------------------------------- UI Interface ----------------------------------------------------------------
const char * RigidBodySystemSimulator::getTestCasesStr(){
	return "Demo1,Demo2,Demo3,Demo4";
}

void RigidBodySystemSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
        m_rigidbodies.clear();
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Time Step", TW_TYPE_FLOAT, &m_timeStep, "min=0.01 step=0.01");
	switch (m_iTestCase)
	{
	case 0:
        // TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		// TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");

        addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        m_rigidbodies[0]->setRotation(Quat(Vec3(0, 0, 1), 90));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

        simulateTimestep(2.0f);
        printf("Linear Velocity: %f,\n Angular Velocity: %f,\n Point Velocity: %f\n", m_rigidbodies[0]->getVelocity(), 
        m_rigidbodies[0]->getAngularVelocity(), m_rigidbodies[0]->getVelocity() + cross(m_rigidbodies[0]->getAngularVelocity(), Vec3(0.3, 0.5, 0.25) - m_rigidbodies[0]->getPosition()));
        break;
	case 1:
        m_rigidbodyIndex = 0;
        m_timeStep = 0.01f;
        addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        m_rigidbodies[0]->setRotation(Quat(Vec3(0, 0, 1), 90));
        applyForceOnBody(0, Vec3(0.3, 0.5, 0.25), Vec3(1, 1, 0));

		break;
	case 2:
        addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        addRigidBody(Vec3(0.8, 1.5, 0.5), Vec3(1, 0.6, 0.5), 2);
        m_rigidbodies[0]->setRotation(Quat(Vec3(1, 0, 0), 45));
        m_rigidbodies[1]->setVelocity(Vec3(-0.5, -0.5, 0.5));
        break;
	default:break;
	}
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	switch (m_iTestCase)
	{
	case 0:
		cout << "Demo 1!\n";
        reset();
		break;
	case 1:
		cout << "Demo 2!\n";
        reset();
		break;
	case 2:
		cout << "Demo 3!\n";
        reset();
		break;
	default:
		cout << "Demo 4!\n";
        reset();
		break;
	}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
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
		applyForceOnBody(m_rigidbodyIndex, inputWorld, inputWorld);
	}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	UpdateTorque(m_timeStep);
    UpdateRotation(m_timeStep);
    UpdateInertia(m_timeStep);
    Collision();
}


void RigidBodySystemSimulator::drawRigidBodies() {
    std::mt19937 eng;
    std::uniform_real_distribution<float> randCol( 0.0f, 1.0f);
    for (int i = 0; i < getNumberOfRigidBodies(); i++) {
        DUC->setUpLighting(Vec3(), 0.4*Vec3(1,1,1), 100, 0.6*Vec3(randCol(eng), randCol(eng), randCol(eng)));
        DUC->drawRigidBody(m_rigidbodies[i]->getPosition(), m_rigidbodies[i]->getSize());
    }
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
    drawRigidBodies();
}


void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// ------------------------------------------------------------------------------------------------

int RigidBodySystemSimulator::getNumberOfRigidBodies() {
    return m_rigidbodies.size();
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i) {
    return m_rigidbodies[i]->getPosition();
}
Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i){
    return m_rigidbodies[i]->getVelocity();
}
Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i) {
    return m_rigidbodies[i]->getAngularVelocity();
}
void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force) {
    m_rigidbodies[i]->applyForce(loc, force);
}
void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass) {
    m_rigidbodies.push_back(new RigidBodySystem(size.x, size.y, size.z, position, Vec3(), Quat(0, 0, 0, 0), (float)mass));
}
void RigidBodySystemSimulator::setOrientationOf(int i,Quat orientation) {
    m_rigidbodies[i]->setRotation(orientation);
}
void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity) {
    m_rigidbodies[i]->setVelocity(velocity);
}

// ------------------------------ ExtraFunctions ----------------------------------------------------

void RigidBodySystemSimulator::UpdateTorque(float timeStep) {
    for (auto body : m_rigidbodies) {
        Vec3 force = Vec3();
        for (int i = 0; i < body->getExternalForce().length; i++) {
            force += body->getExternalForce()[i].force;
            body->setTorque(body->getTorque() + cross(body->getExternalForce()[i].position - body->getPosition(), body->getExternalForce()[i].force));
        }
        // euler step
        body->setPosition(body->getPosition + timeStep * body->getVelocity());
        body->setVelocity(body->getVelocity() + timeStep * force / body->getMass());
        body->getExternalForce().clear();
    }
    
}


void RigidBodySystemSimulator::UpdateRotation(float timeStep) {
    for(auto body : m_rigidbodies) {
        Quat rotation = body->getRotation() + timeStep / 2 * Quat(body->getAngularVelocity().x, body->getAngularVelocity().y, body->getAngularVelocity().z, 0) * body->getRotation();
        rotation /= rotation.norm();
        body->setRotation(rotation);
    }
}

void RigidBodySystemSimulator::UpdateInertia(float timeStep) {
    for(auto body : m_rigidbodies) {
        body->setMomentum(body->getMomentum() + timeStep * body->getTorque());
        body->setICurrent(body->getRotation().getRotMat()*body->getI0()*body->getRotation().getRotMat().transpose());
        body->setAngularVelocity(body->getICurrent()*body->getMomentum());
    }
}

Mat4 RigidBodySystemSimulator::TransferMatrix(RigidBodySystem *rigidBody) {
    Mat4 affine;
    affine.initId();
    affine.initTranslation(rigidBody->getPosition().x, rigidBody->getPosition().y, rigidBody->getPosition().z);
    affine[0][0] = rigidBody->getRotation().getRotMat()[0][0];
    affine[0][1] = rigidBody->getRotation().getRotMat()[0][1];
    affine[0][2] = rigidBody->getRotation().getRotMat()[0][2];
    affine[1][0] = rigidBody->getRotation().getRotMat()[1][0];
    affine[1][1] = rigidBody->getRotation().getRotMat()[1][1];
    affine[1][2] = rigidBody->getRotation().getRotMat()[1][2];
    affine[2][0] = rigidBody->getRotation().getRotMat()[2][0];
    affine[2][1] = rigidBody->getRotation().getRotMat()[2][1];
    affine[2][2] = rigidBody->getRotation().getRotMat()[2][2];
    return affine;
}

void RigidBodySystemSimulator::Collision() {
    RigidBodySystem * A;
    RigidBodySystem * B;
    for(int i = 0; i < m_rigidbodies.size(); i++) {
        A = m_rigidbodies[i];
        for (int j = i + 1; j < m_rigidbodies.size() - i; j++) {
            B = m_rigidbodies[j];
            CollisionInfo collision = checkCollisionSAT(TransferMatrix(A), TransferMatrix(B));
            if (collision.isValid()) {
                Vec3 x_a = collision.collisionPointWorld - A->getPosition();
                Vec3 x_b = collision.collisionPointWorld - B->getPosition();
                float J = (B->getVelocity() - A->getVelocity()) * collision.normalWorld / (1/A->getMass() + 1/B->getMass() + 
                (cross(A->getI0() * cross(x_a, collision.normalWorld), x_a) + cross(B->getI0() * cross(x_b, collision.normalWorld), x_b)) * collision.normalWorld);

                A->setVelocity(A->getVelocity() + J*collision.normalWorld / A->getMass());
                B->setVelocity(B->getVelocity() - J*collision.normalWorld / B->getMass());
                A->setMomentum(A->getMomentum() + cross(x_a, J*collision.normalWorld));
                B->setVelocity(B->getVelocity() - cross(x_b, J*collision.normalWorld));

            }
        }
    }
}