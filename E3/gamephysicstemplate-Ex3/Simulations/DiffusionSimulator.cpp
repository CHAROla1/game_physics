#include "DiffusionSimulator.h"
#include "pcgsolver.h"
using namespace std;

Grid::Grid(int x, int y) {
	this->x = x;
	this->y = y;
	this->m_values = vector<float>();
	this->m_values.resize(x*y);
}

Grid::SetValue(int x, int y, float value) {
	this->m_values[x*this->y + y] = value;
}

Grid::GetValue(int x, int y) {
	if(x>this->x || y>this->y || x<0 || y<0) return 0;
	return this->m_values[x*this->y + y];
}

Grid::GetSize() {
	return Vec3(this->x, this->y, 1);
}


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	// to be implemented
	x = y = 16;
	z = 1;

	T = new Grid(x, y);
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;

}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "x", TW_TYPE_INT32, &x, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "y", TW_TYPE_INT32, &y, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "z", TW_TYPE_INT32, &z, "min=1");
	TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha, "min=0.01 step=0.01");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	//
	//to be implemented
	//
	for(int i = 0; i < T->GetSize().x; i++) {
		for(int j = 0; j < T->GetSize().y; j++) {
			if(i == 0 || j == 0 || i == T->GetSize().x-1 || j == T->GetSize().y-1) T->SetValue(i, j, 0);
			else T->SetValue(i, j, 1);
		}
	}
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::diffuseTemperatureExplicit(float time) {//add your own parameters
	Grid* newT = new Grid(x, y);
	for(int i = 1; i < x-1; i++) {
		for(int j = 1; j < y-1; j++) {
			float oldTemperature = T->GetValue(i, j);
			float newValueX = T->GetValue(i-1, j) - 2*T->GetValue(i, j) + T->GetValue(i+1, j);
			float newValueY = T->GetValue(i, j-1) - 2*T->GetValue(i, j) + T->GetValue(i, j+1);
			newT->SetValue(i, j, newValueX+newValueY);
		}
	}
	for(int i = 0; i < newT->GetSize().x; i=i+newT->GetSize().x-1) {
		for(int j = 0; j < newT->GetSize().y; j=j+newT->GetSize().y-1) {
			newT->SetValue(i, j, 0);
		}
	}
	// to be implemented
	//make sure that the temperature in boundary cells stays zero
	return newT;
}

void setupB(std::vector<Real>& b, float time, Grid* grid) {//add your own parameters
	// to be implemented
	//set vector B[sizeX*sizeY]
	for (int i = 0; i < grid->GetSize().x; i++) {
		for(int j = 0; j < grid->GetSize().y; j++) {
			b.at(i*grid->GetSize().x + j) = 1/time * grid->GetValue(i, j);
		}
	}
}

void fillT(Grid* grid, vector<Real> x) {//add your own parameters
	// to be implemented
	//fill T with solved vector x
	//make sure that the temperature in boundary cells stays zero
	for(int i = 1; i < grid->GetSize().x-1; i++) {
		for(int j = 1; j < grid->GetSize().y-1; j++) {
			grid->SetValue(i, j, x.at(i*grid->GetSize().x + j));
		}
	}
}

void setupA(SparseMatrix<Real>& A, Grid* grid, float time, float alpha) {//add your own parameters
	// to be implemented
	//setup Matrix A[sizeX*sizeY*sizeZ, sizeX*sizeY*sizeZ]
	// set with:  A.set_element( index1, index2 , value );
	// if needed, read with: A(index1, index2);
	// avoid zero rows in A -> set the diagonal value for boundary cells to 1.0
	for (int i = 0; i < grid->GetSize().x*grid->GetSize().y; i++) {
			A.set_element(i, i, 1); // set diagonal
	}

	for(int i = 1; i < grid->GetSize().x-1; i++) {
		for(int j = 1; j < grid->GetSize().y-1; j++) {
			int index = grid->GetSize().x*i + j;
			int left = index - 1;
			int right = index + 1;
			int up = index + grid->GetSize().x;
			int down = index - grid->GetSize().x;

			A.set_element(index, left, -alpha);
			A.set_element(index, right, -alpha);
			A.set_element(index, up, -alpha);
			A.set_element(index, down, -alpha);
			A.set_element(index, index, 1/time+4*alpha);
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float time) {//add your own parameters
	// solve A T = b
	// to be implemented
	const int N = x*y;//N = sizeX*sizeY*sizeZ
	SparseMatrix<Real> *A = new SparseMatrix<Real> (N);
	std::vector<Real> *b = new std::vector<Real>(N);

	setupA(*A, 0.1);
	setupB(*b, time, T);

	// perform solve
	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) { x[j] = 0.; }

	// preconditioners: 0 off, 1 diagonal, 2 incomplete cholesky
	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	// x contains the new temperature values
	fillT(T, x);//copy x to T
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	// to be implemented
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		T = diffuseTemperatureExplicit();
		break;
	case 1:
		diffuseTemperatureImplicit();
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	for (int i = 0; i < x; i++) {
		for(int j = 0; j < y; j++) {
			float positionX = j;
			float positionY = i;

			int degreeBoundry = 500;
			float degreeScale = T->GetValue(i, j) / degreeBoundry;

			if(degreeScale < 0) { 
				degreeScale = 1 + degreeScale;
			}
			Vec3 color = Vec3(1, degreeScale, degreeScale);

			DUC->setUpLighting(Vec3(),0.4*Vec3(1,1,1),100,0.6*color);
			DUC->drawSphere(Vec3(positionX, positionY, 0),Vec3(m_fSphereSize, m_fSphereSize, m_fSphereSize));
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
