#include "Simulator.h"

struct externalForce {
	Vec3 position;
	Vec3 force;
	externalForce(Vec3 p, Vec3 force) : position(p), force(force) {}
};

class RigidBodySystem {
private:
    float _width, _height, _depth;
	Vec3 position;
	Vec3 velocity;
	Vec3 angularVel;
	Mat4 I_current;
	Mat4 I0;
	Quat rotation;

	Vec3 momentum;
	Vec3 torque;

	float mass;
	vector<struct externalForce> forces;

public:
    RigidBodySystem(float width, float height, float depth, Vec3 position, Vec3 velocity, Quat rotation, float mass) : _width(width), _height(height), _depth(depth),
	 position(position), velocity(velocity), rotation(rotation), mass(mass) {
        forces = new vector<struct externalForce>();
        momentum = Vec3(0.0f, 0.0f, 0.0f);
		torque = Vec3(0.0f, 0.0f, 0.0f);

		I0 = Mat4((1/12)*mass*(height*height + depth*depth), 0, 0, 0,
					0, (1/12)*mass*(width*width + height*height), 0, 0,
					0, 0, (1/12)*mass*(width*width + depth*depth), 0,
					0, 0, 0, 0).inverse();
		I_current = rotation.getRotMat()*I0*rotation.getRotMat().transpose();
		angularVel = I_current.inverse()*momentum;
     }

     	// getter
	inline float getWidth() const{return _width;}
	inline float getHeight() const{return _height;}
	inline float getDepth() const{return _depth;}
    inline Vec3 getSize() const{return Vec3(_width, _height, _depth);}
	inline float getMass() const{return mass;}
	inline Quat getRotation() const{return rotation;}
	inline Vec3 getPosition() const{return position;}
	inline Vec3 getVelocity() const{return velocity;}
	inline Vec3 getAngularVelocity() const{return angularVel;}
	inline Mat4 getICurrent() const{return I_current;}
	inline Mat4 getI0() const{return I0;}
	inline Vec3 getMomentum() const{return momentum;}
	inline Vec3 getTorque() const{return torque;}
    inline vector<struct externalForce> getExternalForce() const{return forces;}

	// setter
	inline void setPosition(const Vec3 p) {position = p;}
	inline void setSize(const float width, const float height, const float depth) { _width = width; _height = height; _depth = depth;}
	inline void setVelocity(const Vec3 p) {velocity = p;}
	inline void setAngularVelocity(const Vec3 p) {angularVel = p; }
	inline void setICurrent(const Mat4 I) {I_current = I;}
	inline void setI0(const Mat4 I) {I0 = I;}
	inline void setRotation(const Quat r) {rotation = r;}
	inline void setMomentum(const Vec3 m) {momentum = m;}
	inline void setTorque(const Vec3 t) {torque = t;}
	inline void setMass(const float m) {mass = m;}
    inline void addForce(const Vec3 location, const Vec3 force) {forces.push_back(struct externalForce(location, force));}
}