using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using Unity.VisualScripting;
using UnityEngine;
using static Unity.VisualScripting.Member;
using Vector3 = UnityEngine.Vector3;
using UnityEngine.UI;

public class rigid_body : MonoBehaviour
{
    //public float torque;
    public Rigidbody rb;
    public Slider slider;
    public Slider stiffnessSlider;
    public Slider timeStepSlider;
    private float step;
    public float stiffness;
    private Vector3 torque = new Vector3(0,0,0);
    private Vector3 acc = new Vector3(0, 0, 0);
    private Vector3 momentum = new Vector3(0, 0, 0);
    UnityEngine.Matrix4x4 I0 = UnityEngine.Matrix4x4.identity;
    UnityEngine.Matrix4x4 inertiaTensor_new;
    public Vector3 ini_velocity;

    Vector3 F = new Vector3();
    public struct ExternalForce
    {
        public Vector3 loc;
        public Vector3 force;
        public ExternalForce(Vector3 loc, Vector3 force)
        {
            this.loc = loc;
            this.force = force;
        }
        //public Vector3 loc { get; }
    }
    List<ExternalForce> forces;
    
    // Start is called before the first frame update
    void Start()
    {
        acc = new Vector3(0, -9.8f, 0);
        rb = gameObject.GetComponent<Rigidbody>();
        slider = GameObject.Find("Mass").GetComponent<Slider>();
        stiffnessSlider = GameObject.Find("Stiffness").GetComponent<Slider>();
        timeStepSlider = GameObject.Find("TimeSteps").GetComponent<Slider>();
        step = timeStepSlider.value;
        Vector3 size_A = gameObject.GetComponent<Collider>().bounds.size;

        I0[0, 0] = (rb.mass / 12) * (Mathf.Pow(size_A.y, 2) + Mathf.Pow(size_A.z, 2));
        I0[1, 1] = (rb.mass / 12) * (Mathf.Pow(size_A.x, 2) + Mathf.Pow(size_A.z, 2));
        I0[2, 2] = (rb.mass / 12) * (Mathf.Pow(size_A.x, 2) + Mathf.Pow(size_A.y, 2));
        I0[3, 3] = 1;
        I0 = I0.inverse;
        rb.velocity = ini_velocity;
        slider.onValueChanged.AddListener(OnNumSilderChange);
        //rb.isKinematic = true;

    }

    // Update is called once per frame
    void Update()
    {
        rb.mass = slider.value;
        stiffness = stiffnessSlider.value;
        step = timeStepSlider.value;
        updateLinearVel(step, rb);
        // updateTorque(rb);
        updateOrientation(rb);
        updateMomentum(rb);
        updateInertiaTensor();
        updateAngularVel(rb);
        OutofBounds();


    }
    //UnityEngine.Matrix4x4 getInitialInverseInertiaTensor() 
    //{ return I0; }

    private void OnCollisionEnter(Collision collision)
    {
        collision_handle(collision);
    }

    void collision_handle(Collision collision){
        foreach (ContactPoint contact in collision.contacts)
        {
            //Debug.DrawRay(contact.point, contact.normal, Color.white);
            // Debug.Log("Collision!");
            Rigidbody A = contact.thisCollider.attachedRigidbody;
            Rigidbody B = contact.otherCollider.attachedRigidbody;
            Vector3 size_B = contact.otherCollider.bounds.size;
            //Vector3 posA = A.position;
            //Vector3 posB = B.position;
            Vector3 n = contact.normal;
            Vector3 Xa = new Vector3();
            Vector3 Xb = new Vector3();
            if(B != null) {
                if (contact.point != null) {
                    Xa = contact.point - A.position;
                    Xb = contact.point - B.position;
                }
                Vector3 velOnA = A.velocity + Vector3.Cross(A.angularVelocity, Xa);
                Vector3 velOnB = B.velocity + Vector3.Cross(B.angularVelocity, Xb);
                Vector3 Vrel = velOnA - velOnB;
                if (Vector3.Dot(Vrel, n) <= 0)
                {
                    UnityEngine.Matrix4x4 I0_B = UnityEngine.Matrix4x4.identity;
                    I0_B[0, 0] = (B.mass / 12) * (Mathf.Pow(size_B.y, 2) + Mathf.Pow(size_B.z, 2));
                    I0_B[1, 1] = (B.mass / 12) * (Mathf.Pow(size_B.x, 2) + Mathf.Pow(size_B.z, 2));
                    I0_B[2, 2] = (B.mass / 12) * (Mathf.Pow(size_B.x, 2) + Mathf.Pow(size_B.y, 2));
                    I0_B[3, 3] = 1;
                    I0_B = I0_B.inverse;

                    UnityEngine.Matrix4x4 rotMat = UnityEngine.Matrix4x4.Rotate(B.rotation);
                    //UnityEngine.Matrix4x4 rotMat_transpose = rotMat;
                    UnityEngine.Matrix4x4 rotMat_transpose = rotMat.transpose;
                    UnityEngine.Matrix4x4 currentTensor = rotMat * I0_B * rotMat_transpose;
                    UnityEngine.Matrix4x4 inertiaTensor_new_B = currentTensor;

                    int c = 1;
                    float numerator = -(1 + c) * Vector3.Dot(Vrel, n);
                    float denominator = 1 / A.mass + 1 / B.mass 
                        + Vector3.Dot(Vector3.Cross(inertiaTensor_new * Vector3.Cross(Xa, n), Xa) 
                                    + Vector3.Cross(inertiaTensor_new_B * Vector3.Cross(Xb, n), Xb), n);

                    float J = numerator / denominator;

                    A.velocity = (A.velocity + J * n / A.mass);
                    Vector3 angularMomentum_A = new Vector3(
                        A.inertiaTensor.x * A.angularVelocity.x,
                        A.inertiaTensor.y * A.angularVelocity.y,
                        A.inertiaTensor.z * A.angularVelocity.z
                    );
                    Vector3 new_angularMomentum_A = angularMomentum_A + Vector3.Cross(Xa, J * n);
                    A.angularVelocity = inertiaTensor_new * new_angularMomentum_A; //FIXME
                    //A.setMomentum(A.getMomentum() + cross(Xa, J * n));
                    //A.setMomentum(A.getMomentum() + cross(Xa, J * n));


                    // B.velocity = (B.velocity - J * n / B.mass);
                    // Vector3 angularMomentum_B = new Vector3(
                    //     B.inertiaTensor.x * B.angularVelocity.x,
                    //     B.inertiaTensor.y * B.angularVelocity.y,
                    //     B.inertiaTensor.z * B.angularVelocity.z
                    // );
                    // Vector3 new_angularMomentum_B = angularMomentum_B + Vector3.Cross(Xb, J * n);
                    // B.angularVelocity = B.inertiaTensorRotation * new_angularMomentum_B; //FIXME
                    //B.setMomentum(B.getMomentum() - cross(Xb, J * n));
                }
            }
                
        }

        
        // 
        F = new Vector3(0, -rb.position.y, 0) * stiffness;
    }

    void updateLinearVel(float timeStep, Rigidbody rb)
    {

        rb.MovePosition(rb.position + timeStep * rb.velocity);
        rb.velocity = (rb.velocity + timeStep * acc);
    }

    void updateTorque(Rigidbody rb)
    {

        //vector<ExternalForce> forces = body.getExternalForce();
        Vector3 gravity = new Vector3(0, -9.8f, 0);
        Vector3 newTorque = new Vector3(0,0,0);
        Vector3 newAcc = new Vector3(0,0,0);
        foreach (ExternalForce force in forces)
        {
            newTorque += Vector3.Cross(force.loc - rb.position, force.force);
            newAcc += force.force / rb.mass;
        }
        newAcc += gravity / rb.mass + F / rb.mass;

        torque = torque + newTorque;
        /*body.setAcc(body.getAcc() + this->m_externalForce + newAcc);*/
        acc = acc + newAcc;
        forces.Clear();
    }
    void updateOrientation(Rigidbody rb)
    {
        UnityEngine.Quaternion rot = GetComponent<Rigidbody>().rotation;
        Vector3 ang_v = rb.angularVelocity;
        UnityEngine.Quaternion temp = new UnityEngine.Quaternion(ang_v[0] * (step / 2), ang_v[1] * (step / 2), ang_v[2] * (step / 2), 0); // FIXME
        UnityEngine.Quaternion changed = temp * rot;
        UnityEngine.Quaternion newOrientation = new UnityEngine.Quaternion(rot.x + changed.x,
                                                                            rot.y + changed.y,
                                                                            rot.z + changed.z,
                                                                            rot.w + changed.w);
        UnityEngine.Quaternion.Normalize(newOrientation);
        // print(newOrientation);
        rb.rotation = newOrientation;
        //rot + temp * rot;
                                                                                                 //float x, y, z;
                                                                                                 //    x = body.getAngularVelocity()[0];
                                                                                                 //    y = body.getAngularVelocity()[1];
                                                                                                 //    z = body.getAngularVelocity()[2];

        //    Quat newOrientation = body.getOrientation() + (timeStep / 2) * Quat(x, y, z, 0) * body.getOrientation();
        //    newOrientation /= newOrientation.norm();
        //    body.setOrientation(newOrientation);

    }

    void updateMomentum(Rigidbody rb)
    {
         //Vec3 oldMomentum = body.getMomentum();
         momentum = momentum + step * torque;  
    }
    void updateInertiaTensor()
    {
        
        UnityEngine.Matrix4x4 rotMat = UnityEngine.Matrix4x4.Rotate(rb.rotation);
        //UnityEngine.Matrix4x4 rotMat_transpose = rotMat;
        UnityEngine.Matrix4x4 rotMat_transpose = rotMat.transpose;
        UnityEngine.Matrix4x4 currentTensor = rotMat * I0 * rotMat_transpose;
        inertiaTensor_new = currentTensor;

    }

    void updateAngularVel(Rigidbody rb)
    { 
         Vector3 newAngularVel = inertiaTensor_new * momentum;
         rb.angularVelocity = newAngularVel;
    }

    public void OnNumSilderChange(float value)
    {
        rb.mass = value;
    }

    private void OutofBounds() { 
        if(this.gameObject.transform.position.y < -60) Destroy(this.gameObject);
    }


}





