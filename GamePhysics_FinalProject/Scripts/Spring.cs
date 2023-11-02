using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

public class Spring : MonoBehaviour
{
    public float stiffness;
    public float initLen;
    private float currentLen;
    public float damping;
    public Slider slider;
    public Slider timeStepSlider;
    public float step = 0.01f;
    Rigidbody rb;
    void Start()
    {
        currentLen = initLen;
        rb= gameObject.GetComponent<Rigidbody>();
        slider.onValueChanged.AddListener(OnNumSilderChange);
        timeStepSlider = GameObject.Find("TimeSteps").GetComponent<Slider>();
        step = timeStepSlider.value;
        //rb.velocity = new Vector3(0, -2f, 0);
        //print(rb.velocity);
    }

    // Update is called once per frame
    void Update()
    {
        step = timeStepSlider.value;
        Vector3 currentPos = transform.position;
        float d = Mathf.Abs(currentPos.y - initLen);
        Vector3 force = rb.velocity * -damping - stiffness * (d - Mathf.Abs(initLen)) * new Vector3(0, currentPos.y - initLen, 0) / d;
        Vector3 newPos = currentPos + rb.velocity * step;
        // if(transform.position.y > initLen)
            transform.SetPositionAndRotation(newPos, new Quaternion(0, 0, 0, 0));
        // else
        //     transform.SetPositionAndRotation(new Vector3(transform.position.x, initLen, transform.position.z), new Quaternion(0, 0, 0, 0));
        Vector3 acc = force / rb.mass;
        Vector3 newVel = rb.velocity + acc * step;
        rb.velocity = newVel;

        //print("rb.vel: " + newVel);
        //print("rb.pos: " + newPos);
        //print("rb.acc: " + acc);
        //print("force: " + force);
        //print("=====================");
        //rb.AddForce(new Vector3(0, -9.8f, 0));
    }

    private void OnNumSilderChange(float value)
    {
        stiffness = value;
    }
}
