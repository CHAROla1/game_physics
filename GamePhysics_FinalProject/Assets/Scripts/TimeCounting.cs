using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class TimeCounting : MonoBehaviour
{
    public TextMeshProUGUI time;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        time.text = (float.Parse(time.text) + Time.deltaTime).ToString("f3");
    }
}
