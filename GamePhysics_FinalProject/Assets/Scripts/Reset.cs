using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class Reset : MonoBehaviour
{
    public Button reset;
    // Start is called before the first frame update
    void Start()
    {
        reset.onClick.AddListener(ResetScene);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ResetScene() { 
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
}
