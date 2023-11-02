using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class ChangeScene : MonoBehaviour
{
    public Button changeSceneButton;
    // Start is called before the first frame update
    void Start()
    {
        changeSceneButton.onClick.AddListener(ChangeScenes);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void ChangeScenes() { 
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex + 1);
    }
}
