using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpawnManager : MonoBehaviour
{
    // private float randomRange = 15;
    public GameObject spawnRb;
    public GameObject spawnSpring;
    // public int spawnNum;

    // Start is called before the first frame update
    void Start()
    {
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private Vector3 GenerateSpawnPosition() {
        float randomX = Random.Range(-20, 10);
        float randomZ = Random.Range(-14, 15);
        return new Vector3(randomX, 10, randomZ);
    }

    public void Spawn() {
        Instantiate(spawnRb, GenerateSpawnPosition(), Random.rotation);
    }


}
