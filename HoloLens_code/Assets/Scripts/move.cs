using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class move : MonoBehaviour
{
    // Start is called before the first frame update
    int collected = 0;
    void Start()
    {
        
    }

    public void collect_pos()
    {
        collected += 1;
    }

    // Update is called once per frame
    void Update()
    {
        switch (collected)
        {
            case 0: this.transform.position = new Vector3(0, 0, 0.5f); break;
            case 1: this.transform.position = new Vector3(0.1f, 0, 0.5f); break;
            case 2: this.transform.position = new Vector3(0.5f, 0, 0.3f); break;
            default: this.transform.position = new Vector3(0, 0, 0.5f); break;
        }
    }
}
