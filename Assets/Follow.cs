using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Follow : MonoBehaviour {
    public float speed = 7.0f;
    private Transform target;

    void Start() {
        target = GameObject.FindGameObjectWithTag("Player").GetComponent<Transform>();
    }

    void Update() {
        transform.LookAt(target.position);
        transform.Translate(0.0f, 0.0f, speed * Time.deltaTime);
    }
}
