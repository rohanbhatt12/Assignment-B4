using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.AI;

public class Agent : MonoBehaviour
{
    public float radius;
    public float mass;
    public float perceptionRadius;

    private List<Vector3> path;
    private NavMeshAgent nma;
    private Rigidbody rb;
	private Dictionary<GameObject,Vector3> perWalls = new Dictionary<GameObject, Vector3>();
    private HashSet<GameObject> perceivedNeighbors = new HashSet<GameObject>();

    void Start()
    {
        path = new List<Vector3>();
        nma = GetComponent<NavMeshAgent>();
        rb = GetComponent<Rigidbody>();

        gameObject.transform.localScale = new Vector3(2 * radius, 1, 2 * radius);
        nma.radius = radius;
        rb.mass = mass;
        GetComponent<SphereCollider>().radius = perceptionRadius / 2;
    }

    private void Update()
    {
        if (path.Count > 1 && Vector3.Distance(transform.position, path[0]) < 1.1f)
        {
            path.RemoveAt(0);
        } else if (path.Count == 1 && Vector3.Distance(transform.position, path[0]) < 2f)
        {
            path.RemoveAt(0);

            if (path.Count == 0)
            {
                gameObject.SetActive(false);
                AgentManager.RemoveAgent(gameObject);
            }
        }

        #region Visualization
		
		//CHANGE THIS TO TRUE IF YOU WANT TO SEE THE GREEN AND YELLOW LINES FOR THE PATH
		
        if (false)
        {
            if (path.Count > 0)
            {
                Debug.DrawLine(transform.position, path[0], Color.green);
            }
            for (int i = 0; i < path.Count - 1; i++)
            {
                Debug.DrawLine(path[i], path[i + 1], Color.yellow);
            }
        }
		//CHANGE THIS TO TRUE IF YOU WANT TO SEE WHICH AGENTS ARE COLLIDING WITH EACH OTHER
        if (false)
        {
            foreach (var neighbor in perceivedNeighbors)
            {
                Debug.DrawLine(transform.position, neighbor.transform.position, Color.yellow);
            }
        }

        #endregion
    }

    #region Public Functions

	
    public void ComputePath(Vector3 destination)
    {
        nma.enabled = true;
        var nmPath = new NavMeshPath();
        nma.CalculatePath(destination, nmPath);
        path = nmPath.corners.Skip(1).ToList();
 
    }

    public Vector3 GetVelocity()
    {
        return rb.velocity;
    }

    #endregion

    #region Incomplete Functions
	//TOTAL FORCE ON ALL AGENTS
    private Vector3 ComputeForce()
    {
        var force = CalculateGoalForce(maxspeed : 2) + CalculateAgentForce() + CalculateWallForce();

        if (force != Vector3.zero)
        {
            return force.normalized * Mathf.Min(force.magnitude, Parameters.maxSpeed);
        } else
        {
            return Vector3.zero;
        }
    }
    
	
    private Vector3 CalculateGoalForce(float maxspeed)
    {
		if (path.Count ==0 ){
			return Vector3.zero;
		}
		var temp = path[0] - transform.position;
		var dval = temp.normalized * Mathf.Min(temp.magnitude,maxspeed);
		var actualvalue = rb.velocity;
		var ret = (dval-actualvalue)/Parameters.T;
        return ret;
		
    }

    private Vector3 CalculateAgentForce()
    {

        var agentForce = Vector3.zero;

        foreach (var i in perceivedNeighbors)
        {
            if (!AgentManager.IsAgent(i))
            {
                continue;
            }
            var neighbor = AgentManager.agentsObjs[i];

            var dir = (transform.position - neighbor.transform.position).normalized;
            var overlap = (radius + neighbor.radius) - Vector3.Distance(transform.position, i.transform.position);

            agentForce += Parameters.A * Mathf.Exp(overlap / Parameters.B) * dir;
            agentForce += Parameters.k * (overlap > 0f ? overlap : 0) * dir;

            var tangent = Vector3.Cross(Vector3.up, dir);
            agentForce += Parameters.Kappa * (overlap > 0f ? overlap : 0) * Vector3.Dot(rb.velocity - neighbor.GetVelocity(), tangent) * tangent;
        }
        return agentForce;
    }

    private Vector3 CalculateWallForce()
    {
        var wForce = Vector3.zero;

        foreach (var j in perWalls)
        {
            var dir = (transform.position - j.Value).normalized;
            var overlap = radius - Vector3.Distance(transform.position, j.Value);

            wForce += Parameters.WALL_A * Mathf.Exp(overlap / Parameters.WALL_B) * dir;
            wForce += Parameters.WALL_k * (overlap > 0f ? overlap : 0) * dir;

            var tangent = Vector3.Cross(Vector3.up, dir);
            wForce -= Parameters.WALL_Kappa * (overlap > 0f ? overlap : 0) * Vector3.Dot(GetVelocity(), tangent) * tangent;
        }
        return wForce;
    }

    public void ApplyForce()
    {
        var force = ComputeForce();
        force.y = 0;

        rb.AddForce(force * 10, ForceMode.Force);
    }

    public void OnTriggerEnter(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            perceivedNeighbors.Add(other.gameObject);
        }
    }
    
    public void OnTriggerExit(Collider other)
    {
        if (AgentManager.IsAgent(other.gameObject))
        {
            perceivedNeighbors.Remove(other.gameObject);
        }
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (WallManager.IsWall(collision.gameObject))
        {
            perWalls.Add(collision.gameObject,collision.contacts[0].point);
        }
    }

    public void OnCollisionExit(Collision collision)
    {
        if (WallManager.IsWall(collision.gameObject))
        {
            perWalls.Remove(collision.gameObject);

        }
    }

    #endregion
}
