using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using System;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using System.Text;
using System.IO;
using System.Threading.Tasks;

public class RobotAgent : Agent
{
    public GameObject endEffector;
    public GameObject cube;
    public GameObject robot;
    ROSConnection ros1, ros2;
    public string topicName1 = "Action_Val";
    public string topicName2 = "Pos_rot";
    public float publishMessageFrequency = 0.5f;
    private float timeElapsed;
    
    
    RobotController robotController;
    TouchDetector touchDetector;
    TablePositionRandomizer tablePositionRandomizer;


    void Start()
    {
        robotController = robot.GetComponent<RobotController>();
        touchDetector = cube.GetComponent<TouchDetector>();
        tablePositionRandomizer = cube.GetComponent<TablePositionRandomizer>();
        
        
        //Also start the ROS connection
        ros1 = ROSConnection.instance;
        ros1.RegisterPublisher<ActionValueMsg>(topicName1);

        ros2 = ROSConnection.instance;
        ros2.RegisterPublisher<ActionValueMsg>(topicName2);
        
    }


    // AGENT

    public override void OnEpisodeBegin()
    {
        float[] defaultRotations = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        robotController.ForceJointsToRotations(defaultRotations);
        touchDetector.hasTouchedTarget = false;
        tablePositionRandomizer.Move();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (robotController.joints[0].robotPart == null)
        {
            // No robot is present, no observation should be added
            return;
        }

        // relative cube position
        Vector3 cubePosition = cube.transform.position - robot.transform.position;
        sensor.AddObservation(cubePosition);
        
        

        // relative end position
        Vector3 endPosition = endEffector.transform.position - robot.transform.position;
        sensor.AddObservation(endPosition);
        sensor.AddObservation(cubePosition - endPosition);
        
        // PosRotMsg cubePos = new PosRotMsg(
        //         endEffector.transform.position.x,
        //         endEffector.transform.position.y,
        //         endEffector.transform.position.z,
        //         endEffector.transform.rotation.z,
        //         endEffector.transform.rotation.w,
        //         endEffector.transform.rotation.x,
        //         endEffector.transform.rotation.y
                
        //     );
        // ros2.Send(topicName2, cubePos);
       
        
        
    }
  
    public override void OnActionReceived(float[] vectorAction)
    {
        // move
        List<float> directions =  new List<float>(6) ; //List to hold the directions
        List<float> angles =  new List<float>(5) ;  //hold the degree values in each frame
        for (int jointIndex = 0; jointIndex < vectorAction.Length; jointIndex ++)
        {
            RotationDirection rotationDirection = ActionIndexToRotationDirection((int) vectorAction[jointIndex]);
            float degVal = robotController.RotateJoint(jointIndex, rotationDirection, false);
            directions.Add(vectorAction[jointIndex]);
            if (jointIndex <vectorAction.Length-1)
                angles.Add(Mathf.Deg2Rad*degVal);

        }
        string line = string.Join(", ",angles);

        using (StreamWriter sw = File.AppendText("Angles.txt"))
        {
            sw.WriteLine(line);
        }
        
        Debug.Log(line);
        
        // System.IO.File.WriteAllLines("WriteLines.txt", line);



        // Debug.Log(string.Join(", ",angles));
      
        // ActionValueMsg actions = new ActionValueMsg(directions[0], directions[1], directions[2], directions[3],
        // directions[4], directions[5], directions[6]);

        ActionValueMsg actions = new ActionValueMsg(angles[0], angles[1], angles[2], angles[3],
        angles[4], angles[5], 1);
        


        for (int jointIndex = 0; jointIndex < robotController.joints.Length; jointIndex ++)
        {
            // State of an robotic arm
            PosRotMsg ArmPos = new PosRotMsg(
                robotController.joints[jointIndex].robotPart.transform.position.x,
                robotController.joints[jointIndex].robotPart.transform.position.y,
                robotController.joints[jointIndex].robotPart.transform.position.z,
                robotController.joints[jointIndex].robotPart.transform.rotation.z,
                robotController.joints[jointIndex].robotPart.transform.rotation.w,
                robotController.joints[jointIndex].robotPart.transform.rotation.x,
                robotController.joints[jointIndex].robotPart.transform.rotation.y
                
            );
        ros2.Send(topicName2, ArmPos);

        
        ros1.Send(topicName1, actions);

       

        // Knocked the cube off the table
        if (cube.transform.position.y < -1.0)
        {
            SetReward(-1f);
            EndEpisode();
        }

        // end episode if we touched the cube
        if (touchDetector.hasTouchedTarget)
        {
            SetReward(1f);
            EndEpisode();
        }


        //reward
        // float distanceToCube = Vector3.Distance(endEffector.transform.position, cube.transform.position); // roughly 0.7f


        // var jointHeight = 0f; // This is to reward the agent for keeping high up // max is roughly 3.0f
        // for (int jointIndex = 0; jointIndex < robotController.joints.Length; jointIndex ++)
        // {
        //     jointHeight += robotController.joints[jointIndex].robotPart.transform.position.y - cube.transform.position.y;
        //     PosRotMsg ArmPos = new PosRotMsg(
        //         robotController.joints[jointIndex].robotPart.transform.position.x,
        //         robotController.joints[jointIndex].robotPart.transform.position.y,
        //         robotController.joints[jointIndex].robotPart.transform.position.z,
        //         robotController.joints[jointIndex].robotPart.transform.rotation.z,
        //         robotController.joints[jointIndex].robotPart.transform.rotation.w,
        //         robotController.joints[jointIndex].robotPart.transform.rotation.x,
        //         robotController.joints[jointIndex].robotPart.transform.rotation.y
                
        //     );
        // ros2.Send(topicName2, ArmPos);



        }
        // var reward = - distanceToCube + jointHeight / 100f;

        // SetReward(reward * 0.1f);

    }


    // HELPERS

    static public RotationDirection ActionIndexToRotationDirection(int actionIndex)
    {
        return (RotationDirection)(actionIndex - 1);
    }




}
