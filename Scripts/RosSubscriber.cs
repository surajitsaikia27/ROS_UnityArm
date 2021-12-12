using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg;

public class RosSubscriber : MonoBehaviour
{
    public GameObject cube;

    void Start()
    {
        // subscribe to color topic
        ROSConnection.instance.Subscribe<RosColor>("color", ColorChange);
    }

    void ColorChange(RosColor colorMessage)
    {   //Message recieved from the python script
        cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
}
