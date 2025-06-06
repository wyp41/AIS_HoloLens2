using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;
using UnityEngine.UIElements;

[RequireComponent(typeof(MeshFilter))]
public class CSharpForGIT : MonoBehaviour
{
    Thread mThread;
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25001;
    public int scale = 1;
    IPAddress localAdd;
    TcpListener listener;
    TcpClient client;
    int[] index;
    Vector3[] mesh;
    Vector3[] start_mesh;
    Mesh deformingMesh;
    MeshCollider meshCollider;
    Vector3[] originalVertices, displacedVertices;
    Vector3 currentPos = Vector3.zero;

    bool running;
    bool stopped;
    bool start;

    private void Update()
    {
        //print(transform.position);
        //for (int i = 0; i < displacedVertices.Length; i++)
        //{
        //    UpdateVertex(i);
        //}

        currentPos = transform.position;

        if (mesh.Length > 0)
        {
            for (int i = 0; i < mesh.Length; i++)
            {
                if (stopped)
                {
                    displacedVertices[i] = originalVertices[i];
                }
                else
                {
                    displacedVertices[i] = mesh[index[i]];
                }
            }
            deformingMesh.vertices = displacedVertices;
            deformingMesh.RecalculateNormals();
            //meshCollider.sharedMesh = null;
            meshCollider.sharedMesh = deformingMesh;
        }   
    }

    private void Start()
    {
        start = true;
        deformingMesh = GetComponent<MeshFilter>().mesh;
        meshCollider = GetComponent<MeshCollider>();
        originalVertices = deformingMesh.vertices;
        index = new int[originalVertices.Length];
        displacedVertices = new Vector3[originalVertices.Length];
        mesh = new Vector3[originalVertices.Length];
        start_mesh = new Vector3[originalVertices.Length];
        print(originalVertices.Length);
        for (int i = 0; i < originalVertices.Length; i++)
        {
            print(originalVertices[i]);
            mesh[i] = originalVertices[i];
            start_mesh[i] = originalVertices[i];
            displacedVertices[i] = originalVertices[i];
            index[i] = i;
        }
        ThreadStart ts = new ThreadStart(GetInfo);
        mThread = new Thread(ts);
        mThread.Start();
    }

    void GetInfo()
    {
        localAdd = IPAddress.Parse(connectionIP);
        listener = new TcpListener(IPAddress.Any, connectionPort);
        listener.Start();
        client = listener.AcceptTcpClient();

        stopped = false;
        running = true;
        while (running)
        {
            try
            {
                if (stopped)
                {
                    client = listener.AcceptTcpClient();
                }
                SendAndReceiveData();
                stopped = false;
            }
            catch
            {
                client.Close();
                stopped = true;
            }
        }
        listener.Stop();
    }

    void SendAndReceiveData()
    {
        //print("in");
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        //---receiving Data from the Host----
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string
        //print(dataReceived);
        if (dataReceived != null)
        {
            //---Using received data---
            mesh = StringToVector3List(dataReceived, scale);
            if (start)
            {
                start_mesh = mesh;
                start = false;
                index = sort(start_mesh);
                for (int i = 0; i < originalVertices.Length; i++)
                {
                    print(start_mesh[i]);
                }
            }

            //---Sending Data to Host----
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes(currentPos.ToString()); //Converting string to byte data
            nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }
    }

    int[] sort(Vector3[] bullet_v)
    {
        int[] index_c = new int[bullet_v.Length];
        for (int i = 0; i < bullet_v.Length; i++)
        {
            double dis = 10000;
            int ind = 0;
            for (int j = 0; j < bullet_v.Length; j++)
            {
                double d = Vector3.Distance(originalVertices[i], bullet_v[j]);
                if (d < dis)
                {
                    dis = d;
                    ind = j;
                }
            }
            index_c[i] = ind;
        }
        return index_c;
    } 

    public static Vector3[] StringToVector3List(string sVector, int scale)
    {
        // Remove the parentheses
        sVector = sVector.Replace("(", "");
        sVector = sVector.Replace(")", "");

        // split the items
        string[] sArray = sVector.Split(',');

        Vector3[] VArray = new Vector3[sArray.Length/3];

        for (int i = 0; i < VArray.Length; i++)
        {
            VArray[i].x = -float.Parse(sArray[3 * i]) * scale;
            VArray[i].y = float.Parse(sArray[3 * i + 1]) * scale;
            VArray[i].z = float.Parse(sArray[3 * i + 2]) * scale;
        }

        // store as a Vector3
        //Vector3 result = new Vector3(
        //    float.Parse(sArray[0]),
        //    float.Parse(sArray[1]),
        //    float.Parse(sArray[2]));
        return VArray;
    }
    /*
    public static string GetLocalIPAddress()
    {
        var host = Dns.GetHostEntry(Dns.GetHostName());
        foreach (var ip in host.AddressList)
        {
            if (ip.AddressFamily == AddressFamily.InterNetwork)
            {
                return ip.ToString();
            }
        }
        throw new System.Exception("No network adapters with an IPv4 address in the system!");
    }
    */
}