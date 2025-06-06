using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;
using System.Threading;

public class CSharpForCamera : MonoBehaviour
{
    Thread mThread;
    public string connectionIP = "127.0.0.1";
    public int connectionPort = 25003;
    IPAddress localAdd;
    TcpListener listener;
    TcpClient client;
    Vector3 currentPos = Vector3.zero;
    Vector3 receivedPos = Vector3.zero;
    string receivedCmd = ""; 

    bool running;
    bool stopped;
    bool cmd;
    bool calibed;

    private void Update()
    {
        if (cmd)
        {
            if (calibed){
                print(receivedPos);
                transform.position = receivedPos;
            }
            else{
                Vector3 tempPos = transform.position;
                Vector3 tempRot = transform.eulerAngles;
                if (receivedCmd == "x-"){
                    tempPos.x -= (float)0.1;
                }
                else if (receivedCmd == "x+"){
                    tempPos.x += (float)0.1;
                }
                else if (receivedCmd == "y-"){
                    tempPos.y -= (float)0.1;
                }
                else if (receivedCmd == "y+"){
                    tempPos.y += (float)0.1;
                }
                else if (receivedCmd == "z-"){
                    tempPos.z -= (float)0.1;
                }
                else if (receivedCmd == "z+"){
                    tempPos.z += (float)0.1;
                }
                else if (receivedCmd == "rx-")
                {
                    tempRot.x -= (float)10.0;
                }
                else if (receivedCmd == "rx+")
                {
                    tempRot.x += (float)10.0;
                }
                else if (receivedCmd == "ry-")
                {
                    tempRot.y -= (float)10.0;
                }
                else if (receivedCmd == "ry+")
                {
                    tempRot.y += (float)10.0;
                }
                else if (receivedCmd == "rz-")
                {
                    tempRot.z -= (float)10.0;
                }
                else if (receivedCmd == "rz+")
                {
                    tempRot.z += (float)10.0;
                }
                transform.position = tempPos; //assigning receivedPos in SendAndReceiveData()
                transform.eulerAngles = tempRot;
            }
            cmd = false;
        }
        currentPos = transform.position;
    }

    private void Start()
    {
        cmd = false;
        calibed = false;
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
        NetworkStream nwStream = client.GetStream();
        byte[] buffer = new byte[client.ReceiveBufferSize];

        //---receiving Data from the Host----
        int bytesRead = nwStream.Read(buffer, 0, client.ReceiveBufferSize); //Getting data in Bytes from Python
        string dataReceived = Encoding.UTF8.GetString(buffer, 0, bytesRead); //Converting byte data to string

        if (dataReceived != null)
        {
            //---Using received data---
            if (dataReceived == "x-")
            {
                receivedCmd = "x-";
                cmd = true;
            }
            else if (dataReceived == "x+")
            {
                receivedCmd = "x+";
                cmd = true;
            }
            else if (dataReceived == "y-")
            {
                receivedCmd = "y-";
                cmd = true;
            }
            else if (dataReceived == "y+")
            {
                receivedCmd = "y+";
                cmd = true;
            }
            else if (dataReceived == "z-")
            {
                receivedCmd = "z-";
                cmd = true;
            }
            else if (dataReceived == "z+")
            {
                receivedCmd = "z+";
                cmd = true;
            }
            else if (dataReceived == "rx-")
            {
                receivedCmd = "rx-";
                cmd = true;
            }
            else if (dataReceived == "rx+")
            {
                receivedCmd = "rx+";
                cmd = true;
            }
            else if (dataReceived == "ry-")
            {
                receivedCmd = "ry-";
                cmd = true;
            }
            else if (dataReceived == "ry+")
            {
                receivedCmd = "ry+";
                cmd = true;
            }
            else if (dataReceived == "rz-")
            {
                receivedCmd = "rz-";
                cmd = true;
            }
            else if (dataReceived == "rz+")
            {
                receivedCmd = "rz+";
                cmd = true;
            }
            else if (dataReceived.StartsWith("e"))
            {
                // print(dataReceived.Remove(0,1));
                receivedPos = StringToVector3(dataReceived.Remove(0,1));
                calibed = true;
                cmd = true;
            }
            

            //---Sending Data to Host----
            byte[] myWriteBuffer = Encoding.ASCII.GetBytes(currentPos.ToString()); //Converting string to byte data
            nwStream.Write(myWriteBuffer, 0, myWriteBuffer.Length); //Sending the data in Bytes to Python
        }
    }

    public static Vector3 StringToVector3(string sVector)
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');

        // store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[1]),
            float.Parse(sArray[2]));

        return result;
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