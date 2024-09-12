using Joycon4CS;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Globalization;
using System.IO.Pipes;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace JoyConTest
{
    internal class YAOID
    {
        static string devId = "0";
        bool autoUpdate = false;
        Ipc ipc;

        float currentRoll = 0.0f;
        float currentPitch = 0.0f;
        float currentYaw = 0.0f;

        float driftYaw = 0.0f;

        float currentX = 0.23f;
        float currentY = 1.30f;
        float currentZ = -0.15f;

        public bool leftAttached = false;

        int startId = 0;

        List<string> controllers = new List<string>();

        public void Connect()
        {
            ipc = new Ipc("YAOIDvr");
            ipc.Connect();
        }

        public void SpawnController(bool right = false)
        {
            if (ipc == null) return;
            string controllerStr = (!right ? "LEFT" : "RIGHT");
            string serial = $"CE00{startId}_" + controllerStr;

            Debug.WriteLine(ipc.SendRecv($"addcontroller {serial} {controllerStr}"));
            Debug.WriteLine(ipc.SendRecv($"cfixedpose 0"));
            controllers.Add(serial);
             this.leftAttached = true;
            startId += 1;
        }

        public void ResetYaw(float yaw)
        {
            driftYaw = yaw;
        }

        public void UpdateRotation(Quaternion quat, int deviceId = 0)
        {
            if (ipc == null) return;

            Quaternion q = Quaternion.CreateFromYawPitchRoll(quat.eulerAngles.Z - driftYaw, (quat.eulerAngles.Y + 1.8), -quat.eulerAngles.X);

            string resp =
                ipc.SendRecv($"setpose {deviceId} c {currentX} {currentY} {currentZ} {q.W} {q.X} {q.Y} {q.Z}");
            Debug.WriteLine(resp);
        }

        public void UpdateControllerState(Quaternion quat, ButtonState buttons, int deviceId = 0)
        {
            if (ipc == null) return;

            Quaternion q = Quaternion.CreateFromYawPitchRoll(quat.eulerAngles.Z - driftYaw, (quat.eulerAngles.Y + 1.8), -quat.eulerAngles.X);

            string resp =
                ipc.SendRecv($"cstate {deviceId} {currentX} {currentY} {currentZ} {q.W} {q.X} {q.Y} {q.Z} {buttons.JoyX} {buttons.JoyY} {(buttons.JOYDOWN ? 1 : 0)} 0.0 0.0 0 {(buttons.TRIGGER ? 1 : 0)} {(buttons.A ? 1 : 0)} {(buttons.B ? 1 : 0)} {(buttons.X ? 1 : 0)} {(buttons.Y ? 1 : 0)} {(buttons.MENU ? 1 : 0)} {(buttons.SYSTEM ? 1 : 0)} {(buttons.GRIP ? 1 : 0)}");
            Debug.WriteLine(resp);
        }
    }
    static class CeMath
    {
        public static Quaternion Multiply(this Quaternion lhs, Quaternion rhs)
        {
            return new Quaternion
            {
                W = (lhs.W * rhs.W) - (lhs.X * rhs.X) - (lhs.Y * rhs.Y) - (lhs.Z * rhs.Z),
                X = (lhs.W * rhs.X) + (lhs.X * rhs.W) + (lhs.Y * rhs.Z) - (lhs.Z * rhs.Y),
                Y = (lhs.W * rhs.Y) + (lhs.Y * rhs.W) + (lhs.Z * rhs.X) - (lhs.X * rhs.Z),
                Z = (lhs.W * rhs.Z) + (lhs.Z * rhs.W) + (lhs.X * rhs.Y) - (lhs.Y * rhs.X)
            };
        }

        public static Quaternion quaternionFromRotationX(double rot)
        {
            double ha = rot / 2;
            return new Quaternion()
            {
                W = (float)Math.Cos(ha),
                X = (float)Math.Sin(ha),
                Y = 0.0f,
                Z = 0.0f
            };
        }

        public static Quaternion quaternionFromRotationY(double rot)
        {
            double ha = rot / 2f;
            return new Quaternion()
            {
                W = (float)Math.Cos(ha),
                X = 0.0f,
                Y = (float)Math.Sin(ha),
                Z = 0.0f
            };
        }

        public static Quaternion quaternionFromRotationZ(double rot)
        {
            double ha = rot / 2f;
            return new Quaternion()
            {
                W = (float)Math.Cos(ha),
                X = 0.0f,
                Y = 0.0f,
                Z = (float)Math.Sin(ha)
            };
        }

        public static Quaternion QuaternionFromYawPitchRoll(float yaw, float pitch, float roll)
        {
            return quaternionFromRotationY(yaw) * quaternionFromRotationX(pitch) * quaternionFromRotationZ(roll);
        }
    }
    class DevicePose
    {
        public string DeviceID { get; set; }
        public Vector3 Position { get; set; }
        public Quaternion Rotation { get; set; }
        public Vector3 EulerAngles { get; set; }

        public DevicePose() { }

        public void UpdateFromIpc(string ipcMessage)
        {
            string[] strings;

            strings = ipcMessage.Split(' ');
            strings = strings.Skip(1).ToArray();

            if (strings.Length < 10) return;

            this.DeviceID = strings[1];

            this.Position = new Vector3()
            {
                X = float.Parse(strings[2], CultureInfo.InvariantCulture),
                Y = float.Parse(strings[3], CultureInfo.InvariantCulture),
                Z = float.Parse(strings[4], CultureInfo.InvariantCulture)
            };

            this.Rotation = new Quaternion()
            {
                W = float.Parse(strings[5], CultureInfo.InvariantCulture),
                X = float.Parse(strings[6], CultureInfo.InvariantCulture),
                Y = float.Parse(strings[7], CultureInfo.InvariantCulture),
                Z = float.Parse(strings[8], CultureInfo.InvariantCulture)
            };

            this.EulerAngles = new Vector3()
            {
                // x - roll
                X = float.Parse(strings[11], CultureInfo.InvariantCulture),
                // y - pitch
                Y = float.Parse(strings[10], CultureInfo.InvariantCulture),
                // z - yaw
                Z = float.Parse(strings[9], CultureInfo.InvariantCulture)
            };
        }
    }

    class ButtonState
    {
    public bool A { get; set; } = false;
        public bool B { get; set; } = false;
    public bool X { get; set; } = false;
    public bool Y { get; set; } = false;
    public bool TRIGGER { get; set; } = false;
    public bool MENU { get; set; } = false;
    public bool GRIP { get; set; } = false;
    public bool SYSTEM { get; set; } = false;
    public bool JOYDOWN { get; set; } = false;
    public float JoyX { get; set; } = 0.0f;
        public float JoyY { get; set; } = 0.0f;
}

    internal class Ipc
    {
        static NamedPipeClientStream namedPipeClient;
        private string _pipeName = "YAOIDvr";
        private int _bufferSize = 1024;

        public string pipeName { get => _pipeName; }
        public int bufferSize { get => _bufferSize; }

        public Ipc(string name, int bufferSize = 1024)
        {
            this._pipeName = name;
            this._bufferSize = bufferSize;
            namedPipeClient = new NamedPipeClientStream(pipeName);
        }
        public bool Connect(bool fromSend = false)
        {
            namedPipeClient = new NamedPipeClientStream(pipeName);
            if (!fromSend) Debug.WriteLine($"Connecting to {pipeName}...");
            namedPipeClient.Connect();
            if (!fromSend) Send("bumpinthat", true);
            if (!fromSend) Debug.WriteLine("Connected!");
            if (!fromSend) Receive();
            return true;
        }

        void Reconnect()
        {
            namedPipeClient = new NamedPipeClientStream(pipeName);
            namedPipeClient.Connect();
        }

        public bool Send(string message, bool fromConnect = false)
        {
            byte[] bytes = System.Text.Encoding.ASCII.GetBytes(message);
            try
            {
                if (!fromConnect) this.Reconnect();
                namedPipeClient.Write(bytes, 0, bytes.Length);
                Debug.WriteLine($"sent: {message}");
                return true;
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.ToString());
                return false;
            }
        }

        public string Receive()
        {
            byte[] buffer = new byte[1024];
            int byteFromClient = namedPipeClient.Read(buffer, 0, 1024);
            var str = System.Text.Encoding.Default.GetString(buffer);
            Debug.WriteLine($"received: {str}");
            //namedPipeClient.Close();
            return str;
        }

        public string SendRecv(string message)
        {
            Send(message);
            return Receive();
        }
    }
}
