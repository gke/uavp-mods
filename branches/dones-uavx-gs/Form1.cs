// =================================================================================================
// =                                  UAVX Quadrocopter Controller                                 =
// =                             Copyright (c) 2008 by Prof. Greg Egan                             =
// =                         Instruments Copyright (c) 2008 Guillaume Choutea                      =                                        */
// =                               http://code.google.com/p/uavp-mods/                             =
// =================================================================================================

//    This is part of UAVX.

//    UAVX is free software: you can redistribute it and/or modify it under the terms of the GNU 
//    General Public License as published by the Free Software Foundation, either version 3 of the 
//    License, or (at your option) any later version.

//    UAVX is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even 
//    the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
//    General Public License for more details.

//    You should have received a copy of the GNU General Public License along with this program.  
//    If not, see http://www.gnu.org/licenses/

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Threading;
using System.Windows.Forms;

namespace UAVXGS
{
    public partial class FormMain : Form
    {
        // ASCII Constants
        const byte NUL = 0;
        const byte SOH = 1;
        const byte EOT = 4;
        const byte ACK = 6;
        const byte HT = 9;
        const byte LF = 10;
        const byte CR = 13;
        const byte NAK = 21;
        const byte ESC = 27;

        const int DefaultBaudRate = 9600;

        const Byte RCMaximum = 238;
        const double OUTMaximumScale = 0.5; // 100/200 % for PWM at least

        const byte UnknownPacketTag = 0;
        const byte RestartPacketTag = 8;
        const byte UAVXFlightPacketTag = 13;
        const byte UAVXNavPacketTag = 14;
        const byte UAVXStatsPacketTag = 15;

        const byte WaitRxSentinel = 0;
        const byte WaitRxBody = 1;
        const byte WaitRxESC = 2;
        const byte WaitRxCheck = 3;
        const byte WaitRxCopy = 4;
        const byte WaitUPTag = 5;
        const byte WaitUPLength = 6;
        const byte WaitUPBody = 7;

        const byte DefaultAttitudeToDegrees = 35;
        byte AttitudeToDegrees;

        const int DefaultRangeLimit = 100;
        const int MaximumRangeLimit = 250; // You carry total responsibility if you increase this value
        const int MaximumAltitudeLimit = 121; // You carry total responsibility if you increase this value 

        const double CurrentSensorMax = 50.0; // depends on current ADC used - needs calibration box?
        const double YawGyroRate = 400.0;
        const double RollPitchGyroRate = 400.0;

        /*
        UAVXStatsPacket
         
        */

        short I2CFailsT;
        short GPSFailsT; 
        short AccFailsT; 
        short GyroFailsT; 
        short CompassFailsT; 
        short BaroFailsT; 
        short I2CESCFailsT;
        short RCFailSafesT;

        short GPSAltitudeT;
        short GPSMaxVelT;
        short GPSMinSatsT;
        short GPSMaxSatsT;
        short GPSMinHDiluteT;
        short GPSMaxHDiluteT;
        
        short BaroRelAltitudeT;
        short BaroMinROCT;
        short BaroMaxROCT;

        short MinTempT;
        short MaxTempT;

        short BadT;
        short Unused1T;
        short Unused2T;

        /*
        UAVXFlightPacket
        
        Flags bit values
        Flags[0]
            NavAltitudeHold	
            TurnToWP			
            GyroFailure
            LostModel
            NearLevel
            LowBatt
            GPSValid
            NavValid

        Flags[1]
            BaroFailure
            AccFailure
            CompassFailure
            GPSFailure
            AttitudeHold
            ThrottleMoving
            Hovering
            Navigate
                
        Flags[2]
            ReturnHome
            Proximity
            CloseProximity
            UsingGPSAlt
            UsingRTHAutoDescend
            BaroAltitudeValid
            RangefinderAltitudeValid
            UsingRangefinderAlt
      
        // internal flags not really useful externally
        Flags[3..5]
            Signal
            RCFrameOK
            ParametersValid
            RCNewValues
            NewCommands
            AccelerationsValid
            CompassValid
            CompassMissRead

            GyrosErected
            ReceivingGPS
            GPSSentenceReceived
            NavComputed
            CheckThrottleMoved		
            WayPointsChanged
            UsingSerialPPM
            UsingTxMode2

		    UsingTelemetry
		    TxToBuffer
		    NewBaroValue
		    BeeperInUse 
		    AcquireNewPosition
		    MotorsArmed
		    RFInInches
		    Simulation
            */

        // byte UAVXFlightPacketTag;   
        // byte Length;  
        const byte NoOfFlagBytes = 6;
        const byte NoOfOutputs = 6;
        byte[] Flags = new byte[NoOfFlagBytes];
        byte StateT;                    // 8
        short BatteryVoltsT;            // 9
        short BatteryCurrentT;          // 11
 		short BatteryChargeT;           // 13
        short RCGlitchesT;              // 15
        short DesiredThrottleT;         // 17
        short DesiredRollT;             // 19
        short DesiredPitchT;            // 21
        short DesiredYawT;              // 23
        short RollRateT;                // 25
        short PitchRateT;               // 27
        short YawRateT;                 // 29
        short RollSumT;                 // 31
        short PitchSumT;                // 33
        short YawSumT;                  // 35
        short LRAccT;                   // 37
        short FBAccT;                   // 39
        short DUAccT;                   // 41
        short LRCompT;                  // 43
        short FBCompT;                  // 44
        short DUCompT;                  // 45
        short AltCompT;                 // 46
        byte[] OutputT = new byte[6];   // 47
        int MissionTimeMilliSecT;       // 53

   
        // UAVXNavPacket
        //byte UAVXNavPacketTag;
        //byte Length;
        byte NavStateT;                 // 2
        byte FailStateT;                // 3
        byte GPSNoOfSatsT;              // 4
        byte GPSFixT;                   // 5

        byte CurrWPT;                   // 6

        short BaroROCT;                 // 7 
        int RelBaroAltitudeT;           // 9

        short RangefinderROCT;          // 12
        short RangefinderAltitudeT;     // 14

        short GPSHDiluteT;              // 16
        short HeadingT;                 // 18
        short DesiredCourseT;           // 20

        short GPSVelT;                  // 22
        short GPSROCT;                  // 24
        int GPSRelAltitudeT;            // 26
        int GPSLatitudeT;               // 29
        int GPSLongitudeT;              // 33
        int DesiredAltitudeT;           // 37
        int DesiredLatitudeT;           // 40
        int DesiredLongitudeT;          // 44
        int NavStateTimeoutT;           // 48
        short AmbientTempT;             // 51
        int GPSMissionTimeT;            // 53


        double Distance, LongitudeCorrection, WhereDirection;
        bool FirstGPSCoordinates = true;
        int OriginLatitude, OriginLongitude;
        double EastDiff, NorthDiff;

        byte[] UAVXPacket = new byte[256];

        const short RxQueueLength = 2048;
        const short RxQueueMask = RxQueueLength - 1;
        byte[] RxQueue = new byte[RxQueueLength];
        short RxTail = 0;
        short RxHead = 0;
        int NavPacketsReceived = 0;
        int FlightPacketsReceived = 0;
        int StatsPacketsReceived = 0;

        short RxPacketByteCount;
        byte RxPacketTag, RxPacketLength, PacketLength, PacketRxState;
        byte ReceivedPacketTag;
        bool PacketReceived = false;

        bool CheckSumError;
        short RxLengthErrors = 0, RxCheckSumErrors = 0, RxIllegalErrors = 0;

        byte RxCheckSum;

        int COMBaudRate = DefaultBaudRate;

        double CurrAlt, AltError;

        bool InFlight = false;

        System.IO.FileStream SaveLogFileStream;
        System.IO.BinaryWriter SaveLogFileBinaryWriter;

        System.IO.FileStream SaveTextLogFileStream;
        System.IO.StreamWriter SaveTextLogFileStreamWriter;

        System.IO.FileStream OpenLogFileStream;
        System.IO.BinaryReader OpenLogFileBinaryReader;
        bool DoingLogfileReplay = false;

        public FormMain()
        {
            InitializeComponent();

           // COMSelectComboBox.Items.Add("Select COM Port");
            string[] AvailableCOMPorts = ComPorts.readPorts();
            foreach (string AvailableCOMPort in AvailableCOMPorts)
                COMSelectComboBox.Items.Add(AvailableCOMPort);
            COMSelectComboBox.Text = UAVXGS.Properties.Settings.Default.COMPort;
            COMBaudRateComboBox.Text = string.Format("{0:n0}", UAVXGS.Properties.Settings.Default.COMBaudRate);

            Version vrs = new Version(Application.ProductVersion);
            this.Text = this.Text + " v" + vrs.Major + "." + vrs.Minor + "." + vrs.Build;

        }

        public static class ComPorts
        {
            public static string[] readPorts()
            {
                string[] ComPorts = System.IO.Ports.SerialPort.GetPortNames();
                Array.Sort(ComPorts);
                return ComPorts;
            }
        }

        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                e.Cancel = true; //cancel the fom closing
                Thread CloseDown = new Thread(new ThreadStart(CloseSerialOnExit)); //close port in new thread to avoid hang
                CloseDown.Start(); //close port in new thread to avoid hang
            }
        }

        private void CloseSerialOnExit()
        {
            try
            {
                serialPort1.Close(); //close the serial port
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message); //catch any serial port closing error messages
            }
            this.Invoke(new EventHandler(NowClose)); //now close back in the main thread
        }

        private void NowClose(object sender, EventArgs e)
        {
            this.Close(); //now close the form
        }

        public void SetSerialPort(string portSelected, int speed, ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                serialPort1.PortName = portSelected;
                serialPort1.ReadBufferSize = 8192;
                serialPort1.BaudRate = speed;
                serialPort1.ReceivedBytesThreshold = 1;
                serialPort1.Open();
                serialPort1.Close();
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }
        }


       // private void Form1_FormClosing(object sender, FormClosingEventArgs e)
       // {
       //     if (serialPort1.IsOpen) serialPort1.Close();
       // }

        private void COMSelectComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            UAVXGS.Properties.Settings.Default.COMPort = COMSelectComboBox.Text;
            UAVXGS.Properties.Settings.Default.Save();
        }

        private void COMBaudRateComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            UAVXGS.Properties.Settings.Default.COMBaudRate = Convert.ToInt32(COMBaudRateComboBox.Text);
            UAVXGS.Properties.Settings.Default.Save();
        }

        private void textBox1_KeyPress(object sender, KeyPressEventArgs e)
        {
            // If the port is closed, don't try to send a character.
            if (!serialPort1.IsOpen) return;

            // If the port is Open, declare a char[] array with one element.
            char[] buff = new char[1];

            // Load element 0 with the key character.
            buff[0] = e.KeyChar;

            // Send the one character buffer.
            serialPort1.Write(buff, 0, 1);

            // Set the KeyPress event as handled so the character won't
            // display locally. If you want it to display, omit the next line.
            e.Handled = true;
        }

        private void ReplayButton_Click(object sender, EventArgs e)
        {
            if (!DoingLogfileReplay)
            {
                OpenLogFileDialog.Filter = "UAVX Log File (*.log)|*.log";
                OpenLogFileDialog.InitialDirectory = UAVXGS.Properties.Settings.Default.LogFileFolder;
                if (OpenLogFileDialog.ShowDialog() == DialogResult.OK)
                {
                    UAVXGS.Properties.Settings.Default.LogFileFolder = OpenLogFileDialog.InitialDirectory;
                    OpenLogFileStream = new System.IO.FileStream(OpenLogFileDialog.FileName, System.IO.FileMode.Open);
                    OpenLogFileBinaryReader = new System.IO.BinaryReader(OpenLogFileStream);
                    SaveTextLogFileStream = new System.IO.FileStream("UAVX_Replay.csv", System.IO.FileMode.Create);
                    SaveTextLogFileStreamWriter = new System.IO.StreamWriter(SaveTextLogFileStream, System.Text.Encoding.ASCII);
                    WriteTextLogFileHeader();
                    DoingLogfileReplay = true;

                    ReplayProgressBar.Value = 0;
                    while (DoingLogfileReplay)
                        ReadReplayLogFile();
                }
            }
            else
                CloseReplayLogFile();
        }

        private void CloseReplayLogFile()
        {
            OpenLogFileStream.Close();
            SaveTextLogFileStreamWriter.Flush();
            SaveTextLogFileStreamWriter.Close();
            SaveTextLogFileStream.Close();
        }

        private void CreateSaveLogFile()
        {
            string FileName;

            FileName = "C:/Documents and Settings/All Users/Desktop/UAVX_" +
                DateTime.Now.Year +
                DateTime.Now.Month +
                DateTime.Now.Day + "_" +
                DateTime.Now.Hour +
                DateTime.Now.Minute;

            SaveLogFileStream = new System.IO.FileStream(FileName + ".log", System.IO.FileMode.Create);
            SaveLogFileBinaryWriter = new System.IO.BinaryWriter(SaveLogFileStream);
            SaveTextLogFileStream = new System.IO.FileStream(FileName + ".csv", System.IO.FileMode.Create);
            SaveTextLogFileStreamWriter = new System.IO.StreamWriter(SaveTextLogFileStream, System.Text.Encoding.ASCII);
            WriteTextLogFileHeader();
            NavPacketsReceived = 0;
            FlightPacketsReceived = 0;
            StatsPacketsReceived = 0;
        }

        private void WriteTextLogFileHeader()
        {
            short i;

            SaveTextLogFileStreamWriter.Write("Flight,");
            for (i = 0; i < NoOfFlagBytes; i++)
                SaveTextLogFileStreamWriter.Write("F[" + i + "],");
            SaveTextLogFileStreamWriter.Write("StateT," +
            "BattV," +
            "BattC," +
            "BattCh," +
            "RCGlitches," +
            "DesThr," +
            "DesRoll," +
            "DesPitch," +
            "DesYaw," +
            "RollRate," +
            "PitchRate," +
            "YawRate," +
            "RollSum," +
            "PitchSum," +
            "YawSum," +
            "LRAcc," +
            "FBAcc," +
            "DUAcc," +
            "LRComp," +
            "FBComp," +
            "DUComp," +
            "AltComp,");

            for (i = 0; i < NoOfOutputs; i++)
                SaveTextLogFileStreamWriter.Write("Out[" + i + "],");

            SaveTextLogFileStreamWriter.Write("TimemS,");

            SaveTextLogFileStreamWriter.Write("Nav," +
            "NavState," +
            "FailState," +
            "GPSSats," +
            "GPSFix," +
            "CurrWP," +
            "BaroROC," +
            "RelBaroAlt," +
            "RFROC," +
            "RFAlt," +
            "GPSHD," +
            "HeadingT," +
            "DesCourse," +
            "GPSVelT," +
            "GPSROCT," +
            "GPSRelAlt," +
            "GPSLat," +
            "GPSLon," +
            "DesAlt," +
            "DesLat," +
            "DesLon," +
            "NavStateTimeoutT," +
            "AmbTemp," +
            "GPSTime,");

            SaveTextLogFileStreamWriter.WriteLine("Stats," +
            "I2CFails," +
            "GPSFails," + 
            "AccFails," + 
            "GyroFails," + 
            "CompFails," + 
            "BaroFails," + 
            "I2CESCFails," +
            "RCFSafes," +
            "GPSAlt," +
            "GPSMaxVel," +
            "GPSMinSats," +
            "GPSMaxSats," +
            "GPSMinHD," +
            "GPSMaxHD," +     
            "BaroRelAlt," +
            "BaroMinROC," +
            "BaroMaxROC," +
            "MinTemp," +
            "MaxTemp," +
            "Bad," +
            "Unused1, Unused2");
        }  

        private void FlyingButton_Click(object sender, EventArgs e)
        {
            if (InFlight)
            {
                InFlight = false;
                UAVXCloseTelemetry();
                FlyingButton.Text = "Disconnected";
                FlyingButton.BackColor = System.Drawing.Color.Red;
            }
            else
            {
                FlyingButton.Text = "Connected";
                FlyingButton.BackColor = System.Drawing.Color.Green;
                UAVXOpenTelemetry();
            }
 
         //   e.Handled = true;
        }

        //-----------------------------------------------------------------------
        // UAVX Communications

        private void UAVXOpenTelemetry()
        {
            string sError = "";

            UAVXCloseTelemetry();
            
            SetSerialPort(UAVXGS.Properties.Settings.Default.COMPort, UAVXGS.Properties.Settings.Default.COMBaudRate, ref sError);
            if (sError != "")
            {
                MessageTextBox.Text = sError;
                InFlight = false;
            }
            else
            {
                CreateSaveLogFile();
                RxHead = RxTail = 0;
                InitPollPacket();
                serialPort1.Open();
                if (serialPort1.IsOpen)
                    InFlight = true;
                else
                {
                    UAVXCloseTelemetry();
                    InFlight = false;
                }
            } 
        }

        private void UserAttitudeToDegrees_TextChanged(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(UserAttitudeToDegrees.Text, out outValue))
                AttitudeToDegrees = Convert.ToByte(outValue);
            else
                AttitudeToDegrees = DefaultAttitudeToDegrees;
        }

        private void ReadReplayLogFile()
        {
            byte b;
            long p;
            short NewRxTail;

            NewRxTail = RxTail;
            NewRxTail++;
            NewRxTail &= RxQueueMask;

            while ((NewRxTail != RxHead) && (OpenLogFileStream.Position < OpenLogFileStream.Length)) // inefficient?
            {
                RxTail = NewRxTail;
                b = OpenLogFileBinaryReader.ReadByte();
                RxQueue[RxTail] = b;

                NewRxTail = RxTail;
                NewRxTail++;
                NewRxTail &= RxQueueMask;
            }
            this.Invoke(new EventHandler(UAVXReadTelemetry));
            p = (int) (100 * OpenLogFileStream.Position) / OpenLogFileStream.Length;
            ReplayProgressBar.Value = (int)p;
            if (OpenLogFileStream.Position == OpenLogFileStream.Length)
            {
                DoingLogfileReplay = false;
                CloseReplayLogFile();
            }
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            byte b;
            short NewRxTail;

            if (serialPort1.IsOpen) // flakey serial drivers hence heroics
            {
                while ( serialPort1.IsOpen && (serialPort1.BytesToRead != 0))
                {
                    NewRxTail = RxTail;
                    NewRxTail++;
                    NewRxTail &= RxQueueMask;
                    if (NewRxTail != RxHead)
                    {
                        RxTail = NewRxTail;
                        b = (byte)serialPort1.ReadByte();
                        RxQueue[RxTail] = b;
                        SaveLogFileBinaryWriter.Write((byte) b);
                    }
                    else
                        b = (byte)serialPort1.ReadByte(); // processing overflow - discard
                    SaveLogFileBinaryWriter.Write(b);
                }
                if (serialPort1.IsOpen) 
                    this.Invoke(new EventHandler(UAVXReadTelemetry));
            }
        }

        void InitPollPacket()
        {
              RxPacketByteCount = 0;   
              RxCheckSum = 0;

              Zero(ref UAVXPacket, 256);    
              RxPacketTag = UnknownPacketTag;

              RxPacketLength = 2; // set as minimum
              PacketRxState = WaitRxSentinel;    
        }

        void AddToBuffer(byte ch)
        {
            bool RxPacketError;

            UAVXPacket[RxPacketByteCount++] = ch;
            if (RxPacketByteCount == 1)
            {
                RxPacketTag = ch;
                switch ( RxPacketTag ) {
                case UAVXFlightPacketTag: PacketLength = 54; break;
                case UAVXNavPacketTag: PacketLength = 55; break;
                case UAVXStatsPacketTag: PacketLength = 44; break;
                default:
                    RxIllegalErrors++;
                    RxPacketTag=UnknownPacketTag;  
                    break;
                } // switch
                if (RxPacketTag==UnknownPacketTag)
                    PacketRxState=WaitRxSentinel;
                else
                    PacketRxState=WaitRxBody;
            }
            else
                if (RxPacketByteCount == 2)
                {
                    RxPacketLength = ch; // ignore
                    PacketRxState = WaitRxBody;
                }
                else
                    if(RxPacketByteCount >= 255 ) // zzzsizeof(Packet)) // packet too long for buffer
                    {
                        RxLengthErrors++;
                        PacketRxState = WaitRxSentinel;
                    }
                    else
                        if (RxPacketByteCount >= (PacketLength+3))
                        {
                            RxPacketError = RxCheckSum != 0;

                            CheckSumError = RxPacketError;
                            if (CheckSumError)
                                RxCheckSumErrors++;

                            RxPacketError=false;
                            if (!RxPacketError)
                            {
                                PacketReceived = true;
                                ReceivedPacketTag=RxPacketTag;   
                            }
                            PacketRxState = WaitRxSentinel;
                         //   InitPollPacket(); 
                        }
                        else
                            PacketRxState = WaitRxBody;         
        }

        void ParsePacket(byte ch) 
        {
            switch (PacketRxState) {
            case WaitRxSentinel:
                if (ch == SOH)
                {
                    InitPollPacket();
                    CheckSumError = false;
                    PacketRxState = WaitRxBody;
                }
                break;
            case WaitRxBody:
               if (ch == ESC)
                   PacketRxState = WaitRxESC;
                else
                    if (ch == SOH) // unexpected start of packet
                    {
                        RxLengthErrors++;

                        InitPollPacket();
                        PacketRxState = WaitRxBody;
                    }
                    else
                        if (ch == EOT) // unexpected end of packet 
                        {
                            RxLengthErrors++;
                            PacketRxState = WaitRxSentinel;
                        }
                        else
                            AddToBuffer(ch); 
                break;
            case WaitRxESC:
                AddToBuffer(ch);
                break;
            default: PacketRxState = WaitRxSentinel; break; 
            } 
        }

        public void UAVXReadTelemetry(object sender, EventArgs e)
        {
            byte b, m;
            short i;
  
            while ( RxHead != RxTail) 
            {
                b = RxQueue[RxHead];
                RxHead++;
                RxHead &= RxQueueMask;

                ParsePacket( b );
                
                RxTypeErr.Text = string.Format("{0:n0}", RxIllegalErrors);
                RxCSumErr.Text = string.Format("{0:n0}", RxCheckSumErrors);
                RxLenErr.Text = string.Format("{0:n0}", RxLengthErrors);

                if ( PacketReceived )
                {
                    PacketReceived = false;

                    AttitudeToDegrees = Convert.ToByte(UserAttitudeToDegrees.Text);

                    switch ( RxPacketTag ) {
                    case UAVXStatsPacketTag:
                            StatsPacketsReceived++;
                            I2CFailsT = ExtractShort(ref UAVXPacket, 2);
                            GPSFailsT = ExtractShort(ref UAVXPacket, 4); 
                            AccFailsT = ExtractShort(ref UAVXPacket, 6); 
                            GyroFailsT = ExtractShort(ref UAVXPacket, 8); 
                            CompassFailsT = ExtractShort(ref UAVXPacket, 10); 
                            BaroFailsT = ExtractShort(ref UAVXPacket, 12); 
                            I2CESCFailsT = ExtractShort(ref UAVXPacket, 14);
 
                            RCFailSafesT = ExtractShort(ref UAVXPacket, 16);

                            GPSAltitudeT = ExtractShort(ref UAVXPacket, 18);
                            GPSMaxVelT = ExtractShort(ref UAVXPacket, 20);
                            GPSMinSatsT = ExtractShort(ref UAVXPacket, 22);
                            GPSMaxSatsT = ExtractShort(ref UAVXPacket, 24);
                            GPSMinHDiluteT = ExtractShort(ref UAVXPacket, 26);
                            GPSMaxHDiluteT = ExtractShort(ref UAVXPacket, 28);

                            BaroRelAltitudeT = ExtractShort(ref UAVXPacket, 30);
                            BaroMinROCT = ExtractShort(ref UAVXPacket, 32);
                            BaroMaxROCT = ExtractShort(ref UAVXPacket, 34);

                            MinTempT = ExtractShort(ref UAVXPacket, 36);
                            MaxTempT = ExtractShort(ref UAVXPacket, 38);

                            BadT = ExtractShort(ref UAVXPacket, 40);

                            I2CFailS.Text = string.Format("{0:n0}", I2CFailsT);
                            GPSFailS.Text = string.Format("{0:n0}", GPSFailsT);
                            AccFailS.Text = string.Format("{0:n0}", AccFailsT);
                            GyroFailS.Text = string.Format("{0:n0}", GyroFailsT);
                            CompassFailS.Text = string.Format("{0:n0}", CompassFailsT);
                            BaroFailS.Text = string.Format("{0:n0}", BaroFailsT);
                            I2CESCFailS.Text = string.Format("{0:n0}", I2CESCFailsT);
                            RCFailSafeS.Text = string.Format("{0:n0}", RCFailSafesT);

                            GPSAltitudeS.Text = string.Format("{0:n1}", (float)GPSAltitudeT*0.1);
                            GPSMaxVelS.Text = string.Format("{0:n1}", (float)GPSMaxVelT*0.1);
                            GPSMinSatS.Text = string.Format("{0:n0}", GPSMinSatsT);
                            GPSMaxSatS.Text = string.Format("{0:n0}", GPSMaxSatsT);
                            GPSMinHDiluteS.Text = string.Format("{0:n2}", (float)GPSMinHDiluteT*0.01);
                            GPSMaxHDiluteS.Text = string.Format("{0:n2}", (float)GPSMaxHDiluteT*0.01);

                            BaroRelAltitudeS.Text = string.Format("{0:n1}", (float)BaroRelAltitudeT*0.1);
                            BaroMinROCS.Text = string.Format("{0:n1}", (float)BaroMinROCT*0.1);
                            BaroMaxROCS.Text = string.Format("{0:n1}", (float)BaroMaxROCT*0.1);
                            MinTempS.Text = string.Format("{0:n1}", (float)MinTempT*0.1);
                            MaxTempS.Text = string.Format("{0:n1}", (float)MaxTempT*0.1);

                            BadS.Text = string.Format("{0:n0}", BadT);
                            
                            // 2 unused 16bit stat
                     
                            break;
                    case UAVXFlightPacketTag:
                        FlightPacketsReceived++;
                        for (i = 2; i < (NoOfFlagBytes+2); i++)
                            Flags[i-2] = ExtractByte(ref UAVXPacket, i);

                        if ((Flags[0] & 0x01) != 0) 
                            AltHoldBox.BackColor = System.Drawing.Color.Green;
                        else
                            AltHoldBox.BackColor = System.Drawing.Color.Red;
                        if ((Flags[0] & 0x02) != 0)
                            TurnToWPBox.BackColor = System.Drawing.Color.Green;
                        else
                            TurnToWPBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x04) != 0)
                            GyroFailureBox.BackColor = System.Drawing.Color.Green;
                        else
                            GyroFailureBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x08) != 0)
                            LostModelBox.BackColor = System.Drawing.Color.Red;
                        else
                            LostModelBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[0] & 0x10) != 0)
                            NearLevelBox.BackColor = System.Drawing.Color.Green;
                        else
                            NearLevelBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        if ((Flags[0] & 0x20) != 0)
                        {
                            LowBatteryBox.BackColor = System.Drawing.Color.Red;
                            BatteryVolts.BackColor = System.Drawing.Color.Red;
                        }
                        else
                        {
                            LowBatteryBox.BackColor = FlagsGroupBox.BackColor;
                            BatteryVolts.BackColor = BatteryGroupBox.BackColor;
                        }
                        if ((Flags[0] & 0x40) != 0)
                            GPSValidBox.BackColor = System.Drawing.Color.Green;
                        else
                            GPSValidBox.BackColor = System.Drawing.Color.Red;
                        if ((Flags[0] & 0x80) != 0)
                        {
                            NavValidBox.BackColor = System.Drawing.Color.Green;
                            GPSROC.BackColor = AltitudeGroupBox.BackColor;
                            GPSRelAltitude.BackColor = AltitudeGroupBox.BackColor;
                        }
                        else
                        {
                            NavValidBox.BackColor = System.Drawing.Color.Red;
                            GPSROC.BackColor = System.Drawing.Color.Red;
                            GPSRelAltitude.BackColor = System.Drawing.Color.Red;

                        }

                        if ((Flags[1] & 0x01) != 0)
                            BaroFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            BaroFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x02) != 0)
                            AccFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            AccFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x04) != 0)
                            CompassFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            CompassFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x08) != 0)
                            GPSFailBox.BackColor = System.Drawing.Color.Red;
                        else
                            GPSFailBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x10) != 0)
                            AttitudeHoldBox.BackColor = System.Drawing.Color.Green;
                        else
                            AttitudeHoldBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        if ((Flags[1] & 0x20) != 0)
                            ThrottleMovingBox.BackColor = System.Drawing.Color.LightSteelBlue;
                        else
                            ThrottleMovingBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x40) != 0)
                            HoldingAltBox.BackColor = System.Drawing.Color.Green;
                        else
                            HoldingAltBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[1] & 0x80) != 0)
                            NavigateBox.BackColor = System.Drawing.Color.Green;
                        else
                            NavigateBox.BackColor = FlagsGroupBox.BackColor;
                 
                        if ((Flags[2] & 0x01) != 0)
                            ReturnHomeBox.BackColor = System.Drawing.Color.Green;
                        else
                            ReturnHomeBox.BackColor = FlagsGroupBox.BackColor;

                        if ((Flags[2] & 0x02) != 0)
                            ProximityBox.BackColor = System.Drawing.Color.Green;
                        else
                            ProximityBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x04) != 0)
                            CloseProximityBox.BackColor = System.Drawing.Color.Green;
                        else
                            CloseProximityBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x08) != 0)
                            UsingGPSAltBox.BackColor = System.Drawing.Color.Green;
                        else
                            UsingGPSAltBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x10) != 0)
                            UseRTHAutoDescendBox.BackColor = System.Drawing.Color.Green;
                        else
                            UseRTHAutoDescendBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x20) != 0)
                            BaroAltValidBox.BackColor = System.Drawing.Color.Green;
                        else
                            BaroAltValidBox.BackColor = FlagsGroupBox.BackColor;
                        if ((Flags[2] & 0x40) != 0)
                        {
                            RangefinderAltValidBox.BackColor = System.Drawing.Color.Green;
                            RangefinderAltitude.BackColor = AltitudeGroupBox.BackColor;
                            RangefinderROC.BackColor = AltitudeGroupBox.BackColor;
                            BaroRFError.BackColor = BaroCalibrationGroupBox.BackColor;
                        }
                        else
                        {
                            RangefinderAltValidBox.BackColor = FlagsGroupBox.BackColor;
                            RangefinderAltitude.BackColor = System.Drawing.Color.Orange;
                            RangefinderROC.BackColor = System.Drawing.Color.Orange;
                            BaroRFError.BackColor = System.Drawing.Color.Orange;
                        }
                        if ((Flags[2] & 0x80) != 0)
                            UsingRangefinderBox.BackColor = System.Drawing.Color.Green;
                        else
                            UsingRangefinderBox.BackColor = FlagsGroupBox.BackColor;

                        if ((Flags[5] & 0x80) != 0)
                            SimulationTextBox.Text = "Simulation";
                        else
                            SimulationTextBox.Text = " ";
    
                        StateT = ExtractByte(ref UAVXPacket, 8);
                        switch ( StateT ){
                            case 0: FlightState.Text = "Starting"; break;
                            case 1: FlightState.Text = "Landing"; break;
                            case 2: FlightState.Text = "Landed"; break;
                            case 3: FlightState.Text = "Shutdown"; break;
                            case 4: FlightState.Text = "Flying"; break;
                            default: FlightState.Text = "Unknown"; break;
                        } // switch

                        BatteryVoltsT =  ExtractShort(ref UAVXPacket, 9);
                        BatteryCurrentT = ExtractShort(ref UAVXPacket, 11);
                        BatteryChargeT = ExtractShort(ref UAVXPacket, 13);
                        RCGlitchesT = ExtractShort(ref UAVXPacket, 15);
                        DesiredThrottleT = ExtractShort(ref UAVXPacket, 17);
                        DesiredRollT = ExtractShort(ref UAVXPacket, 19);
                        DesiredPitchT = ExtractShort(ref UAVXPacket, 21);
                        DesiredYawT = ExtractShort(ref UAVXPacket, 23);
                        RollRateT = ExtractShort(ref UAVXPacket, 25);
                        PitchRateT = ExtractShort(ref UAVXPacket, 27);
                        YawRateT = ExtractShort(ref UAVXPacket, 29);
                        RollSumT = ExtractShort(ref UAVXPacket, 31);
                        PitchSumT = ExtractShort(ref UAVXPacket, 33);
                        YawSumT = ExtractShort(ref UAVXPacket, 35);
                        LRAccT = ExtractShort(ref UAVXPacket, 37);
                        FBAccT = ExtractShort(ref UAVXPacket, 39);
                        DUAccT = ExtractShort(ref UAVXPacket, 41);
                        LRCompT = ExtractSignedByte(ref UAVXPacket, 43);
                        FBCompT = ExtractSignedByte(ref UAVXPacket, 44);
                        DUCompT = ExtractSignedByte(ref UAVXPacket, 45);
                        AltCompT = ExtractSignedByte(ref UAVXPacket, 46);

                        for (m = 47; m < (47 + 6 ); m++ )
                            OutputT[m-47] = ExtractByte(ref UAVXPacket, m);

                        MissionTimeMilliSecT = ExtractInt24(ref UAVXPacket, 53);

                        BatteryVolts.Text = string.Format("{0:n1}", (float)BatteryVoltsT * 27.73 / 1024.0); // ADC units 5V * (10K+2K2)/2K2.
                        BatteryCurrent.Text = string.Format("{0:n1}", ((float)BatteryCurrentT * CurrentSensorMax) / 1024.0 ); // ADC units sent
                        BatteryCharge.Text = string.Format("{0:n0}", (float)BatteryChargeT); // mAH // converted as it is used on board

                        RCGlitches.Text = string.Format("{0:n0}", RCGlitchesT); 
                        if ( RCGlitchesT > 20 )
                            RCGlitches.BackColor = System.Drawing.Color.Orange;
                        else
                            RCGlitches.BackColor = EnvGroupBox.BackColor;

                        DesiredThrottle.Text = string.Format("{0:n0}", ((float)DesiredThrottleT * 100.0)/ RCMaximum);
                        DesiredRoll.Text = string.Format("{0:n0}", ((float)DesiredRollT * 200.0)/ RCMaximum); 
                        DesiredPitch.Text = string.Format("{0:n0}", ((float)DesiredPitchT * 200.0)/ RCMaximum);
                        DesiredYaw.Text = string.Format("{0:n0}", ((float)DesiredYawT * 200.0) / RCMaximum); 
                        RollRate.Text = string.Format("{0:n0}", RollRateT); 
                        PitchRate.Text = string.Format("{0:n0}", PitchRateT); 
                        YawRate.Text = string.Format("{0:n0}", YawRateT);
                        RollSum.Text = string.Format("{0:n0}", RollSumT / AttitudeToDegrees);
                        PitchSum.Text = string.Format("{0:n0}", PitchSumT / AttitudeToDegrees);
                        YawSum.Text = string.Format("{0:n0}", YawSumT / AttitudeToDegrees); 
                        LRAcc.Text = string.Format("{0:n2}", (float)LRAccT / 1024.0);
                        FBAcc.Text = string.Format("{0:n2}", (float)FBAccT / 1024.0);
                        DUAcc.Text = string.Format("{0:n2}", (float)DUAccT / 1024.0);
                        LRComp.Text = string.Format("{0:n0}", LRCompT * OUTMaximumScale);
                        FBComp.Text = string.Format("{0:n0}", FBCompT * OUTMaximumScale);
                        DUComp.Text = string.Format("{0:n0}", DUCompT * OUTMaximumScale);
                        AltComp.Text = string.Format("{0:n0}", AltCompT * OUTMaximumScale);

                        OutputT0.Text = string.Format("{0:n0}", OutputT[0] * OUTMaximumScale);
                        OutputT1.Text = string.Format("{0:n0}", OutputT[1] * OUTMaximumScale);
                        OutputT2.Text = string.Format("{0:n0}", OutputT[2] * OUTMaximumScale);
                        OutputT3.Text = string.Format("{0:n0}", OutputT[3] * OUTMaximumScale);
                        OutputT4.Text = string.Format("{0:n0}", OutputT[4] * OUTMaximumScale);
                        OutputT5.Text = string.Format("{0:n0}", OutputT[5] * OUTMaximumScale);

                        MissionTimeSec.Text = string.Format("{0:n3}", MissionTimeMilliSecT * 0.001);

                        attitudeIndicatorInstrumentControl1.SetAttitudeIndicatorParameters(
                            -PitchSumT / AttitudeToDegrees,
                            RollSumT / AttitudeToDegrees
                            );
                           
                        break;
                    case UAVXNavPacketTag:
                        NavPacketsReceived++;
                        NavStateT = ExtractByte(ref UAVXPacket, 2);
                        switch (NavStateT)
                        {
                            case 0: NavState.Text = "Holding";
                                NavState.BackColor = System.Drawing.Color.LightSteelBlue;
                                break;
                            case 1: NavState.Text = "Returning";
                                NavState.BackColor = System.Drawing.Color.Lime;
                                break;
                            case 2: NavState.Text = "@Home";
                                NavState.BackColor = System.Drawing.Color.Green;
                                break;
                            case 3: NavState.Text = "Descending";
                                NavState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 4: NavState.Text = "Touchdown!";
                                NavState.BackColor = System.Drawing.Color.RosyBrown;
                                break;
                            case 5: NavState.Text = "Navigating";
                                NavState.BackColor = System.Drawing.Color.Silver;
                                break;
                            case 6: NavState.Text = "Loitering";
                                NavState.BackColor = System.Drawing.Color.Gold;
                                break;
                            default: NavState.Text = "Unknown"; break;
                        } // switch

                        FailStateT = ExtractByte(ref UAVXPacket, 3);
                        switch (FailStateT)
                        {
                            case 0: FailState.Text = "Monitor Rx";
                                FailState.BackColor = System.Drawing.Color.Green;
                                break;
                            case 1: FailState.Text = "Aborting";
                                FailState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 2: FailState.Text = "Terminating";
                                FailState.BackColor = System.Drawing.Color.Orange;
                                break;
                            case 3: FailState.Text = "Terminated";
                                FailState.BackColor = System.Drawing.Color.Red;
                                break;
                            default: FailState.Text = "Unknown"; break;
                        } // switch
                            
                        GPSNoOfSatsT = ExtractByte(ref UAVXPacket, 4);
                        GPSFixT = ExtractByte(ref UAVXPacket, 5);
                        CurrWPT = ExtractByte(ref UAVXPacket, 6);
                        BaroROCT = ExtractShort(ref UAVXPacket, 7);
                        RelBaroAltitudeT = ExtractInt24(ref UAVXPacket, 9);
                        RangefinderROCT = ExtractShort(ref UAVXPacket, 12);
                        RangefinderAltitudeT = ExtractShort(ref UAVXPacket, 14);
                        GPSHDiluteT = ExtractShort(ref UAVXPacket, 16);
                        HeadingT = ExtractShort(ref UAVXPacket, 18);
                        DesiredCourseT = ExtractShort(ref UAVXPacket, 20);
                        GPSVelT = ExtractShort(ref UAVXPacket, 22);
                        GPSROCT = ExtractShort(ref UAVXPacket, 24);                 
                        GPSRelAltitudeT = ExtractInt24(ref UAVXPacket, 26); 
                        GPSLatitudeT = ExtractInt(ref UAVXPacket, 29);
                        GPSLongitudeT = ExtractInt(ref UAVXPacket, 33);
                        DesiredAltitudeT = ExtractInt24(ref UAVXPacket, 37);
                        DesiredLatitudeT = ExtractInt(ref UAVXPacket, 40);
                        DesiredLongitudeT = ExtractInt(ref UAVXPacket, 44);
                        NavStateTimeoutT = ExtractInt24(ref UAVXPacket, 48);

                        AmbientTempT = ExtractShort(ref UAVXPacket, 51);
                        GPSMissionTimeT = ExtractInt(ref UAVXPacket, 53);

                        if ( FirstGPSCoordinates && ((Flags[0] & 0x80) != 0))
                        {
                            FirstGPSCoordinates = false;
                            OriginLatitude = GPSLatitudeT;
                            OriginLongitude = GPSLongitudeT;
                        }

                        GPSNoOfSats.Text = string.Format("{0:n0}", GPSNoOfSatsT);
                        if ( GPSNoOfSatsT < 6 )
                            GPSNoOfSats.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSNoOfSats.BackColor = GPSStatBox.BackColor;
                        
                        GPSFix.Text = string.Format("{0:n0}", GPSFixT);
                        if ( GPSFixT < 2 )
                            GPSFix.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSFix.BackColor = GPSStatBox.BackColor;

                        GPSHDilute.Text = string.Format("{0:n2}", (float)GPSHDiluteT * 0.01);
                        if (GPSHDiluteT > 130)
                            GPSHDilute.BackColor = System.Drawing.Color.Orange;
                        else
                            GPSHDilute.BackColor = GPSStatBox.BackColor;

                        CurrWP.Text = string.Format("{0:n0}", CurrWPT );
                        //pad1.Text = string.Format("{0:n0}", ExtractByte(ref UAVXPacket, 7));

                        BaroROC.Text = string.Format("{0:n1}", (float)BaroROCT * 0.1);
                        RelBaroAltitude.Text = string.Format("{0:n1}", (float)RelBaroAltitudeT * 0.1);

                        if (RelBaroAltitudeT > ( MaximumAltitudeLimit*10) )
                            RelBaroAltitude.BackColor = System.Drawing.Color.Orange;
                        else
                            RelBaroAltitude.BackColor = AltitudeGroupBox.BackColor;

                        if ( BaroROCT < -100 ) // 1M/S
                            BaroROC.BackColor = System.Drawing.Color.Orange;
                        else
                            BaroROC.BackColor = AltitudeGroupBox.BackColor;

                        RangefinderROC.Text = string.Format("{0:n2}", (float)RangefinderROCT * 0.01);
                        RangefinderAltitude.Text = string.Format("{0:n2}", (float)RangefinderAltitudeT * 0.01 );

                        Heading.Text = string.Format("{0:n0}", ((int)HeadingT * 180) / 3142);
                        
                        if ((Flags[2] & 0x80) != 0)
                        {
                            CurrAlt = RangefinderAltitudeT / 10;
                            AltitudeSource.Text = "Rangefinder";
                        }
                        else
                            if (((Flags[2] & 0x08) != 0) && ((Flags[0] & 0x80) != 0))
                            {
                                CurrAlt = GPSRelAltitudeT;
                                AltitudeSource.Text = "GPS";
                            }
                            else
                            {
                                CurrAlt = RelBaroAltitudeT;
                                AltitudeSource.Text = "Barometer";
                            }

                        CurrentAltitude.Text = string.Format("{0:n0}", (float)CurrAlt * 0.1);
                        if ((CurrAlt * 0.1) > MaximumAltitudeLimit)
                            CurrentAltitude.BackColor = System.Drawing.Color.Orange;
                        else
                            CurrentAltitude.BackColor = NavGroupBox.BackColor;

                        AltError = CurrAlt - DesiredAltitudeT;
                        AltitudeError.Text = string.Format("{0:n1}", (float)AltError * 0.1);

                        BaroGPSError.Text = string.Format("{0:n1}", (((double)RelBaroAltitudeT - (double)GPSRelAltitudeT))/ (double)RelBaroAltitudeT * 100.0);
                        if ((Flags[2] & 0x80) != 0)
                            BaroRFError.Text = string.Format("{0:n1}", (((double)RelBaroAltitudeT -(double)RangefinderAltitudeT) * 0.1) / (double)RelBaroAltitudeT * 100.0);
                        else
                            BaroRFError.Text = " ";

                        GPSVel.Text = string.Format("{0:n1}", (double)GPSVelT * 0.1); // dM/Sec
                        GPSROC.Text = string.Format("{0:n1}", (float)GPSROCT * 0.1);
                        GPSRelAltitude.Text = string.Format("{0:n1}", (double)GPSRelAltitudeT * 0.1);
                        GPSLongitude.Text = string.Format("{0:n6}", (double)GPSLongitudeT / 6000000.0);
                        GPSLatitude.Text = string.Format("{0:n6}", (double)GPSLatitudeT / 6000000.0);

                        if ((Flags[0] & 0x40) != 0) // GPSValid
                        {
                            GPSVel.BackColor = NavGroupBox.BackColor;
                            GPSROC.BackColor = NavGroupBox.BackColor;
                            GPSRelAltitude.BackColor = NavGroupBox.BackColor;
                            GPSLongitude.BackColor = NavGroupBox.BackColor;
                            GPSLatitude.BackColor = NavGroupBox.BackColor;
                            WayHeading.BackColor = NavGroupBox.BackColor;
                            DistanceToDesired.BackColor = NavGroupBox.BackColor;

                            WayHeading.Text = string.Format("{0:n0}", (DesiredCourseT * 180) / 3142);

                            LongitudeCorrection = Math.Cos(Math.PI/180.0 * (DesiredLatitudeT + GPSLatitudeT) / 12000000.0 ); 
                        
                            NorthDiff = (double)(DesiredLatitudeT - GPSLatitudeT); // scale up to decimetres after conversion
                            EastDiff = (double)(DesiredLongitudeT - GPSLongitudeT) * LongitudeCorrection;

                            Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                            DistanceToDesired.Text = string.Format("{0:n1}", Distance);
                        }
                        else
                        {

                            GPSVel.BackColor = System.Drawing.Color.Orange;
                            GPSROC.BackColor = System.Drawing.Color.Orange;
                            GPSRelAltitude.BackColor = System.Drawing.Color.Orange;
                            GPSLongitude.BackColor = System.Drawing.Color.Orange;
                            GPSLatitude.BackColor = System.Drawing.Color.Orange;
                            WayHeading.BackColor = System.Drawing.Color.Orange;
                            DistanceToDesired.BackColor = System.Drawing.Color.Orange;

                            WayHeading.Text = "?";
                            DistanceToDesired.Text = "?";       
                        }

                        if ( !FirstGPSCoordinates ) 
                        {
                            WhereBearing.BackColor = NavGroupBox.BackColor;
                            WhereDistance.BackColor = NavGroupBox.BackColor;

                            NorthDiff = GPSLatitudeT - OriginLatitude;

                            LongitudeCorrection = Math.Cos(Math.PI / 180.0 * (GPSLatitudeT + OriginLatitude) / 12000000.0);

                            EastDiff = (GPSLongitudeT - OriginLongitude) * LongitudeCorrection;
                            WhereDirection = Math.Atan2(EastDiff, NorthDiff) * 180.0 / Math.PI;
                            while (WhereDirection < 0)
                                WhereDirection += 360.0;
                            WhereBearing.Text = string.Format("{0:n0}", WhereDirection);
                            Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                            WhereDistance.Text = string.Format("{0:n0}", Distance);
                            if ( Distance > MaximumRangeLimit )
                                WhereDistance.BackColor = System.Drawing.Color.Orange;
                            else
                                WhereDistance.BackColor = NavGroupBox.BackColor;
                        }
                        else
                        {
                            WhereBearing.BackColor = System.Drawing.Color.Orange;
                            WhereDistance.BackColor = System.Drawing.Color.Orange;

                            WhereBearing.Text = "?";
                            WhereDistance.Text = "?";
                        }

                        if (NavStateTimeoutT >= 0)
                            NavStateTimeout.Text = string.Format("{0:n0}", NavStateTimeoutT / 1000);
                        else
                            NavStateTimeout.Text = " ";

                        AmbientTemp.Text = string.Format("{0:n1}", AmbientTempT * 0.1);
                        if (AmbientTempT < 0.0)
                            AmbientTemp.BackColor = System.Drawing.Color.LightSteelBlue;
                        else
                            AmbientTemp.BackColor = EnvGroupBox.BackColor;

                        headingIndicatorInstrumentControl1.SetHeadingIndicatorParameters(
                            ((HeadingT * 180) / 3142));
 
                        break;
                    default: break;
                    } // switch
                  WriteTextLogFile();
                }
            }
        }

        void WriteTextLogFile()
        {
            short i;

            SaveTextLogFileStreamWriter.Write("Flight,");
            for (i = 0; i < NoOfFlagBytes; i++)
                SaveTextLogFileStreamWriter.Write(Flags[i] + ",");
            SaveTextLogFileStreamWriter.Write(StateT + "," +
            BatteryVoltsT + "," +
            BatteryCurrentT + "," +
            BatteryChargeT + "," +
            RCGlitchesT + "," +
            DesiredThrottleT + "," +
            DesiredRollT + "," +
            DesiredPitchT + "," +
            DesiredYawT + "," +
            RollRateT + "," +
            PitchRateT + "," +
            YawRateT + "," +
            RollSumT + "," +
            PitchSumT + "," +
            YawSumT + "," +
            LRAccT + "," +
            FBAccT + "," +
            DUAccT + "," +
            LRCompT + "," +
            FBCompT + "," +
            DUCompT + "," +
            AltCompT + ",");

            for (i = 0; i < NoOfOutputs; i++)
                SaveTextLogFileStreamWriter.Write(OutputT[i] + ",");

            SaveTextLogFileStreamWriter.Write(MissionTimeMilliSecT + ",");

            SaveTextLogFileStreamWriter.Write("Nav," +
            NavStateT + "," +
            FailStateT + "," +
            GPSNoOfSatsT + "," +
            GPSFixT + "," +
            CurrWPT + "," +
            BaroROCT + "," +
            RelBaroAltitudeT + "," +
            RangefinderROCT + "," +
            RangefinderAltitudeT + "," +
            GPSHDiluteT + "," +
            HeadingT + "," +
            DesiredCourseT + "," +
            GPSVelT + "," +
            GPSROCT + "," +
            GPSRelAltitudeT + "," +
            GPSLatitudeT + "," +
            GPSLongitudeT + "," +
            DesiredAltitudeT + "," +
            DesiredLatitudeT + "," +
            DesiredLongitudeT + "," +
            NavStateTimeoutT + "," +
            AmbientTempT + "," +
            GPSMissionTimeT + ",");

            SaveTextLogFileStreamWriter.WriteLine("Stats," +
            I2CFailsT + "," +
            GPSFailsT + "," + 
            AccFailsT + "," + 
            GyroFailsT + "," + 
            CompassFailsT + "," + 
            BaroFailsT + "," + 
            I2CESCFailsT + "," +
            RCFailSafesT + "," +

            GPSAltitudeT + "," +
            GPSMaxVelT + "," +
            GPSMinSatsT + "," +
            GPSMaxSatsT + "," +
            GPSMinHDiluteT + "," +
            GPSMaxHDiluteT + "," +
        
            BaroRelAltitudeT + "," +
            BaroMinROCT + "," +
            BaroMaxROCT + "," +

            MinTempT + "," +
            MaxTempT + "," +

            BadT + "," +
            Unused1T + "," +
            Unused2T);

        } 

        void UAVXCloseTelemetry()
        {
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();

                SaveLogFileBinaryWriter.Flush();
                SaveLogFileBinaryWriter.Close();
                SaveLogFileStream.Close();
                SaveTextLogFileStreamWriter.Flush();
                SaveTextLogFileStreamWriter.Close();
                SaveTextLogFileStream.Close();
            }
        }

        //----------------------------------------------------------------------- 

        public void Zero(ref byte [] a, short len)
        {
            for (int s = 0; s < len; s++)
                a[s] = 0;
        }

        public byte ExtractByte(ref byte [] a, short p)
        {
            return a[p];
        }

        public short ExtractSignedByte(ref byte[] a, short p)
        {
            short temp;

            temp = a[p];
            if ( temp > 127 )
                temp -= 256;

            return temp;
        }

        public short ExtractShort(ref byte[] a, short p)
        {        
            short temp;
            temp = (short)(a[p + 1] << 8);
            temp |= (short)a[p];
   
            return temp;
        }

        public int ExtractInt24(ref byte[] a, short p)
        {
            int temp;

            temp = ((int)a[p + 2] << 24);
            temp |= ((int)a[p + 1] << 16);
            temp |= (int)a[p] << 8;
            temp /= 256;
            return temp;
        }

        public int ExtractInt(ref byte[] a, short p)
        {
            int temp;

            temp = (int)(a[p + 3] << 24);
            temp |= ((int)a[p + 2] << 16);
            temp |= ((int)a[p + 1] << 8);
            temp |= (int)a[p];
            return temp;
        }

        public double ConvertGPSToM(double c)
        {	// converts to Metres
            // approximately 1.8553257183 cm per LSB at the Equator
            // conversion max is 21Km
            return (c * 0.018553257183);
        }
    
    }
}