using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Runtime.InteropServices;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;
using Utility.ModifyRegistry;
using System.IO;
using System.Text.RegularExpressions;
using System.Net;


using EARTHLib;

namespace UAVXNav
{ // Rewritten and extended for UAVX by by Greg Egan (C) 2010.
  // Based originally on ArduPilotConfigTool(C) 2009 by By Jordi Muñoz && HappyKillmore,
  // and used with the kind permission of Jordi Muñoz - thanks Jordi.
 
    public partial class Form1 : Form
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

        const byte RCMaximum = 238;
        const double OUTMaximumScale = 0.5; // 100/200 % for PWM at least

        const byte UnknownPacketTag = 0;
        const byte RestartPacketTag = 8;
        const byte UAVXFlightPacketTag = 13;
        const byte UAVXNavPacketTag = 14;
        const byte UAVXStatsPacketTag = 15;
        const byte UAVXControlPacketTag = 16;
        const byte UAVXParamsPacketTag = 17;
        const byte UAVXMinPacketTag = 18;
        const byte UAVXArmParamsPacketTag = 19;
        const byte UAVXStickPacketTag = 20;
        const byte UAVXCustomPacketTag = 21;

        const byte FrSkyPacketTag = 99;

        const byte WaitRxSentinel = 0;
        const byte WaitRxBody = 1;
        const byte WaitRxESC = 2;
        const byte WaitRxCheck = 3;
        const byte WaitRxCopy = 4;
        const byte WaitUPTag = 5;
        const byte WaitUPLength = 6;
        const byte WaitUPBody = 7;
        const byte WaitRxTag = 8;

        const byte MAXPARAMS = 64;

        const float MILLIRADDEG = (float)0.057295;
        const float AttitudeToDegrees = 78.0f;

        const int DefaultRangeLimit = 100;
        int MaximumRangeLimit = 250; // You carry total responsibility if you increase this value
        const int MaximumAltitudeLimit = 121; // You carry total responsibility if you increase this value 

        const double CurrentSensorMax = 50.0; // depends on current ADC used - needs calibration box?
        const double YawGyroRate = 400.0;
        const double RollPitchGyroRate = 400.0;


        /*
        UAVXFlightPacket
        
        Flags bit values
        Flags[0]
            AltHoldEnabled	// stick programmed	
            TurnToWP	    // stick programmed		
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
            UsingAccComp
            UsingRTHAutoDescend
            BaroAltitudeValid
            RangefinderAltitudeValid
            UsingRangefinderAlt
         
        Flags[3]
		    AllowNavAltitudeHold	// stick programmed
		    UsingPositionHoldLock
		    Ch5Active
		    Simulation
		    AcquireNewPosition 
		    MotorsArmed
			NavigationActive
		    ForceFailsafe

         Flags[4]
            Signal
		    RCFrameOK
		    ParametersValid
		    RCNewValues
		    NewCommands
		    AccelerationsValid
		    CompassValid
		    CompassMissRead

        Flags[5]
		    UsingPolarCoordinates
		    ReceivingGPS
		    PacketReceived
		    NavComputed
		    AltitudeValid		
		    UsingSerialPPM
		    UsingTxMode2
		    FailsafesEnabled
		    
         */

        bool SimulationB;

        // struct not used - just to document packet format
        // byte UAVXFlightPacketTag;   
        // byte Length;  
        const byte NoOfFlagBytes = 6;
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
        byte[] PWMT = new byte[6];      // 47
        int MissionTimeMilliSecT;       // 53

        /*
UAVXStatsPacket      
*/

        //should be an enum!!!
        const byte GPSAltitudeX = 0;
        const byte BaroRelAltitudeX = 1;
        const byte ESCI2CFailX = 2;
        const byte GPSMinSatsX = 3;
        const byte MinROCX = 4;
        const byte MaxROCX = 5;
        const byte GPSMaxVelX = 6;
        const byte AccFailsX = 7;
        const byte CompassFailsX = 8;
        const byte BaroFailsX = 9;
        const byte GPSInvalidX = 10;
        const byte GPSMaxSatsX = 11;
        const byte NavValidX = 12;
        const byte MinHDiluteX = 13;
        const byte MaxHDiluteX = 14;
        const byte RCGlitchX = 15;
        const byte GPSBaroScaleX = 16;
        const byte GyroFailsX = 17;
        const byte RCFailSafesX = 18;
        const byte I2CFailsX = 19;
        const byte MinTempX = 20;
        const byte MaxTempX = 21;
        const byte BadX = 22;
        const byte BadNumX = 23;

        const byte MaxStats = 32;

        short[] Stats = new short[MaxStats];

        // UAVXNavPacket
        //byte UAVXNavPacketTag;
        //byte Length;
        byte NavStateT;                 // 2
        byte FailStateT;                // 3
        byte GPSNoOfSatsT;              // 4
        byte GPSFixT;                   // 5

        byte CurrWPT;                   // 6

        int ROCT;                     // 7 
        int RelBaroAltitudeT;           // 9

        int CruiseThrottleT;          // 12
        int RangefinderAltitudeT;     // 14

        short GPSHDiluteT;              // 16
        short HeadingT;                 // 18
        short DesiredCourseT;           // 20

        short GPSVelT;                  // 22
        short GPSHeadingT;              // 24
        int GPSRelAltitudeT;            // 26
        int GPSLatitudeT;               // 29
        int GPSLongitudeT;              // 33
        int DesiredAltitudeT;           // 37
        int DesiredLatitudeT;           // 40
        int DesiredLongitudeT;          // 44
        int NavStateTimeoutT;           // 48
        short AmbientTempT;             // 51
        int GPSMissionTimeT;            // 53

        bool WPInvalid;
        bool UAVXArm;
        short AirframeT;

        int PrevGPSLat=0, PrevGPSLon=0;
        int AltError, CurrAlt;
        double EastDiff, NorthDiff, LongitudeCorrection, WhereDirection;
        double Distance;
        int RangeLimit = DefaultRangeLimit;

        byte[] FrSkyPacket = new byte[16];

        const short RxQueueLength = 8192;
        const short RxQueueMask = RxQueueLength - 1;
        byte[] RxQueue = new byte[RxQueueLength];
        short RxTail = 0;
        short RxHead = 0;

        byte RxPacketTag, RxPacketByteCount, RxPacketLength, PacketLength, PacketRxState;
        byte ReceivedPacketTag;
        bool PacketReceived = false;

        bool CheckSumError;
        short RxLengthErrors = 0, RxCheckSumErrors = 0, RxIllegalErrors = 0, RxFrSkyErrors = 0;

        byte RxCheckSum;

        long FrSkyPacketsReceived = 0;
        long TopMotor;
        double TotalOutput;

        short ParamSet, ParamNo;

        short FrSkyRxPacketByteCount;
        short FrSkyRxPacketTag, FrSkyReceivedPacketTag, FrSkyPacketRxState;
        bool FrSkyPacketReceived = false;

        System.IO.FileStream SaveLogFileStream;
        System.IO.BinaryWriter SaveLogFileBinaryWriter;

        System.IO.FileStream OpenLogFileStream;
        System.IO.BinaryReader OpenLogFileStreamReader;

        const int DefAltHold = 20; 
        const byte DefProximityRadius = 5;
        const byte DefProximityAlt = 2;
        const int WPEntrySize = 11;
        const short MaxWayPoints = 21;// more than enough if you only have fuel for 15 minutes!

        bool DoingStartup;
        double nAltHold = DefAltHold;
        double nProximityRadius = DefProximityRadius;
        double nProximityAlt = DefProximityAlt;
        double nHomeAlt;
        string sLastLatLon;
        
        string sLastLoaded;
        bool InFlight = false;
        bool InOnlineMode = true;
        bool UseCompatibleMode = true;
        bool UseOverTerrainMode = false;
        ModifyRegistry myRegistry = new ModifyRegistry();
        string separatorFormat = System.Globalization.CultureInfo.CurrentCulture.NumberFormat.NumberDecimalSeparator;

        //string separatorFormat = new System.Globalization.NumberFormatInfo().NumberDecimalSeparator;
        //separatorFormat = ",";

        public event EventHandler UAVXWriteOK;
        public event EventHandler UAVXWriteFail;
        public event EventHandler UAVXReadOK;
        public event EventHandler UAVXReadFail;

        SerialPort serialPort2 = new SerialPort();
        byte[] Nav = new byte[256];
        byte[] StatsBytes = new byte[256];
        byte[] ReadBuff = new byte[256];
        byte[] UAVXPacket = new byte[256];
        private const byte start_byte = 24;

        public string[] update_available_ports()
        {
            return SerialPort.GetPortNames();
        }

        public void SetSerialPort(string portSelected, int speed, ref string errorMessage)
        {
            try
            {
                errorMessage = "";
                serialPort2.PortName = portSelected;
                serialPort2.ReadBufferSize = 8192;
                serialPort2.BaudRate = speed;
                serialPort2.Open();
                serialPort2.Close();
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                errorMessage = Convert.ToString(er.Message);
            }
        }

        private void COMSelectComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            UAVXNav.Properties.Settings.Default.COMPort = COMSelectComboBox.Text;
            UAVXNav.Properties.Settings.Default.Save();
        }

        private void COMDisarmedBaudRateSelectComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            UAVXNav.Properties.Settings.Default.COMDisarmedBaudRate = Convert.ToInt32(COMDisarmedBaudRateComboBox.Text);
            UAVXNav.Properties.Settings.Default.Save();
        }

        private void COMArmedBaudRateSelectComboBox_SelectedIndexChanged(object sender, EventArgs e)
        {
            UAVXNav.Properties.Settings.Default.COMArmedBaudRate = Convert.ToInt32(COMArmedBaudRateComboBox.Text);
            UAVXNav.Properties.Settings.Default.Save();
        }

        private void OpenLogFileToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OpenLogFileDialog.Filter = "UAVX Log File (*.log)|*.log";
            OpenLogFileDialog.InitialDirectory = UAVXNav.Properties.Settings.Default.LogFileFolder;
            if (OpenLogFileDialog.ShowDialog() == DialogResult.OK)
            {
                UAVXNav.Properties.Settings.Default.LogFileFolder = OpenLogFileDialog.InitialDirectory;
                OpenLogFileStream = new System.IO.FileStream(OpenLogFileDialog.FileName, System.IO.FileMode.Open);
                OpenLogFileStreamReader = new System.IO.BinaryReader(OpenLogFileStream);
            }
        }

        private void CreateSaveLogFile()
        {
            string FileName;

            FileName = "C:\\Program Files\\UAVXLogs\\UAVXNav_" +
                DateTime.Now.Year +
                DateTime.Now.Month +
                DateTime.Now.Day + "_" +
                DateTime.Now.Hour +
                DateTime.Now.Minute + ".log";
                SaveLogFileStream = new System.IO.FileStream(FileName, System.IO.FileMode.Create);
                SaveLogFileBinaryWriter = new System.IO.BinaryWriter(SaveLogFileStream);
        }  

        public Form1()
        {
            DoingStartup = true;
            //options_form.button_change += new EventHandler(options_form_button_change);
            UAVXWriteOK +=new EventHandler(UAVX_WriteOK);
            UAVXReadOK +=new EventHandler(UAVX_ReadOK);
            UAVXWriteFail += new EventHandler(UAVX_WriteFail);
            UAVXReadFail += new EventHandler(UAVX_ReadFail);

            InitializeComponent();

           // headingIndicatorInstrumentControl1.SetHeadingIndicatorParameters(45);

            
            InOnlineMode = true;
            if (checkOnlineMode())
            {
                webBrowser1.Navigate(new Uri(System.AppDomain.CurrentDomain.BaseDirectory + "Maps.html"));
                {
                    do
                    {
                        Application.DoEvents();
                    } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);
                }
            }
            if (!checkGCompatibleMode())
                webBrowser1.Navigate("http://maps.google.com/support/bin/topic.py?topic=10781");

            myRegistry.SubKey = "SOFTWARE\\UAVXNav\\Settings";

            // COMSelectComboBox.Items.Add("Select COM Port");
            string[] AvailableCOMPorts = ComPorts.readPorts();
            foreach (string AvailableCOMPort in AvailableCOMPorts)
                COMSelectComboBox.Items.Add(AvailableCOMPort);
            COMSelectComboBox.Text = UAVXNav.Properties.Settings.Default.COMPort;
            COMDisarmedBaudRateComboBox.Text = Convert.ToString(UAVXNav.Properties.Settings.Default.COMDisarmedBaudRate);
            COMArmedBaudRateComboBox.Text = Convert.ToString(UAVXNav.Properties.Settings.Default.COMArmedBaudRate);

            LocationAddress.Text = myRegistry.Read("SearchAddress","");
            if (myRegistry.Read("SearchType","true") == "false")
                optWP.Checked = true;
            else
                optHome.Checked = true;

            nAltHold = DefAltHold;
            nProximityRadius = DefProximityRadius;
            nProximityAlt = DefProximityAlt;

            LookupAltCheckBox.Checked = false; // Convert.ToBoolean(myRegistry.Read("OverTerrain", "False"));
            LaunchManuallyCheckBox.Checked = Convert.ToBoolean(myRegistry.Read("SetManual", "False"));

            Version vrs = new Version(Application.ProductVersion);
            this.Text = this.Text + " v" + vrs.Major + "." + vrs.Minor + "." + vrs.Build;

            RTHAltitude.Text = string.Format("{0:n0}", nAltHold);
            ProximityRadius.Text = string.Format("{0:n0}",nProximityRadius);
            ProximityAlt.Text = string.Format("{0:n0}", nProximityAlt);

            RangeLimit = Convert.ToInt16(RangeLimitSetting.Text);

            if (InOnlineMode && UseCompatibleMode)
            {
                double nLat;
                double nLon;
                // Woomera
                double.TryParse(myRegistry.Read("homeLat", "-31.199144"), 
                    System.Globalization.NumberStyles.Float, 
                    System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
                double.TryParse(myRegistry.Read("homeLng", "136.824492"), 
                    System.Globalization.NumberStyles.Float, 
                    System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLon);

                LaunchLat.Text = Convert.ToString(nLat);
                UAVXOriginLatitude = (int)(nLat * 6000000);
                LaunchLon.Text = Convert.ToString(nLon);
                UAVXOriginLongitude = (int)(nLon * 6000000);
                setGoogleMapHome(LaunchLat.Text, LaunchLon.Text, true);

                webBrowser1.Document.GetElementById("metersfeet").SetAttribute("value", "1"); // Meters
                webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", 
                    Regex.Replace(LaunchLat.Text, ",", "."));
                webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", 
                    Regex.Replace(LaunchLon.Text, ",", "."));
                webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("lat").SetAttribute("value", 
                    Regex.Replace(LaunchLat.Text, ",", "."));
                webBrowser1.Document.GetElementById("lng").SetAttribute("value", 
                    Regex.Replace(LaunchLon.Text, ",", "."));
                webBrowser1.Document.GetElementById("index").SetAttribute("value", "Home");
            }

            //changeOnlineMode(InOnlineMode);

            FlyingButton.Text = "Landed";
            FlyingButton.BackColor = System.Drawing.Color.Red;

            DoingStartup = false;
            timer1.Enabled = true;
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

        private void AddDataToWPGrid()
        {
            M.Rows.Add(1);
            M[0,M.Rows.Count-1].Value = Convert.ToString(M.Rows.Count);

            //wp_number.Text = Convert.ToString(M.Rows.Count); 
            UAVXNoOfWP = Convert.ToByte(M.Rows.Count);  
        }


        private void launchManually_CheckedChanged(object sender, EventArgs e)
        {
                myRegistry.Write("SetManual", Convert.ToString(LaunchManuallyCheckBox.Checked));
                LaunchLat.Enabled = LaunchManuallyCheckBox.Checked;
                LaunchLon.Enabled = LaunchManuallyCheckBox.Checked;
                if (LaunchManuallyCheckBox.Checked)
                    UAVXOptions |= 0x08; 
                else
                    UAVXOptions &= 0xF7;
        }
 
        private void downloadToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }

        private void uploadToolStripMenuItem_Click(object sender, EventArgs e)
        {
        }
  
        private void optionsToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //options_form.Show();
        }

        private void options_form_button_change(object sender, EventArgs e)
        {
           // not used for UAVXNav
        }

        private void progressBar1_Click(object sender, EventArgs e)
        {            
        }

        void UAVXRead()
        {
            string sError="";

            progressBar1.Value = 0;
            if (InOnlineMode && UseCompatibleMode)
                cmdClear_Click(null,null);

            statusLabel.Text = "Reading Data...";
            progressBar1.Maximum = 120;
            progressBar1.Value = 10;
            this.Refresh();

            UAVXCloseTelemetry();
            FlyingButton.Text = "Landed";
            FlyingButton.BackColor = System.Drawing.Color.Red;

            SetSerialPort(UAVXNav.Properties.Settings.Default.COMPort, UAVXNav.Properties.Settings.Default.COMDisarmedBaudRate, ref sError);

            if (sError != "")
            {
                statusLabel.Text = sError;
                return;
            }
            else
                UAVXReadNav();

            SetSerialPort(UAVXNav.Properties.Settings.Default.COMPort, UAVXNav.Properties.Settings.Default.COMDisarmedBaudRate, ref sError);

            if (sError != "")
            {
                statusLabel.Text = sError;
                return;
            }
            else
                UAVXReadStats();
         
            if (ExtractShort(ref StatsBytes, 11) > 1)
                GPSAltitudeS.Text = string.Format("{0:n0}", ExtractShort(ref StatsBytes, 0) / 10);
            else
                GPSAltitudeS.Text = "?";

            BaroRelAltitudeS.Text = string.Format("{0:n0}", ExtractShort(ref StatsBytes, 2) / 10); 
            RelBaroPressureS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 6));

            GPSMinSatS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 6));
            if ( ExtractShort(ref StatsBytes, 6) < 6 )
                GPSMinSatS.BackColor = System.Drawing.Color.Red;
            else
                GPSMinSatS.BackColor = GPSStatsGroupBox.BackColor;

            BaroMinROCS.Text = string.Format("{0:n1}", ExtractShort(ref StatsBytes, 8) / 100.0);
            BaroMaxROCS.Text = string.Format("{0:n1}", ExtractShort(ref StatsBytes, 10) / 100.0);

            GPSVelS.Text = string.Format("{0:n1}", ExtractShort(ref StatsBytes, 12) / 100.0);

            AccFailS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 14));
            if (ExtractShort(ref StatsBytes, 14) > 0)
                AccFailS.BackColor = System.Drawing.Color.Red;
            else
                AccFailS.BackColor = GPSStatsGroupBox.BackColor;

            CompassFailS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 16));
            if (ExtractShort(ref StatsBytes, 16) > 0)
                CompassFailS.BackColor = System.Drawing.Color.Red;
            else
                CompassFailS.BackColor = GPSStatsGroupBox.BackColor;

            BaroFailS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 18));
            if (ExtractShort(ref StatsBytes, 18) > 0)
                BaroFailS.BackColor = System.Drawing.Color.Red;
            else
                BaroFailS.BackColor = GPSStatsGroupBox.BackColor;

            GPSFailS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 20));
            if (ExtractShort(ref StatsBytes, 20) > 20)
                GPSFailS.BackColor = System.Drawing.Color.Red;
            else
                GPSFailS.BackColor = GPSStatsGroupBox.BackColor;

            GPSMaxSatS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 22));

            GPSMinHDiluteS.Text = string.Format("{0:n1}", ExtractShort(ref StatsBytes, 26) / 100.0);

            GPSMaxHDiluteS.Text = string.Format("{0:n1}", ExtractShort(ref StatsBytes, 28) / 100.0);
            if (ExtractShort(ref StatsBytes, 28) > 150)
                GPSMaxHDiluteS.BackColor = System.Drawing.Color.Red;
            else
                GPSMaxHDiluteS.BackColor = GPSStatsGroupBox.BackColor;

            RCGlitchesS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 30));
            if (ExtractShort(ref StatsBytes, 30) > 20)
                RCGlitchesS.BackColor = System.Drawing.Color.Red;
            else
                RCGlitchesS.BackColor = GPSStatsGroupBox.BackColor;

            GPSBaroScaleS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 32));

            GyroFailS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 34));
            if (ExtractShort(ref StatsBytes, 34) > 0)
                GyroFailS.BackColor = System.Drawing.Color.Orange;
            else
                GyroFailS.BackColor = GPSStatsGroupBox.BackColor;

            RCFailSafeS.Text = Convert.ToString(ExtractShort(ref StatsBytes, 36));

            ProximityRadius.Text = string.Format("{0:n0}", UAVXProximityRadius);
            ProximityAlt.Text = string.Format("{0:n0}", UAVXProximityAlt);

            double nLat;
            double nLon;

            progressBar1.Value = progressBar1.Value + 10;

            double.TryParse(Convert.ToString((double)UAVXOriginLatitude / 6000000), 
                System.Globalization.NumberStyles.Float, 
                System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
            double.TryParse(Convert.ToString((double)UAVXOriginLongitude / 6000000), 
                System.Globalization.NumberStyles.Float, 
                System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLon);

            if (nLat != 0)
                LaunchLat.Text = nLat.ToString();
  
            if (nLon != 0)
                LaunchLon.Text = nLon.ToString();

            launchLat_Leave(null,null);
            launchLon_Leave(null, null);

            OriginAltitude.Text = Convert.ToString(UAVXOriginAltitude);
            RTHAltitude.Text = string.Format("{0:n0}", UAVXRTHAltitude);

            M.Rows.Clear();
            for (int x = 0; x < UAVXNoOfWP; x++)
            {
                progressBar1.Value = progressBar1.Value + 100 / UAVXNoOfWP;

                nLat = (double)UAVXGetLat(x) / 6000000;
                nLon = (double)UAVXGetLon(x) / 6000000;
                ClickMapLabel.Visible = false;
                M.Rows.Add();
                M[0,x].Value = Convert.ToString(x+1);
                M[1,x].Value = Convert.ToString((double)UAVXGetLat(x) / 6000000);
                M[2,x].Value = Convert.ToString((double)UAVXGetLon(x) / 6000000);
                M[5,x].Value = Convert.ToString(GetAltitudeData(nLat,nLon));
                M[6,x].Value = string.Format("{0:n0}", UAVXGetLoiter(x));
                /*
                if (LookupAltCheckBox.Checked)
                {
                    M[3,x].Value = string.Format("{0:n0}", UAVXGetAlt(x) );
                    M[4,x].Value = string.Format("{0:n0}", 
                        (nHomeAlt-Convert.ToDouble(M[5,x].Value) + Convert.ToDouble(M[3,x].Value)));
                }
                else
                */
                {
                    M[4,x].Value = string.Format("{0:n0}", UAVXGetAlt(x));
                    M[3,x].Value = string.Format("{0:n0}", Convert.ToDouble(M[4,x].Value));
                }

                setWPValues(M.Rows.Count,M[1,x].Value.ToString(),M[2,x].Value.ToString(), true);
            }
            if (InOnlineMode && UseCompatibleMode)
                webBrowser1.Document.GetElementById("refreshButton").InvokeMember("click");

            progressBar1.Value = progressBar1.Maximum;

        }

        private void setWPValues(object index, string nlat, string nLon, Boolean addWPClick)
        {
            if (InOnlineMode && UseCompatibleMode)
            {
                webBrowser1.Document.GetElementById("wpIndex").SetAttribute("value", index.ToString());
                webBrowser1.Document.GetElementById("wpLat").SetAttribute("value", 
                    Regex.Replace(nlat, ",", "."));
                webBrowser1.Document.GetElementById("wpLng").SetAttribute("value", 
                    Regex.Replace(nLon, ",", "."));
                if (addWPClick)
                {
                    webBrowser1.Document.GetElementById("addWaypointLatLngButton").InvokeMember("click");
                }
                //Application.DoEvents();
            }
        }

        private void UAVX_ReadOK(object sender, EventArgs e)
        {
            statusLabel.Text = "Read Successful!";
            //update_maps();
            this.Refresh();

            //nAltHold = Convert.ToDouble(latLonAlt[2]);
            nHomeAlt = GetAltitudeData(Convert.ToDouble(LaunchLat.Text), Convert.ToDouble(LaunchLon.Text));
            HomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
        }

        private void UAVX_ReadFail(object sender, EventArgs e)
        {
            statusLabel.Text = "Read failed - are you connected and is the arming switch OFF?";
            this.Refresh();
        }
        
        void UAVXWrite()
        {
            string sError = "";
            int UAVXWPLat, UAVXWPLon;
            short UAVXWPAlt;
            bool MissionValid;

            progressBar1.Value = 0;
            statusLabel.Text = "Writing Data...";
            this.Refresh();

            try
            {
                myRegistry.Write("LastSaved", sLastLoaded);

                //COnverting general vairables
                UAVXProximityRadius = Convert.ToByte(Convert.ToDouble(ProximityRadius.Text));
                UAVXProximityAlt = Convert.ToByte(Convert.ToDouble(ProximityAlt.Text));
                UAVXRTHAltitude = Convert.ToInt16(Convert.ToDouble(RTHAltitude.Text));

                //Verifying if we going to setup home position manually
                if (LaunchManuallyCheckBox.Checked)
                {
                    UAVXOriginLatitude = (int)(Convert.ToDouble(LaunchLat.Text) * (double)6000000);
                    UAVXOriginLongitude = (int)(Convert.ToDouble(LaunchLon.Text) * (double)6000000);
                    UAVXOptions |= 0x08;
                }
                else
                    UAVXOptions &= 0xF7;

                MissionValid = true;
                UAVXNoOfWP = (byte)M.Rows.Count;

                progressBar1.Minimum = 0;
                progressBar1.Maximum = progressBar1.Value + Convert.ToInt32(UAVXNoOfWP)+20;
                for (int WPIndex = 0; WPIndex < UAVXNoOfWP; WPIndex++)
                {
                    progressBar1.Value = progressBar1.Value + 1;
                    UAVXWPLat = Convert.ToInt32((Convert.ToDouble(M[1, WPIndex].Value)) * (double)6000000);
                    UAVXWPLon = Convert.ToInt32((Convert.ToDouble(M[2, WPIndex].Value)) * (double)6000000);
                    UAVXWPAlt = Convert.ToInt16(Convert.ToInt16(M[3, WPIndex].Value));

                    MissionValid &= (DistanceBetween(UAVXOriginLatitude, UAVXOriginLongitude, UAVXWPLat, UAVXWPLon) < RangeLimit);
                    
                    MissionValid &= UAVXWPAlt < MaximumAltitudeLimit;

                    UAVXSetLat(WPIndex, UAVXWPLat);
                    UAVXSetLon(WPIndex, UAVXWPLon);
                    UAVXSetAlt(WPIndex, UAVXWPAlt);
                    UAVXSetLoiter(WPIndex, Convert.ToByte(Convert.ToByte(M[6, WPIndex].Value)));
                    progressBar1.Refresh();
                }

                if (MissionValid)
                {
                    UAVXCloseTelemetry();
                    FlyingButton.Text = "Landed";
                    FlyingButton.BackColor = System.Drawing.Color.Red;

                    SetSerialPort(UAVXNav.Properties.Settings.Default.COMPort, UAVXNav.Properties.Settings.Default.COMDisarmedBaudRate, ref sError);

                    progressBar1.Value = progressBar1.Value + 10;
                    if (sError != "")
                        statusLabel.Text = sError;
                    else
                    {
                        UAVXWriteNav(ref sError);
                        progressBar1.Value = progressBar1.Maximum;
                    }
                }
                else
                {
                    statusLabel.Text = "Invalid Mission - WPs beyond line of sight or above 120M";
                    progressBar1.Value = progressBar1.Maximum;
                }
            }
            catch (Exception er)
            {
                //MessageBox.Show(Convert.ToString(er.Message));
                statusLabel.Text = Convert.ToString(er.Message);
            }
        }

        private double GetAltitudeData(double latitude,double longitude)
        {
            try
            {
                if (!UseOverTerrainMode)
                    return 0;

                double nDoubleReturn;
                // Create a 'WebRequest' object with the specified url. 
                WebRequest myWebRequest = WebRequest.Create(
                    "http://gisdata.usgs.gov/XMLWebServices2/Elevation_Service.asmx/getElevation?X_Value=" + 
                    Regex.Replace(longitude.ToString(), ",", ".") + "&Y_Value=" + 
                    Regex.Replace(latitude.ToString(), ",", ".") + 
                    "&Elevation_Units=METERS&Source_Layer=-1&Elevation_Only=True");

                // Send the 'WebRequest' and wait for response. 
                WebResponse myWebResponse = myWebRequest.GetResponse();

                setOnlineMode(true);

                // Obtain a 'Stream' object associated with the response object. 
                Stream ReceiveStream = myWebResponse.GetResponseStream();

                Encoding encode = System.Text.Encoding.GetEncoding("utf-8");

                // Pipe the stream to a higher level stream reader with the required encoding format. 
                StreamReader readStream = new StreamReader(ReceiveStream, encode);

                string strResponse = readStream.ReadToEnd();

                //System.Diagnostics.Debug.WriteLine(strResponse);
                //Console.WriteLine( strResponse);

                readStream.Close();

                // Release the resources of response object. 
                myWebResponse.Close();

                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(0, strResponse.IndexOf("<"));
                //System.Diagnostics.Debug.WriteLine(strResponse);
                double.TryParse(strResponse, System.Globalization.NumberStyles.Float, 
                    System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nDoubleReturn);

                return Convert.ToDouble(string.Format("{0:n0}", nDoubleReturn));
            }
            catch
            {
                setOnlineMode(false);
                return 0;
            }
        }

        private void setOnlineMode(Boolean IsOnline)
        {
            if (IsOnline && UseOverTerrainMode)
            {
                M.Columns[1].Width = 60;
                M.Columns[2].Width = 65;
                M.Columns[3].Width = 50;
                M.Columns[3].ReadOnly = true;
                M.Columns[3].HeaderText = "Altitude (Home)";
                M.Columns[3].DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
                M.Columns[4].Visible = true;
                M.Columns[5].Visible = true;
                M.Columns[6].Visible = true;
                HomeAltLabel.Visible = true;
                HomeAlt.Visible = true;
            }
            else
            {
                M.Columns[1].Width = 100;
                M.Columns[2].Width = 100;
                M.Columns[3].Width = 75;
                M.Columns[3].Width = 75;
                M.Columns[3].HeaderText = "Altitude";
                M.Columns[3].DefaultCellStyle.Alignment = DataGridViewContentAlignment.MiddleCenter;
                M.Columns[3].ReadOnly = false;
                M.Columns[4].Visible = false;
                M.Columns[5].Visible = false;
                M.Columns[6].Visible = true;
                HomeAltLabel.Visible = false;
                HomeAlt.Visible = false;
            }
            if (!IsOnline)
                InOnlineMode = false;

            OnlineGroupBox.Visible = IsOnline;
            webBrowser1.Visible = IsOnline;

            OfflineGroupBox.Visible = !IsOnline;
            OfflineMap.Visible = !IsOnline;

            if (M.Rows.Count == 0)
            {
                ClickMapLabel.Visible = IsOnline;
                OfflineMapLabel.Visible = !IsOnline;
            }
            else
            {
                ClickMapLabel.Visible = false;
                OfflineMapLabel.Visible = !IsOnline;
            }
        }

        private Boolean checkOnlineMode()
        {
            try
            {
                WebRequest myWebRequest = WebRequest.Create("http://www.google.com");
                WebResponse myWebResponse = myWebRequest.GetResponse();
                setOnlineMode(true);
                return true;
            }
            catch
            {
                setOnlineMode(false);
                return false;
            }
        }

        private Boolean checkGCompatibleMode()
        {
            try
            {
                if (webBrowser1.Document.GetElementById("status").InnerHtml == "Not Compatible")
                {
                    UseCompatibleMode = false;
                    return false;
                }
                else
                {
                    UseCompatibleMode = true;
                    return true;
                }
            }
            catch
            {
                UseCompatibleMode = false;
                return false;
            }
        }

        private void UAVX_WriteOK(object sender, EventArgs e)
        {
            statusLabel.Text = "Write successful!";
            this.Refresh();
        }

        private void UAVX_WriteFail(object sender, EventArgs e)
        {
            statusLabel.Text = "Write failed - are you connected and is the arming switch OFF?";
            this.Refresh();
        }

        private void numericMaps_ValueChanged(object sender, EventArgs e)
        {
            if (!DoingStartup)
            {
                webBrowser1.Document.GetElementById("zoomLevel").SetAttribute("value", 
                    Convert.ToString(ZoomMaps.Value));
                webBrowser1.Document.GetElementById("setZoomButton").InvokeMember("click");
            }
        }

        private void linkLabel1_LinkClicked(object sender, LinkLabelLinkClickedEventArgs e)
        {
        }

        private void updateAltColumns()
        {
            for (int nCount = 0;nCount < M.Rows.Count;nCount++)
            {
                /*
                if (LookupAltCheckBox.Checked)
                {
                    M[5,nCount].Value = string.Format("{0:n0}", 
                        GetAltitudeData(Convert.ToDouble(M[1,nCount].Value),
                        Convert.ToDouble(M[2,nCount].Value)));
                    M[3,nCount].Value = string.Format("{0:n0}", 
                        (Convert.ToDouble(M[4,nCount].Value) + 
                        Convert.ToDouble(M[5,nCount].Value)-nHomeAlt));
                }
                else
                */
                    M[3,nCount].Value = string.Format("{0:n0}", Convert.ToDouble(M[4,nCount].Value));
            }
        }

        private void setGoogleMapHome(string latitude, string longitude, Boolean centerMap)
        {
            if (InOnlineMode && UseCompatibleMode)
            {
                webBrowser1.Document.GetElementById("homeLat").SetAttribute("value", 
                    Regex.Replace(latitude, ",", "."));
                webBrowser1.Document.GetElementById("homeLng").SetAttribute("value", 
                    Regex.Replace(longitude, ",", "."));
                webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");
                if (centerMap)
                    webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            int WPIndex;
            string sWPIndex;
            string WPLat;
            string WPLon;
            string sZoomLevel = "15";
            double nWPLat;
            double nWPLon;
            double nCurrentAlt;

            if ( InFlight )
                PollTelemetry();

            if (!InOnlineMode || !UseCompatibleMode)
                return;

            do
            {
                Application.DoEvents();
            } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);

            DoingStartup = true;
            try
            {
                sZoomLevel = webBrowser1.Document.GetElementById("zoomLevel").GetAttribute("value");
                sWPIndex = webBrowser1.Document.GetElementById("index").GetAttribute("value");
                WPLat = webBrowser1.Document.GetElementById("lat").GetAttribute("value");
                WPLon = webBrowser1.Document.GetElementById("lng").GetAttribute("value");

                WPIndex = -1;

                double.TryParse(WPLat, System.Globalization.NumberStyles.Float, 
                    System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nWPLat);
                double.TryParse(WPLon, System.Globalization.NumberStyles.Float, 
                    System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nWPLon);

                if (LocationAddress.Text.Trim() == "")
                    SearchButton.Enabled = false;
                else
                    SearchButton.Enabled = true;

                if (sLastLatLon != WPLat + "," + WPLon) // has WP marker been moved?
                {
                    sLastLatLon = WPLat + "," + WPLon;
                    if (sWPIndex == "Home")
                    {
                        LaunchLat.Text = string.Format("{0:n6}", nWPLat);
                        LaunchLon.Text = string.Format("{0:n6}", nWPLon);
                        UAVXOriginLatitude = (int)(nWPLat * 6000000);
                        UAVXOriginLongitude = (int)(nWPLon * 6000000);

                        nHomeAlt = GetAltitudeData(nWPLat, nWPLon);
                        HomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
                        //System.Diagnostics.Debug.WriteLine(nHomeAlt);

                        updateAltColumns();
                        DoingStartup = true;
                    }
                    else
                    {
                        WPIndex = Convert.ToInt32(sWPIndex);
                        if (WPIndex > MaxWayPoints)
                        {
                            MessageBox.Show("Too many WP selected.\nUAVX can currently handle up to " + 
                                MaxWayPoints.ToString() + " WP.", "Max WP",MessageBoxButtons.OK,
                                MessageBoxIcon.Exclamation);
                            webBrowser1.Document.GetElementById("dwIndex").SetAttribute(
                                "value",WPIndex.ToString());
                            webBrowser1.Document.GetElementById(
                                "deletewaypointButton").InvokeMember("click");
                            return;
                        }
                    }

                    double nDownloadedAlt;

                    if (WPIndex > M.Rows.Count)
                    {
                        nDownloadedAlt = GetAltitudeData(nWPLat, nWPLon);

                        M.Rows.Add(1);
                        M[0,M.Rows.Count-1].Value = Convert.ToString(M.Rows.Count);
                        statusLabel.Text = "";
                        M[1,M.Rows.Count-1].Value = string.Format("{0:n6}", nWPLat);
                        M[2,M.Rows.Count-1].Value = string.Format("{0:n6}", nWPLon);

                        /*
                        if (LookupAltCheckBox.Checked)
                            M[3,M.Rows.Count-1].Value = string.Format("{0:n0}", 
                                (nDownloadedAlt + nAltHold-nHomeAlt));
                        else
                        */
                            M[3,M.Rows.Count-1].Value = string.Format("{0:n0}", nAltHold);

                        M[4,M.Rows.Count-1].Value = string.Format("{0:n0}", nAltHold);
                        M[5,M.Rows.Count-1].Value = string.Format("{0:n0}", nDownloadedAlt);
                        //wp_number.Text = Convert.ToString(M.Rows.Count);
                        M.CurrentCell.Selected = false;
                        M.Rows[M.Rows.Count-1].Cells[0].Selected = true;
                        M.CurrentCell = M.SelectedCells[0];
                    }
                    else if (WPIndex != -1)
                    {
                        nDownloadedAlt = GetAltitudeData(nWPLat, nWPLon);
                        nCurrentAlt = Convert.ToDouble(M[4,M.Rows.Count-1].Value);

                        statusLabel.Text = "";

                        M[1,WPIndex-1].Value = string.Format("{0:n6}", nWPLat);
                        M[2,WPIndex-1].Value = string.Format("{0:n6}", nWPLon);

                        /*
                        if (LookupAltCheckBox.Checked)
                            M[3,WPIndex-1].Value = string.Format("{0:n0}", 
                                (nDownloadedAlt + nCurrentAlt-nHomeAlt));
                        else
                        */
                            M[3,WPIndex-1].Value = string.Format("{0:n0}", nCurrentAlt);

                        M[4,WPIndex-1].Value = string.Format("{0:n0}", nCurrentAlt);
                        M[5,WPIndex-1].Value = string.Format("{0:n0}", nDownloadedAlt);
                        M.CurrentCell.Selected = false;
                        M.Rows[WPIndex-1].Cells[0].Selected = true;
                        M.CurrentCell = M.SelectedCells[0];
                    }
                    UAVXNoOfWP = Convert.ToByte(M.Rows.Count);

                    M.GridColor = System.Drawing.Color.Black;
                    WPInvalid = false;
                    for (WPIndex = 0; WPIndex < UAVXNoOfWP; WPIndex++)
                    {
                        if (BeyondRangeLimit((int)(Convert.ToDouble(M[1, WPIndex].Value) * 6000000),
                            (int)(Convert.ToDouble(M[2, WPIndex].Value) * 6000000)))
                        {
                            statusLabel.Text = "WP is beyond reasonable line of sight";
                            M.GridColor = System.Drawing.Color.Red;
                            WPInvalid = true;
                        }
                    }

                }
                if (sZoomLevel != "")
                    ZoomMaps.Value = int.Parse(sZoomLevel);
            }
            catch (Exception)
            {
            }
            if (M.Rows.Count > 0)
            {
                OnlineRemoveWPButton.Enabled = true;
                OnlineClearAllButton.Enabled = true;
                CentreButton.Enabled = true;

                OfflineRemoveWPButton.Enabled = true;
                OfflineClearAllButton.Enabled = true;

                ClickMapLabel.Visible = false;
            }
            else
            {
                OnlineRemoveWPButton.Enabled = false;
                OnlineClearAllButton.Enabled = false;
                CentreButton.Enabled = false;

                OfflineRemoveWPButton.Enabled = false;
                OfflineClearAllButton.Enabled = false;

                if (InOnlineMode && UseCompatibleMode)
                    ClickMapLabel.Text = "Click Map to Add WP";
                else
                    ClickMapLabel.Text = "Click Add WP";

                ClickMapLabel.Visible = true;
            }
            DoingStartup = false;
        }

        private void optLocation_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void cmdSearch_Click(object sender, EventArgs e)
        {
            if (optHome.Checked)
            {
                webBrowser1.Document.GetElementById("address").SetAttribute("value", LocationAddress.Text);
                webBrowser1.Document.GetElementById("setHomeAddressButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
            }
            else
            {
                webBrowser1.Document.GetElementById("wpAddress").SetAttribute("value", LocationAddress.Text);
                webBrowser1.Document.GetElementById("addWaypointButton").InvokeMember("click");
                webBrowser1.Document.GetElementById("centerMapWaypointButton").InvokeMember("click");
            }
            myRegistry.Write("SearchAddress", LocationAddress.Text);
            myRegistry.Write("SearchType", Convert.ToString(optHome.Checked));
        }

        private void cmdClear_Click(object sender, EventArgs e)
        {
            webBrowser1.Document.GetElementById("ClearButton").InvokeMember("click");
            M.Rows.Clear();
            UAVXNoOfWP = 0;
        }

        private void cmdClearFlying_Click(object sender, EventArgs e)
        {
            if (InFlight)
                UAVXCloseTelemetry();
            else
                UAVXOpenTelemetry();

            if (InFlight)
            {
                FlyingButton.BackColor = System.Drawing.Color.Green;
                FlyingButton.Text = "Flying";          
            }
            else
            {
                FlyingButton.Text = "Landed";
                FlyingButton.BackColor = System.Drawing.Color.Red;
            }
        }
 
        private void txtAddress_TextChanged(object sender, EventArgs e)
        {
            //Registry.CreateSubKey("Software\\Remzibi OSD\\ArduPilot\\Settings");
        }

        private void M_CellEndEdit(object sender, DataGridViewCellEventArgs e)
        {
            setWPValues(M.CurrentRow.Cells[0].Value,M.CurrentRow.Cells[1].Value.ToString(), 
                M.CurrentRow.Cells[2].Value.ToString(), false);

            if (InOnlineMode && UseCompatibleMode)
                webBrowser1.Document.GetElementById("moveWaypointButton").InvokeMember("click");

            int nCurrentRow = Convert.ToInt32(M.CurrentRow.Cells[0].Value)-1;
            /*
            if (LookupAltCheckBox.Checked)
            {
                M[3, nCurrentRow].Value = string.Format("{0:n0}", 
                    Convert.ToDouble(M[5, nCurrentRow].Value)-nHomeAlt + 
                    Convert.ToDouble(M[4, nCurrentRow].Value));
                M[4, nCurrentRow].Value = string.Format("{0:n0}", 
                    Convert.ToDouble(M[4, nCurrentRow].Value));
            }
            else
            */
            {
                //System.Diagnostics.Debug.WriteLine(Convert.ToInt32(M.CurrentRow.Cells[0].Value)-1);
                M[3, nCurrentRow].Value = string.Format("{0:n0}", Convert.ToDouble(M[3, nCurrentRow].Value));
                M[4, nCurrentRow].Value = string.Format("{0:n0}", Convert.ToDouble(M[4, nCurrentRow].Value));
            }
            //M[4,Convert.ToInt32( M.CurrentRow.Cells[0].Value)-1].Value = string.Format("{0:n0}", 
            //Convert.ToDouble(M[3, Convert.ToInt32(M.CurrentRow.Cells[0].Value)-1].Value));
        }

        private void launchLat_TextChanged(object sender, EventArgs e)
        {
        }

        private void txtAddress_Enter(object sender, EventArgs e)
        {
            this.AcceptButton = SearchButton;
        }

        private void launchLon_TextChanged(object sender, EventArgs e)
        {
        }

        private void MapZoomLevel_Click(object sender, EventArgs e)
        {

        }

        private void M_Enter(object sender, EventArgs e)
        {
            timer1.Enabled = false;
        }

        private void M_Leave(object sender, EventArgs e)
        {
            timer1.Enabled = true;
        }

        private void RTHAltitude_TextChanged(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(RTHAltitude.Text, out outValue))
                nAltHold = Convert.ToDouble(outValue);
            else
                nAltHold = DefAltHold;
        }

        private void ProximityRadius_TextChanged(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(ProximityRadius.Text, out outValue))
                nProximityRadius = Convert.ToDouble(outValue);
            else
                nProximityRadius = DefProximityRadius;
        }

        private void ProximityAlt_TextChanged(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(ProximityAlt.Text, out outValue))
                nProximityAlt = Convert.ToDouble(outValue);
            else
                nProximityRadius = DefProximityRadius;
        }

        private void RTHAltitude_Leave(object sender, EventArgs e)
        {
            double outValue;

            if (double.TryParse(RTHAltitude.Text, out outValue))
                nAltHold = outValue;
            else
                nAltHold = DefAltHold;

            RTHAltitude.Text = string.Format("{0:n0}",outValue);
            myRegistry.Write("AltHold", Convert.ToString(nAltHold));

        }

        private void ProximityRadius_Leave(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(ProximityRadius.Text, out outValue))
                nProximityRadius = outValue;
            else
                nProximityRadius = DefProximityRadius;

            ProximityRadius.Text = string.Format("{0:n0}", outValue);
            myRegistry.Write("ProximityRadius", Convert.ToString(nProximityRadius));
        }

        private void ProximityAlt_Leave(object sender, EventArgs e)
        {
            double outValue;
            if (double.TryParse(ProximityAlt.Text, out outValue))
                nProximityAlt = outValue;
            else
                nProximityAlt = DefProximityAlt;

            ProximityAlt.Text = string.Format("{0:n0}", outValue);
            myRegistry.Write("ProximityAlt", Convert.ToString(nProximityAlt));
        }

        private void RangeLimit_Leave(object sender, EventArgs e)
        {
            int outValue;

            if (int.TryParse(ProximityAlt.Text, out outValue))
                RangeLimit = outValue;
            else
                RangeLimit = DefaultRangeLimit;

            RangeLimitSetting.Text = string.Format("{0:n0}", outValue);
            myRegistry.Write("RangeLimit", Convert.ToString(RangeLimit));
        }

        private void cmdCenter_Click(object sender, EventArgs e)
        {
            if (M.Rows.Count != 0)
            {
                timer1.Enabled = false;
                webBrowser1.Document.GetElementById("index").SetAttribute("value", 
                    Convert.ToString(M.CurrentRow.Cells[0].Value));
                webBrowser1.Document.GetElementById("lat").SetAttribute("value", 
                    Regex.Replace(Convert.ToString(M.CurrentRow.Cells[1].Value), ",", "."));
                webBrowser1.Document.GetElementById("Lng").SetAttribute("value", 
                    Regex.Replace(Convert.ToString(M.CurrentRow.Cells[2].Value), ",", "."));
                webBrowser1.Document.GetElementById("centerMapWaypointButton").InvokeMember("click");
                timer1.Enabled = true;
            }
        }

        private void UAVXNavToolStripMenuItem_Click(object sender, EventArgs e)
        {
            //System.Diagnostics.Process.Start("UAVXNav");
        }

        private void RemoveRow_Click(object sender, EventArgs e)
        {
            int nCount;
            string nSelectedColumn;
            if (M.Rows.Count != 0)
            {
                //M.CurrentCell = M.Rows[index].Cells[0];
                nSelectedColumn = Convert.ToString(M.CurrentRow.Cells[0].Value);
                M.Rows.RemoveAt(Convert.ToInt32(nSelectedColumn)-1);
                webBrowser1.Document.GetElementById("dwIndex").SetAttribute("value", nSelectedColumn);
                webBrowser1.Document.GetElementById("deletewaypointButton").InvokeMember("click");

                for (nCount = 1;nCount <= M.Rows.Count;nCount++)
                    M[0,nCount-1].Value = Convert.ToString(nCount);
            }
            UAVXNoOfWP = Convert.ToByte(M.Rows.Count);
        }

        private void M_CellContentClick(object sender, DataGridViewCellEventArgs e)
        {
            try
            {
                if (InOnlineMode && UseCompatibleMode)
                {
                    webBrowser1.Document.GetElementById("index").SetAttribute("value", 
                        Convert.ToString(M.CurrentRow.Cells[0].Value));
                    webBrowser1.Document.GetElementById("Lat").SetAttribute("value", 
                        Regex.Replace(Convert.ToString(M.CurrentRow.Cells[1].Value), ",", "."));
                    webBrowser1.Document.GetElementById("Lng").SetAttribute("value", 
                        Regex.Replace(Convert.ToString(M.CurrentRow.Cells[2].Value), ",", "."));
                }
            }
            catch { }
        }

        private void toolStripMenuItem14_Click(object sender, EventArgs e)
        {
            this.Close();
        }

        private void toolStripMenuItem8_Click(object sender, EventArgs e)
        {

        }

        private void MenuSaveMission_Click(object sender, EventArgs e)
        {
            int nCount;

            SaveFileDialog dlg = new SaveFileDialog();

            dlg.DefaultExt = "txt";
            dlg.InitialDirectory = Application.StartupPath + "\\Missions";
            dlg.Title = "Mission File";
            dlg.AddExtension = true;
            dlg.Filter = "Mission Files (*.txt)|*.txt|All files (*.*)|*.*";

            if (dlg.ShowDialog() == DialogResult.OK)
            {
                System.IO.FileStream MissionFileStream = new System.IO.FileStream(dlg.FileName, System.IO.FileMode.Create);
                System.IO.StreamWriter MissionFileStreamWriter = new System.IO.StreamWriter(MissionFileStream, System.Text.Encoding.ASCII);

                MissionFileStreamWriter.WriteLine("OPTIONS:" + Convert.ToInt16(Convert.ToDouble(ProximityRadius.Text))
                    + "," + Convert.ToInt16(Convert.ToDouble(ProximityAlt.Text))
                    + "," + LaunchManuallyCheckBox.Checked + "," + LookupAltCheckBox.Checked + 
                    "," + ZoomMaps.Value.ToString());
                MissionFileStreamWriter.WriteLine("HOME:" + Regex.Replace(Convert.ToString(LaunchLat.Text), 
                    ",", ".") + "," + Regex.Replace(Convert.ToString(LaunchLon.Text), 
                    ",", ".") + "," + Convert.ToInt16(Convert.ToDouble(RTHAltitude.Text
                    )));
                for (nCount = 1;nCount <= M.Rows.Count;nCount++)
                    MissionFileStreamWriter.WriteLine(Regex.Replace(Convert.ToString(M[1, nCount - 1].Value), 
                        ",", ".") + "," + Regex.Replace(Convert.ToString(M[2,nCount-1].Value), 
                        ",", ".") 
                        + "," + Convert.ToInt32(Convert.ToDouble(M[4,nCount-1].Value))
                        + "," + Convert.ToInt32(Convert.ToDouble(M[6,nCount-1].Value))
                        );
    
                MissionFileStreamWriter.Flush();
                MissionFileStreamWriter.Close();
                MissionFileStream.Close();

                Application.DoEvents();
                Bitmap b = new Bitmap(webBrowser1.ClientSize.Width, webBrowser1.ClientSize.Height);
                Graphics g = Graphics.FromImage(b);
                g.CopyFromScreen(webBrowser1.Parent.PointToScreen(webBrowser1.Location), 
                    new Point(0, 0), webBrowser1.ClientSize);
                b.Save(dlg.FileName.Substring(0, dlg.FileName.Length-3) + "png");

                myRegistry.Write("LastSaved", dlg.FileName);
            }
        }

        private void MenuLoadMission_Click(object sender, EventArgs e)
        {
            string sLinesFromFile;
            string sValue;

            timer1.Enabled = false;
            OpenFileDialog dlg = new OpenFileDialog();

            dlg.DefaultExt = "txt";
            dlg.InitialDirectory = Application.StartupPath + "\\Missions";
            dlg.Multiselect = false;
            dlg.CheckFileExists = true;
            dlg.Title = "Load Mission File";
            dlg.AddExtension = true;
            dlg.Filter = "Mission Files (*.txt)|*.txt|All files (*.*)|*.*";

            if (dlg.ShowDialog() == DialogResult.OK)
            {
                if (checkOnlineMode())
                    webBrowser1.Document.GetElementById("ClearButton").InvokeMember("click");
                else
                {
                    System.Diagnostics.Debug.WriteLine(dlg.FileName.Substring(0, 
                        dlg.FileName.Length-3) + "png");
                    if (File.Exists(dlg.FileName.Substring(0, dlg.FileName.Length-3) + "png"))
                    {
                        OfflineMap.Load(dlg.FileName.Substring(0, dlg.FileName.Length-3) + "png");
                        OfflineMapLabel.Visible = true;
                        OfflineMap.Visible = true;
                        //this.Width = picMap.Width + picMap.Left + 20;
                        //this.ResizeRedraw();
                    }
                    else
                    {
                        OfflineMap.Image = null;
                        OfflineMap.Invalidate();
                        //picMap.Image.Dispose();
                        OfflineMapLabel.Visible = false;
                        //this.Width = fraStatus.Width + fraStatus.Left + 20;
                        //this.ResizeRedraw();
                    }
                }
                M.Rows.Clear();

                FileStream fileStream = new FileStream(dlg.FileName, FileMode.Open);
                try
                {
                    sLastLoaded = dlg.FileName;
                    System.Text.ASCIIEncoding enc = new System.Text.ASCIIEncoding();
                    byte[] bytes = new byte[fileStream.Length];
                    int numBytesToRead = (int)fileStream.Length;
                    int numBytesRead = 0;
                    while (numBytesToRead > 0)
                    {
                        // Read may return anything from 0 to numBytesToRead.
                        int n = fileStream.Read(bytes, numBytesRead, numBytesToRead);

                        // Break when the end of the file is reached.
                        if (n == 0)
                            break;

                        numBytesRead += n;
                        numBytesToRead -= n;
                    }
                    numBytesToRead = bytes.Length;

                    sLinesFromFile = enc.GetString(bytes);
                    //System.Diagnostics.Debug.WriteLine(sLinesFromFile);
                    char[] sep ={ '\n' };
                    string[] values = sLinesFromFile.Split(sep);
                    string sObjType;
                    double nLat;
                    double nLon;

                    for (int nCount = 0;nCount < values.Length-1;nCount++)
                    {
                        sValue = values[nCount].Substring(values[nCount].IndexOf(":") + 1);
                        sValue = sValue.Substring(0, sValue.Length-1);

                        char[] sep2 ={ ',' };
                        string[] latLonAlt = sValue.Split(sep2);
                        sObjType = "";
                        if (values[nCount].IndexOf(":") != -1)
                            sObjType = values[nCount].Substring(0, values[nCount].IndexOf(":"));

                        double.TryParse(latLonAlt[0], System.Globalization.NumberStyles.Float, 
                            System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLat);
                        double.TryParse(latLonAlt[1], System.Globalization.NumberStyles.Float, 
                            System.Globalization.CultureInfo.GetCultureInfo("en-US"), out nLon);
                        if (nCount % 1 == 0)
                            Application.DoEvents();
  
                        //System.Diagnostics.Debug.WriteLine(latLonAlt[0] + " , " + nLat);

                        //System.Diagnostics.Debug.WriteLine(values[nCount]);
                        //System.Diagnostics.Debug.WriteLine(values[nCount].Substring(0, values[nCount].IndexOf(":")));

                        switch (sObjType)
                        {
                            case "OPTIONS":
                                ProximityRadius.Text = string.Format("{0:n0}", Convert.ToDouble(latLonAlt[0]));
                                ProximityAlt.Text = string.Format("{0:n0}", Convert.ToDouble(latLonAlt[1]));
                                LaunchManuallyCheckBox.Checked = Convert.ToBoolean(latLonAlt[2]);
                                LookupAltCheckBox.Checked = false; // Convert.ToBoolean(latLonAlt[3]);
                                if (latLonAlt.GetUpperBound(0) >= 4)
                                    ZoomMaps.Value = Convert.ToInt32(latLonAlt[4]);
                                break;
                            case "HOME":
                                if (InOnlineMode && UseCompatibleMode)
                                {
                                    webBrowser1.Document.GetElementById("homeLat").SetAttribute(
                                        "value", latLonAlt[0]);
                                    webBrowser1.Document.GetElementById("homeLng").SetAttribute(
                                        "value", latLonAlt[1]);
                                    webBrowser1.Document.GetElementById("setHomeLatLngButton").InvokeMember("click");
                                    webBrowser1.Document.GetElementById("centerMapHomeButton").InvokeMember("click");
                                }
                                nAltHold = Convert.ToDouble(latLonAlt[2]);
                                nHomeAlt = GetAltitudeData(nLat, nLon);
                                HomeAlt.Text = string.Format("{0:n1}", nHomeAlt);
                                RTHAltitude.Text = string.Format("{0:n1}", nAltHold);
                                
                                LaunchLat.Text = Convert.ToString(nLat);
                                LaunchLon.Text = Convert.ToString(nLon);
                                //System.Diagnostics.Debug.WriteLine(values[nCount].Substring(values[nCount].IndexOf(":")+1));
                                break;
                            default:
                                if (M.Rows.Count > MaxWayPoints-1)
                                {
                                    MessageBox.Show("Too many WP selected.\nUAVX can currently can handle up to " + 
                                        MaxWayPoints.ToString() + " WP.", "Max WP",
                                        MessageBoxButtons.OK,MessageBoxIcon.Exclamation);
                                    timer1.Enabled = true;
                                    return;
                                }
                                ClickMapLabel.Visible = false;
                                M.Rows.Add();
                                M[0,M.Rows.Count-1].Value = M.Rows.Count;
                                M[1,M.Rows.Count-1].Value = Convert.ToString(nLat);
                                M[2,M.Rows.Count-1].Value = Convert.ToString(nLon);

                                setWPValues(M.Rows.Count, latLonAlt[0], latLonAlt[1], true);
                                if (latLonAlt.Length > 2)
                                    M[4,M.Rows.Count-1].Value = string.Format("{0:n0}", 
                                        Convert.ToDouble(latLonAlt[2]));
                                else
                                    M[4,M.Rows.Count-1].Value = string.Format("{0:n0}", nAltHold);

                                M[5,M.Rows.Count-1].Value = Convert.ToString(GetAltitudeData(nLat, nLon));

                                /*
                                if (LookupAltCheckBox.Checked)
                                    M[3,M.Rows.Count-1].Value = string.Format("{0:n0}", 
                                        (Convert.ToDouble(M[4,M.Rows.Count-1].Value) + 
                                        Convert.ToDouble(M[5,M.Rows.Count-1].Value)-nHomeAlt));
                                else
                                */
                                    M[3,M.Rows.Count-1].Value = string.Format("{0:n0}", 
                                        Convert.ToDouble(M[4,M.Rows.Count-1].Value));

                                M[6, M.Rows.Count - 1].Value = string.Format("{0:n0}",
                                        Convert.ToDouble(latLonAlt[3]));

                                break;
                        }
                    }
                }
                finally
                {
                    if (InOnlineMode && UseCompatibleMode)
                        webBrowser1.Document.GetElementById("refreshButton").InvokeMember("click");

                    timer1_Tick(sender, e);
                    fileStream.Close();
                    UAVXNoOfWP = Convert.ToByte(M.Rows.Count);
                }
            }
            timer1.Enabled = true;
        }

        private void cboComSelect_SelectedIndexChanged(object sender, EventArgs e)
        {
            MenuFiles.HideDropDown();
            myRegistry.Write("ComPort", Convert.ToString(cboComSelect.Text));
        }

        private void MenuRead_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            UAVXRead();
            timer1.Enabled = true;
        }

        private void MenuWrite_Click(object sender, EventArgs e)
        {
            timer1.Enabled = false;
            UAVXWrite();
            timer1.Enabled = true;
        }

        private void chkLookupAlt_CheckedChanged(object sender, EventArgs e)
        {
            Cursor.Current = Cursors.WaitCursor;
            UseOverTerrainMode = false; // LookupAltCheckBox.Checked;
            setOnlineMode(InOnlineMode);
            updateAltColumns();
            myRegistry.Write("OverTerrain", "false");//Convert.ToString(LookupAltCheckBox.Checked));
            
            if (UseOverTerrainMode)
            {
                //System.Diagnostics.Debug.WriteLine(launchLat.Text.Replace(".",separatorFormat));
                //System.Diagnostics.Debug.WriteLine(separatorFormat);
                nHomeAlt = GetAltitudeData(Convert.ToDouble(LaunchLat.Text.Replace(
                    ".",separatorFormat)), Convert.ToDouble(LaunchLon.Text.Replace(
                    ".",separatorFormat)));
                HomeAlt.Text = string.Format("{0:n0}", nHomeAlt);
            }
            Cursor.Current = Cursors.Default; 
        }

        private void RangeLimit_TextChanged(object sender, EventArgs e)
        { 
            RangeLimit = Convert.ToInt16(RangeLimitSetting.Text);
            if (RangeLimit <= 50)
                RangeLimitSetting.BackColor = System.Drawing.Color.White;
            else
                if ( RangeLimit <= 100 )
                    RangeLimitSetting.BackColor = System.Drawing.Color.Lime;
                else
                    if (RangeLimit <= 150)
                        RangeLimitSetting.BackColor = System.Drawing.Color.Orange;
                    else
                        RangeLimitSetting.BackColor = System.Drawing.Color.Red;
 
        }

        private void cmdRead_Click(object sender, EventArgs e)
        {
            MenuRead_Click(null, null);
        }

        private void cmdWrite_Click(object sender, EventArgs e)
        {
            MenuWrite_Click(null,null);
        }

        private void cmdClearOffWP_Click(object sender, EventArgs e)
        {
            M.Rows.Clear();
        }

        private void cmdRemoveOffWP_Click(object sender, EventArgs e)
        {
            int nCount;
            string nSelectedColumn;
            if (M.Rows.Count != 0)
            {
                nSelectedColumn = Convert.ToString(M.CurrentRow.Cells[0].Value);
                M.Rows.RemoveAt(Convert.ToInt32(nSelectedColumn)-1);

                for (nCount = 1;nCount <= M.Rows.Count;nCount++)
                    M[0,nCount-1].Value = Convert.ToString(nCount);
            }
            UAVXNoOfWP = Convert.ToByte(M.Rows.Count);
        }

        private void cmdAddOffWP_Click(object sender, EventArgs e)
        {
            M.Rows.Add();
            UAVXNoOfWP = Convert.ToByte(M.Rows.Count);
            M[0,M.Rows.Count-1].Value = M.Rows.Count.ToString();
            M[1,M.Rows.Count-1].Value = "00.000000";
            M[2,M.Rows.Count-1].Value = "000.000000";
            M[3,M.Rows.Count-1].Value = RTHAltitude.Text;
        }

        private void launchLat_Leave(object sender, EventArgs e)
        {
            double outValue;
            double.TryParse(LaunchLat.Text, out outValue);
            if (outValue != 0)
            {
                if (!DoingStartup && InOnlineMode && UseCompatibleMode)
                    setGoogleMapHome(LaunchLat.Text, LaunchLon.Text, true);

                myRegistry.Write("homeLat", Regex.Replace(LaunchLat.Text, ",", "."));
            }
        }

        private void launchLon_Leave(object sender, EventArgs e)
        {
            double outValue;
            double.TryParse(LaunchLon.Text, out outValue);
            if (outValue != 0)
            {
                if (!DoingStartup && InOnlineMode && UseCompatibleMode)
                    setGoogleMapHome(LaunchLat.Text, outValue.ToString(), true);

                myRegistry.Write("homeLng", Regex.Replace(LaunchLon.Text, ",", "."));
            }
        }

        private void launchLon_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar.ToString() == "\r")
                launchLon_Leave (sender, e);
        }

        private void launchLat_KeyPress(object sender, KeyPressEventArgs e)
        {
            if (e.KeyChar.ToString() == "\r")
                launchLat_Leave (sender, e);
        }

        //-----------------------------------------------------------------------
        // UAVX Communications

        private void UAVXHello()
        {
            int wait = 0;
            int skips = 0;
            int retries = 0;
            byte one = 0;

            serialPort2.Write(new byte[] { (byte)'?' }, 0, 1);
            do
            { // skip over the UAVX menu prompt
                while (serialPort2.BytesToRead < 1)
                {
                    if (++wait > 200000)
                    {
                        serialPort2.Write(new byte[] { (byte)'?' }, 0, 1);

                        if (retries > 5)
                            break;
                        else
                        {
                            retries++;
                            wait = 0;
                        }
                    }
                }
                if (serialPort2.BytesToRead >= 1)
                    one = (byte)serialPort2.ReadByte();

                if (++skips > 1024)
                    break;
            }
            while (	one != (byte) '>' ); 
        }

        bool UAVXOK()
        {
            byte one = 0;
            int wait = 0;
            bool OK = true;

            do
            {
                if (serialPort2.BytesToRead >= 1)
                    one = (byte)serialPort2.ReadByte();

                if (++wait > 10000)
                {
                    OK = false;
                    break;
                }
            }
            while ( one != (byte) 'W' );

            return OK;
        }

        public void UAVXWriteNav(ref string errorMessage)
        {
            bool OK = true;
            byte csum = 0;

            try
            {
                serialPort2.Open();
                UAVXHello(); 

                serialPort2.Write(new byte[] { (byte)'W', (byte)'1'}, 0, 2);
  
                OK = UAVXOK();
                if ( OK )//verifying reponse...
                {
                   csum = 0;
                   for (int x = 0; x < 255; x++)
                       csum ^= Nav[x];
                   Nav[255] = csum;
                   serialPort2.Write(Nav, 0, 256); // 256 bytes -> UAVX
                   UAVXOnWriteOK(EventArgs.Empty);
                }
                else
                   UAVXOnWriteFail(EventArgs.Empty);

                serialPort2.Close();
            }
            catch (Exception er)
            {
                errorMessage = Convert.ToString(er.Message);
            }
        }

        public void UAVXOnWriteOK(EventArgs e)
        {
            UAVXWriteOK(this, e);
        }
        public void UAVXOnWriteFail(EventArgs e)
        {
            UAVXWriteFail(this, e);
        }

        public void UAVXReadNav()
        {
            int wait = 0;
            bool OK = true;
            byte d = 0;
            byte csum = 0;

            serialPort2.Open();
            UAVXHello(); 

            serialPort2.Write(new byte[] { (byte)'W', (byte)'2' }, 0, 2);
            OK = UAVXOK();

            if ( OK )
            {
                csum = 0;
                for (int x = 0; x < 256; x++) // 256 bytes  <- UAVX
                {
                    wait = 0;
                    while (serialPort2.BytesToRead == 0)
                    {
                        if (++wait > 10000)
                            Application.DoEvents();
                        else if (wait > 20000)
                        {
                            OK = false;
                            break;
                        }
                    }
                    d = (byte)serialPort2.ReadByte(); //Cambiado... 
                    csum ^= d;
                    ReadBuff[x] = d;
                }

                while (serialPort2.BytesToRead == 0)
                {
                    if (++wait > 100000)
                    {
                        OK = false;
                        break;
                    }
                }
                OK = ((byte)serialPort2.ReadByte() == 0x06) && (csum == 0);

                if ( OK )
                {
                    Application.DoEvents();
                    //System.Diagnostics.Debug.WriteLine(serialPort2.ReadByte() + 
                    //" Byte=" + Convert.ToString(0x10));

                    for (int y = 0; y < 256; y++)
                        Nav[y] = ReadBuff[y];

                    UAVXOnReadOK(EventArgs.Empty);
                }
                else
                    UAVXOnReadFail(EventArgs.Empty);
            }
            else
                UAVXOnReadFail(EventArgs.Empty);
 
            serialPort2.Close();
        }

        public void UAVXReadStats()
        {
            int wait = 0;
            bool OK = true;
            byte csum = 0;
            byte d = 0;

            serialPort2.Open();
            UAVXHello(); 

            serialPort2.Write(new byte[] { (byte)'W', (byte)'3' }, 0, 2);
            OK = UAVXOK();

            if ( OK )
            {
                csum = 0;
                for (int x = 0; x < 64; x++) // 64 bytes  <- UAVX
                {
                    wait = 0;
                    while (serialPort2.BytesToRead == 0)
                    {
                        if (++wait > 100000)
                            Application.DoEvents();
                        else if (wait > 20000)
                        {
                            OK = false;
                            break;
                        }
                    }
                    d = (byte)serialPort2.ReadByte(); //Cambiado...
                    csum ^= d;
                    ReadBuff[x] = d;
                }

                while (serialPort2.BytesToRead == 0)
                {
                    if (++wait > 10000)
                    {
                        OK = false;
                        break;
                    }
                }
                OK = ((byte)serialPort2.ReadByte() == 0x06 ) && (csum == 0);

                if ( OK )
                {
                    Application.DoEvents();
                    //System.Diagnostics.Debug.WriteLine(serialPort2.ReadByte() + 
                    //" Byte=" + Convert.ToString(0x10));

                    for (int y = 0; y < 64; y++)
                        StatsBytes[y] = ReadBuff[y];

                    UAVXOnReadOK(EventArgs.Empty);
                }
                else
                    UAVXOnReadFail(EventArgs.Empty);
            }
            else
                UAVXOnReadFail(EventArgs.Empty);
 
            serialPort2.Close();
        }


        //-----------------------------------------------------------------------
        // UAVX Communications

        
        private void UAVXOpenTelemetry()
        {
            string sError = "";

            UAVXCloseTelemetry();

            SetSerialPort(UAVXNav.Properties.Settings.Default.COMPort, UAVXNav.Properties.Settings.Default.COMArmedBaudRate, ref sError);
            if (sError != "")
            {
                InFlight = false;
                statusLabel.Text = sError;
            }
            else
            {
                CreateSaveLogFile();

                InitPollPacket();
                RxHead = RxTail = 0;

                serialPort2.Open();
                if (serialPort2.IsOpen)
                    InFlight = true;
                else
                {
                    UAVXCloseTelemetry();
                    InFlight = false;
                }

               webBrowser1.Document.GetElementById("clearTravelButton").InvokeMember("click");
            }
/*
            if (InFlight)
            {
                GPSMinFixS.Visible = false;
                GPSMinSatS.Visible = false;
                GPSMinHDiluteS.Visible = false;
                BaroMaxROCS.Visible = false;

                GPSBaroScaleSLabel.Visible = false;
                GPSBaroScaleS.Visible = false;

                RelBaroPressureSLabel.Visible = false;
                RelBaroPressureS.Visible = false;
            }
            else
            {
                GPSMinFixS.Visible = true;
                GPSMinSatS.Visible = true;
                GPSMinHDiluteS.Visible = true;
                BaroMaxROCS.Visible = true;

                GPSBaroScaleSLabel.Visible = true;
                GPSBaroScaleS.Visible = true;

                RelBaroPressureSLabel.Visible = true;
                RelBaroPressureS.Visible = true;
            }
*/
        }

        void PollTelemetry()
        {
            byte b;
            short NewRxTail;

            if (serialPort2.IsOpen)
            {
                while (serialPort2.BytesToRead != 0)
                {
                    NewRxTail = RxTail;
                    NewRxTail++;
                    NewRxTail &= RxQueueMask;
                    if (NewRxTail != RxHead)
                    {
                        RxTail = NewRxTail;
                        b = (byte)serialPort2.ReadByte();
                        SaveLogFileBinaryWriter.Write(b);
                        RxQueue[RxTail] = b;
                    }
                    else
                        b = (byte)serialPort2.ReadByte(); // overflow!   
                }
                UAVXReadTelemetry();
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
                PacketRxState = WaitRxBody;
            }
            else
                if (RxPacketByteCount == 2)
                {
                    RxPacketLength = ch;
                    PacketRxState = WaitRxBody;
                }
                else
                    if (RxPacketByteCount >= (RxPacketLength + 3))
                    {
                        RxPacketError = RxCheckSum != 0;

                        CheckSumError = RxPacketError;

                        if (CheckSumError)
                            RxCheckSumErrors++;

                        if (!RxPacketError)
                        {
                            PacketReceived = true;
                            ReceivedPacketTag = RxPacketTag;
                        }
                        PacketRxState = WaitRxSentinel;
                        //   InitPollPacket(); 
                    }
                    else
                        PacketRxState = WaitRxBody;
        }

        void ParseFrSkyPacket(byte ch)
        {
            switch (FrSkyPacketRxState)
            {
                case WaitRxSentinel:
                    if (ch == 0x7e)
                    {
                        FrSkyRxPacketTag = FrSkyPacketTag;
                        FrSkyPacketRxState = WaitRxTag;
                    }
                    break;
                case WaitRxTag:
                    if (ch != 0x7e)
                    {
                        FrSkyReceivedPacketTag = ch;
                        FrSkyRxPacketByteCount = 0;
                        FrSkyPacketRxState = WaitRxBody;
                    }
                    break;
                case WaitRxBody:
                    if (((ch == 0x7d) || (ch == 0x7e)) && (FrSkyRxPacketByteCount != 8))
                        FrSkyPacketRxState = WaitRxESC;
                    else
                    {
                        FrSkyPacket[FrSkyRxPacketByteCount++] = ch;
                        if (FrSkyRxPacketByteCount == 9)
                        {
                            if (ch != 0x7e)
                                RxFrSkyErrors++;
                            else
                                FrSkyPacketReceived = true;
                            FrSkyPacketRxState = WaitRxSentinel;
                        }
                    }
                    break;
                case WaitRxESC:
                    ch ^= 0x20;
                    FrSkyPacket[FrSkyRxPacketByteCount++] = ch;
                    FrSkyPacketRxState = WaitRxBody;
                    break;

                default: FrSkyPacketRxState = WaitRxSentinel; break;
            }
        }

        void ParsePacket(byte ch)
        {
            RxCheckSum ^= ch;
            switch (PacketRxState)
            {
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

        public void DoAirframeLabels()
        {
            
        }

        void UpdateFlags()
        {

            if ((Flags[0] & 0x01) != 0)
                AltHoldEnabledBox.BackColor = System.Drawing.Color.Green;
            else
                AltHoldEnabledBox.BackColor = System.Drawing.Color.Red;
            if ((Flags[0] & 0x02) != 0)
                TurnToWPBox.BackColor = System.Drawing.Color.Green;
            else
                TurnToWPBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[0] & 0x01) != 0)
                AltHoldEnabledBox.BackColor = System.Drawing.Color.Green;
            else
                AltHoldEnabledBox.BackColor = System.Drawing.Color.Red;
            if ((Flags[0] & 0x02) != 0)
                TurnToWPBox.BackColor = System.Drawing.Color.Green;
            else
                TurnToWPBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[0] & 0x20) != 0)
            {
                BatteryVolts.BackColor = System.Drawing.Color.Red;
                GyroFailS.BackColor = System.Drawing.Color.Red;
            }
            else
            {
                BatteryVolts.BackColor = BatteryGroupBox.BackColor;
                GyroFailS.BackColor = ErrorStatsGroupBox.BackColor;
            }
            if ((Flags[0] & 0x40) != 0)
                GPSFailS.BackColor = ErrorStatsGroupBox.BackColor;
            else
                GPSFailS.BackColor = System.Drawing.Color.Red;

            if ((Flags[1] & 0x01) != 0)
                BaroFailS.BackColor = System.Drawing.Color.Red;
            else
                BaroFailS.BackColor = BaroStatsGroupBox.BackColor;

            if ((Flags[1] & 0x02) != 0)
                AccFailS.BackColor = System.Drawing.Color.Red;
            else
                AccFailS.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[1] & 0x04) != 0)
                CompassFailS.BackColor = System.Drawing.Color.Red;
            else
                CompassFailS.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[1] & 0x10) != 0)
                AttitudeHoldBox.BackColor = System.Drawing.Color.Green;
            else
                AttitudeHoldBox.BackColor = System.Drawing.Color.Red;
            /*
            if ((Flags[1] & 0x20) != 0)
                ThrottleMovingBox.BackColor = System.Drawing.Color.LightSteelBlue;
            else
                ThrottleMovingBox.BackColor = FlagsGroupBox.BackColor;
            */
            if ((Flags[1] & 0x40) != 0)
                HoldingAltBox.BackColor = System.Drawing.Color.Green;
            else
                HoldingAltBox.BackColor = ErrorStatsGroupBox.BackColor;
            if ((Flags[1] & 0x80) != 0)
                NavigateBox.BackColor = System.Drawing.Color.Green;
            else
                NavigateBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[2] & 0x01) != 0)
                ReturnHomeBox.BackColor = System.Drawing.Color.Green;
            else
                ReturnHomeBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[2] & 0x02) != 0)
                ProximityBox.BackColor = System.Drawing.Color.Green;
            else
                ProximityBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[2] & 0x04) != 0)
                CloseProximityBox.BackColor = System.Drawing.Color.Green;
            else
                CloseProximityBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[2] & 0x10) != 0)
                UseRTHAutoDescendBox.BackColor = System.Drawing.Color.Green;
            else
                UseRTHAutoDescendBox.BackColor = ErrorStatsGroupBox.BackColor;

            if ((Flags[2] & 0x20) != 0)
                BaroAltValidBox.BackColor = System.Drawing.Color.Green;
            else
                BaroAltValidBox.BackColor = ErrorStatsGroupBox.BackColor;
            /*
                if ((Flags[2] & 0x40) != 0)
                    RangefinderValidBox.BackColor = System.Drawing.Color.Green;
                else
                    RangefinderValidBox.BackColor = ErrorStatsGroupBox.BackColor;
            */
            if ((Flags[2] & 0x80) != 0)
                UsingRangefinderBox.BackColor = System.Drawing.Color.Silver;
            else
                UsingRangefinderBox.BackColor = NavGroupBox.BackColor;

        }

        void DoOrientation()
        {
            /*
            //  do it all of the time in case some one changes orientation but does not shutdown GS if (!DoneOrientation)
            {
                if ((Flags[3] & 0x08) != 0)
                    FlightHeadingOffset = 0.0;
                else
                    FlightHeadingOffset = (OrientT * Math.PI) / 24.0;

                OSO = Math.Sin(FlightHeadingOffset);
                OCO = Math.Cos(FlightHeadingOffset);
            }
             * */
        }

        void UpdateFlightState()
        {
            /*
            switch (StateT)
            {
                case 0: FlightState.Text = "Starting"; break;
                case 1: FlightState.Text = "Landing"; break;
                case 2: FlightState.Text = "Landed"; break;
                case 3: FlightState.Text = "Shutdown"; break;
                case 4: FlightState.Text = "Flying"; break;
                default: FlightState.Text = "Unknown"; break;
            } // switch
             */
        }

        void UpdateNavState()
        {

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
        }

        void UpdateAirframe()
        {
            if (UAVXArm)
                switch (AirframeT)
                {
                    case 0: Airframe.Text = "Arm Quadrocopter"; break;
                    case 1: Airframe.Text = "Arm Tricopter"; break;
                    case 2: Airframe.Text = "Arm VTCopter"; break;
                    case 3: Airframe.Text = "Arm Y6Copter"; break;
                    case 4: Airframe.Text = "Arm Helicopter"; break;
                    case 5: Airframe.Text = "Arm Flying Wing"; break;
                    case 6: Airframe.Text = "Arm Conventional"; break;
                    case 7: Airframe.Text = "Arm Hexacopter"; break;
                }
            else
                switch (AirframeT)
                {
                    case 0: Airframe.Text = "Quadrocopter"; break;
                    case 1: Airframe.Text = "Tricopter"; break;
                    case 2: Airframe.Text = "VTcopter"; break;
                    case 3: Airframe.Text = "Y6copter"; break;
                    case 4: Airframe.Text = "Helicopter"; break;
                    case 5: Airframe.Text = "Flying Wing"; break;
                    case 6: Airframe.Text = "Conventional"; break;
                    case 7: Airframe.Text = "Hexacopter"; break;
                }
        }

       

        void UpdateFailState()
        {
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
                case 4: FailState.Text = "Terminating";
                    FailState.BackColor = System.Drawing.Color.Orange;
                    break;
                default: FailState.Text = "Unknown"; break;
            } // switch
        }

        void UpdateAttitude()
        {
            /*
            if (UAVXArm)
            {
                RollGyro.Text = string.Format("{0:n0}", RollGyroT * MILLIRADDEG);
                PitchGyro.Text = string.Format("{0:n0}", PitchGyroT * MILLIRADDEG);
                YawGyro.Text = string.Format("{0:n0}", YawGyroT * MILLIRADDEG);
                RollAngle.Text = string.Format("{0:n0}", RollAngleT * MILLIRADDEG);
                PitchAngle.Text = string.Format("{0:n0}", PitchAngleT * MILLIRADDEG);
                YawAngle.Text = string.Format("{0:n0}", YawAngleT * MILLIRADDEG);
            }
            else
            {
                RollGyro.Text = string.Format("{0:n0}", RollGyroT);
                PitchGyro.Text = string.Format("{0:n0}", PitchGyroT);
                YawGyro.Text = string.Format("{0:n0}", YawGyroT);
                RollAngle.Text = string.Format("{0:n0}", -RollAngleT / AttitudeToDegrees);
                PitchAngle.Text = string.Format("{0:n0}", -PitchAngleT / AttitudeToDegrees);
                YawAngle.Text = string.Format("{0:n0}", YawAngleT / AttitudeToDegrees);
            }
             */
        }

        void UpdateAccelerations()
        {
            /*
            if (UAVXArm)
            {
                LRAcc.Text = string.Format("{0:n2}", (float)LRAccT * 0.001);
                FBAcc.Text = string.Format("{0:n2}", (float)FBAccT * 0.001);
                DUAcc.Text = string.Format("{0:n2}", (float)DUAccT * 0.001);

                FBAccLabel.Text = "BF";
                DUAccLabel.Text = "UD";
            }
            else
            {
                LRAcc.Text = string.Format("{0:n2}", (float)LRAccT / 1024.0);
                FBAcc.Text = string.Format("{0:n2}", (float)FBAccT / 1024.0);
                DUAcc.Text = string.Format("{0:n2}", (float)DUAccT / 1024.0);

                FBAccLabel.Text = "FB";
                DUAccLabel.Text = "DU";
            }
             */
        }

        void UpdateCompensation()
        {
            /*
            if (UAVXArm)
            {
                IntCorrPitchLabel.Text = "BF";
                AccAltCompLabel.Text = "UD";
            }
            else
            {
                IntCorrPitchLabel.Text = "FB";
                AccAltCompLabel.Text = "DU";
            }
            IntCorrRoll.Text = string.Format("{0:n0}", IntCorrRollT * OUTMaximumScale);
            IntCorrPitch.Text = string.Format("{0:n0}", IntCorrPitchT * OUTMaximumScale);
            AccAltComp.Text = string.Format("{0:n0}", AccAltCompT * OUTMaximumScale);
            AltComp.Text = string.Format("{0:n0}", AltCompT * OUTMaximumScale);
             */
        }

   


        public void UAVXReadTelemetry()
        {
            byte b;
            short i;

            //  textBox1.AppendText(b);

            while (RxHead != RxTail)
            {
                b = RxQueue[RxHead];
                RxHead++;
                RxHead &= RxQueueMask;

                ParsePacket(b);

               // RxTypeErr.Text = string.Format("{0:n0}", RxIllegalErrors);
               // RxCSumErr.Text = string.Format("{0:n0}", RxCheckSumErrors);
               // RxLenErr.Text = string.Format("{0:n0}", RxLengthErrors);

                if (PacketReceived)
                {
                    PacketReceived = false;

                    switch (RxPacketTag)
                    {
                        case UAVXCustomPacketTag:
                            break;
                        case UAVXMinPacketTag:
                            for (i = 2; i < (NoOfFlagBytes + 2); i++)
                                Flags[i - 2] = ExtractByte(ref UAVXPacket, i);

                            UpdateFlags();

                            StateT = ExtractByte(ref UAVXPacket, 8);
                            NavStateT = ExtractByte(ref UAVXPacket, 9);
                            FailStateT = ExtractByte(ref UAVXPacket, 10);
                            BatteryVoltsT = ExtractShort(ref UAVXPacket, 11);
                            BatteryCurrentT = ExtractShort(ref UAVXPacket, 13);
                            BatteryChargeT = ExtractShort(ref UAVXPacket, 15);
                          //  RollAngleT = ExtractShort(ref UAVXPacket, 17);
                          //  PitchAngleT = ExtractShort(ref UAVXPacket, 19);
                            RelBaroAltitudeT = ExtractInt24(ref UAVXPacket, 21);
                            RangefinderAltitudeT = ExtractShort(ref UAVXPacket, 24);
                            HeadingT = ExtractShort(ref UAVXPacket, 26);
                            GPSLatitudeT = ExtractInt(ref UAVXPacket, 28);
                            GPSLongitudeT = ExtractInt(ref UAVXPacket, 32);
                            AirframeT = ExtractByte(ref UAVXPacket, 36);
                          //  OrientT = ExtractByte(ref UAVXPacket, 37);
                            MissionTimeMilliSecT = ExtractInt24(ref UAVXPacket, 38);

                            DesiredAltitudeT = 0;
                          //  ROCT = 0;

                            UAVXArm = (AirframeT & 0x80) != 0;
                            AirframeT &= 0x7f;

                            UpdateFlightState();
                            UpdateNavState();
                            UpdateFailState();
                       //     UpdateBattery();
                       //     UpdateWhere();
                            UpdateFlags();
                            UpdateAirframe();

                        //    UpdateAltitude();
                            UpdateAttitude();

                            DoOrientation();
/*
                            Heading.Text = string.Format("{0:n0}", (float)HeadingT * MILLIRADDEG);

                            GPSLongitude.Text = string.Format("{0:n6}", (double)GPSLongitudeT / 6000000.0);
                           GPSLatitude.Text = string.Format("{0:n6}", (double)GPSLatitudeT / 6000000.0);

                            if ((Flags[0] & 0x40) != 0) // GPSValid
                            {
                                GPSLongitude.BackColor = NavGroupBox.BackColor;
                                GPSLatitude.BackColor = NavGroupBox.BackColor;
                            }
                            else
                            {
                                GPSLongitude.BackColor = System.Drawing.Color.Orange;
                                GPSLatitude.BackColor = System.Drawing.Color.Orange;
                            }

                            if (FirstGPSCoordinates && ((Flags[0] & 0x80) != 0))
                            {
                                FirstGPSCoordinates = false;
                                OriginLatitude = GPSLatitudeT;
                                OriginLongitude = GPSLongitudeT;
                            }
 */

                     //       MissionTimeSec.Text = string.Format("{0:n3}", MissionTimeMilliSecT * 0.001);
                          
                            break;
                        case UAVXParamsPacketTag:
                        
                            break;
                        case UAVXStickPacketTag:
                          
                            break;
                        case UAVXStatsPacketTag:
                           // StatsPacketsReceived++;

                            for (b = 0; b < MaxStats; b++)
                                Stats[b] = ExtractShort(ref UAVXPacket, (byte)(b * 2 + 2));
                            ExtractShort(ref UAVXPacket, 37);

                            AirframeT = ExtractByte(ref UAVXPacket, 2 + MaxStats * 2);
                        //    OrientT = ExtractByte(ref UAVXPacket, 2 + MaxStats * 2 + 1);

                            UAVXArm = (AirframeT & 0x80) != 0;
                            AirframeT &= 0x7f;

                            I2CFailS.Text = string.Format("{0:n0}", Stats[I2CFailsX]);
                            GPSFailS.Text = string.Format("{0:n0}", Stats[GPSInvalidX]);
                            AccFailS.Text = string.Format("{0:n0}", Stats[AccFailsX]);
                            GyroFailS.Text = string.Format("{0:n0}", Stats[GyroFailsX]);
                            CompassFailS.Text = string.Format("{0:n0}", Stats[CompassFailsX]);
                            BaroFailS.Text = string.Format("{0:n0}", Stats[BaroFailsX]);
                         //  I2CESCFailS.Text = string.Format("{0:n0}", Stats[ESCI2CFailX]);
                           RCFailSafeS.Text = string.Format("{0:n0}", Stats[RCFailSafesX]);

                            GPSAltitudeS.Text = string.Format("{0:n1}", (float)Stats[GPSAltitudeX] * 0.01);
                        //    GPSMaxVelS.Text = string.Format("{0:n1}", (float)Stats[GPSMaxVelX] * 0.1);
                            GPSMinSatS.Text = string.Format("{0:n0}", Stats[GPSMinSatsX]);
                            GPSMaxSatS.Text = string.Format("{0:n0}", Stats[GPSMaxSatsX]);
                            GPSMinHDiluteS.Text = string.Format("{0:n2}", (float)Stats[MinHDiluteX] * 0.01);
                            GPSMaxHDiluteS.Text = string.Format("{0:n2}", (float)Stats[MaxHDiluteX] * 0.01);
                           
                            BaroRelAltitudeS.Text = string.Format("{0:n1}", (float)Stats[BaroRelAltitudeX] * 0.01);
                            BaroMinROCS.Text = string.Format("{0:n1}", (float)Stats[MinROCX] * 0.01);
                            BaroMaxROCS.Text = string.Format("{0:n1}", (float)Stats[MaxROCX] * 0.01);
                       //     MinTempS.Text = string.Format("{0:n1}", (float)Stats[MinTempX] * 0.1);
                       //     MaxTempS.Text = string.Format("{0:n1}", (float)Stats[MaxTempX] * 0.1);

                            RCFailSafeS.Text = string.Format("{0:n0}", Stats[RCFailSafesX]);

                            UpdateAirframe();

                            //DoOrientation();

                            if (Stats[AccFailsX] > 0)
                                AccFailS.BackColor = System.Drawing.Color.Red;
                            else
                                AccFailS.BackColor = ErrorStatsGroupBox.BackColor;

                            if (Stats[GyroFailsX] > 0)
                                GyroFailS.BackColor = System.Drawing.Color.Orange;
                            else
                                GyroFailS.BackColor = ErrorStatsGroupBox.BackColor;

                            if (Stats[CompassFailsX] > 50)
                                CompassFailS.BackColor = System.Drawing.Color.Red;
                            else
                                if (Stats[CompassFailsX] > 5)
                                    CompassFailS.BackColor = System.Drawing.Color.Orange;
                                else
                                    CompassFailS.BackColor = ErrorStatsGroupBox.BackColor;

                            if (Stats[BaroFailsX] > 50)
                                BaroFailS.BackColor = System.Drawing.Color.Red;
                            else
                                if (Stats[BaroFailsX] > 5)
                                    BaroFailS.BackColor = System.Drawing.Color.Orange;
                                else
                                    BaroFailS.BackColor = ErrorStatsGroupBox.BackColor;

                            if (Stats[GPSInvalidX] > 20)
                                GPSFailS.BackColor = System.Drawing.Color.Red;
                            else
                                if (Stats[GPSInvalidX] > 5)
                                    GPSFailS.BackColor = System.Drawing.Color.Orange;
                                else
                                    GPSFailS.BackColor = ErrorStatsGroupBox.BackColor;

                            if (Stats[MaxHDiluteX] > 150)
                                GPSMaxHDiluteS.BackColor = System.Drawing.Color.Red;
                            else
                                GPSMaxHDiluteS.BackColor = GPSStatsGroupBox.BackColor;

                            if (Stats[RCFailSafesX] > 10)
                                RCFailSafeS.BackColor = System.Drawing.Color.Red;
                            else
                                if (Stats[RCFailSafesX] > 2)
                                    RCFailSafeS.BackColor = System.Drawing.Color.Orange;
                                else
                                    RCFailSafeS.BackColor = ErrorStatsGroupBox.BackColor;

                            break;
                        case UAVXControlPacketTag:
                            break;
                        case UAVXFlightPacketTag:

                            for (i = 2; i < (NoOfFlagBytes + 2); i++)
                                Flags[i - 2] = ExtractByte(ref UAVXPacket, i);

                            UpdateFlags();

                            SimulationB = (Flags[3] & 0x08) != 0;
                            if ( SimulationB )
                            {
                                FlyingButton.BackColor = System.Drawing.Color.LightSteelBlue;
                                FlyingButton.Text = "Simulation";
                            }
                            
                           
                            StateT = ExtractByte(ref UAVXPacket, 8);
                            switch (StateT)
                            {
                                case 0: FlightState.Text = "Starting"; break;
                                case 1: FlightState.Text = "Landing"; break;
                                case 2: FlightState.Text = "Landed"; break;
                                case 3: FlightState.Text = "Shutdown"; break;
                                case 4: FlightState.Text = "Flying"; break;
                                default: FlightState.Text = "Unknown"; break;
                            } // switch
                            

                            BatteryVoltsT = ExtractShort(ref UAVXPacket, 9);
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

                            // PWMT not used in UAVXNav

                            MissionTimeMilliSecT = ExtractInt24(ref UAVXPacket, 53);

                            BatteryVolts.Text = string.Format("{0:n1}", (float)BatteryVoltsT * 27.73 / 1024.0); // ADC units 5V * (10K+2K2)/2K2.
                            BatteryCurrent.Text = string.Format("{0:n1}", ((float)BatteryCurrentT * CurrentSensorMax) / 1024.0 ); // ADC units sent
                            BatteryCharge.Text = string.Format("{0:n0}", (float)BatteryChargeT); // mAH // converted as it is used on board

                            RCGlitchesS.Text = string.Format("{0:n0}", RCGlitchesT);
                            if (RCGlitchesT > 20)
                                RCGlitchesS.BackColor = System.Drawing.Color.Red;
                            else
                                if (RCGlitchesT > 5)
                                    RCGlitchesS.BackColor = System.Drawing.Color.Orange;
                                else
                                    RCGlitchesS.BackColor = ErrorStatsGroupBox.BackColor;
 
                            DesiredThrottle.Text = string.Format("{0:n0}", (float)DesiredThrottleT * 0.5); //100.0) / RCMaximum);
                            /*
                            DesiredRoll.Text = string.Format("{0:n0}", ((float)DesiredRollT * 200.0) / RCMaximum);
                            DesiredPitch.Text = string.Format("{0:n0}", ((float)DesiredPitchT * 200.0) / RCMaximum);
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
                            */

                            attitudeIndicatorInstrumentControl1.SetAttitudeIndicatorParameters(
                                -PitchSumT / AttitudeToDegrees,
                                RollSumT / AttitudeToDegrees
                                );

                            break;
                        case UAVXNavPacketTag:

                            NavStateT = ExtractByte(ref UAVXPacket, 2);
                            FailStateT = ExtractByte(ref UAVXPacket, 3);

                            UpdateFailState();
                            UpdateNavState();

                            GPSNoOfSatsT = ExtractByte(ref UAVXPacket, 4);
                            GPSFixT = ExtractByte(ref UAVXPacket, 5);
                            CurrWPT = ExtractByte(ref UAVXPacket, 6);
                            ROCT = ((int)ROCT * 7 + (int)ExtractShort(ref UAVXPacket, 7)) / 8;
                            RelBaroAltitudeT = ExtractInt24(ref UAVXPacket, 9);
                            CruiseThrottleT = ExtractShort(ref UAVXPacket, 12);
                            RangefinderAltitudeT = ExtractShort(ref UAVXPacket, 14);
                            GPSHDiluteT = ExtractShort(ref UAVXPacket, 16);
                            HeadingT = ExtractShort(ref UAVXPacket, 18);
                            DesiredCourseT = ExtractShort(ref UAVXPacket, 20);
                            GPSVelT = ExtractShort(ref UAVXPacket, 22);
                            GPSHeadingT = ExtractShort(ref UAVXPacket, 24);
                            GPSRelAltitudeT = ExtractInt24(ref UAVXPacket, 26);
                            GPSLatitudeT = ExtractInt(ref UAVXPacket, 29);
                            GPSLongitudeT = ExtractInt(ref UAVXPacket, 33);
                            DesiredAltitudeT = ExtractInt24(ref UAVXPacket, 37);
                            DesiredLatitudeT = ExtractInt(ref UAVXPacket, 40);
                            DesiredLongitudeT = ExtractInt(ref UAVXPacket, 44);
                            NavStateTimeoutT = ExtractInt24(ref UAVXPacket, 48);

                            AmbientTempT = ExtractShort(ref UAVXPacket, 51);
                            GPSMissionTimeT = ExtractInt(ref UAVXPacket, 53);

                            ROC.Text = string.Format("{0:n1}", (float)ROCT * 0.01);

                            if (Math.Abs(DesiredThrottleT - CruiseThrottleT) < 3)
                                DesiredThrottle.BackColor = System.Drawing.Color.Green;
                            else
                                if (Math.Abs(DesiredThrottleT - CruiseThrottleT) < 7)
                                    DesiredThrottle.BackColor = System.Drawing.Color.LightGreen;
                                else
                                    DesiredThrottle.BackColor = NavGroupBox.BackColor;

                           GPSNoOfSats.Text = string.Format("{0:n0}", GPSNoOfSatsT);
                           if (GPSNoOfSatsT < 6)
                               GPSNoOfSats.BackColor = System.Drawing.Color.Orange;
                           else
                               GPSNoOfSats.BackColor = NavGroupBox.BackColor;

                           GPSFix.Text = string.Format("{0:n0}", GPSFixT);
                           if (GPSFixT < 2)
                               GPSFix.BackColor = System.Drawing.Color.Orange;
                           else
                               GPSFix.BackColor = NavGroupBox.BackColor;

                           GPSHDilute.Text = string.Format("{0:n2}", (float)GPSHDiluteT * 0.01);
                           if (GPSHDiluteT > 150)
                               GPSHDilute.BackColor = System.Drawing.Color.Red;
                           else
                               if (GPSHDiluteT > 130)
                                   GPSHDilute.BackColor = System.Drawing.Color.Orange;
                               else
                                   GPSHDilute.BackColor = NavGroupBox.BackColor;
                          
                            CurrWP.Text = string.Format("{0:n0}", CurrWPT);
                            //pad1.Text = string.Format("{0:n0}", ExtractByte(ref UAVXPacket, 7));

                            //BaroMinROCS.Text = string.Format("{0:n1}", (float)BaroROCT * 0.01);
                            //BaroRelAltitudeS.Text = string.Format("{0:n1}", (float)RelBaroAltitudeT * 0.01);

                            // MaxRangefinderROCS.Text = string.Format("{0:n1}", (float)RangefinderROCT * 0.01);
                            // RangefinderAltitude.Text = string.Format("{0:n1}", (float)RangefinderAltitudeT * 0.01);

                            HeadingS.Text = string.Format("{0:n0}", ((int)HeadingT * 180) / 3142);

                            GPSVelS.Text = string.Format("{0:n1}", (float)GPSVelT * 0.1);
                            //GPSROCS.Text = string.Format("{0:n1}", (float)GPSROCT * 0.01);
                            GPSAltitudeS.Text = string.Format("{0:n1}", (double)GPSRelAltitudeT * 0.01);
                           // GPSLongitudeS.Text = string.Format("{0:n6}", (double)GPSLongitudeT / 6000000.0);
                           // GPSLatitudeS.Text = string.Format("{0:n6}", (double)GPSLatitudeT / 6000000.0);

                            if ((Flags[2] & 0x80) != 0)
                            {
                                CurrAlt = RangefinderAltitudeT;
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

                            CurrentAltitude.Text = string.Format("{0:n0}", (float)CurrAlt * 0.01);
                            if ((CurrAlt * 0.01) > MaximumAltitudeLimit)
                                CurrentAltitude.BackColor = System.Drawing.Color.Orange;
                            else
                                CurrentAltitude.BackColor = NavGroupBox.BackColor;

                            AltError = CurrAlt - DesiredAltitudeT;
                            AltitudeError.Text = string.Format("{0:n1}", (float)AltError * 0.01);

                            if ((Flags[0] & 0x40) != 0) // GPSValid
                            {
                                GPSVelS.BackColor = NavGroupBox.BackColor;
                                //GPSROCS.BackColor = NavGroupBox.BackColor;
                                //GPSRelAltitude.BackColor = NavGroupBox.BackColor;
                                //GPSLongitude.BackColor = NavGroupBox.BackColor;
                                //GPSLatitude.BackColor = NavGroupBox.BackColor;
                                DesiredCourse.BackColor = NavGroupBox.BackColor;
                                DistanceToDesired.BackColor = NavGroupBox.BackColor;

                                DesiredCourse.Text = string.Format("{0:n0}", (DesiredCourseT * 180) / 3142);

                                LongitudeCorrection = Math.Cos(Math.PI / 180.0 * (DesiredLatitudeT + GPSLatitudeT) / 12000000.0);

                                NorthDiff = (double)(DesiredLatitudeT - GPSLatitudeT); // scale up to decimetres after conversion
                                EastDiff = (double)(DesiredLongitudeT - GPSLongitudeT) * LongitudeCorrection;

                                Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                                DistanceToDesired.Text = string.Format("{0:n1}", Distance);
                            }
                            else
                            {
                                GPSVelS.BackColor = System.Drawing.Color.Orange;
                               // GPSROC.BackColor = System.Drawing.Color.Orange;
                               // GPSRelAltitude.BackColor = System.Drawing.Color.Orange;
                               // GPSLongitude.BackColor = System.Drawing.Color.Orange;
                               // GPSLatitude.BackColor = System.Drawing.Color.Orange;
                                DesiredCourse.BackColor = System.Drawing.Color.Orange;
                                DistanceToDesired.BackColor = System.Drawing.Color.Orange;

                                DesiredCourse.Text = "?";
                                DistanceToDesired.Text = "?";
                            }

                            if ((Flags[0] & 0x80) != 0) // NavValid
                            {
                                NavValidBox.BackColor = System.Drawing.Color.Green;

                                WhereBearing.BackColor = WhereGroupBox.BackColor;
                                WhereDistance.BackColor = WhereGroupBox.BackColor;

                                NorthDiff = (double)(GPSLatitudeT - UAVXOriginLatitude);

                                LongitudeCorrection = Math.Cos(Math.PI / 180.0 * (UAVXOriginLatitude + GPSLatitudeT) / 12000000.0);

                                EastDiff = (double)(GPSLongitudeT - UAVXOriginLongitude)* LongitudeCorrection;
                                WhereDirection = Math.Atan2(EastDiff, NorthDiff) * 180.0 / Math.PI;
                                while (WhereDirection < 0)
                                    WhereDirection += 360.0;
                                WhereBearing.Text = string.Format("{0:n0}", WhereDirection);
                                Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
                                WhereDistance.Text = string.Format("{0:n0}", Distance);
                                if (Distance > MaximumRangeLimit)
                                    WhereDistance.BackColor = System.Drawing.Color.Red;
                                else
                                    WhereDistance.BackColor = WhereGroupBox.BackColor;
                            }
                            else
                            {
                                NavValidBox.BackColor = System.Drawing.Color.Orange;

                                WhereBearing.BackColor = System.Drawing.Color.Orange;
                                WhereDistance.BackColor = System.Drawing.Color.Orange;

                                WhereBearing.Text = "?";
                                WhereDistance.Text = "?";
                            }

                            if (NavStateTimeoutT >= 0)
                                NavStateTimeout.Text = string.Format("{0:n0}", NavStateTimeoutT / 1000);
                            else
                                NavStateTimeout.Text = " ";

                            headingIndicatorInstrumentControl1.SetHeadingIndicatorParameters(
                                ((HeadingT * 180) / 3142));

                         //   if (SimulationB) // Prohibit real-time navigation of UAVs using Google Maps/Earth
                                if (DistanceBetween(PrevGPSLat, PrevGPSLon, GPSLatitudeT, GPSLongitudeT) > 2.5)
                                {
                                    webBrowser1.Document.GetElementById("segmentLat").SetAttribute("value",
                                        Convert.ToString((double)GPSLatitudeT / 6000000));
                                    webBrowser1.Document.GetElementById("segmentLng").SetAttribute("value",
                                        Convert.ToString((double)GPSLongitudeT / 6000000));
                                    webBrowser1.Document.GetElementById("addSegmentButton").InvokeMember("click");
                                   // webBrowser1.Document.GetElementById("centerTravelEndButton").InvokeMember("click");
                                    PrevGPSLat = GPSLatitudeT;
                                    PrevGPSLon = GPSLongitudeT;
                                }

                            break;
                        default: break;
                    } // switch
                }
            }
        }

        private void UAVXCloseTelemetry()
        {
            if (serialPort2.IsOpen)
            {
                serialPort2.Close();

                SaveLogFileBinaryWriter.Flush();
                SaveLogFileBinaryWriter.Close();
                SaveLogFileStream.Close();
            }
            InFlight = false;
/*
            GPSMinFixS.Visible = true;
            GPSMinSatS.Visible = true;
            GPSMinHDiluteS.Visible = true;
            BaroMaxROCS.Visible = true;

            GPSBaroScaleSLabel.Visible = true;
            GPSBaroScaleS.Visible = true;

            RelBaroPressureSLabel.Visible = true;
            RelBaroPressureS.Visible = true;
*/
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

        public double DistanceBetween(int LatStart, int LonStart, int LatFinish, int LonFinish)
        {
            double NorthDiff, EastDiff, LongitudeCorrection, Distance;

            NorthDiff = (double)(LatFinish - LatStart);
            LongitudeCorrection = Math.Cos(Math.PI / 180.0 * (LatStart+LatFinish) / 12000000.0);
            EastDiff = (double)(LonFinish - LonStart)* LongitudeCorrection;
            Distance = ConvertGPSToM(Math.Sqrt(NorthDiff * NorthDiff + EastDiff * EastDiff));
            return( Distance );
        }

        public bool BeyondRangeLimit(int Lat, int Lon)
        {
            double Distance;

            Distance = DistanceBetween(UAVXOriginLatitude, UAVXOriginLongitude, Lat, Lon);
            return ( Distance > RangeLimit );
        }

        public double ConvertGPSToM(double c)
        {	// approximately 1.8553257183 cm per LSB at the Equator
            // conversion max is 21Km
            return (c * 0.018553257183);
        } // ConvertGPSToDM
   
        public void UAVXOnReadOK(EventArgs e)
        {
            UAVXReadOK(this, e);
        }

        public void UAVXOnReadFail(EventArgs e)
        {
            UAVXReadFail(this, e);
        }

        //----------------------------------------------------------------------- 
        // General Nav

        public byte UAVXOptions
        {
            get { return Nav[0]; }
            set { Nav[0] = value; }
        }

        public byte UAVXProximityAlt
        {
            get { return Nav[10]; }
            set { Nav[10] = value; }
        }
        
        public byte UAVXProximityRadius
        {
            get { return Nav[11]; }
            set { Nav[11] = value; }
        }

        public Int16 UAVXOriginAltitude
        {
            get { return (Int16)concat_4(12); }
        }

        public int UAVXOriginLatitude
        {
            get { return concat_4(14); }
            set { split_4(14, value); }
        }

        public int UAVXOriginLongitude
        {
            get { return concat_4(18); }
            set { split_4(18, value); }
        }

        public Int16 UAVXRTHAltitude
        {
            get { return (Int16)concat_4(22); }
            set { split_2(22, (int)value); }
        }

        public byte UAVXNoOfWP
        {
            set { Nav[9] = value; }
            get { return Nav[9]; }
        }

        //----------------------------------------------------------------------- 
        // Waypoints

        public int UAVXGetLat(int WP)
        {
            int ber = 0;
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            ber = (int)concat_4(real_position);
            return ber;
        }

        public void UAVXSetLat(int WP, int data)
        {
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            split_4(real_position, data);
        }

        public int UAVXGetLon(int WP)
        {
            int dist = 0;
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            dist = (int)concat_4(real_position + 4);
            return dist;
        }

        public void UAVXSetLon(int WP, int data)
        {
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            split_4(real_position + 4, data);
        }

        public int UAVXGetAlt(int WP)
        {
            int alt = 0;
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            alt = (int)concat_2(real_position + 8);
            return alt;
        }

        public void UAVXSetAlt(int WP, int data)
        {
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            split_2(real_position + 8, data);
        }

        public int UAVXGetLoiter(int WP)
        {
            byte loiter = 0;
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            loiter = Nav[real_position + 10];
            return loiter;
        }

        public void UAVXSetLoiter(int WP, byte data)
        {
            int real_position;
            real_position = start_byte + (WP * WPEntrySize);
            Nav[real_position + 10] = data;
        }

        // --------------------------------------------------------------------------
        // Nav variable access
        
        private void split_2(int startByte, int data)
        {
            Nav[startByte + 1] = (byte)(data >> 8);
            Nav[startByte] = (byte)(data);
        }

        private void split_4(int startByte, long data)
        {
            Nav[startByte + 3] = (byte)(data >> 24);
            Nav[startByte + 2] = (byte)(data >> 16);
            Nav[startByte + 1] = (byte)(data >> 8);
            Nav[startByte] = (byte)(data);

            //System.Diagnostics.Debug.WriteLine((int)(Nav[startByte + 3] << 24));
        }

        private int concat_2(int startByte)
        {
            int temp;
            temp = ((int)Nav[startByte + 1] << 8);
            temp |= (int)Nav[startByte];
            return temp;
        }

        private int concat_4(int startByte)
        {
            int temp;
            temp = (int)(Nav[startByte + 3] << 24);
            temp |= ((int)Nav[startByte + 2] << 16);
            temp |= ((int)Nav[startByte + 1] << 8);
            temp |= (int)Nav[startByte];
            return temp;
        }

        private void CurrentAltitude_Click(object sender, EventArgs e)
        {

        }

        private void ColourButton_Click(object sender, System.EventArgs e)
        {
            ColorDialog MyDialog = new ColorDialog();
            // Keeps the user from selecting a custom color.
            //MyDialog.AllowFullOpen = false;
            // Allows the user to get help. (The default is false.)
            MyDialog.ShowHelp = true;
            // Sets the initial color select to the current text color.
            MyDialog.Color = this.BackColor;

            if (MyDialog.ShowDialog() == DialogResult.OK)
            {
                this.BackColor = MyDialog.Color;
                Properties.Settings.Default.Colour = MyDialog.Color;
            }
        }

    }
}