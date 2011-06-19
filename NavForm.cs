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


namespace UAVXGS
{ // Rewritten and extended for UAVX by by Greg Egan (C) 2010.
  // Based originally on ArduPilotConfigTool(C) 2009 by By Jordi Muñoz && HappyKillmore,
  // and used with the kind permission of Jordi Muñoz - thanks Jordi.
 
    public partial class UAVXNavForm : Form
    {   
        const int DefaultRangeLimit = 100;
        const int MaximumRangeLimit = 250; // You carry total responsibility if you increase this value
        const int MaximumAltitudeLimit = 121; // You carry total responsibility if you increase this value 

        const double CurrentSensorMax = 50.0; // depends on current ADC used - needs calibration box?
        const double YawGyroRate = 400.0;
        const double RollPitchGyroRate = 400.0;
     
        int RangeLimit = DefaultRangeLimit;

        bool WPInvalid;
        byte UAVXOptions;
        byte [] Nav= new byte[1024];
        const short start_byte = 0;
      
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

        public UAVXNavForm()
        {
            DoingStartup = true;
            //options_form.button_change += new EventHandler(options_form_button_change);

            InitializeComponent();
         
            InOnlineMode = true;
            if (checkOnlineMode())
            {

                webBrowser1.Navigate(new Uri(System.AppDomain.CurrentDomain.BaseDirectory + "Maps.html"));
 /*       
                do
                {
                   Application.DoEvents();
                } while (webBrowser1.ReadyState != WebBrowserReadyState.Complete);
 */        
            }
            if (!checkGCompatibleMode())
                webBrowser1.Navigate("http://maps.google.com/support/bin/topic.py?topic=10781");

            myRegistry.SubKey = "SOFTWARE\\UAVXNav\\Settings";

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
            progressBar1.Value = 0;
            if (InOnlineMode && UseCompatibleMode)
                cmdClear_Click(null, null);

            statusLabel.Text = "Reading Mission...";
            progressBar1.Maximum = 120;
            progressBar1.Value = 10;
            this.Refresh();

            ProximityRadius.Text = string.Format("{0:n0}", UAVXProximityRadius);
            ProximityAlt.Text = string.Format("{0:n0}", UAVXProximityAlt);
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
           // string sError = "";
            int UAVXWPLat, UAVXWPLon;
            short UAVXWPAlt;
            bool MissionValid;

            progressBar1.Value = 0;
            statusLabel.Text = "Writing Mission...";
            this.Refresh();

                myRegistry.Write("LastSaved", sLastLoaded);

                //Converting general variables
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
                    //            progressBar1.Value = progressBar1.Value + 1;
                    UAVXWPLat = Convert.ToInt32((Convert.ToDouble(M[1, WPIndex].Value)) * (double)6000000);
                    UAVXWPLon = Convert.ToInt32((Convert.ToDouble(M[2, WPIndex].Value)) * (double)6000000);
                    UAVXWPAlt = Convert.ToInt16(Convert.ToInt16(M[3, WPIndex].Value));

                //    MissionValid &= (DistanceBetween(UAVXOriginLatitude, UAVXOriginLongitude, UAVXWPLat, UAVXWPLon) < RangeLimit);

                    MissionValid &= UAVXWPAlt < MaximumAltitudeLimit;

                    UAVXSetLat(WPIndex, UAVXWPLat);
                    UAVXSetLon(WPIndex, UAVXWPLon);
                    UAVXSetAlt(WPIndex, UAVXWPAlt);
                    UAVXSetLoiter(WPIndex, Convert.ToByte(Convert.ToByte(M[6, WPIndex].Value)));
                    progressBar1.Refresh();
                }

                if (MissionValid)
                {
                    //send mission
                }
   
        }

        private double GetAltitudeData(double latitude,double longitude)
        {  
            try {
                if (!UseOverTerrainMode)
                    return 0;

                double nDoubleReturn;
                // Create a 'WebRequest' CurrObject with the specified url. 
                WebRequest myWebRequest = WebRequest.Create(
                    "http://gisdata.usgs.gov/XMLWebServices2/Elevation_Service.asmx/getElevation?X_Value=" + 
                    Regex.Replace(longitude.ToString(), ",", ".") + "&Y_Value=" + 
                    Regex.Replace(latitude.ToString(), ",", ".") + 
                    "&Elevation_Units=METERS&Source_Layer=-1&Elevation_Only=True");
 
                WebResponse myWebResponse = myWebRequest.GetResponse();
                setOnlineMode(true); 
                Stream ReceiveStream = myWebResponse.GetResponseStream();
                Encoding encode = System.Text.Encoding.GetEncoding("utf-8");
                StreamReader readStream = new StreamReader(ReceiveStream, encode);
                string strResponse = readStream.ReadToEnd();
                readStream.Close();
                myWebResponse.Close();

                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(strResponse.IndexOf(">") + 1);
                strResponse = strResponse.Substring(0, strResponse.IndexOf("<"));
        
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

       //     if ( InFlight )
       //         PollTelemetry();

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
                        /*
                        if (BeyondRangeLimit((int)(Convert.ToDouble(M[1, WPIndex].Value) * 6000000),
                            (int)(Convert.ToDouble(M[2, WPIndex].Value) * 6000000)))
                        {
                         //   statusLabel.Text = "WP is beyond reasonable line of sight";
                            M.GridColor = System.Drawing.Color.Red;
                            WPInvalid = true;
                        }
                         * */
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
            {
              //  UAVXCloseTelemetry();
            }
            else
                UAVXOpenTelemetry();

            if (InFlight)
            {
              //  FlyingButton.BackColor = System.Drawing.Color.Green;
              //  FlyingButton.Text = "Flying";          
            }
            else
            {
              //  FlyingButton.Text = "Landed";
              //  FlyingButton.BackColor = System.Drawing.Color.Red;
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
                    {
                        RangeLimitSetting.BackColor = System.Drawing.Color.Red;
                        if (RangeLimit > MaximumRangeLimit)
                            RangeLimitSetting.Text = string.Format("{0:n0}", MaximumRangeLimit);
                    }
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
       
        private void UAVXOpenTelemetry()
        {
               webBrowser1.Document.GetElementById("clearTravelButton").InvokeMember("click");
        }

        //----------------------------------------------------------------------- 
        // General Nav

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
      
    }
}