namespace UAVXGS
{
    partial class Altitude
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Altitude));
            this.FlyingButton = new System.Windows.Forms.Button();
            this.serialPort1 = new System.IO.Ports.SerialPort(this.components);
            this.RollSum = new System.Windows.Forms.TextBox();
            this.PitchSum = new System.Windows.Forms.TextBox();
            this.BatteryVolts = new System.Windows.Forms.TextBox();
            this.BatteryCurrent = new System.Windows.Forms.TextBox();
            this.RCGlitches = new System.Windows.Forms.TextBox();
            this.DesiredThrottle = new System.Windows.Forms.TextBox();
            this.DesiredRoll = new System.Windows.Forms.TextBox();
            this.DesiredPitch = new System.Windows.Forms.TextBox();
            this.DesiredYaw = new System.Windows.Forms.TextBox();
            this.YawRate = new System.Windows.Forms.TextBox();
            this.PitchRate = new System.Windows.Forms.TextBox();
            this.RollRate = new System.Windows.Forms.TextBox();
            this.LRAcc = new System.Windows.Forms.TextBox();
            this.YawSum = new System.Windows.Forms.TextBox();
            this.DUAcc = new System.Windows.Forms.TextBox();
            this.FBAcc = new System.Windows.Forms.TextBox();
            this.VoltsLabel = new System.Windows.Forms.Label();
            this.LRAccLabel = new System.Windows.Forms.Label();
            this.FBAccLabel = new System.Windows.Forms.Label();
            this.DUAccLabel = new System.Windows.Forms.Label();
            this.DesiredYawLabel = new System.Windows.Forms.Label();
            this.DesiredPitchLabel = new System.Windows.Forms.Label();
            this.DesiredRollLabel = new System.Windows.Forms.Label();
            this.DesiredThrottleLabel = new System.Windows.Forms.Label();
            this.RCGlitchesLabel = new System.Windows.Forms.Label();
            this.CurrentLabel = new System.Windows.Forms.Label();
            this.GPSHDilute = new System.Windows.Forms.TextBox();
            this.RangefinderAltitude = new System.Windows.Forms.TextBox();
            this.RangefinderROC = new System.Windows.Forms.TextBox();
            this.RelBaroAltitude = new System.Windows.Forms.TextBox();
            this.GPSFix = new System.Windows.Forms.TextBox();
            this.BaroROC = new System.Windows.Forms.TextBox();
            this.GPSNoOfSats = new System.Windows.Forms.TextBox();
            this.FailState = new System.Windows.Forms.TextBox();
            this.GPSRelAltitude = new System.Windows.Forms.TextBox();
            this.GPSROC = new System.Windows.Forms.TextBox();
            this.RelAltitudeLabel = new System.Windows.Forms.Label();
            this.ROCLabel = new System.Windows.Forms.Label();
            this.HDiluteLabel = new System.Windows.Forms.Label();
            this.RangefinderAltLabel = new System.Windows.Forms.Label();
            this.GPSFixLabel = new System.Windows.Forms.Label();
            this.GPSNoOfSatsLabel = new System.Windows.Forms.Label();
            this.FailStateLabel = new System.Windows.Forms.Label();
            this.GPSAltLabel = new System.Windows.Forms.Label();
            this.BaroAltitudeLabel = new System.Windows.Forms.Label();
            this.YawRateLabel = new System.Windows.Forms.Label();
            this.PitchRateLabel = new System.Windows.Forms.Label();
            this.RollRateLabel = new System.Windows.Forms.Label();
            this.label5 = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.label7 = new System.Windows.Forms.Label();
            this.RxTypeErr = new System.Windows.Forms.TextBox();
            this.RxCSumErr = new System.Windows.Forms.TextBox();
            this.RxLenErr = new System.Windows.Forms.TextBox();
            this.RxTypeLabel = new System.Windows.Forms.Label();
            this.RxCSumLabel = new System.Windows.Forms.Label();
            this.RxLenLabel = new System.Windows.Forms.Label();
            this.FlightState = new System.Windows.Forms.TextBox();
            this.FlightStateLabel = new System.Windows.Forms.Label();
            this.GyroGroupBox = new System.Windows.Forms.GroupBox();
            this.ControlsGroupBox = new System.Windows.Forms.GroupBox();
            this.AttitudeGroupBox = new System.Windows.Forms.GroupBox();
            this.AccelerationsGroupBox = new System.Windows.Forms.GroupBox();
            this.CommsGroupBox = new System.Windows.Forms.GroupBox();
            this.RCGroupBox = new System.Windows.Forms.GroupBox();
            this.BatteryGroupBox = new System.Windows.Forms.GroupBox();
            this.flowLayoutPanel1 = new System.Windows.Forms.FlowLayoutPanel();
            this.BaroAltValidBox = new System.Windows.Forms.TextBox();
            this.RangefinderAltValidBox = new System.Windows.Forms.TextBox();
            this.GPSValidBox = new System.Windows.Forms.TextBox();
            this.UsingGPSAltBox = new System.Windows.Forms.TextBox();
            this.RTHAltHoldBox = new System.Windows.Forms.TextBox();
            this.TurnToWPBox = new System.Windows.Forms.TextBox();
            this.UseRTHAutoDescendBox = new System.Windows.Forms.TextBox();
            this.NavValidBox = new System.Windows.Forms.TextBox();
            this.ThrottleMovingBox = new System.Windows.Forms.TextBox();
            this.HoveringBox = new System.Windows.Forms.TextBox();
            this.AttitudeHoldBox = new System.Windows.Forms.TextBox();
            this.NavigateBox = new System.Windows.Forms.TextBox();
            this.ReturnHomeBox = new System.Windows.Forms.TextBox();
            this.UsingRangefinderBox = new System.Windows.Forms.TextBox();
            this.CloseProximityBox = new System.Windows.Forms.TextBox();
            this.ProximityBox = new System.Windows.Forms.TextBox();
            this.NearLevelBox = new System.Windows.Forms.TextBox();
            this.LostModelBox = new System.Windows.Forms.TextBox();
            this.MotorsArmedBox = new System.Windows.Forms.TextBox();
            this.LowBatteryBox = new System.Windows.Forms.TextBox();
            this.CompassFailBox = new System.Windows.Forms.TextBox();
            this.BaroFailBox = new System.Windows.Forms.TextBox();
            this.AccFailBox = new System.Windows.Forms.TextBox();
            this.GPSFailBox = new System.Windows.Forms.TextBox();
            this.FlagsGroupBox = new System.Windows.Forms.GroupBox();
            this.GPSStatBox = new System.Windows.Forms.GroupBox();
            this.AltitudeGroupBox = new System.Windows.Forms.GroupBox();
            this.WayHeadingLabel = new System.Windows.Forms.Label();
            this.GPSLongitudeLabel = new System.Windows.Forms.Label();
            this.GPSLatitudeLabel = new System.Windows.Forms.Label();
            this.NavStateLabel = new System.Windows.Forms.Label();
            this.GPSVelLabel = new System.Windows.Forms.Label();
            this.CurrWPLabel = new System.Windows.Forms.Label();
            this.GPSLatitude = new System.Windows.Forms.TextBox();
            this.GPSLongitude = new System.Windows.Forms.TextBox();
            this.NavState = new System.Windows.Forms.TextBox();
            this.CurrWP = new System.Windows.Forms.TextBox();
            this.WayHeading = new System.Windows.Forms.TextBox();
            this.WPAltitude = new System.Windows.Forms.Label();
            this.GPSVel = new System.Windows.Forms.TextBox();
            this.HeadingLabel = new System.Windows.Forms.Label();
            this.AltitudeError = new System.Windows.Forms.TextBox();
            this.Heading = new System.Windows.Forms.TextBox();
            this.WPDistanceLabel = new System.Windows.Forms.Label();
            this.DistanceToDesired = new System.Windows.Forms.TextBox();
            this.NavGroupBox = new System.Windows.Forms.GroupBox();
            this.CurrentAltitude = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.UserAttitudeToDegreesLabel = new System.Windows.Forms.Label();
            this.UserAttitudeToDegrees = new System.Windows.Forms.TextBox();
            this.AltitudeSource = new System.Windows.Forms.TextBox();
            this.GoogleButton = new System.Windows.Forms.Button();
            this.textBox1 = new System.Windows.Forms.TextBox();
            this.LocationBox = new System.Windows.Forms.GroupBox();
            this.WhereBearingLabel = new System.Windows.Forms.Label();
            this.WhereDistanceLabel = new System.Windows.Forms.Label();
            this.WhereDistance = new System.Windows.Forms.Label();
            this.WhereBearing = new System.Windows.Forms.Label();
            this.headingIndicatorInstrumentControl1 = new Instruments.HeadingIndicatorInstrumentControl();
            this.attitudeIndicatorInstrumentControl1 = new Instruments.AttitudeIndicatorInstrumentControl();
            this.GyroGroupBox.SuspendLayout();
            this.ControlsGroupBox.SuspendLayout();
            this.AttitudeGroupBox.SuspendLayout();
            this.AccelerationsGroupBox.SuspendLayout();
            this.CommsGroupBox.SuspendLayout();
            this.RCGroupBox.SuspendLayout();
            this.BatteryGroupBox.SuspendLayout();
            this.flowLayoutPanel1.SuspendLayout();
            this.FlagsGroupBox.SuspendLayout();
            this.GPSStatBox.SuspendLayout();
            this.AltitudeGroupBox.SuspendLayout();
            this.NavGroupBox.SuspendLayout();
            this.LocationBox.SuspendLayout();
            this.SuspendLayout();
            // 
            // FlyingButton
            // 
            this.FlyingButton.BackColor = System.Drawing.Color.Red;
            this.FlyingButton.ForeColor = System.Drawing.SystemColors.ControlText;
            this.FlyingButton.Location = new System.Drawing.Point(12, 9);
            this.FlyingButton.Name = "FlyingButton";
            this.FlyingButton.Size = new System.Drawing.Size(91, 23);
            this.FlyingButton.TabIndex = 0;
            this.FlyingButton.Text = "Disconnected";
            this.FlyingButton.UseVisualStyleBackColor = false;
            this.FlyingButton.Click += new System.EventHandler(this.FlyingButton_Click);
            // 
            // serialPort1
            // 
            this.serialPort1.DataReceived += new System.IO.Ports.SerialDataReceivedEventHandler(this.serialPort1_DataReceived);
            // 
            // RollSum
            // 
            this.RollSum.Location = new System.Drawing.Point(40, 23);
            this.RollSum.Name = "RollSum";
            this.RollSum.ReadOnly = true;
            this.RollSum.Size = new System.Drawing.Size(32, 20);
            this.RollSum.TabIndex = 5;
            this.RollSum.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // PitchSum
            // 
            this.PitchSum.Location = new System.Drawing.Point(39, 49);
            this.PitchSum.Name = "PitchSum";
            this.PitchSum.ReadOnly = true;
            this.PitchSum.Size = new System.Drawing.Size(32, 20);
            this.PitchSum.TabIndex = 6;
            this.PitchSum.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // BatteryVolts
            // 
            this.BatteryVolts.Location = new System.Drawing.Point(53, 19);
            this.BatteryVolts.Name = "BatteryVolts";
            this.BatteryVolts.ReadOnly = true;
            this.BatteryVolts.Size = new System.Drawing.Size(32, 20);
            this.BatteryVolts.TabIndex = 8;
            this.BatteryVolts.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // BatteryCurrent
            // 
            this.BatteryCurrent.Location = new System.Drawing.Point(53, 45);
            this.BatteryCurrent.Name = "BatteryCurrent";
            this.BatteryCurrent.ReadOnly = true;
            this.BatteryCurrent.Size = new System.Drawing.Size(32, 20);
            this.BatteryCurrent.TabIndex = 9;
            this.BatteryCurrent.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RCGlitches
            // 
            this.RCGlitches.Location = new System.Drawing.Point(57, 18);
            this.RCGlitches.Name = "RCGlitches";
            this.RCGlitches.ReadOnly = true;
            this.RCGlitches.Size = new System.Drawing.Size(46, 20);
            this.RCGlitches.TabIndex = 10;
            this.RCGlitches.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DesiredThrottle
            // 
            this.DesiredThrottle.Location = new System.Drawing.Point(40, 19);
            this.DesiredThrottle.Name = "DesiredThrottle";
            this.DesiredThrottle.ReadOnly = true;
            this.DesiredThrottle.Size = new System.Drawing.Size(32, 20);
            this.DesiredThrottle.TabIndex = 11;
            this.DesiredThrottle.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DesiredRoll
            // 
            this.DesiredRoll.Location = new System.Drawing.Point(40, 45);
            this.DesiredRoll.Name = "DesiredRoll";
            this.DesiredRoll.ReadOnly = true;
            this.DesiredRoll.Size = new System.Drawing.Size(32, 20);
            this.DesiredRoll.TabIndex = 12;
            this.DesiredRoll.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DesiredPitch
            // 
            this.DesiredPitch.Location = new System.Drawing.Point(40, 71);
            this.DesiredPitch.Name = "DesiredPitch";
            this.DesiredPitch.ReadOnly = true;
            this.DesiredPitch.Size = new System.Drawing.Size(32, 20);
            this.DesiredPitch.TabIndex = 13;
            this.DesiredPitch.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DesiredYaw
            // 
            this.DesiredYaw.Location = new System.Drawing.Point(40, 97);
            this.DesiredYaw.Name = "DesiredYaw";
            this.DesiredYaw.ReadOnly = true;
            this.DesiredYaw.Size = new System.Drawing.Size(32, 20);
            this.DesiredYaw.TabIndex = 14;
            this.DesiredYaw.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // YawRate
            // 
            this.YawRate.Location = new System.Drawing.Point(40, 71);
            this.YawRate.Name = "YawRate";
            this.YawRate.ReadOnly = true;
            this.YawRate.Size = new System.Drawing.Size(32, 20);
            this.YawRate.TabIndex = 17;
            this.YawRate.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // PitchRate
            // 
            this.PitchRate.Location = new System.Drawing.Point(40, 45);
            this.PitchRate.Name = "PitchRate";
            this.PitchRate.ReadOnly = true;
            this.PitchRate.Size = new System.Drawing.Size(32, 20);
            this.PitchRate.TabIndex = 16;
            this.PitchRate.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RollRate
            // 
            this.RollRate.Location = new System.Drawing.Point(40, 16);
            this.RollRate.Name = "RollRate";
            this.RollRate.ReadOnly = true;
            this.RollRate.Size = new System.Drawing.Size(32, 20);
            this.RollRate.TabIndex = 15;
            this.RollRate.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // LRAcc
            // 
            this.LRAcc.Location = new System.Drawing.Point(39, 19);
            this.LRAcc.Name = "LRAcc";
            this.LRAcc.ReadOnly = true;
            this.LRAcc.Size = new System.Drawing.Size(32, 20);
            this.LRAcc.TabIndex = 21;
            this.LRAcc.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // YawSum
            // 
            this.YawSum.Location = new System.Drawing.Point(39, 75);
            this.YawSum.Name = "YawSum";
            this.YawSum.ReadOnly = true;
            this.YawSum.Size = new System.Drawing.Size(32, 20);
            this.YawSum.TabIndex = 20;
            this.YawSum.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // DUAcc
            // 
            this.DUAcc.Location = new System.Drawing.Point(39, 71);
            this.DUAcc.Name = "DUAcc";
            this.DUAcc.ReadOnly = true;
            this.DUAcc.Size = new System.Drawing.Size(32, 20);
            this.DUAcc.TabIndex = 19;
            this.DUAcc.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // FBAcc
            // 
            this.FBAcc.Location = new System.Drawing.Point(39, 45);
            this.FBAcc.Name = "FBAcc";
            this.FBAcc.ReadOnly = true;
            this.FBAcc.Size = new System.Drawing.Size(32, 20);
            this.FBAcc.TabIndex = 18;
            this.FBAcc.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // VoltsLabel
            // 
            this.VoltsLabel.AutoSize = true;
            this.VoltsLabel.Location = new System.Drawing.Point(6, 22);
            this.VoltsLabel.Name = "VoltsLabel";
            this.VoltsLabel.Size = new System.Drawing.Size(30, 13);
            this.VoltsLabel.TabIndex = 22;
            this.VoltsLabel.Text = "Volts";
            // 
            // LRAccLabel
            // 
            this.LRAccLabel.AutoSize = true;
            this.LRAccLabel.Location = new System.Drawing.Point(5, 25);
            this.LRAccLabel.Name = "LRAccLabel";
            this.LRAccLabel.Size = new System.Drawing.Size(21, 13);
            this.LRAccLabel.TabIndex = 26;
            this.LRAccLabel.Text = "LR";
            // 
            // FBAccLabel
            // 
            this.FBAccLabel.AutoSize = true;
            this.FBAccLabel.Location = new System.Drawing.Point(5, 51);
            this.FBAccLabel.Name = "FBAccLabel";
            this.FBAccLabel.Size = new System.Drawing.Size(20, 13);
            this.FBAccLabel.TabIndex = 27;
            this.FBAccLabel.Text = "FB";
            // 
            // DUAccLabel
            // 
            this.DUAccLabel.AutoSize = true;
            this.DUAccLabel.Location = new System.Drawing.Point(5, 72);
            this.DUAccLabel.Name = "DUAccLabel";
            this.DUAccLabel.Size = new System.Drawing.Size(23, 13);
            this.DUAccLabel.TabIndex = 28;
            this.DUAccLabel.Text = "DU";
            // 
            // DesiredYawLabel
            // 
            this.DesiredYawLabel.AutoSize = true;
            this.DesiredYawLabel.Location = new System.Drawing.Point(6, 100);
            this.DesiredYawLabel.Name = "DesiredYawLabel";
            this.DesiredYawLabel.Size = new System.Drawing.Size(28, 13);
            this.DesiredYawLabel.TabIndex = 34;
            this.DesiredYawLabel.Text = "Yaw";
            // 
            // DesiredPitchLabel
            // 
            this.DesiredPitchLabel.AutoSize = true;
            this.DesiredPitchLabel.Location = new System.Drawing.Point(6, 74);
            this.DesiredPitchLabel.Name = "DesiredPitchLabel";
            this.DesiredPitchLabel.Size = new System.Drawing.Size(31, 13);
            this.DesiredPitchLabel.TabIndex = 33;
            this.DesiredPitchLabel.Text = "Pitch";
            // 
            // DesiredRollLabel
            // 
            this.DesiredRollLabel.AutoSize = true;
            this.DesiredRollLabel.Location = new System.Drawing.Point(6, 48);
            this.DesiredRollLabel.Name = "DesiredRollLabel";
            this.DesiredRollLabel.Size = new System.Drawing.Size(25, 13);
            this.DesiredRollLabel.TabIndex = 32;
            this.DesiredRollLabel.Text = "Roll";
            // 
            // DesiredThrottleLabel
            // 
            this.DesiredThrottleLabel.AutoSize = true;
            this.DesiredThrottleLabel.Location = new System.Drawing.Point(6, 22);
            this.DesiredThrottleLabel.Name = "DesiredThrottleLabel";
            this.DesiredThrottleLabel.Size = new System.Drawing.Size(26, 13);
            this.DesiredThrottleLabel.TabIndex = 31;
            this.DesiredThrottleLabel.Text = "Thr.";
            // 
            // RCGlitchesLabel
            // 
            this.RCGlitchesLabel.AutoSize = true;
            this.RCGlitchesLabel.Location = new System.Drawing.Point(6, 21);
            this.RCGlitchesLabel.Name = "RCGlitchesLabel";
            this.RCGlitchesLabel.Size = new System.Drawing.Size(45, 13);
            this.RCGlitchesLabel.TabIndex = 30;
            this.RCGlitchesLabel.Text = "Glitches";
            // 
            // CurrentLabel
            // 
            this.CurrentLabel.AutoSize = true;
            this.CurrentLabel.Location = new System.Drawing.Point(6, 48);
            this.CurrentLabel.Name = "CurrentLabel";
            this.CurrentLabel.Size = new System.Drawing.Size(33, 13);
            this.CurrentLabel.TabIndex = 29;
            this.CurrentLabel.Text = "Amps";
            // 
            // GPSHDilute
            // 
            this.GPSHDilute.Location = new System.Drawing.Point(52, 71);
            this.GPSHDilute.Name = "GPSHDilute";
            this.GPSHDilute.ReadOnly = true;
            this.GPSHDilute.Size = new System.Drawing.Size(32, 20);
            this.GPSHDilute.TabIndex = 44;
            this.GPSHDilute.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RangefinderAltitude
            // 
            this.RangefinderAltitude.Location = new System.Drawing.Point(122, 29);
            this.RangefinderAltitude.Name = "RangefinderAltitude";
            this.RangefinderAltitude.ReadOnly = true;
            this.RangefinderAltitude.Size = new System.Drawing.Size(32, 20);
            this.RangefinderAltitude.TabIndex = 43;
            this.RangefinderAltitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RangefinderROC
            // 
            this.RangefinderROC.Location = new System.Drawing.Point(122, 55);
            this.RangefinderROC.Name = "RangefinderROC";
            this.RangefinderROC.ReadOnly = true;
            this.RangefinderROC.Size = new System.Drawing.Size(32, 20);
            this.RangefinderROC.TabIndex = 47;
            this.RangefinderROC.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RelBaroAltitude
            // 
            this.RelBaroAltitude.Location = new System.Drawing.Point(84, 29);
            this.RelBaroAltitude.Name = "RelBaroAltitude";
            this.RelBaroAltitude.ReadOnly = true;
            this.RelBaroAltitude.Size = new System.Drawing.Size(32, 20);
            this.RelBaroAltitude.TabIndex = 46;
            this.RelBaroAltitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // GPSFix
            // 
            this.GPSFix.Location = new System.Drawing.Point(52, 45);
            this.GPSFix.Name = "GPSFix";
            this.GPSFix.ReadOnly = true;
            this.GPSFix.Size = new System.Drawing.Size(32, 20);
            this.GPSFix.TabIndex = 51;
            this.GPSFix.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // BaroROC
            // 
            this.BaroROC.Location = new System.Drawing.Point(84, 55);
            this.BaroROC.Name = "BaroROC";
            this.BaroROC.ReadOnly = true;
            this.BaroROC.Size = new System.Drawing.Size(32, 20);
            this.BaroROC.TabIndex = 49;
            this.BaroROC.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // GPSNoOfSats
            // 
            this.GPSNoOfSats.Location = new System.Drawing.Point(52, 19);
            this.GPSNoOfSats.Name = "GPSNoOfSats";
            this.GPSNoOfSats.ReadOnly = true;
            this.GPSNoOfSats.Size = new System.Drawing.Size(32, 20);
            this.GPSNoOfSats.TabIndex = 54;
            this.GPSNoOfSats.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // FailState
            // 
            this.FailState.Location = new System.Drawing.Point(239, 124);
            this.FailState.Name = "FailState";
            this.FailState.ReadOnly = true;
            this.FailState.Size = new System.Drawing.Size(73, 20);
            this.FailState.TabIndex = 53;
            this.FailState.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // GPSRelAltitude
            // 
            this.GPSRelAltitude.Location = new System.Drawing.Point(46, 29);
            this.GPSRelAltitude.Name = "GPSRelAltitude";
            this.GPSRelAltitude.ReadOnly = true;
            this.GPSRelAltitude.Size = new System.Drawing.Size(32, 20);
            this.GPSRelAltitude.TabIndex = 58;
            this.GPSRelAltitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // GPSROC
            // 
            this.GPSROC.Location = new System.Drawing.Point(46, 55);
            this.GPSROC.Name = "GPSROC";
            this.GPSROC.ReadOnly = true;
            this.GPSROC.Size = new System.Drawing.Size(32, 20);
            this.GPSROC.TabIndex = 59;
            this.GPSROC.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RelAltitudeLabel
            // 
            this.RelAltitudeLabel.AutoSize = true;
            this.RelAltitudeLabel.Location = new System.Drawing.Point(9, 29);
            this.RelAltitudeLabel.Name = "RelAltitudeLabel";
            this.RelAltitudeLabel.Size = new System.Drawing.Size(22, 13);
            this.RelAltitudeLabel.TabIndex = 62;
            this.RelAltitudeLabel.Text = "Alt.";
            // 
            // ROCLabel
            // 
            this.ROCLabel.AutoSize = true;
            this.ROCLabel.Location = new System.Drawing.Point(6, 58);
            this.ROCLabel.Name = "ROCLabel";
            this.ROCLabel.Size = new System.Drawing.Size(30, 13);
            this.ROCLabel.TabIndex = 65;
            this.ROCLabel.Text = "ROC";
            // 
            // HDiluteLabel
            // 
            this.HDiluteLabel.AutoSize = true;
            this.HDiluteLabel.Location = new System.Drawing.Point(6, 74);
            this.HDiluteLabel.Name = "HDiluteLabel";
            this.HDiluteLabel.Size = new System.Drawing.Size(42, 13);
            this.HDiluteLabel.TabIndex = 66;
            this.HDiluteLabel.Text = "HDilute";
            // 
            // RangefinderAltLabel
            // 
            this.RangefinderAltLabel.AutoSize = true;
            this.RangefinderAltLabel.Location = new System.Drawing.Point(132, 13);
            this.RangefinderAltLabel.Name = "RangefinderAltLabel";
            this.RangefinderAltLabel.Size = new System.Drawing.Size(21, 13);
            this.RangefinderAltLabel.TabIndex = 67;
            this.RangefinderAltLabel.Text = "RF";
            // 
            // GPSFixLabel
            // 
            this.GPSFixLabel.AutoSize = true;
            this.GPSFixLabel.Location = new System.Drawing.Point(6, 48);
            this.GPSFixLabel.Name = "GPSFixLabel";
            this.GPSFixLabel.Size = new System.Drawing.Size(20, 13);
            this.GPSFixLabel.TabIndex = 69;
            this.GPSFixLabel.Text = "Fix";
            // 
            // GPSNoOfSatsLabel
            // 
            this.GPSNoOfSatsLabel.AutoSize = true;
            this.GPSNoOfSatsLabel.Location = new System.Drawing.Point(6, 22);
            this.GPSNoOfSatsLabel.Name = "GPSNoOfSatsLabel";
            this.GPSNoOfSatsLabel.Size = new System.Drawing.Size(31, 13);
            this.GPSNoOfSatsLabel.TabIndex = 70;
            this.GPSNoOfSatsLabel.Text = "Sats.";
            // 
            // FailStateLabel
            // 
            this.FailStateLabel.AutoSize = true;
            this.FailStateLabel.Location = new System.Drawing.Point(184, 127);
            this.FailStateLabel.Name = "FailStateLabel";
            this.FailStateLabel.Size = new System.Drawing.Size(51, 13);
            this.FailStateLabel.TabIndex = 71;
            this.FailStateLabel.Text = "Fail State";
            // 
            // GPSAltLabel
            // 
            this.GPSAltLabel.AutoSize = true;
            this.GPSAltLabel.Location = new System.Drawing.Point(48, 13);
            this.GPSAltLabel.Name = "GPSAltLabel";
            this.GPSAltLabel.Size = new System.Drawing.Size(29, 13);
            this.GPSAltLabel.TabIndex = 74;
            this.GPSAltLabel.Text = "GPS";
            // 
            // BaroAltitudeLabel
            // 
            this.BaroAltitudeLabel.AutoSize = true;
            this.BaroAltitudeLabel.Location = new System.Drawing.Point(87, 13);
            this.BaroAltitudeLabel.Name = "BaroAltitudeLabel";
            this.BaroAltitudeLabel.Size = new System.Drawing.Size(29, 13);
            this.BaroAltitudeLabel.TabIndex = 75;
            this.BaroAltitudeLabel.Text = "Baro";
            // 
            // YawRateLabel
            // 
            this.YawRateLabel.AutoSize = true;
            this.YawRateLabel.Location = new System.Drawing.Point(6, 74);
            this.YawRateLabel.Name = "YawRateLabel";
            this.YawRateLabel.Size = new System.Drawing.Size(28, 13);
            this.YawRateLabel.TabIndex = 78;
            this.YawRateLabel.Text = "Yaw";
            // 
            // PitchRateLabel
            // 
            this.PitchRateLabel.AutoSize = true;
            this.PitchRateLabel.Location = new System.Drawing.Point(6, 48);
            this.PitchRateLabel.Name = "PitchRateLabel";
            this.PitchRateLabel.Size = new System.Drawing.Size(31, 13);
            this.PitchRateLabel.TabIndex = 77;
            this.PitchRateLabel.Text = "Pitch";
            // 
            // RollRateLabel
            // 
            this.RollRateLabel.AutoSize = true;
            this.RollRateLabel.Location = new System.Drawing.Point(6, 22);
            this.RollRateLabel.Name = "RollRateLabel";
            this.RollRateLabel.Size = new System.Drawing.Size(25, 13);
            this.RollRateLabel.TabIndex = 76;
            this.RollRateLabel.Text = "Roll";
            // 
            // label5
            // 
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(6, 78);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(28, 13);
            this.label5.TabIndex = 81;
            this.label5.Text = "Yaw";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(6, 52);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(31, 13);
            this.label6.TabIndex = 80;
            this.label6.Text = "Pitch";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(6, 26);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(25, 13);
            this.label7.TabIndex = 79;
            this.label7.Text = "Roll";
            // 
            // RxTypeErr
            // 
            this.RxTypeErr.Location = new System.Drawing.Point(65, 18);
            this.RxTypeErr.Name = "RxTypeErr";
            this.RxTypeErr.ReadOnly = true;
            this.RxTypeErr.Size = new System.Drawing.Size(38, 20);
            this.RxTypeErr.TabIndex = 82;
            this.RxTypeErr.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RxCSumErr
            // 
            this.RxCSumErr.Location = new System.Drawing.Point(65, 70);
            this.RxCSumErr.Name = "RxCSumErr";
            this.RxCSumErr.ReadOnly = true;
            this.RxCSumErr.Size = new System.Drawing.Size(38, 20);
            this.RxCSumErr.TabIndex = 83;
            this.RxCSumErr.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RxLenErr
            // 
            this.RxLenErr.Location = new System.Drawing.Point(65, 44);
            this.RxLenErr.Name = "RxLenErr";
            this.RxLenErr.ReadOnly = true;
            this.RxLenErr.Size = new System.Drawing.Size(38, 20);
            this.RxLenErr.TabIndex = 84;
            this.RxLenErr.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // RxTypeLabel
            // 
            this.RxTypeLabel.AutoSize = true;
            this.RxTypeLabel.Location = new System.Drawing.Point(6, 22);
            this.RxTypeLabel.Name = "RxTypeLabel";
            this.RxTypeLabel.Size = new System.Drawing.Size(31, 13);
            this.RxTypeLabel.TabIndex = 85;
            this.RxTypeLabel.Text = "Type";
            // 
            // RxCSumLabel
            // 
            this.RxCSumLabel.AutoSize = true;
            this.RxCSumLabel.Location = new System.Drawing.Point(6, 73);
            this.RxCSumLabel.Name = "RxCSumLabel";
            this.RxCSumLabel.Size = new System.Drawing.Size(59, 13);
            this.RxCSumLabel.TabIndex = 86;
            this.RxCSumLabel.Text = "CheckSum";
            // 
            // RxLenLabel
            // 
            this.RxLenLabel.AutoSize = true;
            this.RxLenLabel.Location = new System.Drawing.Point(6, 48);
            this.RxLenLabel.Name = "RxLenLabel";
            this.RxLenLabel.Size = new System.Drawing.Size(40, 13);
            this.RxLenLabel.TabIndex = 87;
            this.RxLenLabel.Text = "Length";
            // 
            // FlightState
            // 
            this.FlightState.Location = new System.Drawing.Point(239, 98);
            this.FlightState.Name = "FlightState";
            this.FlightState.ReadOnly = true;
            this.FlightState.Size = new System.Drawing.Size(73, 20);
            this.FlightState.TabIndex = 89;
            this.FlightState.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // FlightStateLabel
            // 
            this.FlightStateLabel.AutoSize = true;
            this.FlightStateLabel.Location = new System.Drawing.Point(184, 101);
            this.FlightStateLabel.Name = "FlightStateLabel";
            this.FlightStateLabel.Size = new System.Drawing.Size(32, 13);
            this.FlightStateLabel.TabIndex = 90;
            this.FlightStateLabel.Text = "State";
            // 
            // GyroGroupBox
            // 
            this.GyroGroupBox.Controls.Add(this.RollRate);
            this.GyroGroupBox.Controls.Add(this.PitchRate);
            this.GyroGroupBox.Controls.Add(this.YawRate);
            this.GyroGroupBox.Controls.Add(this.RollRateLabel);
            this.GyroGroupBox.Controls.Add(this.PitchRateLabel);
            this.GyroGroupBox.Controls.Add(this.YawRateLabel);
            this.GyroGroupBox.Location = new System.Drawing.Point(12, 168);
            this.GyroGroupBox.Name = "GyroGroupBox";
            this.GyroGroupBox.Size = new System.Drawing.Size(77, 99);
            this.GyroGroupBox.TabIndex = 92;
            this.GyroGroupBox.TabStop = false;
            this.GyroGroupBox.Text = "Gyros";
            // 
            // ControlsGroupBox
            // 
            this.ControlsGroupBox.Controls.Add(this.DesiredYaw);
            this.ControlsGroupBox.Controls.Add(this.DesiredRoll);
            this.ControlsGroupBox.Controls.Add(this.DesiredPitch);
            this.ControlsGroupBox.Controls.Add(this.DesiredRollLabel);
            this.ControlsGroupBox.Controls.Add(this.DesiredPitchLabel);
            this.ControlsGroupBox.Controls.Add(this.DesiredYawLabel);
            this.ControlsGroupBox.Controls.Add(this.DesiredThrottle);
            this.ControlsGroupBox.Controls.Add(this.DesiredThrottleLabel);
            this.ControlsGroupBox.Location = new System.Drawing.Point(12, 38);
            this.ControlsGroupBox.Name = "ControlsGroupBox";
            this.ControlsGroupBox.Size = new System.Drawing.Size(77, 124);
            this.ControlsGroupBox.TabIndex = 93;
            this.ControlsGroupBox.TabStop = false;
            this.ControlsGroupBox.Text = "Controls (%)";
            // 
            // AttitudeGroupBox
            // 
            this.AttitudeGroupBox.Controls.Add(this.RollSum);
            this.AttitudeGroupBox.Controls.Add(this.PitchSum);
            this.AttitudeGroupBox.Controls.Add(this.YawSum);
            this.AttitudeGroupBox.Controls.Add(this.label7);
            this.AttitudeGroupBox.Controls.Add(this.label6);
            this.AttitudeGroupBox.Controls.Add(this.label5);
            this.AttitudeGroupBox.Location = new System.Drawing.Point(95, 60);
            this.AttitudeGroupBox.Name = "AttitudeGroupBox";
            this.AttitudeGroupBox.Size = new System.Drawing.Size(77, 102);
            this.AttitudeGroupBox.TabIndex = 94;
            this.AttitudeGroupBox.TabStop = false;
            this.AttitudeGroupBox.Text = "Attitude";
            // 
            // AccelerationsGroupBox
            // 
            this.AccelerationsGroupBox.Controls.Add(this.LRAcc);
            this.AccelerationsGroupBox.Controls.Add(this.FBAcc);
            this.AccelerationsGroupBox.Controls.Add(this.DUAcc);
            this.AccelerationsGroupBox.Controls.Add(this.LRAccLabel);
            this.AccelerationsGroupBox.Controls.Add(this.FBAccLabel);
            this.AccelerationsGroupBox.Controls.Add(this.DUAccLabel);
            this.AccelerationsGroupBox.Location = new System.Drawing.Point(95, 168);
            this.AccelerationsGroupBox.Name = "AccelerationsGroupBox";
            this.AccelerationsGroupBox.Size = new System.Drawing.Size(77, 99);
            this.AccelerationsGroupBox.TabIndex = 95;
            this.AccelerationsGroupBox.TabStop = false;
            this.AccelerationsGroupBox.Text = "Accel. (G)";
            // 
            // CommsGroupBox
            // 
            this.CommsGroupBox.Controls.Add(this.RxTypeErr);
            this.CommsGroupBox.Controls.Add(this.RxCSumErr);
            this.CommsGroupBox.Controls.Add(this.RxLenErr);
            this.CommsGroupBox.Controls.Add(this.RxTypeLabel);
            this.CommsGroupBox.Controls.Add(this.RxCSumLabel);
            this.CommsGroupBox.Controls.Add(this.RxLenLabel);
            this.CommsGroupBox.Location = new System.Drawing.Point(12, 425);
            this.CommsGroupBox.Name = "CommsGroupBox";
            this.CommsGroupBox.Size = new System.Drawing.Size(109, 98);
            this.CommsGroupBox.TabIndex = 96;
            this.CommsGroupBox.TabStop = false;
            this.CommsGroupBox.Text = "Telemetry Errors";
            // 
            // RCGroupBox
            // 
            this.RCGroupBox.Controls.Add(this.RCGlitches);
            this.RCGroupBox.Controls.Add(this.RCGlitchesLabel);
            this.RCGroupBox.Location = new System.Drawing.Point(203, 48);
            this.RCGroupBox.Name = "RCGroupBox";
            this.RCGroupBox.Size = new System.Drawing.Size(109, 44);
            this.RCGroupBox.TabIndex = 97;
            this.RCGroupBox.TabStop = false;
            this.RCGroupBox.Text = "RC Status";
            // 
            // BatteryGroupBox
            // 
            this.BatteryGroupBox.Controls.Add(this.BatteryVolts);
            this.BatteryGroupBox.Controls.Add(this.BatteryCurrent);
            this.BatteryGroupBox.Controls.Add(this.VoltsLabel);
            this.BatteryGroupBox.Controls.Add(this.CurrentLabel);
            this.BatteryGroupBox.Location = new System.Drawing.Point(227, 426);
            this.BatteryGroupBox.Name = "BatteryGroupBox";
            this.BatteryGroupBox.Size = new System.Drawing.Size(90, 75);
            this.BatteryGroupBox.TabIndex = 98;
            this.BatteryGroupBox.TabStop = false;
            this.BatteryGroupBox.Text = "Battery";
            // 
            // flowLayoutPanel1
            // 
            this.flowLayoutPanel1.Controls.Add(this.BaroAltValidBox);
            this.flowLayoutPanel1.Controls.Add(this.RangefinderAltValidBox);
            this.flowLayoutPanel1.Controls.Add(this.GPSValidBox);
            this.flowLayoutPanel1.Controls.Add(this.UsingGPSAltBox);
            this.flowLayoutPanel1.Controls.Add(this.RTHAltHoldBox);
            this.flowLayoutPanel1.Controls.Add(this.TurnToWPBox);
            this.flowLayoutPanel1.Controls.Add(this.UseRTHAutoDescendBox);
            this.flowLayoutPanel1.Controls.Add(this.NavValidBox);
            this.flowLayoutPanel1.Controls.Add(this.MotorsArmedBox);
            this.flowLayoutPanel1.Controls.Add(this.ThrottleMovingBox);
            this.flowLayoutPanel1.Controls.Add(this.HoveringBox);
            this.flowLayoutPanel1.Controls.Add(this.AttitudeHoldBox);
            this.flowLayoutPanel1.Controls.Add(this.NearLevelBox);
            this.flowLayoutPanel1.Controls.Add(this.NavigateBox);
            this.flowLayoutPanel1.Controls.Add(this.ReturnHomeBox);
            this.flowLayoutPanel1.Controls.Add(this.UsingRangefinderBox);
            this.flowLayoutPanel1.Controls.Add(this.CloseProximityBox);
            this.flowLayoutPanel1.Controls.Add(this.ProximityBox);
            this.flowLayoutPanel1.Controls.Add(this.LostModelBox);
            this.flowLayoutPanel1.Controls.Add(this.LowBatteryBox);
            this.flowLayoutPanel1.Controls.Add(this.CompassFailBox);
            this.flowLayoutPanel1.Controls.Add(this.BaroFailBox);
            this.flowLayoutPanel1.Controls.Add(this.AccFailBox);
            this.flowLayoutPanel1.Controls.Add(this.GPSFailBox);
            this.flowLayoutPanel1.Location = new System.Drawing.Point(6, 19);
            this.flowLayoutPanel1.Name = "flowLayoutPanel1";
            this.flowLayoutPanel1.Size = new System.Drawing.Size(237, 317);
            this.flowLayoutPanel1.TabIndex = 99;
            // 
            // BaroAltValidBox
            // 
            this.BaroAltValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.BaroAltValidBox.Location = new System.Drawing.Point(3, 3);
            this.BaroAltValidBox.Name = "BaroAltValidBox";
            this.BaroAltValidBox.ReadOnly = true;
            this.BaroAltValidBox.Size = new System.Drawing.Size(109, 20);
            this.BaroAltValidBox.TabIndex = 121;
            this.BaroAltValidBox.Text = "Baro Alt Valid";
            this.BaroAltValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // RangefinderAltValidBox
            // 
            this.RangefinderAltValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.RangefinderAltValidBox.Location = new System.Drawing.Point(118, 3);
            this.RangefinderAltValidBox.Name = "RangefinderAltValidBox";
            this.RangefinderAltValidBox.ReadOnly = true;
            this.RangefinderAltValidBox.Size = new System.Drawing.Size(109, 20);
            this.RangefinderAltValidBox.TabIndex = 122;
            this.RangefinderAltValidBox.Text = "Rangefinder Valid";
            this.RangefinderAltValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // GPSValidBox
            // 
            this.GPSValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.GPSValidBox.Location = new System.Drawing.Point(3, 29);
            this.GPSValidBox.Name = "GPSValidBox";
            this.GPSValidBox.ReadOnly = true;
            this.GPSValidBox.Size = new System.Drawing.Size(109, 20);
            this.GPSValidBox.TabIndex = 105;
            this.GPSValidBox.Text = "GPS Valid";
            this.GPSValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // UsingGPSAltBox
            // 
            this.UsingGPSAltBox.BackColor = System.Drawing.SystemColors.Window;
            this.UsingGPSAltBox.Location = new System.Drawing.Point(118, 29);
            this.UsingGPSAltBox.Name = "UsingGPSAltBox";
            this.UsingGPSAltBox.ReadOnly = true;
            this.UsingGPSAltBox.Size = new System.Drawing.Size(109, 20);
            this.UsingGPSAltBox.TabIndex = 119;
            this.UsingGPSAltBox.Text = "Using GPS Alt";
            this.UsingGPSAltBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // RTHAltHoldBox
            // 
            this.RTHAltHoldBox.BackColor = System.Drawing.SystemColors.Window;
            this.RTHAltHoldBox.Location = new System.Drawing.Point(3, 55);
            this.RTHAltHoldBox.Name = "RTHAltHoldBox";
            this.RTHAltHoldBox.ReadOnly = true;
            this.RTHAltHoldBox.Size = new System.Drawing.Size(109, 20);
            this.RTHAltHoldBox.TabIndex = 100;
            this.RTHAltHoldBox.Text = "RTH Altitude Hold";
            this.RTHAltHoldBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // TurnToWPBox
            // 
            this.TurnToWPBox.BackColor = System.Drawing.SystemColors.Window;
            this.TurnToWPBox.Location = new System.Drawing.Point(118, 55);
            this.TurnToWPBox.Name = "TurnToWPBox";
            this.TurnToWPBox.ReadOnly = true;
            this.TurnToWPBox.Size = new System.Drawing.Size(109, 20);
            this.TurnToWPBox.TabIndex = 101;
            this.TurnToWPBox.Text = "Turn to WP";
            this.TurnToWPBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // UseRTHAutoDescendBox
            // 
            this.UseRTHAutoDescendBox.BackColor = System.Drawing.SystemColors.Window;
            this.UseRTHAutoDescendBox.Location = new System.Drawing.Point(3, 81);
            this.UseRTHAutoDescendBox.Name = "UseRTHAutoDescendBox";
            this.UseRTHAutoDescendBox.ReadOnly = true;
            this.UseRTHAutoDescendBox.Size = new System.Drawing.Size(109, 20);
            this.UseRTHAutoDescendBox.TabIndex = 120;
            this.UseRTHAutoDescendBox.Text = "Auto Land Enabled";
            this.UseRTHAutoDescendBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NavValidBox
            // 
            this.NavValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.NavValidBox.Location = new System.Drawing.Point(118, 81);
            this.NavValidBox.Name = "NavValidBox";
            this.NavValidBox.ReadOnly = true;
            this.NavValidBox.Size = new System.Drawing.Size(109, 20);
            this.NavValidBox.TabIndex = 107;
            this.NavValidBox.Text = "Navigation Enabled";
            this.NavValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ThrottleMovingBox
            // 
            this.ThrottleMovingBox.BackColor = System.Drawing.SystemColors.Window;
            this.ThrottleMovingBox.Location = new System.Drawing.Point(118, 107);
            this.ThrottleMovingBox.Name = "ThrottleMovingBox";
            this.ThrottleMovingBox.ReadOnly = true;
            this.ThrottleMovingBox.Size = new System.Drawing.Size(109, 20);
            this.ThrottleMovingBox.TabIndex = 113;
            this.ThrottleMovingBox.Text = "Throttle Moving";
            this.ThrottleMovingBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // HoveringBox
            // 
            this.HoveringBox.BackColor = System.Drawing.SystemColors.Window;
            this.HoveringBox.Location = new System.Drawing.Point(3, 133);
            this.HoveringBox.Name = "HoveringBox";
            this.HoveringBox.ReadOnly = true;
            this.HoveringBox.Size = new System.Drawing.Size(109, 20);
            this.HoveringBox.TabIndex = 114;
            this.HoveringBox.Text = "Hovering";
            this.HoveringBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // AttitudeHoldBox
            // 
            this.AttitudeHoldBox.BackColor = System.Drawing.SystemColors.Window;
            this.AttitudeHoldBox.Location = new System.Drawing.Point(118, 133);
            this.AttitudeHoldBox.Name = "AttitudeHoldBox";
            this.AttitudeHoldBox.ReadOnly = true;
            this.AttitudeHoldBox.Size = new System.Drawing.Size(109, 20);
            this.AttitudeHoldBox.TabIndex = 112;
            this.AttitudeHoldBox.Text = "Roll&Pitch Hold";
            this.AttitudeHoldBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NavigateBox
            // 
            this.NavigateBox.BackColor = System.Drawing.SystemColors.Window;
            this.NavigateBox.Location = new System.Drawing.Point(118, 159);
            this.NavigateBox.Name = "NavigateBox";
            this.NavigateBox.ReadOnly = true;
            this.NavigateBox.Size = new System.Drawing.Size(109, 20);
            this.NavigateBox.TabIndex = 115;
            this.NavigateBox.Text = "Navigate";
            this.NavigateBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ReturnHomeBox
            // 
            this.ReturnHomeBox.BackColor = System.Drawing.SystemColors.Window;
            this.ReturnHomeBox.Location = new System.Drawing.Point(3, 185);
            this.ReturnHomeBox.Name = "ReturnHomeBox";
            this.ReturnHomeBox.ReadOnly = true;
            this.ReturnHomeBox.Size = new System.Drawing.Size(109, 20);
            this.ReturnHomeBox.TabIndex = 116;
            this.ReturnHomeBox.Text = "Return Home";
            this.ReturnHomeBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // UsingRangefinderBox
            // 
            this.UsingRangefinderBox.BackColor = System.Drawing.SystemColors.Window;
            this.UsingRangefinderBox.Location = new System.Drawing.Point(118, 185);
            this.UsingRangefinderBox.Name = "UsingRangefinderBox";
            this.UsingRangefinderBox.ReadOnly = true;
            this.UsingRangefinderBox.Size = new System.Drawing.Size(109, 20);
            this.UsingRangefinderBox.TabIndex = 123;
            this.UsingRangefinderBox.Text = "Using Rangefinder";
            this.UsingRangefinderBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // CloseProximityBox
            // 
            this.CloseProximityBox.BackColor = System.Drawing.SystemColors.Window;
            this.CloseProximityBox.Location = new System.Drawing.Point(3, 211);
            this.CloseProximityBox.Name = "CloseProximityBox";
            this.CloseProximityBox.ReadOnly = true;
            this.CloseProximityBox.Size = new System.Drawing.Size(109, 20);
            this.CloseProximityBox.TabIndex = 118;
            this.CloseProximityBox.Text = "Centered";
            this.CloseProximityBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ProximityBox
            // 
            this.ProximityBox.BackColor = System.Drawing.SystemColors.Window;
            this.ProximityBox.Location = new System.Drawing.Point(118, 211);
            this.ProximityBox.Name = "ProximityBox";
            this.ProximityBox.ReadOnly = true;
            this.ProximityBox.Size = new System.Drawing.Size(109, 20);
            this.ProximityBox.TabIndex = 117;
            this.ProximityBox.Text = "WP Achieved";
            this.ProximityBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NearLevelBox
            // 
            this.NearLevelBox.BackColor = System.Drawing.SystemColors.Window;
            this.NearLevelBox.Location = new System.Drawing.Point(3, 159);
            this.NearLevelBox.Name = "NearLevelBox";
            this.NearLevelBox.ReadOnly = true;
            this.NearLevelBox.Size = new System.Drawing.Size(109, 20);
            this.NearLevelBox.TabIndex = 104;
            this.NearLevelBox.Text = "Near Level";
            this.NearLevelBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // LostModelBox
            // 
            this.LostModelBox.BackColor = System.Drawing.SystemColors.Window;
            this.LostModelBox.Location = new System.Drawing.Point(3, 237);
            this.LostModelBox.Name = "LostModelBox";
            this.LostModelBox.ReadOnly = true;
            this.LostModelBox.Size = new System.Drawing.Size(109, 20);
            this.LostModelBox.TabIndex = 103;
            this.LostModelBox.Text = "Lost Model";
            this.LostModelBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // MotorsArmedBox
            // 
            this.MotorsArmedBox.BackColor = System.Drawing.SystemColors.Window;
            this.MotorsArmedBox.Location = new System.Drawing.Point(3, 107);
            this.MotorsArmedBox.Name = "MotorsArmedBox";
            this.MotorsArmedBox.ReadOnly = true;
            this.MotorsArmedBox.Size = new System.Drawing.Size(109, 20);
            this.MotorsArmedBox.TabIndex = 102;
            this.MotorsArmedBox.Text = "Motors Armed";
            this.MotorsArmedBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // LowBatteryBox
            // 
            this.LowBatteryBox.BackColor = System.Drawing.SystemColors.Window;
            this.LowBatteryBox.Location = new System.Drawing.Point(118, 237);
            this.LowBatteryBox.Name = "LowBatteryBox";
            this.LowBatteryBox.ReadOnly = true;
            this.LowBatteryBox.Size = new System.Drawing.Size(109, 20);
            this.LowBatteryBox.TabIndex = 106;
            this.LowBatteryBox.Text = "Low Battery";
            this.LowBatteryBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // CompassFailBox
            // 
            this.CompassFailBox.BackColor = System.Drawing.SystemColors.Window;
            this.CompassFailBox.Location = new System.Drawing.Point(3, 263);
            this.CompassFailBox.Name = "CompassFailBox";
            this.CompassFailBox.ReadOnly = true;
            this.CompassFailBox.Size = new System.Drawing.Size(109, 20);
            this.CompassFailBox.TabIndex = 110;
            this.CompassFailBox.Text = "Compass Fail";
            this.CompassFailBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // BaroFailBox
            // 
            this.BaroFailBox.BackColor = System.Drawing.SystemColors.Window;
            this.BaroFailBox.Location = new System.Drawing.Point(118, 263);
            this.BaroFailBox.Name = "BaroFailBox";
            this.BaroFailBox.ReadOnly = true;
            this.BaroFailBox.Size = new System.Drawing.Size(109, 20);
            this.BaroFailBox.TabIndex = 108;
            this.BaroFailBox.Text = "Barometer Fail";
            this.BaroFailBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // AccFailBox
            // 
            this.AccFailBox.BackColor = System.Drawing.SystemColors.Window;
            this.AccFailBox.Location = new System.Drawing.Point(3, 289);
            this.AccFailBox.Name = "AccFailBox";
            this.AccFailBox.ReadOnly = true;
            this.AccFailBox.Size = new System.Drawing.Size(109, 20);
            this.AccFailBox.TabIndex = 109;
            this.AccFailBox.Text = "Accelerometer Fail";
            this.AccFailBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // GPSFailBox
            // 
            this.GPSFailBox.BackColor = System.Drawing.SystemColors.Window;
            this.GPSFailBox.Location = new System.Drawing.Point(118, 289);
            this.GPSFailBox.Name = "GPSFailBox";
            this.GPSFailBox.ReadOnly = true;
            this.GPSFailBox.Size = new System.Drawing.Size(109, 20);
            this.GPSFailBox.TabIndex = 111;
            this.GPSFailBox.Text = "GPS Fail";
            this.GPSFailBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // FlagsGroupBox
            // 
            this.FlagsGroupBox.Controls.Add(this.flowLayoutPanel1);
            this.FlagsGroupBox.Location = new System.Drawing.Point(587, 63);
            this.FlagsGroupBox.Name = "FlagsGroupBox";
            this.FlagsGroupBox.Size = new System.Drawing.Size(249, 342);
            this.FlagsGroupBox.TabIndex = 100;
            this.FlagsGroupBox.TabStop = false;
            this.FlagsGroupBox.Text = "Flags";
            // 
            // GPSStatBox
            // 
            this.GPSStatBox.Controls.Add(this.GPSNoOfSats);
            this.GPSStatBox.Controls.Add(this.GPSFixLabel);
            this.GPSStatBox.Controls.Add(this.GPSNoOfSatsLabel);
            this.GPSStatBox.Controls.Add(this.GPSFix);
            this.GPSStatBox.Controls.Add(this.GPSHDilute);
            this.GPSStatBox.Controls.Add(this.HDiluteLabel);
            this.GPSStatBox.Location = new System.Drawing.Point(128, 425);
            this.GPSStatBox.Name = "GPSStatBox";
            this.GPSStatBox.Size = new System.Drawing.Size(93, 98);
            this.GPSStatBox.TabIndex = 101;
            this.GPSStatBox.TabStop = false;
            this.GPSStatBox.Text = "GPS Status";
            // 
            // AltitudeGroupBox
            // 
            this.AltitudeGroupBox.Controls.Add(this.GPSROC);
            this.AltitudeGroupBox.Controls.Add(this.ROCLabel);
            this.AltitudeGroupBox.Controls.Add(this.RelAltitudeLabel);
            this.AltitudeGroupBox.Controls.Add(this.RangefinderAltitude);
            this.AltitudeGroupBox.Controls.Add(this.GPSRelAltitude);
            this.AltitudeGroupBox.Controls.Add(this.BaroROC);
            this.AltitudeGroupBox.Controls.Add(this.RangefinderROC);
            this.AltitudeGroupBox.Controls.Add(this.RelBaroAltitude);
            this.AltitudeGroupBox.Controls.Add(this.GPSAltLabel);
            this.AltitudeGroupBox.Controls.Add(this.BaroAltitudeLabel);
            this.AltitudeGroupBox.Controls.Add(this.RangefinderAltLabel);
            this.AltitudeGroupBox.Location = new System.Drawing.Point(12, 273);
            this.AltitudeGroupBox.Name = "AltitudeGroupBox";
            this.AltitudeGroupBox.Size = new System.Drawing.Size(160, 83);
            this.AltitudeGroupBox.TabIndex = 102;
            this.AltitudeGroupBox.TabStop = false;
            this.AltitudeGroupBox.Text = "Altitude";
            // 
            // WayHeadingLabel
            // 
            this.WayHeadingLabel.AutoSize = true;
            this.WayHeadingLabel.Location = new System.Drawing.Point(5, 126);
            this.WayHeadingLabel.Name = "WayHeadingLabel";
            this.WayHeadingLabel.Size = new System.Drawing.Size(72, 13);
            this.WayHeadingLabel.TabIndex = 64;
            this.WayHeadingLabel.Text = "Course (Deg.)";
            // 
            // GPSLongitudeLabel
            // 
            this.GPSLongitudeLabel.AutoSize = true;
            this.GPSLongitudeLabel.Location = new System.Drawing.Point(5, 230);
            this.GPSLongitudeLabel.Name = "GPSLongitudeLabel";
            this.GPSLongitudeLabel.Size = new System.Drawing.Size(54, 13);
            this.GPSLongitudeLabel.TabIndex = 63;
            this.GPSLongitudeLabel.Text = "Longitude";
            // 
            // GPSLatitudeLabel
            // 
            this.GPSLatitudeLabel.AutoSize = true;
            this.GPSLatitudeLabel.Location = new System.Drawing.Point(5, 204);
            this.GPSLatitudeLabel.Name = "GPSLatitudeLabel";
            this.GPSLatitudeLabel.Size = new System.Drawing.Size(45, 13);
            this.GPSLatitudeLabel.TabIndex = 61;
            this.GPSLatitudeLabel.Text = "Latitude";
            // 
            // NavStateLabel
            // 
            this.NavStateLabel.AutoSize = true;
            this.NavStateLabel.Location = new System.Drawing.Point(5, 22);
            this.NavStateLabel.Name = "NavStateLabel";
            this.NavStateLabel.Size = new System.Drawing.Size(55, 13);
            this.NavStateLabel.TabIndex = 72;
            this.NavStateLabel.Text = "Nav State";
            // 
            // GPSVelLabel
            // 
            this.GPSVelLabel.AutoSize = true;
            this.GPSVelLabel.Location = new System.Drawing.Point(5, 100);
            this.GPSVelLabel.Name = "GPSVelLabel";
            this.GPSVelLabel.Size = new System.Drawing.Size(47, 13);
            this.GPSVelLabel.TabIndex = 60;
            this.GPSVelLabel.Text = "GPS Vel";
            // 
            // CurrWPLabel
            // 
            this.CurrWPLabel.AutoSize = true;
            this.CurrWPLabel.Location = new System.Drawing.Point(5, 74);
            this.CurrWPLabel.Name = "CurrWPLabel";
            this.CurrWPLabel.Size = new System.Drawing.Size(32, 13);
            this.CurrWPLabel.TabIndex = 73;
            this.CurrWPLabel.Text = "WP#";
            // 
            // GPSLatitude
            // 
            this.GPSLatitude.Location = new System.Drawing.Point(60, 201);
            this.GPSLatitude.Name = "GPSLatitude";
            this.GPSLatitude.ReadOnly = true;
            this.GPSLatitude.Size = new System.Drawing.Size(73, 20);
            this.GPSLatitude.TabIndex = 57;
            this.GPSLatitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // GPSLongitude
            // 
            this.GPSLongitude.Location = new System.Drawing.Point(60, 227);
            this.GPSLongitude.Name = "GPSLongitude";
            this.GPSLongitude.ReadOnly = true;
            this.GPSLongitude.Size = new System.Drawing.Size(73, 20);
            this.GPSLongitude.TabIndex = 56;
            this.GPSLongitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // NavState
            // 
            this.NavState.Location = new System.Drawing.Point(60, 19);
            this.NavState.Name = "NavState";
            this.NavState.ReadOnly = true;
            this.NavState.Size = new System.Drawing.Size(73, 20);
            this.NavState.TabIndex = 52;
            this.NavState.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // CurrWP
            // 
            this.CurrWP.Location = new System.Drawing.Point(96, 71);
            this.CurrWP.Name = "CurrWP";
            this.CurrWP.ReadOnly = true;
            this.CurrWP.Size = new System.Drawing.Size(37, 20);
            this.CurrWP.TabIndex = 50;
            this.CurrWP.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // WayHeading
            // 
            this.WayHeading.Location = new System.Drawing.Point(96, 123);
            this.WayHeading.Name = "WayHeading";
            this.WayHeading.ReadOnly = true;
            this.WayHeading.Size = new System.Drawing.Size(37, 20);
            this.WayHeading.TabIndex = 48;
            this.WayHeading.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // WPAltitude
            // 
            this.WPAltitude.AutoSize = true;
            this.WPAltitude.Location = new System.Drawing.Point(5, 178);
            this.WPAltitude.Name = "WPAltitude";
            this.WPAltitude.Size = new System.Drawing.Size(47, 13);
            this.WPAltitude.TabIndex = 77;
            this.WPAltitude.Text = "Alt. Error";
            // 
            // GPSVel
            // 
            this.GPSVel.Location = new System.Drawing.Point(96, 97);
            this.GPSVel.Name = "GPSVel";
            this.GPSVel.ReadOnly = true;
            this.GPSVel.Size = new System.Drawing.Size(37, 20);
            this.GPSVel.TabIndex = 45;
            this.GPSVel.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // HeadingLabel
            // 
            this.HeadingLabel.AutoSize = true;
            this.HeadingLabel.Location = new System.Drawing.Point(5, 48);
            this.HeadingLabel.Name = "HeadingLabel";
            this.HeadingLabel.Size = new System.Drawing.Size(82, 13);
            this.HeadingLabel.TabIndex = 41;
            this.HeadingLabel.Text = "Compass (Deg.)";
            // 
            // AltitudeError
            // 
            this.AltitudeError.Location = new System.Drawing.Point(96, 175);
            this.AltitudeError.Name = "AltitudeError";
            this.AltitudeError.ReadOnly = true;
            this.AltitudeError.Size = new System.Drawing.Size(37, 20);
            this.AltitudeError.TabIndex = 76;
            this.AltitudeError.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // Heading
            // 
            this.Heading.Location = new System.Drawing.Point(96, 45);
            this.Heading.Name = "Heading";
            this.Heading.ReadOnly = true;
            this.Heading.Size = new System.Drawing.Size(37, 20);
            this.Heading.TabIndex = 7;
            this.Heading.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // WPDistanceLabel
            // 
            this.WPDistanceLabel.AutoSize = true;
            this.WPDistanceLabel.Location = new System.Drawing.Point(5, 152);
            this.WPDistanceLabel.Name = "WPDistanceLabel";
            this.WPDistanceLabel.Size = new System.Drawing.Size(49, 13);
            this.WPDistanceLabel.TabIndex = 75;
            this.WPDistanceLabel.Text = "Distance";
            // 
            // DistanceToDesired
            // 
            this.DistanceToDesired.Location = new System.Drawing.Point(96, 149);
            this.DistanceToDesired.Name = "DistanceToDesired";
            this.DistanceToDesired.ReadOnly = true;
            this.DistanceToDesired.Size = new System.Drawing.Size(37, 20);
            this.DistanceToDesired.TabIndex = 74;
            this.DistanceToDesired.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // NavGroupBox
            // 
            this.NavGroupBox.Controls.Add(this.DistanceToDesired);
            this.NavGroupBox.Controls.Add(this.NavState);
            this.NavGroupBox.Controls.Add(this.WayHeadingLabel);
            this.NavGroupBox.Controls.Add(this.WPDistanceLabel);
            this.NavGroupBox.Controls.Add(this.GPSLongitudeLabel);
            this.NavGroupBox.Controls.Add(this.GPSLatitudeLabel);
            this.NavGroupBox.Controls.Add(this.Heading);
            this.NavGroupBox.Controls.Add(this.NavStateLabel);
            this.NavGroupBox.Controls.Add(this.GPSVelLabel);
            this.NavGroupBox.Controls.Add(this.AltitudeError);
            this.NavGroupBox.Controls.Add(this.CurrWPLabel);
            this.NavGroupBox.Controls.Add(this.GPSLatitude);
            this.NavGroupBox.Controls.Add(this.GPSLongitude);
            this.NavGroupBox.Controls.Add(this.CurrWP);
            this.NavGroupBox.Controls.Add(this.HeadingLabel);
            this.NavGroupBox.Controls.Add(this.GPSVel);
            this.NavGroupBox.Controls.Add(this.WayHeading);
            this.NavGroupBox.Controls.Add(this.WPAltitude);
            this.NavGroupBox.Location = new System.Drawing.Point(179, 150);
            this.NavGroupBox.Name = "NavGroupBox";
            this.NavGroupBox.Size = new System.Drawing.Size(139, 257);
            this.NavGroupBox.TabIndex = 91;
            this.NavGroupBox.TabStop = false;
            this.NavGroupBox.Text = "Nav";
            // 
            // CurrentAltitude
            // 
            this.CurrentAltitude.BackColor = System.Drawing.SystemColors.Control;
            this.CurrentAltitude.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.CurrentAltitude.Font = new System.Drawing.Font("Microsoft Sans Serif", 30F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.CurrentAltitude.Location = new System.Drawing.Point(372, 37);
            this.CurrentAltitude.Name = "CurrentAltitude";
            this.CurrentAltitude.Size = new System.Drawing.Size(159, 46);
            this.CurrentAltitude.TabIndex = 103;
            this.CurrentAltitude.Text = "-1";
            this.CurrentAltitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(635, 14);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(200, 13);
            this.label1.TabIndex = 104;
            this.label1.Text = "UAVX Groundstation (C) G.K. Egan 2010";
            // 
            // UserAttitudeToDegreesLabel
            // 
            this.UserAttitudeToDegreesLabel.AutoSize = true;
            this.UserAttitudeToDegreesLabel.Location = new System.Drawing.Point(101, 41);
            this.UserAttitudeToDegreesLabel.Name = "UserAttitudeToDegreesLabel";
            this.UserAttitudeToDegreesLabel.Size = new System.Drawing.Size(34, 13);
            this.UserAttitudeToDegreesLabel.TabIndex = 106;
            this.UserAttitudeToDegreesLabel.Text = "Scale";
            // 
            // UserAttitudeToDegrees
            // 
            this.UserAttitudeToDegrees.Location = new System.Drawing.Point(135, 38);
            this.UserAttitudeToDegrees.Name = "UserAttitudeToDegrees";
            this.UserAttitudeToDegrees.Size = new System.Drawing.Size(32, 20);
            this.UserAttitudeToDegrees.TabIndex = 105;
            this.UserAttitudeToDegrees.Text = "35";
            this.UserAttitudeToDegrees.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            this.UserAttitudeToDegrees.TextChanged += new System.EventHandler(this.UserAttitudeToDegrees_TextChanged);
            // 
            // AltitudeSource
            // 
            this.AltitudeSource.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.AltitudeSource.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.AltitudeSource.Location = new System.Drawing.Point(389, 11);
            this.AltitudeSource.Name = "AltitudeSource";
            this.AltitudeSource.ReadOnly = true;
            this.AltitudeSource.Size = new System.Drawing.Size(121, 13);
            this.AltitudeSource.TabIndex = 107;
            this.AltitudeSource.Text = "Barometer";
            this.AltitudeSource.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // GoogleButton
            // 
            this.GoogleButton.BackColor = System.Drawing.Color.Silver;
            this.GoogleButton.ForeColor = System.Drawing.SystemColors.ControlText;
            this.GoogleButton.Location = new System.Drawing.Point(109, 9);
            this.GoogleButton.Name = "GoogleButton";
            this.GoogleButton.Size = new System.Drawing.Size(91, 23);
            this.GoogleButton.TabIndex = 108;
            this.GoogleButton.Text = "Google";
            this.GoogleButton.UseVisualStyleBackColor = false;
            // 
            // textBox1
            // 
            this.textBox1.Location = new System.Drawing.Point(587, 410);
            this.textBox1.Multiline = true;
            this.textBox1.Name = "textBox1";
            this.textBox1.Size = new System.Drawing.Size(248, 91);
            this.textBox1.TabIndex = 110;
            // 
            // LocationBox
            // 
            this.LocationBox.Controls.Add(this.WhereBearingLabel);
            this.LocationBox.Controls.Add(this.WhereDistanceLabel);
            this.LocationBox.Controls.Add(this.WhereDistance);
            this.LocationBox.Controls.Add(this.WhereBearing);
            this.LocationBox.Location = new System.Drawing.Point(12, 362);
            this.LocationBox.Name = "LocationBox";
            this.LocationBox.Size = new System.Drawing.Size(103, 57);
            this.LocationBox.TabIndex = 152;
            this.LocationBox.TabStop = false;
            this.LocationBox.Text = "Where?";
            // 
            // WhereBearingLabel
            // 
            this.WhereBearingLabel.AutoSize = true;
            this.WhereBearingLabel.Location = new System.Drawing.Point(6, 18);
            this.WhereBearingLabel.Name = "WhereBearingLabel";
            this.WhereBearingLabel.Size = new System.Drawing.Size(46, 13);
            this.WhereBearingLabel.TabIndex = 54;
            this.WhereBearingLabel.Text = "Bearing:";
            // 
            // WhereDistanceLabel
            // 
            this.WhereDistanceLabel.AutoSize = true;
            this.WhereDistanceLabel.Location = new System.Drawing.Point(6, 39);
            this.WhereDistanceLabel.Name = "WhereDistanceLabel";
            this.WhereDistanceLabel.Size = new System.Drawing.Size(52, 13);
            this.WhereDistanceLabel.TabIndex = 53;
            this.WhereDistanceLabel.Text = "Distance:";
            // 
            // WhereDistance
            // 
            this.WhereDistance.Location = new System.Drawing.Point(58, 39);
            this.WhereDistance.Name = "WhereDistance";
            this.WhereDistance.Size = new System.Drawing.Size(39, 13);
            this.WhereDistance.TabIndex = 52;
            this.WhereDistance.Text = "0";
            this.WhereDistance.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // WhereBearing
            // 
            this.WhereBearing.Location = new System.Drawing.Point(68, 18);
            this.WhereBearing.Name = "WhereBearing";
            this.WhereBearing.Size = new System.Drawing.Size(29, 12);
            this.WhereBearing.TabIndex = 51;
            this.WhereBearing.Text = "0";
            this.WhereBearing.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // headingIndicatorInstrumentControl1
            // 
            this.headingIndicatorInstrumentControl1.Location = new System.Drawing.Point(369, 349);
            this.headingIndicatorInstrumentControl1.Name = "headingIndicatorInstrumentControl1";
            this.headingIndicatorInstrumentControl1.Size = new System.Drawing.Size(162, 162);
            this.headingIndicatorInstrumentControl1.TabIndex = 4;
            this.headingIndicatorInstrumentControl1.Text = "headingIndicatorInstrumentControl1";
            // 
            // attitudeIndicatorInstrumentControl1
            // 
            this.attitudeIndicatorInstrumentControl1.Location = new System.Drawing.Point(323, 89);
            this.attitudeIndicatorInstrumentControl1.Name = "attitudeIndicatorInstrumentControl1";
            this.attitudeIndicatorInstrumentControl1.Size = new System.Drawing.Size(258, 259);
            this.attitudeIndicatorInstrumentControl1.TabIndex = 3;
            this.attitudeIndicatorInstrumentControl1.Text = "attitudeIndicatorInstrumentControl1";
            // 
            // Altitude
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(845, 522);
            this.Controls.Add(this.LocationBox);
            this.Controls.Add(this.textBox1);
            this.Controls.Add(this.GoogleButton);
            this.Controls.Add(this.AltitudeSource);
            this.Controls.Add(this.UserAttitudeToDegreesLabel);
            this.Controls.Add(this.UserAttitudeToDegrees);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.CurrentAltitude);
            this.Controls.Add(this.AltitudeGroupBox);
            this.Controls.Add(this.GPSStatBox);
            this.Controls.Add(this.FailState);
            this.Controls.Add(this.FlagsGroupBox);
            this.Controls.Add(this.BatteryGroupBox);
            this.Controls.Add(this.RCGroupBox);
            this.Controls.Add(this.AccelerationsGroupBox);
            this.Controls.Add(this.AttitudeGroupBox);
            this.Controls.Add(this.ControlsGroupBox);
            this.Controls.Add(this.GyroGroupBox);
            this.Controls.Add(this.CommsGroupBox);
            this.Controls.Add(this.NavGroupBox);
            this.Controls.Add(this.FlightStateLabel);
            this.Controls.Add(this.FlightState);
            this.Controls.Add(this.headingIndicatorInstrumentControl1);
            this.Controls.Add(this.attitudeIndicatorInstrumentControl1);
            this.Controls.Add(this.FailStateLabel);
            this.Controls.Add(this.FlyingButton);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Altitude";
            this.Text = "UAVXGS";
            this.Load += new System.EventHandler(this.Altitude_Load);
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.Form1_FormClosing);
            this.GyroGroupBox.ResumeLayout(false);
            this.GyroGroupBox.PerformLayout();
            this.ControlsGroupBox.ResumeLayout(false);
            this.ControlsGroupBox.PerformLayout();
            this.AttitudeGroupBox.ResumeLayout(false);
            this.AttitudeGroupBox.PerformLayout();
            this.AccelerationsGroupBox.ResumeLayout(false);
            this.AccelerationsGroupBox.PerformLayout();
            this.CommsGroupBox.ResumeLayout(false);
            this.CommsGroupBox.PerformLayout();
            this.RCGroupBox.ResumeLayout(false);
            this.RCGroupBox.PerformLayout();
            this.BatteryGroupBox.ResumeLayout(false);
            this.BatteryGroupBox.PerformLayout();
            this.flowLayoutPanel1.ResumeLayout(false);
            this.flowLayoutPanel1.PerformLayout();
            this.FlagsGroupBox.ResumeLayout(false);
            this.GPSStatBox.ResumeLayout(false);
            this.GPSStatBox.PerformLayout();
            this.AltitudeGroupBox.ResumeLayout(false);
            this.AltitudeGroupBox.PerformLayout();
            this.NavGroupBox.ResumeLayout(false);
            this.NavGroupBox.PerformLayout();
            this.LocationBox.ResumeLayout(false);
            this.LocationBox.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.Button FlyingButton;
        private System.IO.Ports.SerialPort serialPort1;
        private Instruments.AttitudeIndicatorInstrumentControl attitudeIndicatorInstrumentControl1;
        private Instruments.HeadingIndicatorInstrumentControl headingIndicatorInstrumentControl1;
        private System.Windows.Forms.TextBox RollSum;
        private System.Windows.Forms.TextBox PitchSum;
        private System.Windows.Forms.TextBox BatteryVolts;
        private System.Windows.Forms.TextBox BatteryCurrent;
        private System.Windows.Forms.TextBox RCGlitches;
        private System.Windows.Forms.TextBox DesiredThrottle;
        private System.Windows.Forms.TextBox DesiredRoll;
        private System.Windows.Forms.TextBox DesiredPitch;
        private System.Windows.Forms.TextBox DesiredYaw;
        private System.Windows.Forms.TextBox YawRate;
        private System.Windows.Forms.TextBox PitchRate;
        private System.Windows.Forms.TextBox RollRate;
        private System.Windows.Forms.TextBox LRAcc;
        private System.Windows.Forms.TextBox YawSum;
        private System.Windows.Forms.TextBox DUAcc;
        private System.Windows.Forms.TextBox FBAcc;
        private System.Windows.Forms.Label VoltsLabel;
        private System.Windows.Forms.Label LRAccLabel;
        private System.Windows.Forms.Label FBAccLabel;
        private System.Windows.Forms.Label DUAccLabel;
        private System.Windows.Forms.Label DesiredYawLabel;
        private System.Windows.Forms.Label DesiredPitchLabel;
        private System.Windows.Forms.Label DesiredRollLabel;
        private System.Windows.Forms.Label DesiredThrottleLabel;
        private System.Windows.Forms.Label RCGlitchesLabel;
        private System.Windows.Forms.Label CurrentLabel;
        private System.Windows.Forms.TextBox GPSHDilute;
        private System.Windows.Forms.TextBox RangefinderAltitude;
        private System.Windows.Forms.TextBox RangefinderROC;
        private System.Windows.Forms.TextBox RelBaroAltitude;
        private System.Windows.Forms.TextBox GPSFix;
        private System.Windows.Forms.TextBox BaroROC;
        private System.Windows.Forms.TextBox GPSNoOfSats;
        private System.Windows.Forms.TextBox FailState;
        private System.Windows.Forms.TextBox GPSRelAltitude;
        private System.Windows.Forms.TextBox GPSROC;
        private System.Windows.Forms.Label RelAltitudeLabel;
        private System.Windows.Forms.Label ROCLabel;
        private System.Windows.Forms.Label HDiluteLabel;
        private System.Windows.Forms.Label RangefinderAltLabel;
        private System.Windows.Forms.Label GPSFixLabel;
        private System.Windows.Forms.Label GPSNoOfSatsLabel;
        private System.Windows.Forms.Label FailStateLabel;
        private System.Windows.Forms.Label GPSAltLabel;
        private System.Windows.Forms.Label BaroAltitudeLabel;
        private System.Windows.Forms.Label YawRateLabel;
        private System.Windows.Forms.Label PitchRateLabel;
        private System.Windows.Forms.Label RollRateLabel;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.TextBox RxTypeErr;
        private System.Windows.Forms.TextBox RxCSumErr;
        private System.Windows.Forms.TextBox RxLenErr;
        private System.Windows.Forms.Label RxTypeLabel;
        private System.Windows.Forms.Label RxCSumLabel;
        private System.Windows.Forms.Label RxLenLabel;
        private System.Windows.Forms.TextBox FlightState;
        private System.Windows.Forms.Label FlightStateLabel;
        private System.Windows.Forms.GroupBox GyroGroupBox;
        private System.Windows.Forms.GroupBox ControlsGroupBox;
        private System.Windows.Forms.GroupBox AttitudeGroupBox;
        private System.Windows.Forms.GroupBox AccelerationsGroupBox;
        private System.Windows.Forms.GroupBox CommsGroupBox;
        private System.Windows.Forms.GroupBox RCGroupBox;
        private System.Windows.Forms.GroupBox BatteryGroupBox;
        private System.Windows.Forms.FlowLayoutPanel flowLayoutPanel1;
        private System.Windows.Forms.TextBox RTHAltHoldBox;
        private System.Windows.Forms.TextBox TurnToWPBox;
        private System.Windows.Forms.TextBox MotorsArmedBox;
        private System.Windows.Forms.TextBox LostModelBox;
        private System.Windows.Forms.TextBox NearLevelBox;
        private System.Windows.Forms.TextBox LowBatteryBox;
        private System.Windows.Forms.TextBox GPSValidBox;
        private System.Windows.Forms.TextBox NavValidBox;
        private System.Windows.Forms.TextBox BaroFailBox;
        private System.Windows.Forms.TextBox AccFailBox;
        private System.Windows.Forms.TextBox CompassFailBox;
        private System.Windows.Forms.TextBox GPSFailBox;
        private System.Windows.Forms.TextBox AttitudeHoldBox;
        private System.Windows.Forms.TextBox ThrottleMovingBox;
        private System.Windows.Forms.TextBox HoveringBox;
        private System.Windows.Forms.TextBox NavigateBox;
        private System.Windows.Forms.TextBox ReturnHomeBox;
        private System.Windows.Forms.TextBox ProximityBox;
        private System.Windows.Forms.TextBox CloseProximityBox;
        private System.Windows.Forms.TextBox UsingGPSAltBox;
        private System.Windows.Forms.TextBox UseRTHAutoDescendBox;
        private System.Windows.Forms.TextBox BaroAltValidBox;
        private System.Windows.Forms.TextBox RangefinderAltValidBox;
        private System.Windows.Forms.TextBox UsingRangefinderBox;
        private System.Windows.Forms.GroupBox FlagsGroupBox;
        private System.Windows.Forms.GroupBox GPSStatBox;
        private System.Windows.Forms.GroupBox AltitudeGroupBox;
        private System.Windows.Forms.Label WayHeadingLabel;
        private System.Windows.Forms.Label GPSLongitudeLabel;
        private System.Windows.Forms.Label GPSLatitudeLabel;
        private System.Windows.Forms.Label NavStateLabel;
        private System.Windows.Forms.Label GPSVelLabel;
        private System.Windows.Forms.Label CurrWPLabel;
        private System.Windows.Forms.TextBox GPSLatitude;
        private System.Windows.Forms.TextBox GPSLongitude;
        private System.Windows.Forms.TextBox NavState;
        private System.Windows.Forms.TextBox CurrWP;
        private System.Windows.Forms.TextBox WayHeading;
        private System.Windows.Forms.Label WPAltitude;
        private System.Windows.Forms.TextBox GPSVel;
        private System.Windows.Forms.Label HeadingLabel;
        private System.Windows.Forms.TextBox AltitudeError;
        private System.Windows.Forms.TextBox Heading;
        private System.Windows.Forms.Label WPDistanceLabel;
        private System.Windows.Forms.TextBox DistanceToDesired;
        private System.Windows.Forms.GroupBox NavGroupBox;
        private System.Windows.Forms.TextBox CurrentAltitude;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Label UserAttitudeToDegreesLabel;
        private System.Windows.Forms.TextBox UserAttitudeToDegrees;
        private System.Windows.Forms.TextBox AltitudeSource;
        private System.Windows.Forms.Button GoogleButton;
        private System.Windows.Forms.TextBox textBox1;
        private System.Windows.Forms.GroupBox LocationBox;
        private System.Windows.Forms.Label WhereBearingLabel;
        private System.Windows.Forms.Label WhereDistanceLabel;
        private System.Windows.Forms.Label WhereDistance;
        private System.Windows.Forms.Label WhereBearing;
    }
}

