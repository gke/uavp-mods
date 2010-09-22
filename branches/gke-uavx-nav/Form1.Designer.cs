namespace UAVXNav
{
    partial class Form1
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
        /// Required method for Designer support-do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle3 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle4 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle5 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle6 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle7 = new System.Windows.Forms.DataGridViewCellStyle();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.M = new System.Windows.Forms.DataGridView();
            this.Number = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Lat = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Lon = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Altitude = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Hidden_Alt = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.SeaLevelAlt = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.Loiter = new System.Windows.Forms.DataGridViewTextBoxColumn();
            this.LaunchLat = new System.Windows.Forms.TextBox();
            this.LaunchLon = new System.Windows.Forms.TextBox();
            this.LaunchManuallyCheckBox = new System.Windows.Forms.CheckBox();
            this.MenuMain = new System.Windows.Forms.MenuStrip();
            this.UAVXNavToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.COMSelectComboBox = new System.Windows.Forms.ToolStripComboBox();
            this.COMDisarmedBaudRateComboBox = new System.Windows.Forms.ToolStripComboBox();
            this.COMArmedBaudRateComboBox = new System.Windows.Forms.ToolStripComboBox();
            this.MenuFiles = new System.Windows.Forms.ToolStripMenuItem();
            this.MenuLoadMission = new System.Windows.Forms.ToolStripMenuItem();
            this.MenuSaveMission = new System.Windows.Forms.ToolStripMenuItem();
            this.MenuSaveMissionAs = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator5 = new System.Windows.Forms.ToolStripSeparator();
            this.cboComSelect = new System.Windows.Forms.ToolStripComboBox();
            this.toolStripSeparator6 = new System.Windows.Forms.ToolStripSeparator();
            this.MenuRead = new System.Windows.Forms.ToolStripMenuItem();
            this.MenuWrite = new System.Windows.Forms.ToolStripMenuItem();
            this.toolStripSeparator7 = new System.Windows.Forms.ToolStripSeparator();
            this.MenuExit = new System.Windows.Forms.ToolStripMenuItem();
            this.OriginAltitude = new System.Windows.Forms.TextBox();
            this.OriginGroupBox = new System.Windows.Forms.GroupBox();
            this.RTHAltitude = new System.Windows.Forms.Label();
            this.HomeAlt = new System.Windows.Forms.Label();
            this.HomeAltLabel = new System.Windows.Forms.Label();
            this.RTHAltitudeLabel = new System.Windows.Forms.Label();
            this.LaunchLonLabel = new System.Windows.Forms.Label();
            this.LaunchLatLabel = new System.Windows.Forms.Label();
            this.LookupAltCheckBox = new System.Windows.Forms.CheckBox();
            this.OriginAltitudeLabel = new System.Windows.Forms.Label();
            this.label8 = new System.Windows.Forms.Label();
            this.webBrowser1 = new System.Windows.Forms.WebBrowser();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.StatusGroupBox = new System.Windows.Forms.GroupBox();
            this.cmdWrite = new System.Windows.Forms.Button();
            this.cmdRead = new System.Windows.Forms.Button();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.statusLabel = new System.Windows.Forms.Label();
            this.ClickMapLabel = new System.Windows.Forms.Label();
            this.OtherGroupBox = new System.Windows.Forms.GroupBox();
            this.RangeLimitLabel = new System.Windows.Forms.Label();
            this.RangeLimitSetting = new System.Windows.Forms.TextBox();
            this.ProximityAltLabel = new System.Windows.Forms.Label();
            this.ProximityAlt = new System.Windows.Forms.TextBox();
            this.ProxRadiusLabel = new System.Windows.Forms.Label();
            this.ProximityRadius = new System.Windows.Forms.TextBox();
            this.GPSAltitudeLabel = new System.Windows.Forms.Label();
            this.GPSVelLabel = new System.Windows.Forms.Label();
            this.GPSVelS = new System.Windows.Forms.Label();
            this.GPSAltitudeS = new System.Windows.Forms.Label();
            this.OfflineGroupBox = new System.Windows.Forms.GroupBox();
            this.OfflineAddWPButton = new System.Windows.Forms.Button();
            this.OfflineClearAllButton = new System.Windows.Forms.Button();
            this.OfflineRemoveWPButton = new System.Windows.Forms.Button();
            this.OfflineMap = new System.Windows.Forms.PictureBox();
            this.OnlineGroupBox = new System.Windows.Forms.GroupBox();
            this.CentreButton = new System.Windows.Forms.Button();
            this.OnlineClearAllButton = new System.Windows.Forms.Button();
            this.optWP = new System.Windows.Forms.RadioButton();
            this.optHome = new System.Windows.Forms.RadioButton();
            this.SearchButton = new System.Windows.Forms.Button();
            this.AddressLabel = new System.Windows.Forms.Label();
            this.LocationAddress = new System.Windows.Forms.TextBox();
            this.MapZoomLevel = new System.Windows.Forms.Label();
            this.ZoomMaps = new System.Windows.Forms.NumericUpDown();
            this.OnlineRemoveWPButton = new System.Windows.Forms.Button();
            this.OfflineMapLabel = new System.Windows.Forms.Label();
            this.GPSStatsGroupBox = new System.Windows.Forms.GroupBox();
            this.GPSFixLabel = new System.Windows.Forms.Label();
            this.MinGPSFixS = new System.Windows.Forms.Label();
            this.MaxGPSFixS = new System.Windows.Forms.Label();
            this.HDiluteLabel = new System.Windows.Forms.Label();
            this.MaxHDiluteS = new System.Windows.Forms.Label();
            this.NavValidSLabel = new System.Windows.Forms.Label();
            this.MinHDiluteS = new System.Windows.Forms.Label();
            this.NavValidS = new System.Windows.Forms.Label();
            this.SatsLabel = new System.Windows.Forms.Label();
            this.MinGPSNoOfSatsS = new System.Windows.Forms.Label();
            this.MaxGPSNoOfSatsS = new System.Windows.Forms.Label();
            this.ErrorStatsGroupBox = new System.Windows.Forms.GroupBox();
            this.GyroSLabel = new System.Windows.Forms.Label();
            this.GyroFailS = new System.Windows.Forms.Label();
            this.GPSInvalidSLabel = new System.Windows.Forms.Label();
            this.GPSFailS = new System.Windows.Forms.Label();
            this.RCGlitchesS = new System.Windows.Forms.Label();
            this.CompassFailLabel = new System.Windows.Forms.Label();
            this.AccFailLabel = new System.Windows.Forms.Label();
            this.CompassFailS = new System.Windows.Forms.Label();
            this.BaroFailLabel = new System.Windows.Forms.Label();
            this.BaroFailS = new System.Windows.Forms.Label();
            this.RCGlitchesSLabel = new System.Windows.Forms.Label();
            this.AccFailS = new System.Windows.Forms.Label();
            this.BaroStatsGroupBox = new System.Windows.Forms.GroupBox();
            this.GPSBaroScaleSLabel = new System.Windows.Forms.Label();
            this.GPSBaroScaleS = new System.Windows.Forms.Label();
            this.RelBaroPressureSLabel = new System.Windows.Forms.Label();
            this.RelBaroPressureS = new System.Windows.Forms.Label();
            this.ROCLabel = new System.Windows.Forms.Label();
            this.MaxBaroROCS = new System.Windows.Forms.Label();
            this.RelBaroAltitudeLabel = new System.Windows.Forms.Label();
            this.RelBaroAltitudeS = new System.Windows.Forms.Label();
            this.MinBaroROCS = new System.Windows.Forms.Label();
            this.NavGroupBox = new System.Windows.Forms.GroupBox();
            this.NavStateTimeout = new System.Windows.Forms.Label();
            this.NavStateTimeoutLabel = new System.Windows.Forms.Label();
            this.AltitudeError = new System.Windows.Forms.Label();
            this.label6 = new System.Windows.Forms.Label();
            this.WPHeadingLabel = new System.Windows.Forms.Label();
            this.WPDistanceLabel = new System.Windows.Forms.Label();
            this.DistanceToDesired = new System.Windows.Forms.Label();
            this.CurrWPLabel = new System.Windows.Forms.Label();
            this.CurrWP = new System.Windows.Forms.Label();
            this.DesiredCourse = new System.Windows.Forms.Label();
            this.HeadingSLabel = new System.Windows.Forms.Label();
            this.HeadingS = new System.Windows.Forms.Label();
            this.FlyingButton = new System.Windows.Forms.Button();
            this.BatteryVoltsLabel = new System.Windows.Forms.Label();
            this.BatteryVolts = new System.Windows.Forms.Label();
            this.CurrentAltitude = new System.Windows.Forms.Label();
            this.serialPort2 = new System.IO.Ports.SerialPort(this.components);
            this.BatteryGroupBox = new System.Windows.Forms.GroupBox();
            this.BatteryChargeLabel = new System.Windows.Forms.Label();
            this.BatteryCharge = new System.Windows.Forms.Label();
            this.BatteryCurrentLabel = new System.Windows.Forms.Label();
            this.BatteryCurrent = new System.Windows.Forms.Label();
            this.FlagsGroupBox = new System.Windows.Forms.FlowLayoutPanel();
            this.RTHAltHoldBox = new System.Windows.Forms.TextBox();
            this.UsingGPSAltBox = new System.Windows.Forms.TextBox();
            this.RangefinderValidBox = new System.Windows.Forms.TextBox();
            this.HoldingAltBox = new System.Windows.Forms.TextBox();
            this.ReturnHomeBox = new System.Windows.Forms.TextBox();
            this.CloseProximityBox = new System.Windows.Forms.TextBox();
            this.TurnToWPBox = new System.Windows.Forms.TextBox();
            this.UseRTHAutoDescendBox = new System.Windows.Forms.TextBox();
            this.NavValidBox = new System.Windows.Forms.TextBox();
            this.AttitudeHoldBox = new System.Windows.Forms.TextBox();
            this.NavigateBox = new System.Windows.Forms.TextBox();
            this.ProximityBox = new System.Windows.Forms.TextBox();
            this.NavState = new System.Windows.Forms.TextBox();
            this.FailState = new System.Windows.Forms.TextBox();
            this.AltitudeSource = new System.Windows.Forms.TextBox();
            this.WhereGroupBox = new System.Windows.Forms.GroupBox();
            this.WhereBearingLabel = new System.Windows.Forms.Label();
            this.WhereDistanceLabel = new System.Windows.Forms.Label();
            this.WhereDistance = new System.Windows.Forms.Label();
            this.WhereBearing = new System.Windows.Forms.Label();
            this.SaveLogFileDialog = new System.Windows.Forms.SaveFileDialog();
            this.OpenLogFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.headingIndicatorInstrumentControl1 = new Instruments.HeadingIndicatorInstrumentControl();
            this.attitudeIndicatorInstrumentControl1 = new Instruments.AttitudeIndicatorInstrumentControl();
            this.eventLog1 = new System.Diagnostics.EventLog();
            ((System.ComponentModel.ISupportInitialize)(this.M)).BeginInit();
            this.MenuMain.SuspendLayout();
            this.OriginGroupBox.SuspendLayout();
            this.StatusGroupBox.SuspendLayout();
            this.OtherGroupBox.SuspendLayout();
            this.OfflineGroupBox.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.OfflineMap)).BeginInit();
            this.OnlineGroupBox.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ZoomMaps)).BeginInit();
            this.GPSStatsGroupBox.SuspendLayout();
            this.ErrorStatsGroupBox.SuspendLayout();
            this.BaroStatsGroupBox.SuspendLayout();
            this.NavGroupBox.SuspendLayout();
            this.BatteryGroupBox.SuspendLayout();
            this.FlagsGroupBox.SuspendLayout();
            this.WhereGroupBox.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.eventLog1)).BeginInit();
            this.SuspendLayout();
            // 
            // M
            // 
            this.M.AllowUserToAddRows = false;
            this.M.AllowUserToDeleteRows = false;
            this.M.AllowUserToResizeColumns = false;
            this.M.AllowUserToResizeRows = false;
            this.M.BackgroundColor = System.Drawing.Color.White;
            this.M.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            this.M.Columns.AddRange(new System.Windows.Forms.DataGridViewColumn[] {
            this.Number,
            this.Lat,
            this.Lon,
            this.Altitude,
            this.Hidden_Alt,
            this.SeaLevelAlt,
            this.Loiter});
            this.M.Location = new System.Drawing.Point(117, 31);
            this.M.Name = "M";
            this.M.RowHeadersVisible = false;
            this.M.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing;
            this.M.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.M.Size = new System.Drawing.Size(346, 229);
            this.M.TabIndex = 1;
            this.M.Enter += new System.EventHandler(this.M_Enter);
            this.M.Leave += new System.EventHandler(this.M_Leave);
            this.M.CellEndEdit += new System.Windows.Forms.DataGridViewCellEventHandler(this.M_CellEndEdit);
            this.M.CellContentClick += new System.Windows.Forms.DataGridViewCellEventHandler(this.M_CellContentClick);
            // 
            // Number
            // 
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.Number.DefaultCellStyle = dataGridViewCellStyle1;
            this.Number.Frozen = true;
            this.Number.HeaderText = "#";
            this.Number.Name = "Number";
            this.Number.ReadOnly = true;
            this.Number.Resizable = System.Windows.Forms.DataGridViewTriState.False;
            this.Number.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Number.Width = 30;
            // 
            // Lat
            // 
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.Lat.DefaultCellStyle = dataGridViewCellStyle2;
            this.Lat.HeaderText = "Latitude";
            this.Lat.Name = "Lat";
            this.Lat.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Lat.Width = 60;
            // 
            // Lon
            // 
            dataGridViewCellStyle3.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.Lon.DefaultCellStyle = dataGridViewCellStyle3;
            this.Lon.HeaderText = "Longitude";
            this.Lon.Name = "Lon";
            this.Lon.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Lon.Width = 65;
            // 
            // Altitude
            // 
            dataGridViewCellStyle4.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle4.Format = "N0";
            dataGridViewCellStyle4.NullValue = null;
            this.Altitude.DefaultCellStyle = dataGridViewCellStyle4;
            this.Altitude.HeaderText = "Altitude (Home)";
            this.Altitude.Name = "Altitude";
            this.Altitude.ReadOnly = true;
            this.Altitude.SortMode = System.Windows.Forms.DataGridViewColumnSortMode.NotSortable;
            this.Altitude.Width = 50;
            // 
            // Hidden_Alt
            // 
            dataGridViewCellStyle5.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle5.NullValue = null;
            this.Hidden_Alt.DefaultCellStyle = dataGridViewCellStyle5;
            this.Hidden_Alt.HeaderText = "Altitude Offset";
            this.Hidden_Alt.Name = "Hidden_Alt";
            this.Hidden_Alt.Width = 50;
            // 
            // SeaLevelAlt
            // 
            dataGridViewCellStyle6.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            dataGridViewCellStyle6.NullValue = null;
            this.SeaLevelAlt.DefaultCellStyle = dataGridViewCellStyle6;
            this.SeaLevelAlt.HeaderText = "Altitude (Sea)";
            this.SeaLevelAlt.Name = "SeaLevelAlt";
            this.SeaLevelAlt.ReadOnly = true;
            this.SeaLevelAlt.Width = 50;
            // 
            // Loiter
            // 
            dataGridViewCellStyle7.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.Loiter.DefaultCellStyle = dataGridViewCellStyle7;
            this.Loiter.HeaderText = "Loiter (Sec.)";
            this.Loiter.Name = "Loiter";
            this.Loiter.Width = 40;
            // 
            // LaunchLat
            // 
            this.LaunchLat.Enabled = false;
            this.LaunchLat.Location = new System.Drawing.Point(7, 50);
            this.LaunchLat.Name = "LaunchLat";
            this.LaunchLat.Size = new System.Drawing.Size(92, 20);
            this.LaunchLat.TabIndex = 3;
            this.LaunchLat.Text = "0.000000";
            this.LaunchLat.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.LaunchLat.TextChanged += new System.EventHandler(this.launchLat_TextChanged);
            this.LaunchLat.Leave += new System.EventHandler(this.launchLat_Leave);
            this.LaunchLat.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.launchLat_KeyPress);
            // 
            // LaunchLon
            // 
            this.LaunchLon.Enabled = false;
            this.LaunchLon.Location = new System.Drawing.Point(7, 89);
            this.LaunchLon.Name = "LaunchLon";
            this.LaunchLon.Size = new System.Drawing.Size(93, 20);
            this.LaunchLon.TabIndex = 4;
            this.LaunchLon.Text = "0.000000";
            this.LaunchLon.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.LaunchLon.TextChanged += new System.EventHandler(this.launchLon_TextChanged);
            this.LaunchLon.Leave += new System.EventHandler(this.launchLon_Leave);
            this.LaunchLon.KeyPress += new System.Windows.Forms.KeyPressEventHandler(this.launchLon_KeyPress);
            // 
            // LaunchManuallyCheckBox
            // 
            this.LaunchManuallyCheckBox.AutoSize = true;
            this.LaunchManuallyCheckBox.Location = new System.Drawing.Point(7, 134);
            this.LaunchManuallyCheckBox.Name = "LaunchManuallyCheckBox";
            this.LaunchManuallyCheckBox.Size = new System.Drawing.Size(87, 17);
            this.LaunchManuallyCheckBox.TabIndex = 6;
            this.LaunchManuallyCheckBox.Text = "Set Manually";
            this.LaunchManuallyCheckBox.UseVisualStyleBackColor = true;
            this.LaunchManuallyCheckBox.CheckedChanged += new System.EventHandler(this.launchManually_CheckedChanged);
            // 
            // MenuMain
            // 
            this.MenuMain.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.UAVXNavToolStripMenuItem,
            this.COMSelectComboBox,
            this.COMDisarmedBaudRateComboBox,
            this.COMArmedBaudRateComboBox,
            this.MenuFiles});
            this.MenuMain.Location = new System.Drawing.Point(0, 0);
            this.MenuMain.Name = "MenuMain";
            this.MenuMain.Size = new System.Drawing.Size(1014, 25);
            this.MenuMain.TabIndex = 13;
            this.MenuMain.Text = "menuStrip1";
            // 
            // UAVXNavToolStripMenuItem
            // 
            this.UAVXNavToolStripMenuItem.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.UAVXNavToolStripMenuItem.Name = "UAVXNavToolStripMenuItem";
            this.UAVXNavToolStripMenuItem.Size = new System.Drawing.Size(102, 21);
            this.UAVXNavToolStripMenuItem.Text = "G.K. Egan (2010)";
            this.UAVXNavToolStripMenuItem.Click += new System.EventHandler(this.UAVXNavToolStripMenuItem_Click);
            // 
            // COMSelectComboBox
            // 
            this.COMSelectComboBox.Name = "COMSelectComboBox";
            this.COMSelectComboBox.Size = new System.Drawing.Size(75, 21);
            this.COMSelectComboBox.Text = "COM1:";
            this.COMSelectComboBox.SelectedIndexChanged += new System.EventHandler(this.COMSelectComboBox_SelectedIndexChanged);
            // 
            // COMDisarmedBaudRateComboBox
            // 
            this.COMDisarmedBaudRateComboBox.Items.AddRange(new object[] {
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "128000"});
            this.COMDisarmedBaudRateComboBox.Name = "COMDisarmedBaudRateComboBox";
            this.COMDisarmedBaudRateComboBox.Size = new System.Drawing.Size(75, 21);
            this.COMDisarmedBaudRateComboBox.SelectedIndexChanged += new System.EventHandler(this.COMDisarmedBaudRateSelectComboBox_SelectedIndexChanged);
            // 
            // COMArmedBaudRateComboBox
            // 
            this.COMArmedBaudRateComboBox.Items.AddRange(new object[] {
            "9600",
            "19200",
            "38400",
            "57600",
            "115200",
            "128000"});
            this.COMArmedBaudRateComboBox.Name = "COMArmedBaudRateComboBox";
            this.COMArmedBaudRateComboBox.Size = new System.Drawing.Size(75, 21);
            this.COMArmedBaudRateComboBox.SelectedIndexChanged += new System.EventHandler(this.COMArmedBaudRateSelectComboBox_SelectedIndexChanged);
            // 
            // MenuFiles
            // 
            this.MenuFiles.DropDownItems.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.MenuLoadMission,
            this.MenuSaveMission,
            this.MenuSaveMissionAs,
            this.toolStripSeparator5,
            this.cboComSelect,
            this.toolStripSeparator6,
            this.MenuRead,
            this.MenuWrite,
            this.toolStripSeparator7,
            this.MenuExit});
            this.MenuFiles.Name = "MenuFiles";
            this.MenuFiles.Size = new System.Drawing.Size(72, 21);
            this.MenuFiles.Text = "Mission File";
            this.MenuFiles.Click += new System.EventHandler(this.toolStripMenuItem8_Click);
            // 
            // MenuLoadMission
            // 
            this.MenuLoadMission.Name = "MenuLoadMission";
            this.MenuLoadMission.Size = new System.Drawing.Size(200, 22);
            this.MenuLoadMission.Text = "Load Mission";
            this.MenuLoadMission.Click += new System.EventHandler(this.MenuLoadMission_Click);
            // 
            // MenuSaveMission
            // 
            this.MenuSaveMission.Name = "MenuSaveMission";
            this.MenuSaveMission.Size = new System.Drawing.Size(200, 22);
            this.MenuSaveMission.Text = "Save Mission";
            this.MenuSaveMission.Click += new System.EventHandler(this.MenuSaveMission_Click);
            // 
            // MenuSaveMissionAs
            // 
            this.MenuSaveMissionAs.Name = "MenuSaveMissionAs";
            this.MenuSaveMissionAs.Size = new System.Drawing.Size(200, 22);
            this.MenuSaveMissionAs.Text = "Save Mission As...";
            this.MenuSaveMissionAs.Visible = false;
            // 
            // toolStripSeparator5
            // 
            this.toolStripSeparator5.Name = "toolStripSeparator5";
            this.toolStripSeparator5.Size = new System.Drawing.Size(197, 6);
            // 
            // cboComSelect
            // 
            this.cboComSelect.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cboComSelect.DropDownWidth = 100;
            this.cboComSelect.Name = "cboComSelect";
            this.cboComSelect.Size = new System.Drawing.Size(140, 21);
            this.cboComSelect.SelectedIndexChanged += new System.EventHandler(this.cboComSelect_SelectedIndexChanged);
            // 
            // toolStripSeparator6
            // 
            this.toolStripSeparator6.Name = "toolStripSeparator6";
            this.toolStripSeparator6.Size = new System.Drawing.Size(197, 6);
            // 
            // MenuRead
            // 
            this.MenuRead.Name = "MenuRead";
            this.MenuRead.Size = new System.Drawing.Size(200, 22);
            this.MenuRead.Text = "Read";
            this.MenuRead.Click += new System.EventHandler(this.MenuRead_Click);
            // 
            // MenuWrite
            // 
            this.MenuWrite.Name = "MenuWrite";
            this.MenuWrite.Size = new System.Drawing.Size(200, 22);
            this.MenuWrite.Text = "Write";
            this.MenuWrite.Click += new System.EventHandler(this.MenuWrite_Click);
            // 
            // toolStripSeparator7
            // 
            this.toolStripSeparator7.Name = "toolStripSeparator7";
            this.toolStripSeparator7.Size = new System.Drawing.Size(197, 6);
            // 
            // MenuExit
            // 
            this.MenuExit.Name = "MenuExit";
            this.MenuExit.Size = new System.Drawing.Size(200, 22);
            this.MenuExit.Text = "Exit";
            this.MenuExit.Click += new System.EventHandler(this.toolStripMenuItem14_Click);
            // 
            // OriginAltitude
            // 
            this.OriginAltitude.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.OriginAltitude.Location = new System.Drawing.Point(68, 22);
            this.OriginAltitude.Name = "OriginAltitude";
            this.OriginAltitude.ReadOnly = true;
            this.OriginAltitude.Size = new System.Drawing.Size(30, 13);
            this.OriginAltitude.TabIndex = 23;
            this.OriginAltitude.Text = "0";
            this.OriginAltitude.TextAlign = System.Windows.Forms.HorizontalAlignment.Right;
            // 
            // OriginGroupBox
            // 
            this.OriginGroupBox.Controls.Add(this.RTHAltitude);
            this.OriginGroupBox.Controls.Add(this.HomeAlt);
            this.OriginGroupBox.Controls.Add(this.HomeAltLabel);
            this.OriginGroupBox.Controls.Add(this.RTHAltitudeLabel);
            this.OriginGroupBox.Controls.Add(this.LaunchLonLabel);
            this.OriginGroupBox.Controls.Add(this.LaunchLatLabel);
            this.OriginGroupBox.Controls.Add(this.LaunchLat);
            this.OriginGroupBox.Controls.Add(this.LaunchManuallyCheckBox);
            this.OriginGroupBox.Controls.Add(this.LaunchLon);
            this.OriginGroupBox.Location = new System.Drawing.Point(6, 31);
            this.OriginGroupBox.Name = "OriginGroupBox";
            this.OriginGroupBox.Size = new System.Drawing.Size(105, 156);
            this.OriginGroupBox.TabIndex = 24;
            this.OriginGroupBox.TabStop = false;
            this.OriginGroupBox.Text = "Origin";
            // 
            // RTHAltitude
            // 
            this.RTHAltitude.Location = new System.Drawing.Point(64, 118);
            this.RTHAltitude.Name = "RTHAltitude";
            this.RTHAltitude.Size = new System.Drawing.Size(33, 13);
            this.RTHAltitude.TabIndex = 53;
            this.RTHAltitude.Text = "0";
            this.RTHAltitude.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // HomeAlt
            // 
            this.HomeAlt.Location = new System.Drawing.Point(66, 16);
            this.HomeAlt.Name = "HomeAlt";
            this.HomeAlt.Size = new System.Drawing.Size(33, 13);
            this.HomeAlt.TabIndex = 51;
            this.HomeAlt.Text = "0";
            this.HomeAlt.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // HomeAltLabel
            // 
            this.HomeAltLabel.AutoSize = true;
            this.HomeAltLabel.Location = new System.Drawing.Point(7, 16);
            this.HomeAltLabel.Name = "HomeAltLabel";
            this.HomeAltLabel.Size = new System.Drawing.Size(22, 13);
            this.HomeAltLabel.TabIndex = 52;
            this.HomeAltLabel.Text = "Alt:";
            // 
            // RTHAltitudeLabel
            // 
            this.RTHAltitudeLabel.AutoSize = true;
            this.RTHAltitudeLabel.Location = new System.Drawing.Point(4, 118);
            this.RTHAltitudeLabel.Name = "RTHAltitudeLabel";
            this.RTHAltitudeLabel.Size = new System.Drawing.Size(51, 13);
            this.RTHAltitudeLabel.TabIndex = 28;
            this.RTHAltitudeLabel.Text = "RTH Alt.:";
            // 
            // LaunchLonLabel
            // 
            this.LaunchLonLabel.AutoSize = true;
            this.LaunchLonLabel.Location = new System.Drawing.Point(7, 73);
            this.LaunchLonLabel.Name = "LaunchLonLabel";
            this.LaunchLonLabel.Size = new System.Drawing.Size(57, 13);
            this.LaunchLonLabel.TabIndex = 27;
            this.LaunchLonLabel.Text = "Longitude:";
            // 
            // LaunchLatLabel
            // 
            this.LaunchLatLabel.AutoSize = true;
            this.LaunchLatLabel.Location = new System.Drawing.Point(7, 34);
            this.LaunchLatLabel.Name = "LaunchLatLabel";
            this.LaunchLatLabel.Size = new System.Drawing.Size(48, 13);
            this.LaunchLatLabel.TabIndex = 26;
            this.LaunchLatLabel.Text = "Latitude:";
            // 
            // LookupAltCheckBox
            // 
            this.LookupAltCheckBox.AutoSize = true;
            this.LookupAltCheckBox.Location = new System.Drawing.Point(6, 45);
            this.LookupAltCheckBox.Name = "LookupAltCheckBox";
            this.LookupAltCheckBox.Size = new System.Drawing.Size(85, 17);
            this.LookupAltCheckBox.TabIndex = 30;
            this.LookupAltCheckBox.Text = "Over Terrain";
            this.LookupAltCheckBox.UseVisualStyleBackColor = true;
            this.LookupAltCheckBox.Visible = false;
            this.LookupAltCheckBox.CheckedChanged += new System.EventHandler(this.chkLookupAlt_CheckedChanged);
            // 
            // OriginAltitudeLabel
            // 
            this.OriginAltitudeLabel.AutoSize = true;
            this.OriginAltitudeLabel.Location = new System.Drawing.Point(5, 22);
            this.OriginAltitudeLabel.Name = "OriginAltitudeLabel";
            this.OriginAltitudeLabel.Size = new System.Drawing.Size(48, 13);
            this.OriginAltitudeLabel.TabIndex = 29;
            this.OriginAltitudeLabel.Text = "Alt (Ref):";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Font = new System.Drawing.Font("Microsoft Sans Serif", 6F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label8.Location = new System.Drawing.Point(5, 646);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(0, 9);
            this.label8.TabIndex = 28;
            // 
            // webBrowser1
            // 
            this.webBrowser1.Location = new System.Drawing.Point(469, 27);
            this.webBrowser1.MinimumSize = new System.Drawing.Size(20, 20);
            this.webBrowser1.Name = "webBrowser1";
            this.webBrowser1.ScrollBarsEnabled = false;
            this.webBrowser1.Size = new System.Drawing.Size(533, 382);
            this.webBrowser1.TabIndex = 0;
            this.webBrowser1.WebBrowserShortcutsEnabled = false;
            // 
            // timer1
            // 
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // StatusGroupBox
            // 
            this.StatusGroupBox.Controls.Add(this.cmdWrite);
            this.StatusGroupBox.Controls.Add(this.cmdRead);
            this.StatusGroupBox.Controls.Add(this.progressBar1);
            this.StatusGroupBox.Controls.Add(this.statusLabel);
            this.StatusGroupBox.Location = new System.Drawing.Point(6, 554);
            this.StatusGroupBox.Name = "StatusGroupBox";
            this.StatusGroupBox.Size = new System.Drawing.Size(457, 101);
            this.StatusGroupBox.TabIndex = 43;
            this.StatusGroupBox.TabStop = false;
            this.StatusGroupBox.Text = "Status";
            // 
            // cmdWrite
            // 
            this.cmdWrite.Location = new System.Drawing.Point(350, 10);
            this.cmdWrite.Name = "cmdWrite";
            this.cmdWrite.Size = new System.Drawing.Size(97, 22);
            this.cmdWrite.TabIndex = 20;
            this.cmdWrite.Text = "Write";
            this.cmdWrite.UseVisualStyleBackColor = true;
            this.cmdWrite.Click += new System.EventHandler(this.cmdWrite_Click);
            // 
            // cmdRead
            // 
            this.cmdRead.Location = new System.Drawing.Point(250, 10);
            this.cmdRead.Name = "cmdRead";
            this.cmdRead.Size = new System.Drawing.Size(97, 22);
            this.cmdRead.TabIndex = 19;
            this.cmdRead.Text = "Read";
            this.cmdRead.UseVisualStyleBackColor = true;
            this.cmdRead.Click += new System.EventHandler(this.cmdRead_Click);
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(10, 19);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(230, 10);
            this.progressBar1.TabIndex = 18;
            // 
            // statusLabel
            // 
            this.statusLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.statusLabel.Location = new System.Drawing.Point(9, 35);
            this.statusLabel.Name = "statusLabel";
            this.statusLabel.Size = new System.Drawing.Size(443, 60);
            this.statusLabel.TabIndex = 17;
            this.statusLabel.Text = "Hello.";
            this.statusLabel.UseCompatibleTextRendering = true;
            // 
            // ClickMapLabel
            // 
            this.ClickMapLabel.AutoSize = true;
            this.ClickMapLabel.BackColor = System.Drawing.Color.White;
            this.ClickMapLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.ClickMapLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.ClickMapLabel.ForeColor = System.Drawing.Color.Blue;
            this.ClickMapLabel.Location = new System.Drawing.Point(201, 180);
            this.ClickMapLabel.Name = "ClickMapLabel";
            this.ClickMapLabel.Size = new System.Drawing.Size(155, 18);
            this.ClickMapLabel.TabIndex = 46;
            this.ClickMapLabel.Text = "Click Map to Add WP";
            this.ClickMapLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // OtherGroupBox
            // 
            this.OtherGroupBox.Controls.Add(this.RangeLimitLabel);
            this.OtherGroupBox.Controls.Add(this.RangeLimitSetting);
            this.OtherGroupBox.Controls.Add(this.ProximityAltLabel);
            this.OtherGroupBox.Controls.Add(this.ProximityAlt);
            this.OtherGroupBox.Controls.Add(this.ProxRadiusLabel);
            this.OtherGroupBox.Controls.Add(this.OriginAltitudeLabel);
            this.OtherGroupBox.Controls.Add(this.LookupAltCheckBox);
            this.OtherGroupBox.Controls.Add(this.ProximityRadius);
            this.OtherGroupBox.Controls.Add(this.OriginAltitude);
            this.OtherGroupBox.Location = new System.Drawing.Point(6, 193);
            this.OtherGroupBox.Name = "OtherGroupBox";
            this.OtherGroupBox.Size = new System.Drawing.Size(105, 142);
            this.OtherGroupBox.TabIndex = 48;
            this.OtherGroupBox.TabStop = false;
            this.OtherGroupBox.Text = "Other Settings";
            // 
            // RangeLimitLabel
            // 
            this.RangeLimitLabel.AutoSize = true;
            this.RangeLimitLabel.Location = new System.Drawing.Point(4, 67);
            this.RangeLimitLabel.Name = "RangeLimitLabel";
            this.RangeLimitLabel.Size = new System.Drawing.Size(63, 13);
            this.RangeLimitLabel.TabIndex = 59;
            this.RangeLimitLabel.Text = "Range Limit";
            // 
            // RangeLimitSetting
            // 
            this.RangeLimitSetting.Location = new System.Drawing.Point(72, 64);
            this.RangeLimitSetting.Name = "RangeLimitSetting";
            this.RangeLimitSetting.Size = new System.Drawing.Size(28, 20);
            this.RangeLimitSetting.TabIndex = 58;
            this.RangeLimitSetting.Text = "100";
            this.RangeLimitSetting.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.RangeLimitSetting.TextChanged += new System.EventHandler(this.RangeLimit_TextChanged);
            // 
            // ProximityAltLabel
            // 
            this.ProximityAltLabel.AutoSize = true;
            this.ProximityAltLabel.Location = new System.Drawing.Point(4, 119);
            this.ProximityAltLabel.Name = "ProximityAltLabel";
            this.ProximityAltLabel.Size = new System.Drawing.Size(69, 13);
            this.ProximityAltLabel.TabIndex = 57;
            this.ProximityAltLabel.Text = "Prox. Altitude";
            // 
            // ProximityAlt
            // 
            this.ProximityAlt.Location = new System.Drawing.Point(77, 116);
            this.ProximityAlt.Name = "ProximityAlt";
            this.ProximityAlt.Size = new System.Drawing.Size(22, 20);
            this.ProximityAlt.TabIndex = 56;
            this.ProximityAlt.Text = "2";
            this.ProximityAlt.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ProximityAlt.TextChanged += new System.EventHandler(this.ProximityAlt_TextChanged);
            this.ProximityAlt.Leave += new System.EventHandler(this.ProximityAlt_Leave);
            // 
            // ProxRadiusLabel
            // 
            this.ProxRadiusLabel.AutoSize = true;
            this.ProxRadiusLabel.Location = new System.Drawing.Point(4, 93);
            this.ProxRadiusLabel.Name = "ProxRadiusLabel";
            this.ProxRadiusLabel.Size = new System.Drawing.Size(67, 13);
            this.ProxRadiusLabel.TabIndex = 55;
            this.ProxRadiusLabel.Text = "Prox. Radius";
            // 
            // ProximityRadius
            // 
            this.ProximityRadius.Location = new System.Drawing.Point(77, 90);
            this.ProximityRadius.Name = "ProximityRadius";
            this.ProximityRadius.Size = new System.Drawing.Size(22, 20);
            this.ProximityRadius.TabIndex = 53;
            this.ProximityRadius.Text = "10";
            this.ProximityRadius.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ProximityRadius.TextChanged += new System.EventHandler(this.ProximityRadius_TextChanged);
            this.ProximityRadius.Leave += new System.EventHandler(this.ProximityRadius_Leave);
            // 
            // GPSAltitudeLabel
            // 
            this.GPSAltitudeLabel.AutoSize = true;
            this.GPSAltitudeLabel.Location = new System.Drawing.Point(6, 16);
            this.GPSAltitudeLabel.Name = "GPSAltitudeLabel";
            this.GPSAltitudeLabel.Size = new System.Drawing.Size(22, 13);
            this.GPSAltitudeLabel.TabIndex = 50;
            this.GPSAltitudeLabel.Text = "Alt:";
            // 
            // GPSVelLabel
            // 
            this.GPSVelLabel.AutoSize = true;
            this.GPSVelLabel.Location = new System.Drawing.Point(6, 36);
            this.GPSVelLabel.Name = "GPSVelLabel";
            this.GPSVelLabel.Size = new System.Drawing.Size(28, 13);
            this.GPSVelLabel.TabIndex = 49;
            this.GPSVelLabel.Text = "Vel.:";
            // 
            // GPSVelS
            // 
            this.GPSVelS.Location = new System.Drawing.Point(39, 36);
            this.GPSVelS.Name = "GPSVelS";
            this.GPSVelS.Size = new System.Drawing.Size(20, 13);
            this.GPSVelS.TabIndex = 48;
            this.GPSVelS.Text = "0";
            this.GPSVelS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // GPSAltitudeS
            // 
            this.GPSAltitudeS.Location = new System.Drawing.Point(30, 16);
            this.GPSAltitudeS.Name = "GPSAltitudeS";
            this.GPSAltitudeS.Size = new System.Drawing.Size(29, 13);
            this.GPSAltitudeS.TabIndex = 47;
            this.GPSAltitudeS.Text = "0";
            this.GPSAltitudeS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // OfflineGroupBox
            // 
            this.OfflineGroupBox.Controls.Add(this.OfflineAddWPButton);
            this.OfflineGroupBox.Controls.Add(this.OfflineClearAllButton);
            this.OfflineGroupBox.Controls.Add(this.OfflineRemoveWPButton);
            this.OfflineGroupBox.Location = new System.Drawing.Point(117, 361);
            this.OfflineGroupBox.Name = "OfflineGroupBox";
            this.OfflineGroupBox.Size = new System.Drawing.Size(346, 48);
            this.OfflineGroupBox.TabIndex = 49;
            this.OfflineGroupBox.TabStop = false;
            this.OfflineGroupBox.Text = "WP Setup (Offline Mode)";
            // 
            // OfflineAddWPButton
            // 
            this.OfflineAddWPButton.Location = new System.Drawing.Point(6, 19);
            this.OfflineAddWPButton.Name = "OfflineAddWPButton";
            this.OfflineAddWPButton.Size = new System.Drawing.Size(100, 23);
            this.OfflineAddWPButton.TabIndex = 54;
            this.OfflineAddWPButton.Text = "Add WP";
            this.OfflineAddWPButton.UseVisualStyleBackColor = true;
            this.OfflineAddWPButton.Click += new System.EventHandler(this.cmdAddOffWP_Click);
            // 
            // OfflineClearAllButton
            // 
            this.OfflineClearAllButton.Enabled = false;
            this.OfflineClearAllButton.Location = new System.Drawing.Point(222, 19);
            this.OfflineClearAllButton.Name = "OfflineClearAllButton";
            this.OfflineClearAllButton.Size = new System.Drawing.Size(118, 23);
            this.OfflineClearAllButton.TabIndex = 53;
            this.OfflineClearAllButton.Text = "Clear All WPs";
            this.OfflineClearAllButton.UseVisualStyleBackColor = true;
            this.OfflineClearAllButton.Click += new System.EventHandler(this.cmdClearOffWP_Click);
            // 
            // OfflineRemoveWPButton
            // 
            this.OfflineRemoveWPButton.Enabled = false;
            this.OfflineRemoveWPButton.Location = new System.Drawing.Point(112, 19);
            this.OfflineRemoveWPButton.Name = "OfflineRemoveWPButton";
            this.OfflineRemoveWPButton.Size = new System.Drawing.Size(104, 23);
            this.OfflineRemoveWPButton.TabIndex = 52;
            this.OfflineRemoveWPButton.Text = "Remove WP";
            this.OfflineRemoveWPButton.UseVisualStyleBackColor = true;
            this.OfflineRemoveWPButton.Click += new System.EventHandler(this.cmdRemoveOffWP_Click);
            // 
            // OfflineMap
            // 
            this.OfflineMap.Location = new System.Drawing.Point(469, 27);
            this.OfflineMap.Name = "OfflineMap";
            this.OfflineMap.Size = new System.Drawing.Size(533, 382);
            this.OfflineMap.TabIndex = 50;
            this.OfflineMap.TabStop = false;
            // 
            // OnlineGroupBox
            // 
            this.OnlineGroupBox.Controls.Add(this.CentreButton);
            this.OnlineGroupBox.Controls.Add(this.OnlineClearAllButton);
            this.OnlineGroupBox.Controls.Add(this.optWP);
            this.OnlineGroupBox.Controls.Add(this.optHome);
            this.OnlineGroupBox.Controls.Add(this.SearchButton);
            this.OnlineGroupBox.Controls.Add(this.AddressLabel);
            this.OnlineGroupBox.Controls.Add(this.LocationAddress);
            this.OnlineGroupBox.Controls.Add(this.MapZoomLevel);
            this.OnlineGroupBox.Controls.Add(this.ZoomMaps);
            this.OnlineGroupBox.Controls.Add(this.OnlineRemoveWPButton);
            this.OnlineGroupBox.FlatStyle = System.Windows.Forms.FlatStyle.System;
            this.OnlineGroupBox.Location = new System.Drawing.Point(118, 260);
            this.OnlineGroupBox.Name = "OnlineGroupBox";
            this.OnlineGroupBox.Size = new System.Drawing.Size(345, 95);
            this.OnlineGroupBox.TabIndex = 51;
            this.OnlineGroupBox.TabStop = false;
            this.OnlineGroupBox.Text = "WP Setup (Online Mode)";
            // 
            // CentreButton
            // 
            this.CentreButton.Enabled = false;
            this.CentreButton.Location = new System.Drawing.Point(138, 16);
            this.CentreButton.Name = "CentreButton";
            this.CentreButton.Size = new System.Drawing.Size(81, 23);
            this.CentreButton.TabIndex = 52;
            this.CentreButton.Text = "Center";
            this.CentreButton.UseVisualStyleBackColor = true;
            this.CentreButton.Click += new System.EventHandler(this.cmdCenter_Click);
            // 
            // OnlineClearAllButton
            // 
            this.OnlineClearAllButton.Enabled = false;
            this.OnlineClearAllButton.Location = new System.Drawing.Point(225, 16);
            this.OnlineClearAllButton.Name = "OnlineClearAllButton";
            this.OnlineClearAllButton.Size = new System.Drawing.Size(114, 23);
            this.OnlineClearAllButton.TabIndex = 51;
            this.OnlineClearAllButton.Text = "Clear All WPs";
            this.OnlineClearAllButton.UseVisualStyleBackColor = true;
            this.OnlineClearAllButton.Click += new System.EventHandler(this.cmdClear_Click);
            // 
            // optWP
            // 
            this.optWP.AutoSize = true;
            this.optWP.Location = new System.Drawing.Point(166, 43);
            this.optWP.Name = "optWP";
            this.optWP.Size = new System.Drawing.Size(43, 17);
            this.optWP.TabIndex = 50;
            this.optWP.Text = "WP";
            this.optWP.UseVisualStyleBackColor = true;
            // 
            // optHome
            // 
            this.optHome.AutoSize = true;
            this.optHome.Checked = true;
            this.optHome.Location = new System.Drawing.Point(107, 43);
            this.optHome.Name = "optHome";
            this.optHome.Size = new System.Drawing.Size(53, 17);
            this.optHome.TabIndex = 49;
            this.optHome.TabStop = true;
            this.optHome.Text = "Home";
            this.optHome.UseVisualStyleBackColor = true;
            // 
            // SearchButton
            // 
            this.SearchButton.Location = new System.Drawing.Point(180, 66);
            this.SearchButton.Name = "SearchButton";
            this.SearchButton.Size = new System.Drawing.Size(70, 23);
            this.SearchButton.TabIndex = 48;
            this.SearchButton.Text = "Search";
            this.SearchButton.UseVisualStyleBackColor = true;
            this.SearchButton.Click += new System.EventHandler(this.cmdSearch_Click);
            // 
            // AddressLabel
            // 
            this.AddressLabel.AutoSize = true;
            this.AddressLabel.Location = new System.Drawing.Point(6, 45);
            this.AddressLabel.Name = "AddressLabel";
            this.AddressLabel.Size = new System.Drawing.Size(85, 13);
            this.AddressLabel.TabIndex = 47;
            this.AddressLabel.Text = "Address Search:";
            // 
            // LocationAddress
            // 
            this.LocationAddress.Location = new System.Drawing.Point(4, 67);
            this.LocationAddress.Name = "LocationAddress";
            this.LocationAddress.Size = new System.Drawing.Size(170, 20);
            this.LocationAddress.TabIndex = 46;
            this.LocationAddress.TextChanged += new System.EventHandler(this.txtAddress_TextChanged);
            this.LocationAddress.Enter += new System.EventHandler(this.txtAddress_Enter);
            // 
            // MapZoomLevel
            // 
            this.MapZoomLevel.AutoSize = true;
            this.MapZoomLevel.Location = new System.Drawing.Point(250, 45);
            this.MapZoomLevel.Name = "MapZoomLevel";
            this.MapZoomLevel.Size = new System.Drawing.Size(87, 13);
            this.MapZoomLevel.TabIndex = 45;
            this.MapZoomLevel.Text = "Map Zoom Level";
            // 
            // ZoomMaps
            // 
            this.ZoomMaps.Location = new System.Drawing.Point(261, 67);
            this.ZoomMaps.Maximum = new decimal(new int[] {
            20,
            0,
            0,
            0});
            this.ZoomMaps.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.ZoomMaps.Name = "ZoomMaps";
            this.ZoomMaps.Size = new System.Drawing.Size(76, 20);
            this.ZoomMaps.TabIndex = 44;
            this.ZoomMaps.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            this.ZoomMaps.Value = new decimal(new int[] {
            17,
            0,
            0,
            0});
            this.ZoomMaps.ValueChanged += new System.EventHandler(this.numericMaps_ValueChanged);
            // 
            // OnlineRemoveWPButton
            // 
            this.OnlineRemoveWPButton.Enabled = false;
            this.OnlineRemoveWPButton.Location = new System.Drawing.Point(6, 16);
            this.OnlineRemoveWPButton.Name = "OnlineRemoveWPButton";
            this.OnlineRemoveWPButton.Size = new System.Drawing.Size(126, 23);
            this.OnlineRemoveWPButton.TabIndex = 43;
            this.OnlineRemoveWPButton.Text = "Remove WP";
            this.OnlineRemoveWPButton.UseVisualStyleBackColor = true;
            this.OnlineRemoveWPButton.Click += new System.EventHandler(this.RemoveRow_Click);
            // 
            // OfflineMapLabel
            // 
            this.OfflineMapLabel.AutoSize = true;
            this.OfflineMapLabel.BackColor = System.Drawing.Color.White;
            this.OfflineMapLabel.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.OfflineMapLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 9.75F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.OfflineMapLabel.ForeColor = System.Drawing.Color.Red;
            this.OfflineMapLabel.Location = new System.Drawing.Point(645, 383);
            this.OfflineMapLabel.Name = "OfflineMapLabel";
            this.OfflineMapLabel.Size = new System.Drawing.Size(203, 18);
            this.OfflineMapLabel.TabIndex = 53;
            this.OfflineMapLabel.Text = "Saved Image (Offline Mode)";
            this.OfflineMapLabel.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            // 
            // GPSStatsGroupBox
            // 
            this.GPSStatsGroupBox.Controls.Add(this.GPSFixLabel);
            this.GPSStatsGroupBox.Controls.Add(this.MinGPSFixS);
            this.GPSStatsGroupBox.Controls.Add(this.MaxGPSFixS);
            this.GPSStatsGroupBox.Controls.Add(this.HDiluteLabel);
            this.GPSStatsGroupBox.Controls.Add(this.MaxHDiluteS);
            this.GPSStatsGroupBox.Controls.Add(this.NavValidSLabel);
            this.GPSStatsGroupBox.Controls.Add(this.MinHDiluteS);
            this.GPSStatsGroupBox.Controls.Add(this.NavValidS);
            this.GPSStatsGroupBox.Controls.Add(this.GPSAltitudeLabel);
            this.GPSStatsGroupBox.Controls.Add(this.GPSVelLabel);
            this.GPSStatsGroupBox.Controls.Add(this.GPSVelS);
            this.GPSStatsGroupBox.Controls.Add(this.GPSAltitudeS);
            this.GPSStatsGroupBox.Controls.Add(this.SatsLabel);
            this.GPSStatsGroupBox.Controls.Add(this.MinGPSNoOfSatsS);
            this.GPSStatsGroupBox.Controls.Add(this.MaxGPSNoOfSatsS);
            this.GPSStatsGroupBox.Location = new System.Drawing.Point(7, 415);
            this.GPSStatsGroupBox.Name = "GPSStatsGroupBox";
            this.GPSStatsGroupBox.Size = new System.Drawing.Size(175, 78);
            this.GPSStatsGroupBox.TabIndex = 54;
            this.GPSStatsGroupBox.TabStop = false;
            this.GPSStatsGroupBox.Text = "GPS Stats";
            // 
            // GPSFixLabel
            // 
            this.GPSFixLabel.AutoSize = true;
            this.GPSFixLabel.Location = new System.Drawing.Point(80, 36);
            this.GPSFixLabel.Name = "GPSFixLabel";
            this.GPSFixLabel.Size = new System.Drawing.Size(23, 13);
            this.GPSFixLabel.TabIndex = 68;
            this.GPSFixLabel.Text = "Fix:";
            // 
            // MinGPSFixS
            // 
            this.MinGPSFixS.Location = new System.Drawing.Point(120, 36);
            this.MinGPSFixS.Name = "MinGPSFixS";
            this.MinGPSFixS.Size = new System.Drawing.Size(27, 13);
            this.MinGPSFixS.TabIndex = 67;
            this.MinGPSFixS.Text = "0";
            this.MinGPSFixS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // MaxGPSFixS
            // 
            this.MaxGPSFixS.Location = new System.Drawing.Point(143, 36);
            this.MaxGPSFixS.Name = "MaxGPSFixS";
            this.MaxGPSFixS.Size = new System.Drawing.Size(26, 13);
            this.MaxGPSFixS.TabIndex = 66;
            this.MaxGPSFixS.Text = "0";
            this.MaxGPSFixS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // HDiluteLabel
            // 
            this.HDiluteLabel.AutoSize = true;
            this.HDiluteLabel.Location = new System.Drawing.Point(80, 56);
            this.HDiluteLabel.Name = "HDiluteLabel";
            this.HDiluteLabel.Size = new System.Drawing.Size(45, 13);
            this.HDiluteLabel.TabIndex = 63;
            this.HDiluteLabel.Text = "HDilute:";
            // 
            // MaxHDiluteS
            // 
            this.MaxHDiluteS.Location = new System.Drawing.Point(143, 56);
            this.MaxHDiluteS.Name = "MaxHDiluteS";
            this.MaxHDiluteS.Size = new System.Drawing.Size(26, 13);
            this.MaxHDiluteS.TabIndex = 61;
            this.MaxHDiluteS.Text = "0";
            this.MaxHDiluteS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // NavValidSLabel
            // 
            this.NavValidSLabel.AutoSize = true;
            this.NavValidSLabel.Location = new System.Drawing.Point(6, 56);
            this.NavValidSLabel.Name = "NavValidSLabel";
            this.NavValidSLabel.Size = new System.Drawing.Size(33, 13);
            this.NavValidSLabel.TabIndex = 65;
            this.NavValidSLabel.Text = "Valid:";
            // 
            // MinHDiluteS
            // 
            this.MinHDiluteS.Location = new System.Drawing.Point(120, 56);
            this.MinHDiluteS.Name = "MinHDiluteS";
            this.MinHDiluteS.Size = new System.Drawing.Size(27, 13);
            this.MinHDiluteS.TabIndex = 62;
            this.MinHDiluteS.Text = "0";
            this.MinHDiluteS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // NavValidS
            // 
            this.NavValidS.Location = new System.Drawing.Point(30, 56);
            this.NavValidS.Name = "NavValidS";
            this.NavValidS.Size = new System.Drawing.Size(39, 13);
            this.NavValidS.TabIndex = 64;
            this.NavValidS.Text = "false";
            this.NavValidS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // SatsLabel
            // 
            this.SatsLabel.AutoSize = true;
            this.SatsLabel.Location = new System.Drawing.Point(80, 16);
            this.SatsLabel.Name = "SatsLabel";
            this.SatsLabel.Size = new System.Drawing.Size(34, 13);
            this.SatsLabel.TabIndex = 57;
            this.SatsLabel.Text = "Sats.:";
            // 
            // MinGPSNoOfSatsS
            // 
            this.MinGPSNoOfSatsS.Location = new System.Drawing.Point(126, 16);
            this.MinGPSNoOfSatsS.Name = "MinGPSNoOfSatsS";
            this.MinGPSNoOfSatsS.Size = new System.Drawing.Size(20, 13);
            this.MinGPSNoOfSatsS.TabIndex = 56;
            this.MinGPSNoOfSatsS.Text = "0";
            this.MinGPSNoOfSatsS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // MaxGPSNoOfSatsS
            // 
            this.MaxGPSNoOfSatsS.Location = new System.Drawing.Point(152, 16);
            this.MaxGPSNoOfSatsS.Name = "MaxGPSNoOfSatsS";
            this.MaxGPSNoOfSatsS.Size = new System.Drawing.Size(17, 13);
            this.MaxGPSNoOfSatsS.TabIndex = 55;
            this.MaxGPSNoOfSatsS.Text = "0";
            this.MaxGPSNoOfSatsS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // ErrorStatsGroupBox
            // 
            this.ErrorStatsGroupBox.Controls.Add(this.GyroSLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.GyroFailS);
            this.ErrorStatsGroupBox.Controls.Add(this.GPSInvalidSLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.GPSFailS);
            this.ErrorStatsGroupBox.Controls.Add(this.RCGlitchesS);
            this.ErrorStatsGroupBox.Controls.Add(this.CompassFailLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.AccFailLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.CompassFailS);
            this.ErrorStatsGroupBox.Controls.Add(this.BaroFailLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.BaroFailS);
            this.ErrorStatsGroupBox.Controls.Add(this.RCGlitchesSLabel);
            this.ErrorStatsGroupBox.Controls.Add(this.AccFailS);
            this.ErrorStatsGroupBox.Location = new System.Drawing.Point(314, 415);
            this.ErrorStatsGroupBox.Name = "ErrorStatsGroupBox";
            this.ErrorStatsGroupBox.Size = new System.Drawing.Size(149, 78);
            this.ErrorStatsGroupBox.TabIndex = 55;
            this.ErrorStatsGroupBox.TabStop = false;
            this.ErrorStatsGroupBox.Text = "Error Stats";
            // 
            // GyroSLabel
            // 
            this.GyroSLabel.AutoSize = true;
            this.GyroSLabel.Location = new System.Drawing.Point(75, 56);
            this.GyroSLabel.Name = "GyroSLabel";
            this.GyroSLabel.Size = new System.Drawing.Size(35, 13);
            this.GyroSLabel.TabIndex = 69;
            this.GyroSLabel.Text = "Gyro.:";
            // 
            // GyroFailS
            // 
            this.GyroFailS.Location = new System.Drawing.Point(101, 56);
            this.GyroFailS.Name = "GyroFailS";
            this.GyroFailS.Size = new System.Drawing.Size(35, 13);
            this.GyroFailS.TabIndex = 68;
            this.GyroFailS.Text = "0";
            this.GyroFailS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // GPSInvalidSLabel
            // 
            this.GPSInvalidSLabel.AutoSize = true;
            this.GPSInvalidSLabel.Location = new System.Drawing.Point(6, 56);
            this.GPSInvalidSLabel.Name = "GPSInvalidSLabel";
            this.GPSInvalidSLabel.Size = new System.Drawing.Size(32, 13);
            this.GPSInvalidSLabel.TabIndex = 67;
            this.GPSInvalidSLabel.Text = "GPS:";
            // 
            // GPSFailS
            // 
            this.GPSFailS.Location = new System.Drawing.Point(37, 56);
            this.GPSFailS.Name = "GPSFailS";
            this.GPSFailS.Size = new System.Drawing.Size(32, 13);
            this.GPSFailS.TabIndex = 66;
            this.GPSFailS.Text = "0";
            this.GPSFailS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // RCGlitchesS
            // 
            this.RCGlitchesS.Location = new System.Drawing.Point(25, 16);
            this.RCGlitchesS.Name = "RCGlitchesS";
            this.RCGlitchesS.Size = new System.Drawing.Size(44, 13);
            this.RCGlitchesS.TabIndex = 65;
            this.RCGlitchesS.Text = "0";
            this.RCGlitchesS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // CompassFailLabel
            // 
            this.CompassFailLabel.AutoSize = true;
            this.CompassFailLabel.Location = new System.Drawing.Point(75, 36);
            this.CompassFailLabel.Name = "CompassFailLabel";
            this.CompassFailLabel.Size = new System.Drawing.Size(40, 13);
            this.CompassFailLabel.TabIndex = 60;
            this.CompassFailLabel.Text = "Comp.:";
            // 
            // AccFailLabel
            // 
            this.AccFailLabel.AutoSize = true;
            this.AccFailLabel.Location = new System.Drawing.Point(6, 36);
            this.AccFailLabel.Name = "AccFailLabel";
            this.AccFailLabel.Size = new System.Drawing.Size(32, 13);
            this.AccFailLabel.TabIndex = 60;
            this.AccFailLabel.Text = "Acc.:";
            // 
            // CompassFailS
            // 
            this.CompassFailS.BackColor = System.Drawing.SystemColors.Control;
            this.CompassFailS.Location = new System.Drawing.Point(110, 36);
            this.CompassFailS.Name = "CompassFailS";
            this.CompassFailS.Size = new System.Drawing.Size(26, 13);
            this.CompassFailS.TabIndex = 59;
            this.CompassFailS.Text = "0";
            this.CompassFailS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // BaroFailLabel
            // 
            this.BaroFailLabel.AutoSize = true;
            this.BaroFailLabel.Location = new System.Drawing.Point(75, 16);
            this.BaroFailLabel.Name = "BaroFailLabel";
            this.BaroFailLabel.Size = new System.Drawing.Size(35, 13);
            this.BaroFailLabel.TabIndex = 58;
            this.BaroFailLabel.Text = "Baro.:";
            // 
            // BaroFailS
            // 
            this.BaroFailS.Location = new System.Drawing.Point(107, 16);
            this.BaroFailS.Name = "BaroFailS";
            this.BaroFailS.Size = new System.Drawing.Size(29, 13);
            this.BaroFailS.TabIndex = 55;
            this.BaroFailS.Text = "0";
            this.BaroFailS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // RCGlitchesSLabel
            // 
            this.RCGlitchesSLabel.AutoSize = true;
            this.RCGlitchesSLabel.Location = new System.Drawing.Point(6, 16);
            this.RCGlitchesSLabel.Name = "RCGlitchesSLabel";
            this.RCGlitchesSLabel.Size = new System.Drawing.Size(23, 13);
            this.RCGlitchesSLabel.TabIndex = 52;
            this.RCGlitchesSLabel.Text = "Rx:";
            // 
            // AccFailS
            // 
            this.AccFailS.Location = new System.Drawing.Point(37, 36);
            this.AccFailS.Name = "AccFailS";
            this.AccFailS.Size = new System.Drawing.Size(32, 13);
            this.AccFailS.TabIndex = 51;
            this.AccFailS.Text = "0";
            this.AccFailS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // BaroStatsGroupBox
            // 
            this.BaroStatsGroupBox.Controls.Add(this.GPSBaroScaleSLabel);
            this.BaroStatsGroupBox.Controls.Add(this.GPSBaroScaleS);
            this.BaroStatsGroupBox.Controls.Add(this.RelBaroPressureSLabel);
            this.BaroStatsGroupBox.Controls.Add(this.RelBaroPressureS);
            this.BaroStatsGroupBox.Controls.Add(this.ROCLabel);
            this.BaroStatsGroupBox.Controls.Add(this.MaxBaroROCS);
            this.BaroStatsGroupBox.Controls.Add(this.RelBaroAltitudeLabel);
            this.BaroStatsGroupBox.Controls.Add(this.RelBaroAltitudeS);
            this.BaroStatsGroupBox.Controls.Add(this.MinBaroROCS);
            this.BaroStatsGroupBox.Location = new System.Drawing.Point(187, 415);
            this.BaroStatsGroupBox.Name = "BaroStatsGroupBox";
            this.BaroStatsGroupBox.Size = new System.Drawing.Size(121, 78);
            this.BaroStatsGroupBox.TabIndex = 56;
            this.BaroStatsGroupBox.TabStop = false;
            this.BaroStatsGroupBox.Text = "Baro Stats";
            // 
            // GPSBaroScaleSLabel
            // 
            this.GPSBaroScaleSLabel.AutoSize = true;
            this.GPSBaroScaleSLabel.Location = new System.Drawing.Point(66, 16);
            this.GPSBaroScaleSLabel.Name = "GPSBaroScaleSLabel";
            this.GPSBaroScaleSLabel.Size = new System.Drawing.Size(33, 13);
            this.GPSBaroScaleSLabel.TabIndex = 71;
            this.GPSBaroScaleSLabel.Text = "Calib:";
            // 
            // GPSBaroScaleS
            // 
            this.GPSBaroScaleS.Location = new System.Drawing.Point(97, 16);
            this.GPSBaroScaleS.Name = "GPSBaroScaleS";
            this.GPSBaroScaleS.Size = new System.Drawing.Size(18, 13);
            this.GPSBaroScaleS.TabIndex = 70;
            this.GPSBaroScaleS.Text = "0";
            this.GPSBaroScaleS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // RelBaroPressureSLabel
            // 
            this.RelBaroPressureSLabel.AutoSize = true;
            this.RelBaroPressureSLabel.Location = new System.Drawing.Point(7, 56);
            this.RelBaroPressureSLabel.Name = "RelBaroPressureSLabel";
            this.RelBaroPressureSLabel.Size = new System.Drawing.Size(36, 13);
            this.RelBaroPressureSLabel.TabIndex = 69;
            this.RelBaroPressureSLabel.Text = "Press:";
            // 
            // RelBaroPressureS
            // 
            this.RelBaroPressureS.Location = new System.Drawing.Point(49, 56);
            this.RelBaroPressureS.Name = "RelBaroPressureS";
            this.RelBaroPressureS.Size = new System.Drawing.Size(31, 13);
            this.RelBaroPressureS.TabIndex = 68;
            this.RelBaroPressureS.Text = "0";
            this.RelBaroPressureS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // ROCLabel
            // 
            this.ROCLabel.AutoSize = true;
            this.ROCLabel.Location = new System.Drawing.Point(7, 36);
            this.ROCLabel.Name = "ROCLabel";
            this.ROCLabel.Size = new System.Drawing.Size(33, 13);
            this.ROCLabel.TabIndex = 53;
            this.ROCLabel.Text = "ROC:";
            // 
            // MaxBaroROCS
            // 
            this.MaxBaroROCS.Location = new System.Drawing.Point(86, 36);
            this.MaxBaroROCS.Name = "MaxBaroROCS";
            this.MaxBaroROCS.Size = new System.Drawing.Size(29, 13);
            this.MaxBaroROCS.TabIndex = 52;
            this.MaxBaroROCS.Text = "0";
            this.MaxBaroROCS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // RelBaroAltitudeLabel
            // 
            this.RelBaroAltitudeLabel.AutoSize = true;
            this.RelBaroAltitudeLabel.Location = new System.Drawing.Point(7, 16);
            this.RelBaroAltitudeLabel.Name = "RelBaroAltitudeLabel";
            this.RelBaroAltitudeLabel.Size = new System.Drawing.Size(22, 13);
            this.RelBaroAltitudeLabel.TabIndex = 49;
            this.RelBaroAltitudeLabel.Text = "Alt:";
            // 
            // RelBaroAltitudeS
            // 
            this.RelBaroAltitudeS.Location = new System.Drawing.Point(27, 16);
            this.RelBaroAltitudeS.Name = "RelBaroAltitudeS";
            this.RelBaroAltitudeS.Size = new System.Drawing.Size(34, 13);
            this.RelBaroAltitudeS.TabIndex = 48;
            this.RelBaroAltitudeS.Text = "0";
            this.RelBaroAltitudeS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // MinBaroROCS
            // 
            this.MinBaroROCS.Location = new System.Drawing.Point(38, 36);
            this.MinBaroROCS.Name = "MinBaroROCS";
            this.MinBaroROCS.Size = new System.Drawing.Size(42, 13);
            this.MinBaroROCS.TabIndex = 51;
            this.MinBaroROCS.Text = "0";
            this.MinBaroROCS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // NavGroupBox
            // 
            this.NavGroupBox.Controls.Add(this.NavStateTimeout);
            this.NavGroupBox.Controls.Add(this.NavStateTimeoutLabel);
            this.NavGroupBox.Controls.Add(this.AltitudeError);
            this.NavGroupBox.Controls.Add(this.label6);
            this.NavGroupBox.Controls.Add(this.WPHeadingLabel);
            this.NavGroupBox.Controls.Add(this.WPDistanceLabel);
            this.NavGroupBox.Controls.Add(this.DistanceToDesired);
            this.NavGroupBox.Controls.Add(this.CurrWPLabel);
            this.NavGroupBox.Controls.Add(this.CurrWP);
            this.NavGroupBox.Controls.Add(this.DesiredCourse);
            this.NavGroupBox.Controls.Add(this.HeadingSLabel);
            this.NavGroupBox.Controls.Add(this.HeadingS);
            this.NavGroupBox.Location = new System.Drawing.Point(469, 498);
            this.NavGroupBox.Name = "NavGroupBox";
            this.NavGroupBox.Size = new System.Drawing.Size(147, 157);
            this.NavGroupBox.TabIndex = 57;
            this.NavGroupBox.TabStop = false;
            this.NavGroupBox.Text = "Navigation";
            // 
            // NavStateTimeout
            // 
            this.NavStateTimeout.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.NavStateTimeout.Location = new System.Drawing.Point(91, 37);
            this.NavStateTimeout.Name = "NavStateTimeout";
            this.NavStateTimeout.Size = new System.Drawing.Size(50, 20);
            this.NavStateTimeout.TabIndex = 66;
            this.NavStateTimeout.Text = "-1";
            this.NavStateTimeout.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // NavStateTimeoutLabel
            // 
            this.NavStateTimeoutLabel.AutoSize = true;
            this.NavStateTimeoutLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.NavStateTimeoutLabel.Location = new System.Drawing.Point(6, 42);
            this.NavStateTimeoutLabel.Name = "NavStateTimeoutLabel";
            this.NavStateTimeoutLabel.Size = new System.Drawing.Size(48, 17);
            this.NavStateTimeoutLabel.TabIndex = 67;
            this.NavStateTimeoutLabel.Text = "Delay:";
            // 
            // AltitudeError
            // 
            this.AltitudeError.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.AltitudeError.Location = new System.Drawing.Point(91, 62);
            this.AltitudeError.Name = "AltitudeError";
            this.AltitudeError.Size = new System.Drawing.Size(50, 20);
            this.AltitudeError.TabIndex = 55;
            this.AltitudeError.Text = "-1";
            this.AltitudeError.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.label6.Location = new System.Drawing.Point(6, 64);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(47, 17);
            this.label6.TabIndex = 56;
            this.label6.Text = "AltErr:";
            // 
            // WPHeadingLabel
            // 
            this.WPHeadingLabel.AutoSize = true;
            this.WPHeadingLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.WPHeadingLabel.Location = new System.Drawing.Point(6, 108);
            this.WPHeadingLabel.Name = "WPHeadingLabel";
            this.WPHeadingLabel.Size = new System.Drawing.Size(57, 17);
            this.WPHeadingLabel.TabIndex = 54;
            this.WPHeadingLabel.Text = "Course:";
            // 
            // WPDistanceLabel
            // 
            this.WPDistanceLabel.AutoSize = true;
            this.WPDistanceLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.WPDistanceLabel.Location = new System.Drawing.Point(6, 86);
            this.WPDistanceLabel.Name = "WPDistanceLabel";
            this.WPDistanceLabel.Size = new System.Drawing.Size(40, 17);
            this.WPDistanceLabel.TabIndex = 53;
            this.WPDistanceLabel.Text = "Dist.:";
            // 
            // DistanceToDesired
            // 
            this.DistanceToDesired.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.DistanceToDesired.Location = new System.Drawing.Point(81, 84);
            this.DistanceToDesired.Name = "DistanceToDesired";
            this.DistanceToDesired.Size = new System.Drawing.Size(60, 20);
            this.DistanceToDesired.TabIndex = 52;
            this.DistanceToDesired.Text = "-1";
            this.DistanceToDesired.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // CurrWPLabel
            // 
            this.CurrWPLabel.AutoSize = true;
            this.CurrWPLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.CurrWPLabel.Location = new System.Drawing.Point(6, 20);
            this.CurrWPLabel.Name = "CurrWPLabel";
            this.CurrWPLabel.Size = new System.Drawing.Size(38, 17);
            this.CurrWPLabel.TabIndex = 65;
            this.CurrWPLabel.Text = "WP#";
            // 
            // CurrWP
            // 
            this.CurrWP.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.CurrWP.Location = new System.Drawing.Point(91, 18);
            this.CurrWP.Name = "CurrWP";
            this.CurrWP.Size = new System.Drawing.Size(50, 20);
            this.CurrWP.TabIndex = 64;
            this.CurrWP.Text = "-1";
            this.CurrWP.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // DesiredCourse
            // 
            this.DesiredCourse.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.DesiredCourse.Location = new System.Drawing.Point(81, 106);
            this.DesiredCourse.Name = "DesiredCourse";
            this.DesiredCourse.Size = new System.Drawing.Size(60, 20);
            this.DesiredCourse.TabIndex = 51;
            this.DesiredCourse.Text = "-1";
            this.DesiredCourse.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // HeadingSLabel
            // 
            this.HeadingSLabel.AutoSize = true;
            this.HeadingSLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HeadingSLabel.Location = new System.Drawing.Point(6, 130);
            this.HeadingSLabel.Name = "HeadingSLabel";
            this.HeadingSLabel.Size = new System.Drawing.Size(70, 17);
            this.HeadingSLabel.TabIndex = 63;
            this.HeadingSLabel.Text = "Compass:";
            // 
            // HeadingS
            // 
            this.HeadingS.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.HeadingS.Location = new System.Drawing.Point(81, 128);
            this.HeadingS.Name = "HeadingS";
            this.HeadingS.Size = new System.Drawing.Size(60, 20);
            this.HeadingS.TabIndex = 62;
            this.HeadingS.Text = "-1";
            this.HeadingS.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // FlyingButton
            // 
            this.FlyingButton.BackColor = System.Drawing.Color.Red;
            this.FlyingButton.Location = new System.Drawing.Point(897, 552);
            this.FlyingButton.Name = "FlyingButton";
            this.FlyingButton.Size = new System.Drawing.Size(93, 20);
            this.FlyingButton.TabIndex = 55;
            this.FlyingButton.Text = "Landed";
            this.FlyingButton.UseVisualStyleBackColor = false;
            this.FlyingButton.Click += new System.EventHandler(this.cmdClearFlying_Click);
            // 
            // BatteryVoltsLabel
            // 
            this.BatteryVoltsLabel.AutoSize = true;
            this.BatteryVoltsLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryVoltsLabel.Location = new System.Drawing.Point(6, 16);
            this.BatteryVoltsLabel.Name = "BatteryVoltsLabel";
            this.BatteryVoltsLabel.Size = new System.Drawing.Size(39, 17);
            this.BatteryVoltsLabel.TabIndex = 53;
            this.BatteryVoltsLabel.Text = "Volts";
            // 
            // BatteryVolts
            // 
            this.BatteryVolts.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryVolts.Location = new System.Drawing.Point(66, 14);
            this.BatteryVolts.Name = "BatteryVolts";
            this.BatteryVolts.Size = new System.Drawing.Size(50, 20);
            this.BatteryVolts.TabIndex = 52;
            this.BatteryVolts.Text = "-1";
            this.BatteryVolts.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // CurrentAltitude
            // 
            this.CurrentAltitude.Font = new System.Drawing.Font("Microsoft Sans Serif", 30F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.CurrentAltitude.Location = new System.Drawing.Point(473, 438);
            this.CurrentAltitude.Name = "CurrentAltitude";
            this.CurrentAltitude.Size = new System.Drawing.Size(137, 61);
            this.CurrentAltitude.TabIndex = 51;
            this.CurrentAltitude.Text = "-1";
            this.CurrentAltitude.TextAlign = System.Drawing.ContentAlignment.MiddleCenter;
            this.CurrentAltitude.Click += new System.EventHandler(this.CurrentAltitude_Click);
            // 
            // BatteryGroupBox
            // 
            this.BatteryGroupBox.Controls.Add(this.BatteryChargeLabel);
            this.BatteryGroupBox.Controls.Add(this.BatteryCharge);
            this.BatteryGroupBox.Controls.Add(this.BatteryCurrentLabel);
            this.BatteryGroupBox.Controls.Add(this.BatteryCurrent);
            this.BatteryGroupBox.Controls.Add(this.BatteryVoltsLabel);
            this.BatteryGroupBox.Controls.Add(this.BatteryVolts);
            this.BatteryGroupBox.Location = new System.Drawing.Point(874, 578);
            this.BatteryGroupBox.Name = "BatteryGroupBox";
            this.BatteryGroupBox.Size = new System.Drawing.Size(122, 77);
            this.BatteryGroupBox.TabIndex = 62;
            this.BatteryGroupBox.TabStop = false;
            this.BatteryGroupBox.Text = "Battery";
            // 
            // BatteryChargeLabel
            // 
            this.BatteryChargeLabel.AutoSize = true;
            this.BatteryChargeLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryChargeLabel.Location = new System.Drawing.Point(6, 55);
            this.BatteryChargeLabel.Name = "BatteryChargeLabel";
            this.BatteryChargeLabel.Size = new System.Drawing.Size(38, 17);
            this.BatteryChargeLabel.TabIndex = 57;
            this.BatteryChargeLabel.Text = "mAH";
            // 
            // BatteryCharge
            // 
            this.BatteryCharge.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryCharge.ForeColor = System.Drawing.Color.LightSalmon;
            this.BatteryCharge.Location = new System.Drawing.Point(66, 56);
            this.BatteryCharge.Name = "BatteryCharge";
            this.BatteryCharge.Size = new System.Drawing.Size(50, 15);
            this.BatteryCharge.TabIndex = 56;
            this.BatteryCharge.Text = "-1";
            this.BatteryCharge.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // BatteryCurrentLabel
            // 
            this.BatteryCurrentLabel.AutoSize = true;
            this.BatteryCurrentLabel.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryCurrentLabel.Location = new System.Drawing.Point(6, 36);
            this.BatteryCurrentLabel.Name = "BatteryCurrentLabel";
            this.BatteryCurrentLabel.Size = new System.Drawing.Size(43, 17);
            this.BatteryCurrentLabel.TabIndex = 55;
            this.BatteryCurrentLabel.Text = "Amps";
            // 
            // BatteryCurrent
            // 
            this.BatteryCurrent.BackColor = System.Drawing.SystemColors.Control;
            this.BatteryCurrent.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.BatteryCurrent.ForeColor = System.Drawing.Color.LightSalmon;
            this.BatteryCurrent.Location = new System.Drawing.Point(66, 37);
            this.BatteryCurrent.Name = "BatteryCurrent";
            this.BatteryCurrent.Size = new System.Drawing.Size(50, 15);
            this.BatteryCurrent.TabIndex = 54;
            this.BatteryCurrent.Text = "-1";
            this.BatteryCurrent.TextAlign = System.Drawing.ContentAlignment.MiddleRight;
            // 
            // FlagsGroupBox
            // 
            this.FlagsGroupBox.Controls.Add(this.RTHAltHoldBox);
            this.FlagsGroupBox.Controls.Add(this.UsingGPSAltBox);
            this.FlagsGroupBox.Controls.Add(this.RangefinderValidBox);
            this.FlagsGroupBox.Controls.Add(this.HoldingAltBox);
            this.FlagsGroupBox.Controls.Add(this.ReturnHomeBox);
            this.FlagsGroupBox.Controls.Add(this.CloseProximityBox);
            this.FlagsGroupBox.Controls.Add(this.TurnToWPBox);
            this.FlagsGroupBox.Controls.Add(this.UseRTHAutoDescendBox);
            this.FlagsGroupBox.Controls.Add(this.NavValidBox);
            this.FlagsGroupBox.Controls.Add(this.AttitudeHoldBox);
            this.FlagsGroupBox.Controls.Add(this.NavigateBox);
            this.FlagsGroupBox.Controls.Add(this.ProximityBox);
            this.FlagsGroupBox.Location = new System.Drawing.Point(6, 499);
            this.FlagsGroupBox.Name = "FlagsGroupBox";
            this.FlagsGroupBox.Size = new System.Drawing.Size(377, 56);
            this.FlagsGroupBox.TabIndex = 64;
            // 
            // RTHAltHoldBox
            // 
            this.RTHAltHoldBox.BackColor = System.Drawing.SystemColors.Window;
            this.RTHAltHoldBox.Location = new System.Drawing.Point(3, 3);
            this.RTHAltHoldBox.Name = "RTHAltHoldBox";
            this.RTHAltHoldBox.ReadOnly = true;
            this.RTHAltHoldBox.Size = new System.Drawing.Size(55, 20);
            this.RTHAltHoldBox.TabIndex = 124;
            this.RTHAltHoldBox.Text = "Alt Hold";
            this.RTHAltHoldBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // UsingGPSAltBox
            // 
            this.UsingGPSAltBox.BackColor = System.Drawing.SystemColors.Window;
            this.UsingGPSAltBox.Location = new System.Drawing.Point(64, 3);
            this.UsingGPSAltBox.Name = "UsingGPSAltBox";
            this.UsingGPSAltBox.ReadOnly = true;
            this.UsingGPSAltBox.Size = new System.Drawing.Size(55, 20);
            this.UsingGPSAltBox.TabIndex = 143;
            this.UsingGPSAltBox.Text = "GPS Alt";
            this.UsingGPSAltBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // RangefinderValidBox
            // 
            this.RangefinderValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.RangefinderValidBox.Location = new System.Drawing.Point(125, 3);
            this.RangefinderValidBox.Name = "RangefinderValidBox";
            this.RangefinderValidBox.ReadOnly = true;
            this.RangefinderValidBox.Size = new System.Drawing.Size(55, 20);
            this.RangefinderValidBox.TabIndex = 147;
            this.RangefinderValidBox.Text = "R. Finder";
            this.RangefinderValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // HoldingAltBox
            // 
            this.HoldingAltBox.BackColor = System.Drawing.SystemColors.Window;
            this.HoldingAltBox.Location = new System.Drawing.Point(186, 3);
            this.HoldingAltBox.Name = "HoldingAltBox";
            this.HoldingAltBox.ReadOnly = true;
            this.HoldingAltBox.Size = new System.Drawing.Size(55, 20);
            this.HoldingAltBox.TabIndex = 138;
            this.HoldingAltBox.Text = "Hold Alt";
            this.HoldingAltBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ReturnHomeBox
            // 
            this.ReturnHomeBox.BackColor = System.Drawing.SystemColors.Window;
            this.ReturnHomeBox.Location = new System.Drawing.Point(247, 3);
            this.ReturnHomeBox.Name = "ReturnHomeBox";
            this.ReturnHomeBox.ReadOnly = true;
            this.ReturnHomeBox.Size = new System.Drawing.Size(55, 20);
            this.ReturnHomeBox.TabIndex = 140;
            this.ReturnHomeBox.Text = "RTH";
            this.ReturnHomeBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // CloseProximityBox
            // 
            this.CloseProximityBox.BackColor = System.Drawing.SystemColors.Window;
            this.CloseProximityBox.Location = new System.Drawing.Point(308, 3);
            this.CloseProximityBox.Name = "CloseProximityBox";
            this.CloseProximityBox.ReadOnly = true;
            this.CloseProximityBox.Size = new System.Drawing.Size(55, 20);
            this.CloseProximityBox.TabIndex = 142;
            this.CloseProximityBox.Text = "Centered";
            this.CloseProximityBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // TurnToWPBox
            // 
            this.TurnToWPBox.BackColor = System.Drawing.SystemColors.Window;
            this.TurnToWPBox.Location = new System.Drawing.Point(3, 29);
            this.TurnToWPBox.Name = "TurnToWPBox";
            this.TurnToWPBox.ReadOnly = true;
            this.TurnToWPBox.Size = new System.Drawing.Size(55, 20);
            this.TurnToWPBox.TabIndex = 125;
            this.TurnToWPBox.Text = "WP Turn";
            this.TurnToWPBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // UseRTHAutoDescendBox
            // 
            this.UseRTHAutoDescendBox.BackColor = System.Drawing.SystemColors.Window;
            this.UseRTHAutoDescendBox.Location = new System.Drawing.Point(64, 29);
            this.UseRTHAutoDescendBox.Name = "UseRTHAutoDescendBox";
            this.UseRTHAutoDescendBox.ReadOnly = true;
            this.UseRTHAutoDescendBox.Size = new System.Drawing.Size(55, 20);
            this.UseRTHAutoDescendBox.TabIndex = 144;
            this.UseRTHAutoDescendBox.Text = "Auto Land";
            this.UseRTHAutoDescendBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NavValidBox
            // 
            this.NavValidBox.BackColor = System.Drawing.SystemColors.Window;
            this.NavValidBox.Location = new System.Drawing.Point(125, 29);
            this.NavValidBox.Name = "NavValidBox";
            this.NavValidBox.ReadOnly = true;
            this.NavValidBox.Size = new System.Drawing.Size(55, 20);
            this.NavValidBox.TabIndex = 131;
            this.NavValidBox.Text = "Home Set";
            this.NavValidBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // AttitudeHoldBox
            // 
            this.AttitudeHoldBox.BackColor = System.Drawing.SystemColors.Window;
            this.AttitudeHoldBox.Location = new System.Drawing.Point(186, 29);
            this.AttitudeHoldBox.Name = "AttitudeHoldBox";
            this.AttitudeHoldBox.ReadOnly = true;
            this.AttitudeHoldBox.Size = new System.Drawing.Size(55, 20);
            this.AttitudeHoldBox.TabIndex = 136;
            this.AttitudeHoldBox.Text = "Hold R&P";
            this.AttitudeHoldBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NavigateBox
            // 
            this.NavigateBox.BackColor = System.Drawing.SystemColors.Window;
            this.NavigateBox.Location = new System.Drawing.Point(247, 29);
            this.NavigateBox.Name = "NavigateBox";
            this.NavigateBox.ReadOnly = true;
            this.NavigateBox.Size = new System.Drawing.Size(55, 20);
            this.NavigateBox.TabIndex = 139;
            this.NavigateBox.Text = "Navigate";
            this.NavigateBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // ProximityBox
            // 
            this.ProximityBox.BackColor = System.Drawing.SystemColors.Window;
            this.ProximityBox.Location = new System.Drawing.Point(308, 29);
            this.ProximityBox.Name = "ProximityBox";
            this.ProximityBox.ReadOnly = true;
            this.ProximityBox.Size = new System.Drawing.Size(55, 20);
            this.ProximityBox.TabIndex = 141;
            this.ProximityBox.Text = "Achieved";
            this.ProximityBox.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // NavState
            // 
            this.NavState.BackColor = System.Drawing.SystemColors.Control;
            this.NavState.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.NavState.Location = new System.Drawing.Point(381, 505);
            this.NavState.Name = "NavState";
            this.NavState.ReadOnly = true;
            this.NavState.Size = new System.Drawing.Size(82, 13);
            this.NavState.TabIndex = 149;
            this.NavState.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // FailState
            // 
            this.FailState.BackColor = System.Drawing.SystemColors.Control;
            this.FailState.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.FailState.Location = new System.Drawing.Point(381, 531);
            this.FailState.Name = "FailState";
            this.FailState.ReadOnly = true;
            this.FailState.Size = new System.Drawing.Size(82, 13);
            this.FailState.TabIndex = 148;
            this.FailState.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // AltitudeSource
            // 
            this.AltitudeSource.BackColor = System.Drawing.SystemColors.Control;
            this.AltitudeSource.BorderStyle = System.Windows.Forms.BorderStyle.None;
            this.AltitudeSource.Font = new System.Drawing.Font("Microsoft Sans Serif", 10F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.AltitudeSource.Location = new System.Drawing.Point(494, 419);
            this.AltitudeSource.Name = "AltitudeSource";
            this.AltitudeSource.ReadOnly = true;
            this.AltitudeSource.Size = new System.Drawing.Size(99, 16);
            this.AltitudeSource.TabIndex = 150;
            this.AltitudeSource.TextAlign = System.Windows.Forms.HorizontalAlignment.Center;
            // 
            // WhereGroupBox
            // 
            this.WhereGroupBox.Controls.Add(this.WhereBearingLabel);
            this.WhereGroupBox.Controls.Add(this.WhereDistanceLabel);
            this.WhereGroupBox.Controls.Add(this.WhereDistance);
            this.WhereGroupBox.Controls.Add(this.WhereBearing);
            this.WhereGroupBox.Location = new System.Drawing.Point(7, 341);
            this.WhereGroupBox.Name = "WhereGroupBox";
            this.WhereGroupBox.Size = new System.Drawing.Size(103, 57);
            this.WhereGroupBox.TabIndex = 151;
            this.WhereGroupBox.TabStop = false;
            this.WhereGroupBox.Text = "Where?";
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
            // OpenLogFileDialog
            // 
            this.OpenLogFileDialog.FileName = "openFileDialog1";
            // 
            // headingIndicatorInstrumentControl1
            // 
            this.headingIndicatorInstrumentControl1.Location = new System.Drawing.Point(874, 415);
            this.headingIndicatorInstrumentControl1.Name = "headingIndicatorInstrumentControl1";
            this.headingIndicatorInstrumentControl1.Size = new System.Drawing.Size(128, 132);
            this.headingIndicatorInstrumentControl1.TabIndex = 60;
            this.headingIndicatorInstrumentControl1.Text = "headingIndicatorInstrumentControl1";
            // 
            // attitudeIndicatorInstrumentControl1
            // 
            this.attitudeIndicatorInstrumentControl1.Location = new System.Drawing.Point(622, 415);
            this.attitudeIndicatorInstrumentControl1.Name = "attitudeIndicatorInstrumentControl1";
            this.attitudeIndicatorInstrumentControl1.Size = new System.Drawing.Size(237, 240);
            this.attitudeIndicatorInstrumentControl1.TabIndex = 59;
            this.attitudeIndicatorInstrumentControl1.Text = "attitudeIndicatorInstrumentControl1";
            // 
            // eventLog1
            // 
            this.eventLog1.SynchronizingObject = this;
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(1014, 659);
            this.Controls.Add(this.WhereGroupBox);
            this.Controls.Add(this.AltitudeSource);
            this.Controls.Add(this.FlagsGroupBox);
            this.Controls.Add(this.BatteryGroupBox);
            this.Controls.Add(this.headingIndicatorInstrumentControl1);
            this.Controls.Add(this.attitudeIndicatorInstrumentControl1);
            this.Controls.Add(this.NavGroupBox);
            this.Controls.Add(this.NavState);
            this.Controls.Add(this.FailState);
            this.Controls.Add(this.FlyingButton);
            this.Controls.Add(this.BaroStatsGroupBox);
            this.Controls.Add(this.ErrorStatsGroupBox);
            this.Controls.Add(this.CurrentAltitude);
            this.Controls.Add(this.GPSStatsGroupBox);
            this.Controls.Add(this.OriginGroupBox);
            this.Controls.Add(this.OfflineMapLabel);
            this.Controls.Add(this.OtherGroupBox);
            this.Controls.Add(this.ClickMapLabel);
            this.Controls.Add(this.StatusGroupBox);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.M);
            this.Controls.Add(this.MenuMain);
            this.Controls.Add(this.OfflineMap);
            this.Controls.Add(this.webBrowser1);
            this.Controls.Add(this.OnlineGroupBox);
            this.Controls.Add(this.OfflineGroupBox);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.MainMenuStrip = this.MenuMain;
            this.MaximizeBox = false;
            this.Name = "Form1";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "UAVXNav ";
            ((System.ComponentModel.ISupportInitialize)(this.M)).EndInit();
            this.MenuMain.ResumeLayout(false);
            this.MenuMain.PerformLayout();
            this.OriginGroupBox.ResumeLayout(false);
            this.OriginGroupBox.PerformLayout();
            this.StatusGroupBox.ResumeLayout(false);
            this.OtherGroupBox.ResumeLayout(false);
            this.OtherGroupBox.PerformLayout();
            this.OfflineGroupBox.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.OfflineMap)).EndInit();
            this.OnlineGroupBox.ResumeLayout(false);
            this.OnlineGroupBox.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ZoomMaps)).EndInit();
            this.GPSStatsGroupBox.ResumeLayout(false);
            this.GPSStatsGroupBox.PerformLayout();
            this.ErrorStatsGroupBox.ResumeLayout(false);
            this.ErrorStatsGroupBox.PerformLayout();
            this.BaroStatsGroupBox.ResumeLayout(false);
            this.BaroStatsGroupBox.PerformLayout();
            this.NavGroupBox.ResumeLayout(false);
            this.NavGroupBox.PerformLayout();
            this.BatteryGroupBox.ResumeLayout(false);
            this.BatteryGroupBox.PerformLayout();
            this.FlagsGroupBox.ResumeLayout(false);
            this.FlagsGroupBox.PerformLayout();
            this.WhereGroupBox.ResumeLayout(false);
            this.WhereGroupBox.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.eventLog1)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.DataGridView M;
        private System.Windows.Forms.TextBox LaunchLat;
        private System.Windows.Forms.TextBox LaunchLon;
        private System.Windows.Forms.CheckBox LaunchManuallyCheckBox;
        private System.Windows.Forms.MenuStrip MenuMain;
        private System.Windows.Forms.TextBox OriginAltitude;
        private System.Windows.Forms.GroupBox OriginGroupBox;
        private System.Windows.Forms.Label RTHAltitudeLabel;
        private System.Windows.Forms.Label LaunchLonLabel;
        private System.Windows.Forms.Label LaunchLatLabel;
        private System.Windows.Forms.Label OriginAltitudeLabel;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.WebBrowser webBrowser1;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.GroupBox StatusGroupBox;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Label statusLabel;
        private System.Windows.Forms.ToolStripMenuItem UAVXNavToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem MenuFiles;
        private System.Windows.Forms.ToolStripMenuItem MenuLoadMission;
        private System.Windows.Forms.ToolStripMenuItem MenuSaveMission;
        private System.Windows.Forms.ToolStripMenuItem MenuSaveMissionAs;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator5;
        private System.Windows.Forms.ToolStripComboBox cboComSelect;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator6;
        private System.Windows.Forms.ToolStripMenuItem MenuRead;
        private System.Windows.Forms.ToolStripMenuItem MenuWrite;
        private System.Windows.Forms.ToolStripSeparator toolStripSeparator7;
        private System.Windows.Forms.ToolStripMenuItem MenuExit;
        private System.Windows.Forms.CheckBox LookupAltCheckBox;
        private System.Windows.Forms.Button cmdWrite;
        private System.Windows.Forms.Button cmdRead;
        private System.Windows.Forms.Label ClickMapLabel;
        private System.Windows.Forms.GroupBox OtherGroupBox;
        private System.Windows.Forms.TextBox ProximityRadius;
        private System.Windows.Forms.Label HomeAltLabel;
        private System.Windows.Forms.Label HomeAlt;
        private System.Windows.Forms.Label GPSAltitudeLabel;
        private System.Windows.Forms.Label GPSVelLabel;
        private System.Windows.Forms.Label GPSVelS;
        private System.Windows.Forms.Label GPSAltitudeS;
        private System.Windows.Forms.GroupBox OfflineGroupBox;
        private System.Windows.Forms.Button OfflineAddWPButton;
        private System.Windows.Forms.Button OfflineClearAllButton;
        private System.Windows.Forms.Button OfflineRemoveWPButton;
        private System.Windows.Forms.PictureBox OfflineMap;
        private System.Windows.Forms.GroupBox OnlineGroupBox;
        private System.Windows.Forms.Button CentreButton;
        private System.Windows.Forms.Button OnlineClearAllButton;
        private System.Windows.Forms.RadioButton optWP;
        private System.Windows.Forms.RadioButton optHome;
        private System.Windows.Forms.Button SearchButton;
        private System.Windows.Forms.Label AddressLabel;
        private System.Windows.Forms.TextBox LocationAddress;
        private System.Windows.Forms.Label MapZoomLevel;
        private System.Windows.Forms.NumericUpDown ZoomMaps;
        private System.Windows.Forms.Button OnlineRemoveWPButton;
        private System.Windows.Forms.Label OfflineMapLabel;
        private System.Windows.Forms.GroupBox GPSStatsGroupBox;
        private System.Windows.Forms.Label SatsLabel;
        private System.Windows.Forms.Label MinGPSNoOfSatsS;
        private System.Windows.Forms.Label MaxGPSNoOfSatsS;
        private System.Windows.Forms.Label HDiluteLabel;
        private System.Windows.Forms.Label MinHDiluteS;
        private System.Windows.Forms.Label MaxHDiluteS;
        private System.Windows.Forms.GroupBox ErrorStatsGroupBox;
        private System.Windows.Forms.Label RCGlitchesS;
        private System.Windows.Forms.Label CompassFailLabel;
        private System.Windows.Forms.Label AccFailLabel;
        private System.Windows.Forms.Label CompassFailS;
        private System.Windows.Forms.Label BaroFailLabel;
        private System.Windows.Forms.Label BaroFailS;
        private System.Windows.Forms.Label RCGlitchesSLabel;
        private System.Windows.Forms.Label AccFailS;
        private System.Windows.Forms.GroupBox BaroStatsGroupBox;
        private System.Windows.Forms.Label RelBaroAltitudeLabel;
        private System.Windows.Forms.Label RelBaroAltitudeS;
        private System.Windows.Forms.Label MinBaroROCS;
        private System.Windows.Forms.Label MaxBaroROCS;
        private System.Windows.Forms.Label ROCLabel;
        private System.Windows.Forms.Label ProximityAltLabel;
        private System.Windows.Forms.TextBox ProximityAlt;
        private System.Windows.Forms.Label ProxRadiusLabel;
        private System.Windows.Forms.Label GPSInvalidSLabel;
        private System.Windows.Forms.Label GPSFailS;
        private System.Windows.Forms.Label RelBaroPressureSLabel;
        private System.Windows.Forms.Label RelBaroPressureS;
        private System.Windows.Forms.Label GPSBaroScaleSLabel;
        private System.Windows.Forms.Label GPSBaroScaleS;
        private System.Windows.Forms.Label GyroSLabel;
        private System.Windows.Forms.Label GyroFailS;
        private System.Windows.Forms.Label NavValidSLabel;
        private System.Windows.Forms.Label NavValidS;
        private System.Windows.Forms.GroupBox NavGroupBox;
        private System.Windows.Forms.Label BatteryVoltsLabel;
        private System.Windows.Forms.Label BatteryVolts;
        private System.Windows.Forms.Label CurrentAltitude;
        private System.Windows.Forms.DataGridViewTextBoxColumn Number;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lat;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lon;
        private System.Windows.Forms.DataGridViewTextBoxColumn Altitude;
        private System.Windows.Forms.DataGridViewTextBoxColumn Hidden_Alt;
        private System.Windows.Forms.DataGridViewTextBoxColumn SeaLevelAlt;
        private System.Windows.Forms.DataGridViewTextBoxColumn Loiter;
        private System.Windows.Forms.Label RTHAltitude;
        private System.Windows.Forms.Button FlyingButton;
        private Instruments.AttitudeIndicatorInstrumentControl attitudeIndicatorInstrumentControl1;
        private Instruments.HeadingIndicatorInstrumentControl headingIndicatorInstrumentControl1;
        private System.Windows.Forms.Label WPHeadingLabel;
        private System.Windows.Forms.Label WPDistanceLabel;
        private System.Windows.Forms.Label DistanceToDesired;
        private System.Windows.Forms.Label DesiredCourse;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.Label AltitudeError;
        private System.Windows.Forms.Label GPSFixLabel;
        private System.Windows.Forms.Label MinGPSFixS;
        private System.Windows.Forms.Label MaxGPSFixS;
        private System.Windows.Forms.Label HeadingSLabel;
        private System.Windows.Forms.Label HeadingS;
        private System.Windows.Forms.Label CurrWPLabel;
        private System.Windows.Forms.Label CurrWP;
        private System.Windows.Forms.GroupBox BatteryGroupBox;
        private System.Windows.Forms.Label BatteryCurrentLabel;
        private System.Windows.Forms.Label BatteryCurrent;
        private System.Windows.Forms.FlowLayoutPanel FlagsGroupBox;
        private System.Windows.Forms.TextBox NavValidBox;
        private System.Windows.Forms.TextBox HoldingAltBox;
        private System.Windows.Forms.TextBox AttitudeHoldBox;
        private System.Windows.Forms.TextBox NavigateBox;
        private System.Windows.Forms.TextBox ReturnHomeBox;
        private System.Windows.Forms.TextBox ProximityBox;
        private System.Windows.Forms.TextBox CloseProximityBox;
        private System.Windows.Forms.TextBox UseRTHAutoDescendBox;
        private System.Windows.Forms.TextBox RangefinderValidBox;
        private System.Windows.Forms.TextBox TurnToWPBox;
        private System.Windows.Forms.TextBox RTHAltHoldBox;
        private System.Windows.Forms.TextBox UsingGPSAltBox;
        private System.Windows.Forms.TextBox NavState;
        private System.Windows.Forms.TextBox FailState;
        private System.Windows.Forms.TextBox AltitudeSource;
        private System.Windows.Forms.GroupBox WhereGroupBox;
        private System.Windows.Forms.Label WhereBearingLabel;
        private System.Windows.Forms.Label WhereDistanceLabel;
        private System.Windows.Forms.Label WhereDistance;
        private System.Windows.Forms.Label WhereBearing;
        private System.Windows.Forms.Label RangeLimitLabel;
        private System.Windows.Forms.TextBox RangeLimitSetting;
        private System.Windows.Forms.Label BatteryChargeLabel;
        private System.Windows.Forms.Label BatteryCharge;
        private System.Windows.Forms.Label NavStateTimeout;
        private System.Windows.Forms.Label NavStateTimeoutLabel;
        private System.Windows.Forms.ToolStripComboBox COMSelectComboBox;
        private System.Windows.Forms.ToolStripComboBox COMDisarmedBaudRateComboBox;
        private System.Windows.Forms.SaveFileDialog SaveLogFileDialog;
        private System.Windows.Forms.OpenFileDialog OpenLogFileDialog;
        private System.Windows.Forms.ToolStripComboBox COMArmedBaudRateComboBox;
        private System.Diagnostics.EventLog eventLog1;
    }
}

