namespace UAVXGS
{
    partial class UAVXNavForm
    {

        private System.ComponentModel.IContainer components = null;

     /*    protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }
*/
        #region Windows Form Designer generated code

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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(UAVXNavForm));
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
            this.webBrowser1 = new System.Windows.Forms.WebBrowser();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.ClickMapLabel = new System.Windows.Forms.Label();
            this.OtherGroupBox = new System.Windows.Forms.GroupBox();
            this.RangeLimitLabel = new System.Windows.Forms.Label();
            this.RangeLimitSetting = new System.Windows.Forms.TextBox();
            this.ProximityAltLabel = new System.Windows.Forms.Label();
            this.ProximityAlt = new System.Windows.Forms.TextBox();
            this.ProxRadiusLabel = new System.Windows.Forms.Label();
            this.ProximityRadius = new System.Windows.Forms.TextBox();
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
            this.AltitudeSource = new System.Windows.Forms.TextBox();
            this.SaveLogFileDialog = new System.Windows.Forms.SaveFileDialog();
            this.OpenLogFileDialog = new System.Windows.Forms.OpenFileDialog();
            this.eventLog1 = new System.Diagnostics.EventLog();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.statusLabel = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.M)).BeginInit();
            this.MenuMain.SuspendLayout();
            this.OriginGroupBox.SuspendLayout();
            this.OtherGroupBox.SuspendLayout();
            this.OfflineGroupBox.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.OfflineMap)).BeginInit();
            this.OnlineGroupBox.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ZoomMaps)).BeginInit();
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
            this.MenuFiles});
            this.MenuMain.Location = new System.Drawing.Point(0, 0);
            this.MenuMain.Name = "MenuMain";
            this.MenuMain.Size = new System.Drawing.Size(1019, 24);
            this.MenuMain.TabIndex = 13;
            this.MenuMain.Text = "menuStrip1";
            // 
            // UAVXNavToolStripMenuItem
            // 
            this.UAVXNavToolStripMenuItem.Alignment = System.Windows.Forms.ToolStripItemAlignment.Right;
            this.UAVXNavToolStripMenuItem.Name = "UAVXNavToolStripMenuItem";
            this.UAVXNavToolStripMenuItem.Size = new System.Drawing.Size(102, 20);
            this.UAVXNavToolStripMenuItem.Text = "G.K. Egan (2010)";
            this.UAVXNavToolStripMenuItem.Click += new System.EventHandler(this.UAVXNavToolStripMenuItem_Click);
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
            this.MenuFiles.Size = new System.Drawing.Size(72, 20);
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
            this.OtherGroupBox.Size = new System.Drawing.Size(105, 162);
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
            this.OfflineMap.Location = new System.Drawing.Point(463, 31);
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
            // OpenLogFileDialog
            // 
            this.OpenLogFileDialog.FileName = "openFileDialog1";
            // 
            // eventLog1
            // 
            this.eventLog1.SynchronizingObject = this;
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(6, 380);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(100, 23);
            this.progressBar1.TabIndex = 151;
            // 
            // statusLabel
            // 
            this.statusLabel.Location = new System.Drawing.Point(194, 429);
            this.statusLabel.Name = "statusLabel";
            this.statusLabel.Size = new System.Drawing.Size(100, 20);
            this.statusLabel.TabIndex = 152;
            // 
            // UAVXNavForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.AutoSize = true;
            this.ClientSize = new System.Drawing.Size(1019, 454);
            this.Controls.Add(this.statusLabel);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.AltitudeSource);
            this.Controls.Add(this.OriginGroupBox);
            this.Controls.Add(this.OfflineMapLabel);
            this.Controls.Add(this.OtherGroupBox);
            this.Controls.Add(this.ClickMapLabel);
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
            this.Name = "UAVXNavForm";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "UAVXNav ";
            ((System.ComponentModel.ISupportInitialize)(this.M)).EndInit();
            this.MenuMain.ResumeLayout(false);
            this.MenuMain.PerformLayout();
            this.OriginGroupBox.ResumeLayout(false);
            this.OriginGroupBox.PerformLayout();
            this.OtherGroupBox.ResumeLayout(false);
            this.OtherGroupBox.PerformLayout();
            this.OfflineGroupBox.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.OfflineMap)).EndInit();
            this.OnlineGroupBox.ResumeLayout(false);
            this.OnlineGroupBox.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.ZoomMaps)).EndInit();
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
        private System.Windows.Forms.WebBrowser webBrowser1;
        private System.Windows.Forms.Timer timer1;
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
        private System.Windows.Forms.Label ClickMapLabel;
        private System.Windows.Forms.GroupBox OtherGroupBox;
        private System.Windows.Forms.TextBox ProximityRadius;
        private System.Windows.Forms.Label HomeAltLabel;
        private System.Windows.Forms.Label HomeAlt;
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
        private System.Windows.Forms.Label ProximityAltLabel;
        private System.Windows.Forms.TextBox ProximityAlt;
        private System.Windows.Forms.Label ProxRadiusLabel;
        private System.Windows.Forms.DataGridViewTextBoxColumn Number;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lat;
        private System.Windows.Forms.DataGridViewTextBoxColumn Lon;
        private System.Windows.Forms.DataGridViewTextBoxColumn Altitude;
        private System.Windows.Forms.DataGridViewTextBoxColumn Hidden_Alt;
        private System.Windows.Forms.DataGridViewTextBoxColumn SeaLevelAlt;
        private System.Windows.Forms.DataGridViewTextBoxColumn Loiter;
        private System.Windows.Forms.Label RTHAltitude;
        private System.Windows.Forms.TextBox AltitudeSource;
        private System.Windows.Forms.Label RangeLimitLabel;
        private System.Windows.Forms.TextBox RangeLimitSetting;
        private System.Windows.Forms.SaveFileDialog SaveLogFileDialog;
        private System.Windows.Forms.OpenFileDialog OpenLogFileDialog;
        private System.Diagnostics.EventLog eventLog1;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.TextBox statusLabel;
    }
}

