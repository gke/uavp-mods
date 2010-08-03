; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

[Setup]
AppName=UAVPSet
AppVerName=UAVPSet 2.4
AppPublisher=Universal Aerial Video Platform
AppPublisherURL=http://www.uavp.de
AppSupportURL=http://www.uavp.de
AppUpdatesURL=http://www.uavp.de
DefaultDirName={pf}\UAVPSet
DefaultGroupName=UAVPSet
AllowNoIcons=yes
LicenseFile=D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\License.txt
InfoBeforeFile=D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\InstallInfo.txt
InfoAfterFile=D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\ChangeLog.txt
OutputDir=D:\Data\Visual Studio 2008\Projects\UAVPSet\setup
OutputBaseFilename=UAVPSet_2.4_setup
SetupIconFile=D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\uavpset.ico
Compression=lzma
SolidCompression=yes
ChangesAssociations=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "german"; MessagesFile: "compiler:Languages\German.isl"
Name: "french"; MessagesFile: "compiler:Languages\French.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\UAVPSet.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\UAVPSet.exe.config"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\UAVPSet.exe.manifest"; DestDir: "{app}"; Flags: ignoreversion
;Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\ZedGraph.dll"; DestDir: "{app}"; Flags: ignoreversion
;Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\ZedGraph.xml"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\ChangeLog.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\InstallInfo.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\License.txt"; DestDir: "{app}"; Flags: ignoreversion
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\de\*"; DestDir: "{app}\de"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "D:\Data\Visual Studio 2008\Projects\UAVPSet\Trunk\UAVPSet\bin\Release\fr\*"; DestDir: "{app}\fr"; Flags: ignoreversion recursesubdirs createallsubdirs
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{group}\UAVPSet"; Filename: "{app}\UAVPSet.exe"
Name: "{group}\{cm:ProgramOnTheWeb,UAVPSet}"; Filename: "http://www.uavp.de"
Name: "{group}\{cm:UninstallProgram,UAVPSet}"; Filename: "{uninstallexe}"
Name: "{commondesktop}\UAVPSet"; Filename: "{app}\UAVPSet.exe"; Tasks: desktopicon
Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\UAVPSet"; Filename: "{app}\UAVPSet.exe"; Tasks: quicklaunchicon

[Run]
Filename: "{app}\UAVPSet.exe"; Description: "{cm:LaunchProgram,UAVPSet}"; Flags: nowait postinstall skipifsilent

[Registry]
Root: HKCR; Subkey: ".auv"; ValueType: string; ValueName: ""; ValueData: "UAVPSetParameter"; Flags: uninsdeletevalue
Root: HKCR; Subkey: "UAVPSetParameter"; ValueType: string; ValueName: ""; ValueData: "UAVPSet Parameter File"; Flags: uninsdeletekey
Root: HKCR; Subkey: "UAVPSetParameter\DefaultIcon"; ValueType: string; ValueName: ""; ValueData: "{app}\UAVPSet.exe,0"
Root: HKCR; Subkey: "UAVPSetParameter\shell\open\command"; ValueType: string; ValueName: ""; ValueData: """{app}\UAVPSet.exe"" ""%1"""

