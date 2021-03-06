; Script generated by the Inno Setup Script Wizard.
; SEE THE DOCUMENTATION FOR DETAILS ON CREATING INNO SETUP SCRIPT FILES!

#define MyAppName "Electrostimulator"
#define MyAppVersion "1.0"
#define MyAppPublisher "Buso Technology"
#define MyAppURL "http://www.busotech.com/"
#define MyAppExeName "Serial.exe"

[Setup]
; NOTE: The value of AppId uniquely identifies this application.
; Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{2BA55B03-7C1B-4451-A955-7F7D2BC10B52}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
AppPublisherURL={#MyAppURL}
AppSupportURL={#MyAppURL}
AppUpdatesURL={#MyAppURL}
DefaultDirName={pf}\{#MyAppName}
DefaultGroupName={#MyAppName}
LicenseFile=E:\KLab\Electric Stimulator\Electrostimulation\License.txt
OutputDir=E:\KLab\Electric Stimulator\Electrostimulation\Software\Installer
OutputBaseFilename=serial-setup
SetupIconFile=E:\KLab\Electric Stimulator\Electrostimulation\Software\Serial\Logo.ico
Compression=lzma
SolidCompression=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 0,6.1

[Files]
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\Serial.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\dialog.o"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\icudt52.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\icuin52.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\icuuc52.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\libgcc_s_dw2-1.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\libstdc++-6.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\libwinpthread-1.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\main.o"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\moc_dialog.cpp"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\moc_dialog.o"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\myapp_res.o"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\Qt5Core.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\Qt5Gui.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\Qt5SerialPort.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "E:\KLab\Electric Stimulator\Electrostimulation\Software\build-Serial-Desktop_Qt_5_3_MinGW_32bit-Release\release\Qt5Widgets.dll"; DestDir: "{app}"; Flags: ignoreversion
; NOTE: Don't use "Flags: ignoreversion" on any shared system files

[Icons]
Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
Name: "{commondesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon
Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; Tasks: quicklaunchicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent

