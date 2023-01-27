@echo off
echo Set oWS = WScript.CreateObject("WScript.Shell") > CreateShortcut.vbs
echo sLinkFile = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\16Y Project Manager.lnk" >> CreateShortcut.vbs
echo Set oLink = oWS.CreateShortcut(sLinkFile) >> CreateShortcut.vbs
echo oLink.TargetPath = "%0\..\run_project_manager.bat" >> CreateShortcut.vbs
echo oLink.IconLocation = "%0\..\documentation\fruitfly.ico" >> CreateShortcut.vbs
echo oLink.WorkingDirectory = "%0\..\" >> CreateShortcut.vbs
echo oLink.Save >> CreateShortcut.vbs
cscript CreateShortcut.vbs
del CreateShortcut.vbs

echo Set oWS = WScript.CreateObject("WScript.Shell") > CreateShortcut.vbs
echo sLinkFile = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\16Y Experimenter.lnk" >> CreateShortcut.vbs
echo Set oLink = oWS.CreateShortcut(sLinkFile) >> CreateShortcut.vbs
echo oLink.TargetPath = "%0\..\run_experiment.bat" >> CreateShortcut.vbs
echo oLink.IconLocation = "%0\..\documentation\fruitfly.ico" >> CreateShortcut.vbs
echo oLink.WorkingDirectory = "%0\..\" >> CreateShortcut.vbs
echo oLink.Save >> CreateShortcut.vbs
cscript CreateShortcut.vbs
del CreateShortcut.vbs

echo Set oWS = WScript.CreateObject("WScript.Shell") > CreateShortcut.vbs
echo sLinkFile = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\16Y Mask Designer.lnk" >> CreateShortcut.vbs
echo Set oLink = oWS.CreateShortcut(sLinkFile) >> CreateShortcut.vbs
echo oLink.TargetPath = "%0\..\run_mask_designer.bat" >> CreateShortcut.vbs
echo oLink.IconLocation = "%0\..\documentation\fruitfly.ico" >> CreateShortcut.vbs
echo oLink.WorkingDirectory = "%0\..\" >> CreateShortcut.vbs
echo oLink.Save >> CreateShortcut.vbs
cscript CreateShortcut.vbs
del CreateShortcut.vbs

echo Set oWS = WScript.CreateObject("WScript.Shell") > CreateShortcut.vbs
echo sLinkFile = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\16Y Rig Configurator.lnk" >> CreateShortcut.vbs
echo Set oLink = oWS.CreateShortcut(sLinkFile) >> CreateShortcut.vbs
echo oLink.TargetPath = "%0\..\run_rig_configurator.bat" >> CreateShortcut.vbs
echo oLink.IconLocation = "%0\..\documentation\fruitfly.ico" >> CreateShortcut.vbs
echo oLink.WorkingDirectory = "%0\..\" >> CreateShortcut.vbs
echo oLink.Save >> CreateShortcut.vbs
cscript CreateShortcut.vbs
del CreateShortcut.vbs