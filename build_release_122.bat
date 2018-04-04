cd /D "%~dp0"

md "_release"
md "_release\GameData122"
md "_release\GameData122\GameData"
md "_release\GameData122\GameData\KerbalJointReinforcement"
md "_release\GameData122\GameData\KerbalJointReinforcement\Plugins"
md "_release\GameData122\GameData\KerbalJointReinforcement\Plugins\PluginData"
md "_release\GameData122\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement"

copy "GameData\KerbalJointReinforcement\KerbalJointReinforcement_1.2.2.version" "_release\GameData122\GameData\KerbalJointReinforcement\KerbalJointReinforcement.version"

copy "KerbalJointReinforcement\KerbalJointReinforcement\bin\Release 1.2.2\KerbalJointReinforcement.dll" "_release\GameData122\GameData\KerbalJointReinforcement\Plugins\KerbalJointReinforcement.dll"

copy "KerbalJointReinforcement\KerbalJointReinforcement\bin\Release 1.2.2\KerbalJointReinforcement_Redist.dll" "_release\GameData122\GameData\KerbalJointReinforcement\Plugins\KerbalJointReinforcement_Redist.dll"

copy "GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml" "_release\GameData122\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml"

C:\PACL\PACOMP.EXE -a -r -p "_release\KerbalJointReinforcement_v3.4.1_for_1.2.2.zip" "_release\GameData122\*"
