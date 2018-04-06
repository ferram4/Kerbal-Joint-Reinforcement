cd /D "%~dp0"

md "_release"
md "_release\GameData131"
md "_release\GameData131\GameData"
md "_release\GameData131\GameData\KerbalJointReinforcement"
md "_release\GameData131\GameData\KerbalJointReinforcement\Plugins"
md "_release\GameData131\GameData\KerbalJointReinforcement\Plugins\PluginData"
md "_release\GameData131\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement"

copy "GameData\KerbalJointReinforcement\KerbalJointReinforcement_1.3.version" "_release\GameData131\GameData\KerbalJointReinforcement\KerbalJointReinforcement.version"

copy "KerbalJointReinforcement\KerbalJointReinforcement\bin\Release 1.3.1\KerbalJointReinforcement.dll" "_release\GameData131\GameData\KerbalJointReinforcement\Plugins\KerbalJointReinforcement.dll"

copy "GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml" "_release\GameData131\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml"

del "_release\KerbalJointReinforcement_v3.4.1_for_1.3.1.zip"
C:\PACL\PACOMP.EXE -a -r -p "_release\KerbalJointReinforcement_v3.4.1_for_1.3.1.zip" "_release\GameData131\*"
