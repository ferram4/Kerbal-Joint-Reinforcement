cd /D "%~dp0"

md "_release"
md "_release\GameData143"
md "_release\GameData143\GameData"
md "_release\GameData143\GameData\KerbalJointReinforcement"
md "_release\GameData143\GameData\KerbalJointReinforcement\AssetBundles"
md "_release\GameData143\GameData\KerbalJointReinforcement\Plugins"
md "_release\GameData143\GameData\KerbalJointReinforcement\Plugins\PluginData"
md "_release\GameData143\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement"

copy "GameData\KerbalJointReinforcement\KerbalJointReinforcement_1.4.version" "_release\GameData143\GameData\KerbalJointReinforcement\KerbalJointReinforcement.version"

copy "GameData\KerbalJointReinforcement\AssetBundles\kjr_ui_objects.ksp" "_release\GameData143\GameData\KerbalJointReinforcement\AssetBundles\kjr_ui_objects.ksp"

copy "KerbalJointReinforcement\KerbalJointReinforcement\bin\Release 1.4\KerbalJointReinforcement.dll" "_release\GameData143\GameData\KerbalJointReinforcement\Plugins\KerbalJointReinforcement.dll"

copy "GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml" "_release\GameData143\GameData\KerbalJointReinforcement\Plugins\PluginData\KerbalJointReinforcement\config.xml"

del "_release\KerbalJointReinforcement_v3.4.1_for_1.4.zip"
C:\PACL\PACOMP.EXE -a -r -p "_release\KerbalJointReinforcement_v3.4.1_for_1.4.zip" "_release\GameData143\*"
