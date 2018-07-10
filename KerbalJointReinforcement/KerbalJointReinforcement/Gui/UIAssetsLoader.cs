using System;
using System.IO;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;


namespace KerbalJointReinforcement
{
#if IncludeAnalyzer

	[KSPAddon(KSPAddon.Startup.MainMenu, false)]
	public class UIAssetsLoader : MonoBehaviour
	{
		internal static AssetBundle KJRAssetBundle;

		// windows
		internal static GameObject settingsWindowPrefab;

		internal static GameObject basicTooltipPrefab;

		// images and icons
		internal static List<Texture2D> iconAssets;
		internal static List<UnityEngine.Sprite> spriteAssets;
		
		public static bool allPrefabsReady = false;

		public IEnumerator LoadBundle(string location)
		{
			while(!Caching.ready)
				yield return null;

			using (WWW www = WWW.LoadFromCacheOrDownload(location, 1))
			{
				yield return www;
				KJRAssetBundle = www.assetBundle;

				LoadBundleAssets();
			}
		}
		
		private void LoadBundleAssets()
		{
			var prefabs = KJRAssetBundle.LoadAllAssets<GameObject>();
			int prefabCounter = 0;

			for(int i = 0; i < prefabs.Length; i++)
			{
				if(prefabs[i].name == "KJRSettingsWindowPrefab")
				{
					settingsWindowPrefab = prefabs[i] as GameObject;
					prefabCounter++;
					Logger.Log("Successfully loaded settings window prefab", Logger.Level.Debug);
				}
			/*
				if(prefabs[i].name == "UISettingsWindowPrefab")
				{
					uiSettingsWindowPrefab = prefabs[i] as GameObject;
					prefabCounter++;
					Logger.Log("Successfully loaded UI settings window prefab", Logger.Level.Debug);
				}
			*/
				if(prefabs[i].name == "BasicTooltipPrefab")
				{
					basicTooltipPrefab = prefabs[i] as GameObject;
					prefabCounter++;
					Logger.Log("Successfully loaded BasicTooltipPrefab", Logger.Level.Debug);
				}
			}

			allPrefabsReady = (prefabCounter >= 2);

			spriteAssets = new List<UnityEngine.Sprite>();
			var sprites = KJRAssetBundle.LoadAllAssets<UnityEngine.Sprite>();

			for(int i = 0; i < sprites.Length; i++)
			{
				if(sprites[i] != null)
				{
					spriteAssets.Add(sprites[i]);
					Logger.Log("Successfully loaded Sprite " + sprites[i].name, Logger.Level.Debug);
				}
			}

			iconAssets = new List<Texture2D>();
			var icons = KJRAssetBundle.LoadAllAssets<Texture2D>();

			for(int i = 0; i < icons.Length; i++)
			{
				if(icons[i] != null)
				{
					iconAssets.Add(icons[i]);
					Logger.Log("Successfully loaded texture " + icons[i].name, Logger.Level.Debug);
				}
			}
		}

		public void Start()
		{
			if(allPrefabsReady)
				return;

			var assemblyFile = Assembly.GetExecutingAssembly().Location;
			var bundlePath = "file://" + assemblyFile.Replace(new FileInfo(assemblyFile).Name, "").Replace("\\","/") + "../AssetBundles/";

			Logger.Log("Loading bundles from BundlePath: " + bundlePath, Logger.Level.Debug);

			Caching.CleanCache();

			StartCoroutine(LoadBundle(bundlePath + "kjr_ui_objects.ksp"));
		}

		public void OnDestroy()
		{
		}
	}

#endif
}

